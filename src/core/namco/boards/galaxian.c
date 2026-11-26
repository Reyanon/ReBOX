/*  Namco Galaxian hardware and its derivatives: Galaxian and its clones / bootlegs, Moon Cresta, King & Balloon,
    Pisces, Uniwar S, Black Hole, Zig Zag, Jump Bug, Check Man, Moon Shuttle, Fantastic, Time Fighter, Ghost Muncher,
    Crazy Kong and Bagman bootlegs, ... and the Konami boards with the Scramble sound board: Scramble, Super Cobra,
    The End, Frogger, Amidar, Turtles, Anteater, Calipso, Lost Tomb, Armored Car, Moon War, Tazz-Mania, Mighty Monkey,
    Ozon I, Explorer, Take Off, Namennayo ...
    Ported from MAME 0.289 src/mame/galaxian/galaxian.cpp, galaxian.h, galaxian_v.cpp (BSD-3-Clause, copyright-holders:
    Aaron Giles, Couriersud, Robbbert), the Konami sound board's filters of nl_konami.cpp (CC0-1.0, Couriersud), the
    Crazy Climber sample player of src/mame/shared/cclimber_a.cpp (BSD-3-Clause, Nicola Salmoria, hap) and MAME's
    compute_resistor_weights (src/emu/video/resnet.cpp, BSD-3-Clause, Couriersud).
    A Z80 at 3.072 MHz (Sidam bootlegs 3 MHz), a 32 x 32 tilemap of 8x8 2 bpp characters with a scroll and colour per
    column, eight 16x16 sprites a generator, the shells and missiles counted in the line and the 17-bit LFSR stars.
    The frame (256 x 224 at 60.6 Hz) is drawn at vblank from a copy of the video and object RAM: the tilemap from a cache
    redrawn where a tile changed, the stars from the list of the LFSR's star positions (about 250 in its period).
    MAME draws at 3 pixels a 6 MHz pixel for the stars' two clocks; here a pixel each, the star shown when either of its
    two RNG clocks has one. Sound: the Galaxian / Moon Cresta discrete circuit (galaxian_a.c), the Konami sound board
    (a Z80 at 1.79 MHz, one or two AY-3-8910s, each channel through its switched RC filter as MAME's older flt_rc model of
    the board: 1k in, 5.1k out, 0.22 / 0.047 uF), or the bootlegs' AY-3-8910s and DACs.
    The memory maps are MAME's address maps as tables (the later entry wins, mirrors as MAME's), resolved into direct
    256-byte pages where a page is plain ROM or RAM.
    This file is part of ReBOX. GPL-2.0-or-later.

    SF-X and Monster Zero: the sample board's third Z80, its PPI and 8-bit DAC(s), the tilemap in columns with a scroll
    a row, ORIENTATION_FLIP_X (the 216 visible pixels mirrored here). Kong and Time Fighter's PROMs are not dumped (MAME
    draws them black): a plain palette instead.
    Not done (the configurations not listed by gx_supported): the boards with a speech chip or a CPU of another kind
    (Space Battle: I8039 + SP0250, Scorpion: Digitalker, BMX Stunts: 6502 + SN76489A), the Ten Spot game selection CPU;
    the watchdog, the coin counters, lockout and lamps; the mid-frame video RAM changes (the whole frame at vblank, Time
    Fighter's sprite bases per 64 lines kept).
*/
#include "../../rbx.h"
#include "../../cpu/z80/z80.h"
#include "../../sound/ay8910.h"
#include "../../sound/filter_rc.h"
#include "../../machine/i8255.h"
#include "../../rbx_prof.h"
#include "galaxian_a.h"

#include <stdlib.h>
#include <string.h>

#define VIS_W 256
#define VIS_H 224
#define VIS_Y0 16
#define LINES 264
#define STAR_P ((1 << 17) - 1)
#define NPENS 360
#define POOL_SIZE 0x4000

// ---- palette layout (the PROM's colours first)
#define PEN_STAR 256
#define PEN_BLACK 320
#define PEN_BUL 321          // (8: the bullets' colours)
#define PEN_BUL_TE 329       // (8: The End's, blue and green swapped)
#define PEN_YELLOW 337
#define PEN_MSH 338          // (8: Moon Shuttle's)
#define PEN_PURPLE 346
#define PEN_BLUE 347         // (0, 0, 0x56: the Scramble background)
#define PEN_FROGBLUE 348     // (0, 0, 0x47: Frogger's river)
#define PEN_TURTLES 349      // (8: Turtles' background combinations)

// ---- the RAM pool
#define PO_VRAM 0x0000
#define PO_OBJ 0x0800
#define PO_EXT 0x0c00
#define PO_RA 0x1000
#define PO_RB 0x2000
#define PO_RC 0x3000

enum { K_NONE, K_ROM, K_RAM, K_PORT, K_FN, K_NOP, K_BANK };
typedef struct { uint16_t lo, hi, mirror; uint8_t rk, wk; uint32_t ra, wa; } me_t;
#define E_ROM(lo, hi)            { lo, hi, 0, K_ROM, K_NONE, lo, 0 }
#define E_ROMR(lo, hi, off)      { lo, hi, 0, K_ROM, K_NONE, off, 0 }
#define E_RAM(lo, hi, m, po)     { lo, hi, m, K_RAM, K_RAM, po, po }
#define E_PR(lo, m, port)        { lo, lo, m, K_PORT, K_NONE, port, 0 }
#define E_R(lo, hi, m, fn)       { lo, hi, m, K_FN, K_NONE, fn, 0 }
#define E_W(lo, hi, m, fn)       { lo, hi, m, K_NONE, K_FN, 0, fn }
#define E_RW(lo, hi, m, fn)      { lo, hi, m, K_FN, K_FN, fn, fn }
#define E_NOPW(lo, hi, m)        { lo, hi, m, K_NONE, K_NOP, 0, 0 }
#define E_NOPR(lo, hi, m)        { lo, hi, m, K_NOP, K_NONE, 0, 0 }
#define E_NOPRW(lo, hi, m)       { lo, hi, m, K_NOP, K_NOP, 0, 0 }
#define E_BANK(lo, hi, bank)     { lo, hi, 0, K_BANK, K_NONE, bank, 0 }
#define E_END                    { 0, 0, 0, 0xff, 0xff, 0, 0 }

// input ports by tag
enum { P_IN0, P_IN1, P_IN2, P_IN3, P_DSW, P_FAKE, P_COINAGE, P_DIAL1, P_DIAL2, P_GAMESEL, P_GLIN0, P_GLIN1, P_GLIN2,
       P_GMIN0, P_GMIN1, P_GMIN2, P_COUNT };
static const char *const s_port_tags[P_COUNT] = { "IN0", "IN1", "IN2", "IN3", "DSW", "FAKE", "COINAGE", "P1_DIAL",
                                                  "P2_DIAL", "GAMESEL", "GLIN0", "GLIN1", "GLIN2", "GMIN0", "GMIN1",
                                                  "GMIN2" };

// handlers (read / write function ids)
enum {
	F_NONE,
	F_PPI0, F_PPI1, F_PPI2, F_THEEND_PPI, F_FROGGER_PPI, F_FROGF_PPI, F_TURTLES_PPI0, F_TURTLES_PPI1, F_FROGGERAM_PPI,
	F_AY0_ADDR, F_AY0_DATA, F_AY1_ADDR, F_AY1_DATA,
	F_JUMPBUG_PROT, F_CHECKMAJ_PROT, F_DINGO_3000, F_DINGO_3035, F_DINGOE_3001,
	F_IRQEN, F_STARS, F_FLIPX, F_FLIPY, F_FLIPXY, F_GFXBANK, F_BG_EN, F_BG_R, F_BG_G, F_BG_B,
	F_LFO, F_SOUND, F_PITCH,
	F_SOUNDLATCH, F_KSND_CTRL, F_EXPLORER_SCTRL, F_FROGGERMC_SCTRL,
	F_ZZ_AY, F_ZZ_BANK, F_FOURPLAY_BANK, F_VIDEIGHT_BANK, F_VIDEIGHT_GFX, F_GUTT_BANK,
	F_KB_SOUND1, F_KB_SOUND2, F_KB_SPEECH,
	F_MSH_CS, F_CC_TRIG, F_CC_RATE, F_CC_VOL,
	F_CHECKMAN_CMD, F_MONSTERZ_PROT,
};

// ---- the memory maps (MAME's address maps; galaxian.cpp)
#define GX_BASE \
	E_ROM(0x0000, 0x3fff), E_RAM(0x4000, 0x43ff, 0x0400, PO_RA), E_RAM(0x5000, 0x53ff, 0x0400, PO_VRAM), \
	E_RAM(0x5800, 0x58ff, 0x0700, PO_OBJ), E_PR(0x6000, 0x07ff, P_IN0), E_PR(0x6800, 0x07ff, P_IN1), \
	E_PR(0x7000, 0x07ff, P_IN2), E_W(0x7001, 0x7001, 0x07f8, F_IRQEN), E_W(0x7004, 0x7004, 0x07f8, F_STARS), \
	E_W(0x7006, 0x7006, 0x07f8, F_FLIPX), E_W(0x7007, 0x7007, 0x07f8, F_FLIPY), E_NOPR(0x7800, 0x7800, 0x07ff)
#define GX_DISC \
	E_W(0x6004, 0x6007, 0x07f8, F_LFO), E_W(0x6800, 0x6807, 0x07f8, F_SOUND), E_W(0x7800, 0x7800, 0x07ff, F_PITCH)
#define MC_BASE \
	E_ROM(0x0000, 0x3fff), E_RAM(0x8000, 0x83ff, 0x0400, PO_RA), E_RAM(0x9000, 0x93ff, 0x0400, PO_VRAM), \
	E_RAM(0x9800, 0x98ff, 0x0700, PO_OBJ), E_PR(0xa000, 0x07ff, P_IN0), E_PR(0xa800, 0x07ff, P_IN1), \
	E_PR(0xb000, 0x07ff, P_IN2), E_W(0xb000, 0xb000, 0x07f8, F_IRQEN), E_W(0xb004, 0xb004, 0x07f8, F_STARS), \
	E_W(0xb006, 0xb006, 0x07f8, F_FLIPX), E_W(0xb007, 0xb007, 0x07f8, F_FLIPY), E_NOPR(0xb800, 0xb800, 0x07ff)
#define MC_DISC \
	E_W(0xa004, 0xa007, 0x07f8, F_LFO), E_W(0xa800, 0xa807, 0x07f8, F_SOUND), E_W(0xb800, 0xb800, 0x07ff, F_PITCH)
#define MC_MAP MC_BASE, MC_DISC, E_W(0xa000, 0xa002, 0x07f8, F_GFXBANK)
#define CKG_BASE \
	E_ROMR(0x0000, 0x5fff, 0), E_RAM(0x6000, 0x67ff, 0, PO_RA), E_RAM(0x9000, 0x93ff, 0, PO_VRAM), \
	E_RAM(0x9800, 0x9bff, 0, PO_OBJ)
#define CKG_MAP \
	CKG_BASE, E_RAM(0x6800, 0x6bff, 0, PO_RB), E_PR(0xc000, 0, P_IN0), E_W(0xc004, 0xc007, 0, F_LFO), \
	E_PR(0xc400, 0, P_IN1), E_W(0xc400, 0xc407, 0, F_SOUND), E_PR(0xc800, 0, P_IN2), E_W(0xc801, 0xc801, 0, F_IRQEN), \
	E_NOPW(0xc804, 0xc804, 0), E_W(0xc806, 0xc806, 0, F_FLIPX), E_W(0xc807, 0xc807, 0, F_FLIPY), \
	E_NOPR(0xcc00, 0xcc00, 0), E_W(0xcc00, 0xcc00, 0, F_PITCH)
#define THEEND_MAP \
	E_ROM(0x0000, 0x3fff), E_RAM(0x4000, 0x47ff, 0, PO_RA), E_RAM(0x4800, 0x4bff, 0x0400, PO_VRAM), \
	E_RAM(0x5000, 0x50ff, 0x0700, PO_OBJ), E_W(0x6801, 0x6801, 0x07f8, F_IRQEN), E_W(0x6803, 0x6803, 0x07f8, F_BG_EN), \
	E_W(0x6804, 0x6804, 0x07f8, F_STARS), E_W(0x6806, 0x6806, 0x07f8, F_FLIPX), E_W(0x6807, 0x6807, 0x07f8, F_FLIPY), \
	E_NOPR(0x7000, 0x7000, 0x07ff), E_RW(0x8000, 0xffff, 0, F_THEEND_PPI)
#define SCOBRA_MAP \
	E_ROM(0x0000, 0x7fff), E_RAM(0x8000, 0x87ff, 0x4000, PO_RA), E_RAM(0x8800, 0x8bff, 0x4400, PO_VRAM), \
	E_RAM(0x9000, 0x90ff, 0x4700, PO_OBJ), E_RW(0x9800, 0x9803, 0x47fc, F_PPI0), E_RW(0xa000, 0xa003, 0x47fc, F_PPI1), \
	E_W(0xa801, 0xa801, 0x47f8, F_IRQEN), E_W(0xa803, 0xa803, 0x47f8, F_BG_EN), E_W(0xa804, 0xa804, 0x47f8, F_STARS), \
	E_W(0xa806, 0xa806, 0x47f8, F_FLIPX), E_W(0xa807, 0xa807, 0x47f8, F_FLIPY), E_NOPR(0xb000, 0xb000, 0x47ff)
#define JUMPBUGBRF_MAP \
	E_ROM(0x0000, 0x3fff), E_RAM(0x4000, 0x47ff, 0, PO_RA), E_RAM(0x4800, 0x4bff, 0x0400, PO_VRAM), \
	E_RAM(0x5000, 0x50ff, 0x0700, PO_OBJ), E_PR(0x6000, 0x07ff, P_IN0), E_W(0x6002, 0x6006, 0x07f8, F_GFXBANK), \
	E_PR(0x6800, 0x07ff, P_IN1), E_PR(0x7000, 0x07ff, P_IN2), E_W(0x7001, 0x7001, 0x07f8, F_IRQEN), \
	E_W(0x7004, 0x7004, 0x07f8, F_STARS), E_W(0x7006, 0x7006, 0x07f8, F_FLIPX), E_W(0x7007, 0x7007, 0x07f8, F_FLIPY), \
	E_ROM(0x8000, 0xafff), E_R(0xb000, 0xbfff, 0, F_JUMPBUG_PROT)
#define BONGO_MAP \
	MC_BASE, E_ROMR(0x0000, 0x5fff, 0), E_NOPW(0xb000, 0xb000, 0x07f8), E_W(0xb001, 0xb001, 0x07f8, F_IRQEN), \
	E_NOPW(0xb800, 0xb800, 0x07ff)
#define EXPLORER_MAP \
	E_ROM(0x0000, 0x3fff), E_RAM(0x4000, 0x47ff, 0, PO_RA), E_RAM(0x4800, 0x4bff, 0x0400, PO_VRAM), \
	E_RAM(0x5000, 0x50ff, 0x0700, PO_OBJ), E_NOPW(0x6800, 0x6800, 0x07f8), E_W(0x6801, 0x6801, 0x07f8, F_IRQEN), \
	E_W(0x6806, 0x6806, 0x07f8, F_FLIPX), E_W(0x6807, 0x6807, 0x07f8, F_FLIPY), E_NOPRW(0x7000, 0x7000, 0x07ff), \
	E_PR(0x8000, 0x0ffc, P_IN0), E_W(0x8000, 0x8000, 0x0fff, F_SOUNDLATCH), E_PR(0x8001, 0x0ffc, P_IN1), \
	E_PR(0x8002, 0x0ffc, P_IN2), E_PR(0x8003, 0x0ffc, P_IN3), E_W(0x9000, 0x9000, 0x0fff, F_EXPLORER_SCTRL)

static const me_t map_galaxian[] = { GX_BASE, GX_DISC, E_END };
static const me_t map_galaxian_base[] = { GX_BASE, E_END };
static const me_t map_pisces[] = { GX_BASE, GX_DISC, E_W(0x6002, 0x6002, 0x07f8, F_GFXBANK), E_END };
static const me_t map_frogg[] = { GX_BASE, GX_DISC, E_RAM(0x4400, 0x47ff, 0, PO_RB), E_END };
static const me_t map_mandingarf[] = { GX_BASE, GX_DISC, E_RAM(0x4400, 0x47ff, 0, PO_RB), E_ROMR(0xc000, 0xc7ff, 0xc000), E_END };
static const me_t map_victoryc[] = { GX_BASE, GX_DISC, E_NOPW(0x7004, 0x7004, 0x07f8), E_RAM(0x8000, 0x87ff, 0, PO_RB), E_END };
static const me_t map_mooncrst[] = { MC_MAP, E_END };
static const me_t map_froggermc[] = {
	MC_BASE, E_RAM(0x8400, 0x87ff, 0, PO_RB), E_W(0xa800, 0xa800, 0x07ff, F_SOUNDLATCH),
	E_W(0xb001, 0xb001, 0x07f8, F_FROGGERMC_SCTRL), E_END };
static const me_t map_thepitm[] = {
	MC_MAP, E_ROMR(0x0000, 0x47ff, 0), E_NOPW(0xb000, 0xb000, 0x07f8), E_W(0xb001, 0xb001, 0x07f8, F_IRQEN),
	E_NOPW(0xb004, 0xb004, 0x07f8), E_END };
static const me_t map_porter[] = {
	MC_MAP, E_ROMR(0x0000, 0x4fff, 0), E_RAM(0x8400, 0x87ff, 0, PO_RB), E_W(0xa002, 0xa002, 0x07f8, F_GFXBANK), E_END };
static const me_t map_skybase[] = {
	MC_MAP, E_ROMR(0x0000, 0x5fff, 0), E_RAM(0x8400, 0x87ff, 0, PO_RB), E_W(0xa002, 0xa002, 0x07f8, F_GFXBANK), E_END };
static const me_t map_kong[] = { MC_MAP, E_ROMR(0x0000, 0x7fff, 0), E_RAM(0x8400, 0x87ff, 0, PO_RB), E_END };
static const me_t map_scorpnmc[] = {
	MC_MAP, E_RAM(0x4000, 0x47ff, 0, PO_RB), E_ROMR(0x5000, 0x67ff, 0x5000), E_NOPRW(0x8400, 0x87ff, 0),
	E_NOPW(0xb000, 0xb000, 0x07f8), E_W(0xb001, 0xb001, 0x07f8, F_IRQEN), E_END };
static const me_t map_bongo[] = { BONGO_MAP, E_END };
static const me_t map_bongog[] = { BONGO_MAP, MC_DISC, E_PR(0xb000, 0x07ff, P_DSW), E_END };
static const me_t map_ckongg[] = { CKG_MAP, E_END };
static const me_t map_bigkonggx[] = { CKG_MAP, E_ROM(0xd400, 0xe3ff), E_END };
static const me_t map_ckongmc[] = {
	CKG_BASE, MC_DISC, E_RAM(0x6800, 0x6bff, 0, PO_RB), E_PR(0xa000, 0, P_IN0), E_PR(0xa800, 0, P_IN1),
	E_PR(0xb000, 0, P_IN2), E_W(0xb001, 0xb001, 0, F_IRQEN), E_NOPW(0xb004, 0xb004, 0), E_W(0xb006, 0xb006, 0, F_FLIPX),
	E_W(0xb007, 0xb007, 0, F_FLIPY), E_NOPR(0xb800, 0xb800, 0), E_END };
static const me_t map_bagmanmc[] = {
	CKG_BASE, MC_DISC, E_PR(0xa000, 0, P_IN0), E_PR(0xa800, 0, P_IN1), E_PR(0xb000, 0, P_IN2), E_NOPW(0xb000, 0xb000, 0),
	E_W(0xb001, 0xb001, 0, F_IRQEN), E_W(0xb002, 0xb002, 0, F_GFXBANK), E_W(0xb006, 0xb006, 0, F_FLIPX),
	E_W(0xb007, 0xb007, 0, F_FLIPY), E_NOPR(0xb800, 0xb800, 0), E_END };
static const me_t map_fantastc[] = {
	E_ROM(0x0000, 0x7fff), E_RAM(0x8000, 0x87ff, 0, PO_RA), E_W(0x8803, 0x8803, 0, F_AY0_ADDR),
	E_R(0x8807, 0x8807, 0, F_AY0_DATA), E_W(0x880b, 0x880b, 0, F_AY0_DATA), E_W(0x880c, 0x880c, 0, F_AY1_ADDR),
	E_R(0x880d, 0x880d, 0, F_AY1_DATA), E_W(0x880e, 0x880e, 0, F_AY1_DATA), E_RAM(0x9000, 0x93ff, 0x0400, PO_VRAM),
	E_RAM(0x9800, 0x9bff, 0x0400, PO_OBJ), E_PR(0xa000, 0x07ff, P_IN0), E_PR(0xa800, 0x07ff, P_IN1),
	E_PR(0xb000, 0x07ff, P_IN2), E_W(0xb000, 0xb000, 0x07f8, F_IRQEN), E_NOPR(0xb800, 0xb800, 0x07ff),
	E_NOPRW(0xfffe, 0xfffe, 0), E_END };
static const me_t map_zigzag[] = {
	E_ROM(0x0000, 0x1fff), E_BANK(0x2000, 0x2fff, 0), E_BANK(0x3000, 0x3fff, 1), E_RAM(0x4000, 0x47ff, 0, PO_RA),
	E_W(0x4800, 0x4fff, 0, F_ZZ_AY), E_RAM(0x5000, 0x53ff, 0x0400, PO_VRAM), E_RAM(0x5800, 0x58ff, 0x0700, PO_OBJ),
	E_PR(0x6000, 0x07ff, P_IN0), E_PR(0x6800, 0x07ff, P_IN1), E_PR(0x7000, 0x07ff, P_IN2),
	E_W(0x7001, 0x7001, 0x07f8, F_IRQEN), E_W(0x7002, 0x7002, 0x07f8, F_ZZ_BANK), E_W(0x7004, 0x7004, 0x07f8, F_STARS),
	E_W(0x7006, 0x7006, 0x07f8, F_FLIPX), E_W(0x7007, 0x7007, 0x07f8, F_FLIPY), E_NOPR(0x7800, 0x7800, 0x07ff), E_END };
static const me_t map_gmgalax[] = { GX_BASE, GX_DISC, E_BANK(0x0000, 0x3fff, 0), E_END };
static const me_t map_ozon1[] = {
	E_ROM(0x0000, 0x2fff), E_RAM(0x4000, 0x43ff, 0, PO_RA), E_RAM(0x4800, 0x4bff, 0x0400, PO_VRAM),
	E_RAM(0x5000, 0x50ff, 0, PO_OBJ), E_W(0x6801, 0x6801, 0, F_IRQEN), E_W(0x6806, 0x6806, 0, F_FLIPX),
	E_W(0x6807, 0x6807, 0, F_FLIPY), E_RW(0x8100, 0x8103, 0, F_PPI0), E_END };
static const me_t map_theend[] = { THEEND_MAP, E_END };
static const me_t map_froggervd[] = { THEEND_MAP, E_NOPR(0x7800, 0x7800, 0x07ff), E_END };
static const me_t map_namenayo[] = {
	E_ROM(0x0000, 0x3fff), E_RAM(0x4000, 0x4fff, 0, PO_RA), E_ROM(0x5000, 0x6fff), E_RAM(0xc800, 0xc8ff, 0, PO_OBJ),
	E_RAM(0xd000, 0xd3ff, 0, PO_VRAM), E_NOPW(0xd800, 0xd800, 0), E_RAM(0xe000, 0xe01f, 0, PO_EXT),
	E_W(0xe801, 0xe801, 0, F_IRQEN), E_W(0xe806, 0xe806, 0x07f8, F_FLIPX), E_W(0xe807, 0xe807, 0x07f8, F_FLIPY),
	E_RW(0xf900, 0xf903, 0, F_PPI0), E_RW(0xfa00, 0xfa03, 0, F_PPI1), E_NOPR(0xf000, 0xf000, 0), E_END };
static const me_t map_mandinka[] = {
	E_ROM(0x0000, 0x3fff), E_RAM(0x8000, 0x87ff, 0, PO_RA), E_RAM(0x9000, 0x97ff, 0, PO_VRAM),
	E_RAM(0x9800, 0x98ff, 0, PO_OBJ), E_W(0xa001, 0xa001, 0, F_IRQEN), E_W(0xa003, 0xa003, 0, F_BG_EN),
	E_W(0xa006, 0xa006, 0, F_FLIPX), E_W(0xa007, 0xa007, 0, F_FLIPY), E_NOPR(0xa800, 0xa800, 0),
	E_RW(0xb000, 0xb003, 0, F_PPI0), E_RW(0xb800, 0xb803, 0, F_PPI1), E_END };
static const me_t map_explorer[] = { EXPLORER_MAP, E_END };
static const me_t map_astroamb[] = {
	E_ROM(0x0000, 0x3fff), E_RAM(0x4000, 0x47ff, 0, PO_RA), E_RAM(0x4800, 0x4bff, 0x0400, PO_VRAM),
	E_RAM(0x5000, 0x50ff, 0x0700, PO_OBJ), E_PR(0x6000, 0, P_IN0), E_W(0x6004, 0x6007, 0, F_LFO), E_PR(0x6800, 0, P_IN1),
	E_W(0x6800, 0x6807, 0, F_SOUND), E_PR(0x7000, 0, P_IN2), E_W(0x7001, 0x7001, 0, F_IRQEN),
	E_W(0x7003, 0x7003, 0, F_BG_EN), E_W(0x7004, 0x7004, 0, F_STARS), E_W(0x7006, 0x7006, 0, F_FLIPX),
	E_W(0x7007, 0x7007, 0, F_FLIPY), E_NOPR(0x7800, 0x7800, 0), E_W(0x7800, 0x7800, 0, F_PITCH), E_END };
static const me_t map_scobra[] = { SCOBRA_MAP, E_END };
static const me_t map_mimonkey[] = {
	E_ROM(0x0000, 0x3fff), E_RAM(0x8000, 0x87ff, 0, PO_RA), E_RAM(0x8800, 0x8bff, 0x0400, PO_VRAM),
	E_RAM(0x9000, 0x90ff, 0, PO_OBJ), E_RW(0x9800, 0x9803, 0, F_PPI0), E_RW(0xa000, 0xa003, 0, F_PPI1),
	E_W(0xa800, 0xa802, 0, F_GFXBANK), E_W(0xa801, 0xa801, 0, F_IRQEN), E_W(0xa804, 0xa804, 0, F_BG_EN),
	E_W(0xa806, 0xa806, 0, F_FLIPX), E_W(0xa807, 0xa807, 0, F_FLIPY), E_NOPR(0xb000, 0xb000, 0), E_ROM(0xc000, 0xffff),
	E_END };
static const me_t map_mimonscr[] = {
	E_ROM(0x0000, 0x3fff), E_RAM(0x4000, 0x47ff, 0, PO_RA), E_RAM(0x4800, 0x4bff, 0x0400, PO_VRAM),
	E_RAM(0x5000, 0x50ff, 0, PO_OBJ), E_W(0x6800, 0x6802, 0, F_GFXBANK), E_W(0x6801, 0x6801, 0, F_IRQEN),
	E_W(0x6804, 0x6804, 0, F_BG_EN), E_W(0x6806, 0x6806, 0, F_FLIPX), E_W(0x6807, 0x6807, 0, F_FLIPY),
	E_NOPR(0x7000, 0x7000, 0), E_RW(0x8100, 0x8103, 0, F_PPI0), E_RW(0x8200, 0x8203, 0, F_PPI1), E_ROM(0xc000, 0xffff),
	E_END };
static const me_t map_ckongs[] = {
	CKG_BASE, E_RAM(0x6800, 0x6bff, 0, PO_RB), E_RW(0x7000, 0x7003, 0, F_PPI0), E_RW(0x7800, 0x7803, 0, F_PPI1),
	E_W(0xa801, 0xa801, 0, F_IRQEN), E_NOPW(0xa803, 0xa803, 0), E_NOPW(0xa805, 0xa805, 0), E_W(0xa806, 0xa806, 0, F_FLIPX),
	E_W(0xa807, 0xa807, 0, F_FLIPY), E_NOPR(0xb000, 0xb000, 0), E_END };
static const me_t map_anteateruk[] = {
	E_ROM(0x0000, 0x03ff), E_RAM(0x0400, 0x0bff, 0, PO_RA), E_RAM(0x0c00, 0x0fff, 0, PO_VRAM),
	E_W(0x1001, 0x1001, 0x01f8, F_IRQEN), E_W(0x1003, 0x1003, 0x01f8, F_BG_EN), E_W(0x1004, 0x1004, 0x01f8, F_STARS),
	E_W(0x1006, 0x1006, 0x01f8, F_FLIPX), E_W(0x1007, 0x1007, 0x01f8, F_FLIPY), E_RAM(0x1200, 0x12ff, 0x0100, PO_OBJ),
	E_NOPR(0x1400, 0x1400, 0x03ff), E_ROM(0x4000, 0xbfff), E_RW(0xc000, 0xc003, 0x3efc, F_PPI1),
	E_RW(0xc100, 0xc103, 0x3efc, F_PPI0), E_END };
static const me_t map_anteaterg[] = {
	E_ROM(0x0000, 0x03ff), E_RAM(0x0400, 0x0bff, 0, PO_RA), E_RAM(0x0c00, 0x0fff, 0, PO_VRAM),
	E_RAM(0x2000, 0x20ff, 0x0300, PO_OBJ), E_RW(0x2400, 0x2403, 0x01fc, F_PPI1), E_W(0x2601, 0x2601, 0x01f8, F_IRQEN),
	E_W(0x2603, 0x2603, 0x01f8, F_BG_EN), E_W(0x2604, 0x2604, 0x01f8, F_STARS), E_W(0x2606, 0x2606, 0x01f8, F_FLIPX),
	E_W(0x2607, 0x2607, 0x01f8, F_FLIPY), E_ROM(0x4000, 0x7bff), E_RAM(0x7c00, 0x7fff, 0, PO_VRAM),
	E_ROM(0x8000, 0xbfff), E_NOPR(0xf400, 0xf400, 0x01ff), E_RW(0xf600, 0xf603, 0x01fc, F_PPI0), E_END };
static const me_t map_anteatergg[] = {
	GX_DISC, E_ROM(0x0000, 0x3fff), E_RAM(0x4000, 0x4fff, 0, PO_RA), E_RAM(0x5000, 0x53ff, 0, PO_VRAM),
	E_RAM(0x5800, 0x58ff, 0, PO_OBJ), E_PR(0x6000, 0x07ff, P_IN0), E_PR(0x6800, 0x07ff, P_IN1), E_PR(0x7000, 0x07ff, P_IN2),
	E_W(0x7001, 0x7001, 0x07f8, F_IRQEN), E_W(0x7004, 0x7004, 0x07f8, F_STARS), E_W(0x7006, 0x7006, 0x07f8, F_FLIPX),
	E_W(0x7007, 0x7007, 0x07f8, F_FLIPY), E_NOPR(0x7800, 0x7800, 0), E_END };
static const me_t map_jungsub[] = {
	E_ROM(0x0000, 0x3fff), E_RAM(0x4000, 0x4fff, 0, PO_RA), E_RAM(0x5000, 0x53ff, 0, PO_VRAM),
	E_RAM(0x5800, 0x58ff, 0, PO_OBJ), E_NOPRW(0x5900, 0x5aff, 0), E_PR(0x6000, 0x07ff, P_IN0), E_NOPW(0x6003, 0x6003, 0),
	E_NOPW(0x6184, 0x6184, 0), E_NOPW(0x6186, 0x6186, 0), E_PR(0x6800, 0x07ff, P_IN1), E_W(0x7001, 0x7001, 0x07f8, F_IRQEN),
	E_W(0x7006, 0x7006, 0x07f8, F_FLIPX), E_W(0x7007, 0x7007, 0x07f8, F_FLIPY), E_NOPR(0x7800, 0x7800, 0),
	E_NOPW(0x7800, 0x7800, 0), E_END };
static const me_t map_frogger[] = {
	E_ROM(0x0000, 0x3fff), E_RAM(0x8000, 0x87ff, 0, PO_RA), E_NOPR(0x8800, 0x8800, 0x07ff),
	E_RAM(0xa800, 0xabff, 0x0400, PO_VRAM), E_RAM(0xb000, 0xb0ff, 0x0700, PO_OBJ), E_W(0xb808, 0xb808, 0x07e3, F_IRQEN),
	E_W(0xb80c, 0xb80c, 0x07e3, F_FLIPY), E_W(0xb810, 0xb810, 0x07e3, F_FLIPX), E_RW(0xc000, 0xffff, 0, F_FROGGER_PPI),
	E_END };
static const me_t map_turtles[] = {
	E_ROM(0x0000, 0x7fff), E_RAM(0x8000, 0x87ff, 0x4000, PO_RA), E_RAM(0x9000, 0x93ff, 0x4400, PO_VRAM),
	E_RAM(0x9800, 0x98ff, 0x4700, PO_OBJ), E_W(0xa000, 0xa000, 0x47c7, F_BG_R), E_W(0xa008, 0xa008, 0x47c7, F_IRQEN),
	E_W(0xa010, 0xa010, 0x47c7, F_FLIPY), E_W(0xa018, 0xa018, 0x47c7, F_FLIPX), E_W(0xa020, 0xa020, 0x47c7, F_BG_G),
	E_W(0xa028, 0xa028, 0x47c7, F_BG_B), E_NOPR(0xa800, 0xa800, 0x47ff), E_RW(0xb000, 0xb03f, 0x47c0, F_TURTLES_PPI0),
	E_RW(0xb800, 0xb83f, 0x47c0, F_TURTLES_PPI1), E_END };
static const me_t map_turpinnv[] = {
	E_ROM(0x0000, 0x3fff), E_RAM(0x4000, 0x47ff, 0, PO_RA), E_RAM(0x4800, 0x4bff, 0x0400, PO_VRAM),
	E_RAM(0x5000, 0x50ff, 0x0700, PO_OBJ), E_W(0x6801, 0x6801, 0x07f8, F_IRQEN), E_W(0x6803, 0x6803, 0x07f8, F_BG_B),
	E_W(0x6804, 0x6804, 0x07f8, F_BG_G), E_W(0x6805, 0x6805, 0x07f8, F_BG_R), E_W(0x6806, 0x6806, 0x07f8, F_FLIPX),
	E_W(0x6807, 0x6807, 0x07f8, F_FLIPY), E_NOPR(0x7000, 0x7000, 0x07ff), E_RW(0x8100, 0x8103, 0, F_PPI0),
	E_RW(0x8200, 0x8203, 0, F_PPI1), E_ROMR(0xc000, 0xcfff, 0x4000), E_END };
static const me_t map_amigo2[] = {
	E_ROM(0x0000, 0x3fff), E_PR(0x4000, 0, P_IN0), E_W(0x4000, 0x4000, 0, F_SOUNDLATCH), E_PR(0x4001, 0, P_IN1),
	E_PR(0x4002, 0, P_IN2), E_PR(0x4003, 0, P_IN3), E_W(0x5000, 0x5000, 0, F_KSND_CTRL), E_RAM(0x8000, 0x87ff, 0, PO_RA),
	E_RAM(0x8800, 0x8bff, 0, PO_VRAM), E_RAM(0x9000, 0x90ff, 0, PO_OBJ), E_W(0xa000, 0xa000, 0, F_BG_R),
	E_W(0xa008, 0xa008, 0, F_IRQEN), E_W(0xa010, 0xa010, 0, F_FLIPY), E_W(0xa018, 0xa018, 0, F_FLIPX),
	E_W(0xa020, 0xa020, 0, F_BG_G), E_W(0xa028, 0xa028, 0, F_BG_B), E_NOPR(0xa800, 0xa800, 0), E_END };
static const me_t map_turpins[] = {
	E_ROM(0x0000, 0x7fff), E_RAM(0x8000, 0x87ff, 0, PO_RA), E_RAM(0x9000, 0x93ff, 0x0400, PO_VRAM),
	E_RAM(0x9800, 0x98ff, 0, PO_OBJ), E_RW(0xa000, 0xa003, 0, F_PPI0), E_W(0xa801, 0xa801, 0, F_IRQEN),
	E_W(0xa806, 0xa806, 0, F_FLIPX), E_W(0xa807, 0xa807, 0, F_FLIPY), E_NOPR(0xb800, 0xb800, 0),
	E_RW(0xc000, 0xc003, 0, F_PPI1), E_END };
static const me_t map_jumpbugbrf[] = { JUMPBUGBRF_MAP, E_END };
static const me_t map_jumpbug[] = {
	JUMPBUGBRF_MAP, E_W(0x5800, 0x5800, 0x00ff, F_AY0_DATA), E_W(0x5900, 0x5900, 0x00ff, F_AY0_ADDR), E_END };
static const me_t map_froggeram[] = {
	E_ROM(0x0000, 0x2fff), E_RW(0x4000, 0x43ff, 0, F_FROGGERAM_PPI), E_RAM(0x8000, 0x87ff, 0, PO_RA),
	E_NOPR(0x8800, 0x8800, 0), E_RAM(0xa800, 0xabff, 0, PO_VRAM), E_RAM(0xb000, 0xb0ff, 0, PO_OBJ),
	E_W(0xb801, 0xb801, 0, F_IRQEN), E_W(0xb806, 0xb806, 0, F_FLIPX), E_W(0xb807, 0xb807, 0, F_FLIPY), E_END };
static const me_t map_frogf[] = {
	E_ROM(0x0000, 0x3fff), E_RAM(0x8000, 0x87ff, 0, PO_RA), E_RAM(0x8800, 0x8bff, 0x0400, PO_VRAM),
	E_RAM(0x9000, 0x90ff, 0x0700, PO_OBJ), E_W(0xa802, 0xa802, 0x07f1, F_FLIPX), E_W(0xa804, 0xa804, 0x07f1, F_IRQEN),
	E_W(0xa806, 0xa806, 0x07f1, F_FLIPY), E_NOPR(0xb800, 0xb800, 0x07ff), E_RW(0xc000, 0xffff, 0, F_FROGF_PPI), E_END };
static const me_t map_mshuttle[] = {
	E_ROM(0x0000, 0x7fff), E_RAM(0x8000, 0x83ff, 0, PO_RA), E_RAM(0x9000, 0x93ff, 0x0400, PO_VRAM),
	E_RAM(0x9800, 0x98ff, 0x0700, PO_OBJ), E_PR(0xa000, 0, P_IN0), E_W(0xa000, 0xa000, 0, F_IRQEN),
	E_W(0xa001, 0xa001, 0, F_STARS), E_W(0xa002, 0xa002, 0, F_FLIPXY), E_W(0xa004, 0xa004, 0, F_CC_TRIG),
	E_W(0xa007, 0xa007, 0, F_MSH_CS), E_PR(0xa800, 0, P_IN1), E_W(0xa800, 0xa800, 0, F_CC_RATE), E_PR(0xb000, 0, P_IN2),
	E_W(0xb000, 0xb000, 0, F_CC_VOL), E_NOPR(0xb800, 0xb800, 0), E_END };
static const me_t map_fourplay[] = {
	E_BANK(0x0000, 0x3fff, 0), E_RAM(0x4000, 0x47ff, 0, PO_RA), E_RAM(0x5000, 0x53ff, 0x0400, PO_VRAM),
	E_RAM(0x5800, 0x58ff, 0x0700, PO_OBJ), E_PR(0x6000, 0, P_IN0), E_PR(0x6800, 0, P_IN1), E_PR(0x7000, 0, P_IN2),
	E_NOPR(0x7800, 0x7fff, 0), E_NOPW(0x6002, 0x6002, 0), E_W(0x6004, 0x6007, 0, F_LFO), E_W(0x6800, 0x6807, 0, F_SOUND),
	E_W(0x7001, 0x7001, 0, F_IRQEN), E_W(0x7002, 0x7003, 0, F_FOURPLAY_BANK), E_W(0x7004, 0x7004, 0, F_STARS),
	E_NOPW(0x7005, 0x7005, 0), E_W(0x7006, 0x7006, 0, F_FLIPX), E_W(0x7007, 0x7007, 0, F_FLIPY), E_NOPW(0x7008, 0x7008, 0),
	E_W(0x7800, 0x7800, 0, F_PITCH), E_END };
static const me_t map_videight[] = {
	E_BANK(0x0000, 0x3fff, 0), E_RAM(0x4000, 0x47ff, 0, PO_RA), E_RAM(0x5000, 0x53ff, 0x0400, PO_VRAM),
	E_RAM(0x5800, 0x58ff, 0x0700, PO_OBJ), E_PR(0x6000, 0, P_IN0), E_PR(0x6800, 0, P_IN1), E_PR(0x7000, 0, P_IN2),
	E_NOPR(0x7800, 0x7fff, 0), E_W(0x6000, 0x6002, 0, F_VIDEIGHT_GFX), E_W(0x6004, 0x6007, 0, F_LFO),
	E_W(0x6800, 0x6807, 0, F_SOUND), E_NOPW(0x6808, 0x68ff, 0), E_W(0x7001, 0x7001, 0, F_IRQEN),
	E_W(0x7002, 0x7005, 0, F_VIDEIGHT_BANK), E_W(0x7006, 0x7006, 0, F_FLIPX), E_W(0x7007, 0x7007, 0, F_FLIPY),
	E_NOPW(0x7008, 0x7008, 0), E_W(0x7800, 0x7800, 0, F_PITCH), E_END };
static const me_t map_guttangt[] = {
	E_ROM(0x0000, 0x1fff), E_NOPW(0x0000, 0x1fff, 0), E_BANK(0x2000, 0x27ff, 0), E_ROM(0x2800, 0x3fff),
	E_NOPW(0x2800, 0x3fff, 0), E_RAM(0x4000, 0x47ff, 0, PO_RA), E_RAM(0x5000, 0x53ff, 0, PO_VRAM),
	E_RAM(0x5800, 0x58ff, 0, PO_OBJ), E_PR(0x6000, 0, P_IN0), E_W(0x6000, 0x6000, 0, F_GUTT_BANK), E_PR(0x6800, 0, P_IN1),
	GX_DISC, E_PR(0x7000, 0, P_IN2), E_W(0x7001, 0x7001, 0, F_IRQEN), E_W(0x7006, 0x7006, 0, F_FLIPX),
	E_W(0x7007, 0x7007, 0, F_FLIPY), E_NOPR(0x7800, 0x7800, 0), E_END };
static const me_t map_guttangts3[] = {
	GX_DISC, E_ROM(0x0000, 0x3fff), E_NOPW(0x0000, 0x3fff, 0), E_RAM(0x4000, 0x47ff, 0, PO_RA),
	E_RAM(0x5000, 0x53ff, 0, PO_VRAM), E_RAM(0x5800, 0x58ff, 0, PO_OBJ), E_PR(0x6000, 0, P_IN0), E_PR(0x6800, 0, P_IN1),
	E_PR(0x7000, 0, P_IN2), E_W(0x7001, 0x7001, 0, F_IRQEN), E_W(0x7006, 0x7006, 0, F_FLIPX),
	E_W(0x7007, 0x7007, 0, F_FLIPY), E_NOPR(0x7800, 0x7800, 0), E_ROMR(0x8000, 0x87ff, 0x4000), E_END };
static const me_t map_kingball[] = {
	MC_MAP, E_W(0xb000, 0xb000, 0x07f8, F_KB_SOUND1), E_W(0xb001, 0xb001, 0x07f8, F_IRQEN),
	E_W(0xb002, 0xb002, 0x07f8, F_KB_SOUND2), E_W(0xb003, 0xb003, 0x07f8, F_KB_SPEECH), E_NOPW(0xb004, 0xb004, 0x07f8),
	E_END };
static const me_t map_spactrai[] = {
	E_ROM(0x0000, 0x01ff), E_RAM(0x0200, 0x05ff, 0, PO_RA), E_ROM(0x0600, 0x11ff), E_RAM(0x1200, 0x15ff, 0, PO_VRAM),
	E_ROM(0x1600, 0x4fff), E_W(0x6004, 0x6007, 0x07f8, F_LFO), E_W(0x6800, 0x6807, 0x07f8, F_SOUND),
	E_W(0x7800, 0x7800, 0x07ff, F_PITCH), E_W(0x7001, 0x7001, 0x07f8, F_IRQEN), E_W(0x7004, 0x7004, 0x07f8, F_STARS),
	E_W(0x7006, 0x7006, 0x07f8, F_FLIPX), E_W(0x7007, 0x7007, 0x07f8, F_FLIPY), E_PR(0x6000, 0, P_IN0),
	E_PR(0x6800, 0, P_IN1), E_PR(0x7000, 0, P_IN2), E_NOPR(0x7800, 0x7800, 0), E_RAM(0xfe00, 0xfeff, 0, PO_OBJ), E_END };
#define SFX_LATCHES \
	E_RAM(0x4000, 0x47ff, 0, PO_RA), E_RAM(0x4800, 0x4bff, 0x0400, PO_VRAM), E_RAM(0x5000, 0x50ff, 0x0700, PO_OBJ), \
	E_W(0x6800, 0x6800, 0x07f8, F_BG_R), E_W(0x6801, 0x6801, 0x07f8, F_IRQEN), E_W(0x6803, 0x6803, 0x07f8, F_BG_B), \
	E_W(0x6804, 0x6804, 0x07f8, F_STARS), E_W(0x6805, 0x6805, 0x07f8, F_BG_G), E_W(0x6806, 0x6806, 0x07f8, F_FLIPX), \
	E_W(0x6807, 0x6807, 0x07f8, F_FLIPY), E_RW(0x8000, 0xbfff, 0, F_THEEND_PPI)
static const me_t map_sfx[] = { E_ROM(0x0000, 0x3fff), SFX_LATCHES, E_ROM(0x7000, 0x7fff), E_ROM(0xc000, 0xefff), E_END };
static const me_t map_monsterz[] = {
	E_ROM(0x0000, 0x37ff), E_RAM(0x3800, 0x3fff, 0, PO_RB), SFX_LATCHES, E_NOPR(0x7000, 0x7000, 0), E_ROM(0xc000, 0xd7ff),
	E_R(0xd800, 0xdfff, 0, F_MONSTERZ_PROT), E_END };
// (the driver inits' changes to the maps)
static const me_t patch_mooncrgx[] = { E_W(0x6000, 0x6002, 0x07f8, F_GFXBANK), E_END };
static const me_t patch_checkman[] = { E_NOPW(0xb000, 0xb000, 0x07f8), E_W(0xb001, 0xb001, 0x07f8, F_IRQEN), E_END };
static const me_t patch_checkmaj[] = {
	E_W(0x7800, 0x7800, 0x07ff, F_CHECKMAN_CMD), E_R(0x3800, 0x3800, 0, F_CHECKMAJ_PROT), E_END };
static const me_t patch_dingo[] = {
	E_W(0x7800, 0x7800, 0x07ff, F_CHECKMAN_CMD), E_R(0x3000, 0x3000, 0, F_DINGO_3000), E_R(0x3035, 0x3035, 0, F_DINGO_3035),
	E_END };
static const me_t patch_dingoe[] = {
	E_NOPW(0xb000, 0xb000, 0x07f8), E_W(0xb001, 0xb001, 0x07f8, F_IRQEN), E_R(0x3001, 0x3001, 0, F_DINGOE_3001), E_END };
static const me_t patch_pacmanbl[] = { E_NOPW(0x6002, 0x6002, 0x07f8), E_END };

// ---- the machine configurations
enum { DISC_NONE, DISC_GAL, DISC_MCR, DISC_BONGO };
enum { SND_NONE, SND_KONAMI2, SND_KONAMI1, SND_AMIGO2, SND_EXPLORER, SND_TAKEOFF, SND_TURPINS, SND_BONGO, SND_ZIGZAG,
       SND_FANTASTC, SND_JUMPBUG, SND_OZON1, SND_CHECKMAN, SND_CHECKMAJ, SND_JUNGSUB, SND_KINGBALL, SND_CCLIMBER };
enum { IO_NONE, IO_BONGO, IO_OZON1, IO_JUNGSUB, IO_MSHUTTLE };
#define MF_IRQ0     0x0001   // (the vblank on INT, not NMI)
#define MF_SIDAM    0x0002   // (12 MHz: the CPU at 3 MHz, 59.19 Hz)
#define MF_2SPR     0x0004   // (two sprite generators)
#define MF_BUL_C0   0x0008   // (bullets at $c0)
#define MF_GFX2     0x0010   // (sprites from gfx2)
#define MF_KONAMI   0x0020   // (konami_base: the two PPIs)
#define MF_STARTMR  0x0040   // (the Scramble stars' blink timer)
#define MF_EAGLE    0x0080   // (RGB -> GBR)
#define MF_MOONWAR  0x0100
#define MF_PROT     0x0200   // (theend / scramble: the 6J protection on PPI1 port C)
#define MF_SCRAMMR  0x0400
#define MF_THEEND   0x0800   // (PPI0 port C: the coin counter)
#define MF_FROGTMR  0x1000   // (frogger_sound_timer_r on a two-AY board)
#define MF_TIMEFGTR 0x2000
#define MF_NAMENAYO 0x4000   // (64 colours, the sprites' from the second PROM)
#define MF_NOPPI1   0x8000   // (ozon1: one PPI)
#define MF_SFX      0x10000  // (the SF-X sample board: a third Z80, a PPI and an 8-bit DAC)
#define MF_MONSTERZ 0x20000

typedef struct { const char *name; const me_t *map; uint8_t disc, snd, io; uint32_t flags; } gx_machine;
static const gx_machine s_machines[] = {
	{ "galaxian",   map_galaxian,    DISC_GAL,   SND_NONE,     IO_NONE,     0 },
	{ "galartic",   map_galaxian,    DISC_GAL,   SND_NONE,     IO_NONE,     0 },
	{ "pisces",     map_pisces,      DISC_GAL,   SND_NONE,     IO_NONE,     0 },
	{ "ladybugg2",  map_pisces,      DISC_GAL,   SND_NONE,     IO_NONE,     0 },
	{ "victoryc",   map_victoryc,    DISC_GAL,   SND_NONE,     IO_NONE,     0 },
	{ "spactrai",   map_spactrai,    DISC_GAL,   SND_NONE,     IO_NONE,     0 },
	{ "frogg",      map_frogg,       DISC_GAL,   SND_NONE,     IO_NONE,     0 },
	{ "mandingarf", map_mandingarf,  DISC_GAL,   SND_NONE,     IO_NONE,     0 },
	{ "mandinka",   map_mandinka,    DISC_NONE,  SND_KONAMI2,  IO_NONE,     MF_KONAMI | MF_STARTMR },
	{ "pacmanbl",   map_galaxian,    DISC_GAL,   SND_NONE,     IO_NONE,     MF_GFX2 },
	{ "devilfshg",  map_galaxian,    DISC_GAL,   SND_NONE,     IO_NONE,     MF_GFX2 | MF_IRQ0 },
	{ "zigzag",     map_zigzag,      DISC_NONE,  SND_ZIGZAG,   IO_NONE,     MF_GFX2 | MF_2SPR },
	{ "gmgalax",    map_gmgalax,     DISC_GAL,   SND_NONE,     IO_NONE,     0 },
	{ "highroll",   map_galaxian,    DISC_GAL,   SND_NONE,     IO_NONE,     0 },
	{ "mooncrst",   map_mooncrst,    DISC_MCR,   SND_NONE,     IO_NONE,     0 },
	{ "eagle",      map_mooncrst,    DISC_MCR,   SND_NONE,     IO_NONE,     MF_EAGLE },
	{ "moonqsr",    map_mooncrst,    DISC_MCR,   SND_NONE,     IO_NONE,     0 },
	{ "thepitm",    map_thepitm,     DISC_MCR,   SND_NONE,     IO_NONE,     0 },
	{ "porter",     map_porter,      DISC_MCR,   SND_NONE,     IO_NONE,     0 },
	{ "skybase",    map_skybase,     DISC_MCR,   SND_NONE,     IO_NONE,     0 },
	{ "kong",       map_kong,        DISC_MCR,   SND_NONE,     IO_NONE,     0 },
	{ "scorpnmc",   map_scorpnmc,    DISC_MCR,   SND_NONE,     IO_NONE,     0 },
	{ "bongo",      map_bongo,       DISC_NONE,  SND_BONGO,    IO_BONGO,    0 },
	{ "bongog",     map_bongog,      DISC_BONGO, SND_NONE,     IO_NONE,     0 },
	{ "ckongg",     map_ckongg,      DISC_GAL,   SND_NONE,     IO_NONE,     0 },
	{ "bigkonggx",  map_bigkonggx,   DISC_GAL,   SND_NONE,     IO_NONE,     0 },
	{ "ckongmc",    map_ckongmc,     DISC_MCR,   SND_NONE,     IO_NONE,     0 },
	{ "bagmanmc",   map_bagmanmc,    DISC_MCR,   SND_NONE,     IO_NONE,     MF_IRQ0 },
	{ "fantastc",   map_fantastc,    DISC_NONE,  SND_FANTASTC, IO_NONE,     MF_2SPR | MF_BUL_C0 },
	{ "timefgtr",   map_fantastc,    DISC_NONE,  SND_FANTASTC, IO_NONE,     MF_2SPR | MF_BUL_C0 | MF_TIMEFGTR },
	{ "guttangt",   map_guttangt,    DISC_GAL,   SND_NONE,     IO_NONE,     0 },
	{ "guttangts3", map_guttangts3,  DISC_GAL,   SND_NONE,     IO_NONE,     0 },
	{ "jumpbugbrf", map_jumpbugbrf,  DISC_NONE,  SND_NONE,     IO_NONE,     0 },
	{ "jumpbug",    map_jumpbug,     DISC_NONE,  SND_JUMPBUG,  IO_NONE,     0 },
	{ "checkman",   map_mooncrst,    DISC_MCR,   SND_CHECKMAN, IO_NONE,     0 },
	{ "checkmaj",   map_galaxian_base, DISC_NONE, SND_CHECKMAJ, IO_NONE,    0 },
	{ "mshuttle",   map_mshuttle,    DISC_NONE,  SND_CCLIMBER, IO_MSHUTTLE, MF_IRQ0 },
	{ "kingball",   map_kingball,    DISC_MCR,   SND_KINGBALL, IO_NONE,     0 },
	{ "frogger",    map_frogger,     DISC_NONE,  SND_KONAMI1,  IO_NONE,     MF_KONAMI },
	{ "froggermc",  map_froggermc,   DISC_NONE,  SND_KONAMI1,  IO_NONE,     0 },
	{ "froggers",   map_theend,      DISC_NONE,  SND_KONAMI1,  IO_NONE,     MF_KONAMI },
	{ "froggervd",  map_froggervd,   DISC_NONE,  SND_KONAMI1,  IO_NONE,     MF_KONAMI },
	{ "frogf",      map_frogf,       DISC_NONE,  SND_KONAMI1,  IO_NONE,     MF_KONAMI },
	{ "turtles",    map_turtles,     DISC_NONE,  SND_KONAMI2,  IO_NONE,     MF_KONAMI },
	{ "turpinnv",   map_turpinnv,    DISC_NONE,  SND_KONAMI2,  IO_NONE,     MF_KONAMI },
	{ "theend",     map_theend,      DISC_NONE,  SND_KONAMI2,  IO_NONE,     MF_KONAMI | MF_THEEND | MF_PROT },
	{ "ozon1",      map_ozon1,       DISC_NONE,  SND_OZON1,    IO_OZON1,    MF_KONAMI | MF_NOPPI1 },
	{ "namenayo",   map_namenayo,    DISC_NONE,  SND_KONAMI2,  IO_NONE,     MF_KONAMI | MF_NAMENAYO },
	{ "scramble",   map_theend,      DISC_NONE,  SND_KONAMI2,  IO_NONE,     MF_KONAMI | MF_STARTMR | MF_PROT },
	{ "scrammr",    map_theend,      DISC_NONE,  SND_KONAMI2,  IO_NONE,     MF_KONAMI | MF_STARTMR | MF_PROT | MF_SCRAMMR },
	{ "jungsub",    map_jungsub,     DISC_NONE,  SND_JUNGSUB,  IO_JUNGSUB,  0 },
	{ "explorer",   map_explorer,    DISC_NONE,  SND_EXPLORER, IO_NONE,     MF_SIDAM },
	{ "takeoff",    map_explorer,    DISC_NONE,  SND_TAKEOFF,  IO_NONE,     MF_SIDAM },
	{ "astroamb",   map_astroamb,    DISC_GAL,   SND_NONE,     IO_NONE,     0 },
	{ "amigo2",     map_amigo2,      DISC_NONE,  SND_AMIGO2,   IO_NONE,     MF_SIDAM },
	{ "ckongs",     map_ckongs,      DISC_NONE,  SND_KONAMI2,  IO_NONE,     MF_KONAMI | MF_STARTMR },
	{ "scobra",     map_scobra,      DISC_NONE,  SND_KONAMI2,  IO_NONE,     MF_KONAMI | MF_STARTMR },
	{ "anteatergg", map_anteatergg,  DISC_GAL,   SND_NONE,     IO_NONE,     0 },
	{ "mimonkey",   map_mimonkey,    DISC_NONE,  SND_KONAMI2,  IO_NONE,     MF_KONAMI | MF_STARTMR },
	{ "mimonscr",   map_mimonscr,    DISC_NONE,  SND_KONAMI2,  IO_NONE,     MF_KONAMI | MF_STARTMR },
	{ "quaak",      map_scobra,      DISC_NONE,  SND_KONAMI2,  IO_NONE,     MF_KONAMI | MF_FROGTMR },
	{ "froggeram",  map_froggeram,    DISC_NONE,  SND_KONAMI2,  IO_NONE,     MF_KONAMI | MF_FROGTMR },
	{ "turpins",    map_turpins,     DISC_NONE,  SND_TURPINS,  IO_NONE,     MF_KONAMI | MF_STARTMR },
	{ "anteater",   map_scobra,      DISC_NONE,  SND_KONAMI2,  IO_NONE,     MF_KONAMI | MF_STARTMR },
	{ "anteateruk", map_anteateruk,  DISC_NONE,  SND_KONAMI2,  IO_NONE,     MF_KONAMI | MF_STARTMR },
	{ "anteaterg",  map_anteaterg,   DISC_NONE,  SND_KONAMI2,  IO_NONE,     MF_KONAMI | MF_STARTMR },
	{ "moonwar",    map_scobra,      DISC_NONE,  SND_KONAMI2,  IO_NONE,     MF_KONAMI | MF_STARTMR | MF_MOONWAR },
	{ "fourplay",   map_fourplay,    DISC_GAL,   SND_NONE,     IO_NONE,     0 },
	{ "videight",   map_videight,    DISC_GAL,   SND_NONE,     IO_NONE,     0 },
	{ "sfx",        map_sfx,         DISC_NONE,  SND_KONAMI2,  IO_NONE,     MF_KONAMI | MF_STARTMR | MF_SFX },
	{ "monsterz",   map_monsterz,    DISC_NONE,  SND_KONAMI2,  IO_NONE,     MF_KONAMI | MF_STARTMR | MF_SFX | MF_MONSTERZ },
};

// ---- the driver inits (video extensions, decryptions, map changes)
enum { BUL_NONE, BUL_GAL, BUL_MSH, BUL_SCR, BUL_THEEND, BUL_SFX };
enum { BG_NULL, BG_GAL, BG_SCR, BG_ANTEATER, BG_JUMPBUG, BG_TURTLES, BG_FROGGER, BG_NAMENAYO, BG_SFX };
enum { TX_NONE, TX_UPPER, TX_FROGGER, TX_BAGMANMC, TX_PISCES, TX_MIMONKEY, TX_BATMAN2, TX_MOONCRST, TX_MOONQSR,
       TX_MSHUTTLE, TX_JUMPBUG, TX_NAMENAYO, TX_VIDEIGHT };
enum { SX_NONE, SX_UPPER, SX_FROGGER, SX_BAGMANMC, SX_PISCES, SX_MIMONKEY, SX_MOONCRST, SX_MOONQSR, SX_MSHUTTLE,
       SX_CALIPSO, SX_JUMPBUG, SX_NAMENAYO, SX_GUTTANGT, SX_VIDEIGHT };
enum { DEC_NONE, DEC_HIGHROLL, DEC_WAROFBUGG, DEC_LADYBUGG2, DEC_VICTORYC, DEC_BIGKONGGX, DEC_CKONGGX, DEC_GUTTANGTS3,
       DEC_MOONCRST, DEC_MOONQSR, DEC_DEVILFSHG, DEC_JUMPBUGBC, DEC_CHECKMAN, DEC_DINGOE, DEC_MSHUTTLE, DEC_MSHUTTLJ,
       DEC_FANTASTC, DEC_SCOBRAE, DEC_LOSTTOMB, DEC_FROGGER, DEC_FROGGERMC, DEC_FROGGERS, DEC_ANTEATER, DEC_SUPERBON,
       DEC_CRAZYM, DEC_JUNGSUB, DEC_MIMONKEY };
enum { CU_NONE, CU_AZURIAN, CU_KINGBALL, CU_THEEND, CU_CKONGG, CU_CKONGMC, CU_CKONGS, CU_MOONWAR, CU_GMGALAX };
#define IF_FROGADJ 1
#define IF_SFXADJ 2   // (SF-X: the tilemap in columns with a scroll a row, the sprites' line match the other way)
typedef struct { const char *name; uint8_t bul, bg, tx, sx, dec, flags; const me_t *patch; } gx_initcfg;
static const gx_initcfg s_inits[] = {
	{ "galaxian",   BUL_GAL,    BG_GAL,      TX_NONE,     SX_NONE,     DEC_NONE,       0, NULL },
	{ "nolock",     BUL_GAL,    BG_GAL,      TX_NONE,     SX_NONE,     DEC_NONE,       0, NULL },
	{ "azurian",    BUL_SCR,    BG_GAL,      TX_NONE,     SX_NONE,     DEC_NONE,       0, NULL },
	{ "warofbugg",  BUL_GAL,    BG_GAL,      TX_NONE,     SX_NONE,     DEC_WAROFBUGG,  0, NULL },
	{ "crazym",     BUL_GAL,    BG_GAL,      TX_NONE,     SX_UPPER,    DEC_CRAZYM,     0, NULL },
	{ "gmgalax",    BUL_GAL,    BG_GAL,      TX_BAGMANMC, SX_BAGMANMC, DEC_NONE,       0, NULL },
	{ "pisces",     BUL_GAL,    BG_GAL,      TX_PISCES,   SX_PISCES,   DEC_NONE,       0, NULL },
	{ "batman2",    BUL_GAL,    BG_GAL,      TX_BATMAN2,  SX_UPPER,    DEC_NONE,       0, NULL },
	{ "ladybugg2",  BUL_GAL,    BG_GAL,      TX_BATMAN2,  SX_UPPER,    DEC_LADYBUGG2,  0, NULL },
	{ "frogg",      BUL_GAL,    BG_FROGGER,  TX_FROGGER,  SX_FROGGER,  DEC_NONE,       0, NULL },
	{ "victoryc",   BUL_GAL,    BG_GAL,      TX_NONE,     SX_NONE,     DEC_VICTORYC,   0, NULL },
	{ "bigkonggx",  BUL_SCR,    BG_SCR,      TX_NONE,     SX_MSHUTTLE, DEC_BIGKONGGX,  0, NULL },
	{ "ckonggx",    BUL_SCR,    BG_SCR,      TX_NONE,     SX_MSHUTTLE, DEC_CKONGGX,    0, NULL },
	{ "fourplay",   BUL_GAL,    BG_GAL,      TX_PISCES,   SX_PISCES,   DEC_NONE,       0, NULL },
	{ "videight",   BUL_GAL,    BG_GAL,      TX_VIDEIGHT, SX_VIDEIGHT, DEC_NONE,       0, NULL },
	{ "guttangt",   BUL_GAL,    BG_GAL,      TX_NONE,     SX_GUTTANGT, DEC_NONE,       0, NULL },
	{ "guttangts3", BUL_GAL,    BG_GAL,      TX_NONE,     SX_GUTTANGT, DEC_GUTTANGTS3, 0, NULL },
	{ "mooncrst",   BUL_GAL,    BG_GAL,      TX_MOONCRST, SX_MOONCRST, DEC_MOONCRST,   0, NULL },
	{ "mooncrsu",   BUL_GAL,    BG_GAL,      TX_MOONCRST, SX_MOONCRST, DEC_NONE,       0, NULL },
	{ "mooncrgx",   BUL_GAL,    BG_GAL,      TX_MOONCRST, SX_MOONCRST, DEC_NONE,       0, patch_mooncrgx },
	{ "moonqsr",    BUL_GAL,    BG_GAL,      TX_MOONQSR,  SX_MOONQSR,  DEC_MOONQSR,    0, NULL },
	{ "pacmanbl",   BUL_GAL,    BG_GAL,      TX_NONE,     SX_NONE,     DEC_NONE,       0, patch_pacmanbl },
	{ "ghostmun",   BUL_GAL,    BG_GAL,      TX_NONE,     SX_NONE,     DEC_NONE,       0, patch_pacmanbl },
	{ "devilfshg",  BUL_GAL,    BG_GAL,      TX_NONE,     SX_NONE,     DEC_DEVILFSHG,  0, NULL },
	{ "bagmanmc",   BUL_GAL,    BG_GAL,      TX_BAGMANMC, SX_BAGMANMC, DEC_NONE,       0, NULL },
	{ "zigzag",     BUL_NONE,   BG_GAL,      TX_NONE,     SX_NONE,     DEC_NONE,       0, NULL },
	{ "jumpbug",    BUL_SCR,    BG_JUMPBUG,  TX_JUMPBUG,  SX_JUMPBUG,  DEC_NONE,       0, NULL },
	{ "jumpbugbc",  BUL_SCR,    BG_JUMPBUG,  TX_JUMPBUG,  SX_JUMPBUG,  DEC_JUMPBUGBC,  0, NULL },
	{ "checkman",   BUL_GAL,    BG_GAL,      TX_MOONCRST, SX_MOONCRST, DEC_CHECKMAN,   0, patch_checkman },
	{ "checkmaj",   BUL_GAL,    BG_GAL,      TX_NONE,     SX_NONE,     DEC_NONE,       0, patch_checkmaj },
	{ "dingo",      BUL_GAL,    BG_GAL,      TX_NONE,     SX_NONE,     DEC_NONE,       0, patch_dingo },
	{ "dingoe",     BUL_GAL,    BG_GAL,      TX_MOONCRST, SX_MOONCRST, DEC_DINGOE,     0, patch_dingoe },
	{ "kong",       BUL_GAL,    BG_GAL,      TX_NONE,     SX_UPPER,    DEC_NONE,       0, NULL },
	{ "mshuttle",   BUL_MSH,    BG_GAL,      TX_MSHUTTLE, SX_MSHUTTLE, DEC_MSHUTTLE,   0, NULL },
	{ "mshuttlj",   BUL_MSH,    BG_GAL,      TX_MSHUTTLE, SX_MSHUTTLE, DEC_MSHUTTLJ,   0, NULL },
	{ "fantastc",   BUL_GAL,    BG_GAL,      TX_NONE,     SX_UPPER,    DEC_FANTASTC,   0, NULL },
	{ "timefgtr",   BUL_GAL,    BG_GAL,      TX_NONE,     SX_UPPER,    DEC_NONE,       0, NULL },
	{ "theend",     BUL_THEEND, BG_GAL,      TX_NONE,     SX_NONE,     DEC_NONE,       0, NULL },
	{ "scramble",   BUL_SCR,    BG_SCR,      TX_NONE,     SX_NONE,     DEC_NONE,       0, NULL },
	{ "mandingaeg", BUL_SCR,    BG_SCR,      TX_NONE,     SX_NONE,     DEC_NONE,       0, NULL },
	{ "mandinga",   BUL_SCR,    BG_SCR,      TX_NONE,     SX_NONE,     DEC_NONE,       0, NULL },
	{ "atlantis",   BUL_SCR,    BG_SCR,      TX_NONE,     SX_NONE,     DEC_NONE,       0, NULL },
	{ "explorer",   BUL_SCR,    BG_NULL,     TX_NONE,     SX_NONE,     DEC_NONE,       0, NULL },
	{ "scobra",     BUL_SCR,    BG_SCR,      TX_NONE,     SX_NONE,     DEC_NONE,       0, NULL },
	{ "scobrae",    BUL_SCR,    BG_SCR,      TX_NONE,     SX_NONE,     DEC_SCOBRAE,    0, NULL },
	{ "losttomb",   BUL_SCR,    BG_SCR,      TX_NONE,     SX_NONE,     DEC_LOSTTOMB,   0, NULL },
	{ "frogger",    BUL_GAL,    BG_FROGGER,  TX_FROGGER,  SX_FROGGER,  DEC_FROGGER,    IF_FROGADJ, NULL },
	{ "quaak",      BUL_GAL,    BG_FROGGER,  TX_FROGGER,  SX_FROGGER,  DEC_NONE,       0, NULL },
	{ "froggermc",  BUL_GAL,    BG_FROGGER,  TX_FROGGER,  SX_FROGGER,  DEC_FROGGERMC,  0, NULL },
	{ "froggers",   BUL_GAL,    BG_FROGGER,  TX_FROGGER,  SX_FROGGER,  DEC_FROGGERS,   0, NULL },
	{ "froggrs",    BUL_GAL,    BG_FROGGER,  TX_FROGGER,  SX_FROGGER,  DEC_FROGGER,    0, NULL },
	{ "turtles",    BUL_GAL,    BG_TURTLES,  TX_NONE,     SX_NONE,     DEC_NONE,       0, NULL },
	{ "ckongs",     BUL_SCR,    BG_SCR,      TX_NONE,     SX_MSHUTTLE, DEC_NONE,       0, NULL },
	{ "anteater",   BUL_SCR,    BG_ANTEATER, TX_NONE,     SX_NONE,     DEC_ANTEATER,   0, NULL },
	{ "anteateruk", BUL_SCR,    BG_ANTEATER, TX_NONE,     SX_NONE,     DEC_NONE,       0, NULL },
	{ "superbon",   BUL_SCR,    BG_SCR,      TX_NONE,     SX_NONE,     DEC_SUPERBON,   0, NULL },
	{ "calipso",    BUL_SCR,    BG_SCR,      TX_NONE,     SX_CALIPSO,  DEC_NONE,       0, NULL },
	{ "jungsub",    BUL_SCR,    BG_SCR,      TX_NONE,     SX_NONE,     DEC_JUNGSUB,    0, NULL },
	{ "mimonkeyb",  BUL_SCR,    BG_SCR,      TX_MIMONKEY, SX_MIMONKEY, DEC_NONE,       0, NULL },
	{ "mimonkey",   BUL_SCR,    BG_SCR,      TX_MIMONKEY, SX_MIMONKEY, DEC_MIMONKEY,   0, NULL },
	{ "namenayo",   BUL_SCR,    BG_NAMENAYO, TX_NAMENAYO, SX_NAMENAYO, DEC_NONE,       0, NULL },
	{ "highroll",   BUL_GAL,    BG_GAL,      TX_NONE,     SX_NONE,     DEC_HIGHROLL,   0, NULL },
	{ "sfx",        BUL_SFX,    BG_SFX,      TX_UPPER,    SX_NONE,     DEC_NONE,       IF_SFXADJ, NULL },
	{ "monsterz",   BUL_SFX,    BG_GAL,      TX_UPPER,    SX_NONE,     DEC_NONE,       IF_SFXADJ, NULL },
};

// ---- the set's configuration
static const gx_machine *s_m;
static const gx_initcfg *s_i;
static const me_t *s_ents[2];          // (the machine's map and the init's changes)
static me_t s_map[160];
static int s_nmap;
static int s_custom;
static int s_port[P_COUNT];
static uint8_t *s_rom, *s_srom, *s_opcodes, *s_samples;
static uint32_t s_rom_size, s_srom_size, s_proms_size;
static uint8_t *s_proms;
static uint8_t *s_chr, *s_spr;         // (decoded, a byte a pixel)
static uint32_t s_nchr, s_nspr;
static uint32_t s_pal[NPENS];
static int s_snd_clock;
static uint32_t s_frame_mhz, s_rate, s_cpu_clock;
static uint8_t s_unmap;                // (the maps' unmap value: 0xff for the maps with unmap_value_high)
static const uint8_t *s_bank_base[2];
static uint32_t s_bank_stride[2];

static z80 s_main, s_snd, s_smp;   // (s_smp: the SF-X sample board's Z80)
static const uint8_t *s_rmap[256];
static uint8_t *s_wmap[256];
// a handled page (s_rmap / s_wmap NULL): its entry (s_rpe / s_wpe >= 0, -1 unmapped) or, -2, an entry a byte
static int16_t s_rpe[256], s_wpe[256];
static uint8_t *s_rent[256], *s_went[256];
#define ENT_PAGES 48
static uint8_t s_ent_mem[ENT_PAGES][256];
static int s_ent_used;
static const uint8_t *s_zmap[256], *s_smap[256];
static uint8_t *s_arom;           // (the sample board's ROM: audio2)
static uint32_t s_arom_size;
static i8255_t s_ppi[3];
static ay8910_t s_ay[2];
static galsnd_t s_galsnd;

// ---- the board's state (saved)
static struct {
	uint8_t pool[POOL_SIZE];
	uint8_t zram[0x400], zram2[0x400];
	uint8_t soundlatch2, sample_ctrl, mz_portb, mz_portc, dac2;
	int32_t smp_int;
	uint32_t mz_shift, mz_shift2;
	uint32_t stgt2;
	uint64_t smp_frac;
	uint8_t irq_enabled, nmi_line;
	int32_t int_line;
	uint8_t flipx, flipy, stars_enabled, stars_blink, bg_en, bg_r, bg_g, bg_b;
	uint8_t gfxbank[5];
	uint8_t soundlatch, snd_ctrl, snd_mute, snd_nmi_line, latch_pending;
	int32_t snd_int;
	uint8_t prot_result;
	uint16_t prot_state;
	uint8_t bank[2];
	uint8_t kb_speech, kb_sound;
	uint8_t msh_cs, zz_latch, gm_sel;
	uint8_t cc_rate, cc_trig, cc_vol, cc_start, cc_loop;
	uint32_t cc_addr, cc_cnt;   // (the sample player's address and its clock count to the next tick)
	uint8_t cc_dac;
	uint8_t dac;
	uint8_t mw_sel, mw_dir[2], mw_cnt[2], mw_last[2];
	uint32_t star_origin, star_origin_frame, frame;
	uint32_t blink_acc;
	uint32_t tgt, stgt;
	uint64_t cyc_frac, snd_frac, ay_frac[2];
	int32_t dc[2];              // (the DC blocker of the AYs and DACs)
	uint8_t dc_init;
	filter_rc_t flt[2][3];
	int32_t lfsr;
} b;

// ---- the drawing's copy
typedef struct {
	uint8_t vram[0x400], obj[0x400], ext[0x20];
	uint8_t flipx, flipy, stars_enabled, stars_blink, bg_en, bg_r, bg_g, bg_b;
	uint8_t gfxbank[5];
	uint32_t star_origin;
} snap_t;
static snap_t *s_snap;
static uint8_t *s_tiles;              // (the tilemap's pixels, 256 x 256: colour << 2 | pen, 0 = transparent)
static uint32_t *s_tile_key;
static uint32_t *s_stars;             // (the LFSR positions with a star: index << 8 | colour)
static int s_nstars;

enum { GX_PA_MAIN = RBX_PA_DRV, GX_PA_SOUND_CPU, GX_PA_SOUND, GX_PA_VBLANK, GX_PA_END };
static const char *const s_pa_names[] = { "gx main Z80", "gx sound Z80", "gx sound chips", "gx vblank copy" };

static inline uint8_t bitswap8(uint8_t v, int b7, int b6, int b5, int b4, int b3, int b2, int b1, int b0)
{
	return (uint8_t)(((v >> b7) & 1) << 7 | ((v >> b6) & 1) << 6 | ((v >> b5) & 1) << 5 | ((v >> b4) & 1) << 4 |
	                 ((v >> b3) & 1) << 3 | ((v >> b2) & 1) << 2 | ((v >> b1) & 1) << 1 | ((v >> b0) & 1));
}
#define BIT(v, n) (((v) >> (n)) & 1)

static inline uint8_t raw_port(int p) { return (uint8_t)rbx_port_read(s_port[p]); }

// ---- the input ports with their custom bits (PORT_CUSTOM_MEMBER / PORT_READ_LINE_MEMBER)
static uint8_t moonwar_dial(void)
{
	const int s = b.mw_sel;
	const uint8_t dial = raw_port(s ? P_DIAL2 : P_DIAL1);
	const int8_t d = (int8_t)(dial - b.mw_last[s]);
	if (d < 0)
		b.mw_dir[s] = 0x00;
	else if (d != 0)
		b.mw_dir[s] = 0x10;
	b.mw_cnt[s] = (uint8_t)((b.mw_cnt[s] + (d < 0 ? -d : d)) & 0x0f);
	b.mw_last[s] = dial;
	return (uint8_t)(b.mw_cnt[s] | b.mw_dir[s]);
}

static uint8_t port_in(int p)
{
	uint8_t v = raw_port(p);
	switch (s_custom) {
	case CU_AZURIAN:
		if (p == P_IN1) v = (uint8_t)((v & ~0x40) | (BIT(raw_port(P_FAKE), 0) << 6));
		else if (p == P_IN2) v = (uint8_t)((v & ~0x04) | (BIT(raw_port(P_FAKE), 1) << 2));
		break;
	case CU_KINGBALL:
		if (p == P_IN0) v = (uint8_t)((v & ~0x40) | (BIT(raw_port(P_FAKE), b.kb_speech & 7) << 6));
		else if (p == P_IN1) {
			b.lfsr = b.lfsr * 1103515245 + 12345;
			v = (uint8_t)((v & ~0x20) | (((uint32_t)b.lfsr >> 16) & 1) << 5);
		}
		break;
	case CU_THEEND:
		if (p == P_IN2) {
			const uint8_t bit = (b.prot_result >> 7) & 1;
			v = (uint8_t)((v & ~0xa0) | bit << 5 | bit << 7);
		}
		break;
	case CU_CKONGG:
		if (p == P_IN1) v = (uint8_t)((v & ~0x40) | (raw_port(P_COINAGE) & 0x40));
		else if (p == P_IN2) v = (uint8_t)((v & ~0x0c) | (raw_port(P_COINAGE) & 0x0c));
		break;
	case CU_CKONGMC:
		if (p == P_IN1) v = (uint8_t)((v & ~0xc0) | (raw_port(P_COINAGE) & 0xc0));
		else if (p == P_IN2) v = (uint8_t)((v & ~0x01) | (raw_port(P_COINAGE) & 0x01));
		break;
	case CU_CKONGS: {
		const uint8_t c = raw_port(P_COINAGE);
		if (p == P_IN1) v = (uint8_t)((v & ~0x03) | ((c & 1) ? 1 : 0) | ((c & 2) ? 2 : 0));
		else if (p == P_IN2) v = (uint8_t)((v & ~0x08) | ((c & 4) ? 8 : 0));
		break;
	}
	case CU_MOONWAR:
		if (p == P_IN0) v = (uint8_t)((v & ~0x1f) | (moonwar_dial() & 0x1f));
		break;
	case CU_GMGALAX:
		if (p <= P_IN2) v = raw_port((b.gm_sel ? P_GLIN0 : P_GMIN0) + p);
		break;
	default:
		break;
	}
	return v;
}

// ---- the PPIs (konami_base)
static uint8_t ppi_in_a(void *u) { return (intptr_t)u == 0 ? port_in(P_IN0) : (intptr_t)u == 2 ? b.soundlatch2 : 0xff; }
static uint8_t ppi_in_b(void *u) { return (intptr_t)u == 0 ? port_in(P_IN1) : 0xff; }
static uint8_t ppi_in_c(void *u)
{
	if ((intptr_t)u == 2)
		return 0xff;
	if ((intptr_t)u == 0)
		return port_in(P_IN2);
	if (s_m->flags & MF_SCRAMMR) {   // scrammr_protection_r (MAME's hack on the PC)
		switch (s_main.pc) {
		case 0x12e: case 0x1ca: case 0xc3c: return 0x90;
		case 0xbf1: return 0xa0;
		default: return 0x00;
		}
	}
	if (s_m->flags & MF_PROT)
		return b.prot_result;
	return port_in(P_IN3);
}
static void konami_sound_control_w(uint8_t data);
static void ppi_out_a(void *u, uint8_t v)
{
	if ((intptr_t)u == 1)
		b.soundlatch = v;
}
static void ppi_out_b(void *u, uint8_t v)
{
	if ((intptr_t)u == 1)
		konami_sound_control_w(v);
}
static void ppi_out_c(void *u, uint8_t v)
{
	if ((intptr_t)u == 2) {
		if ((s_m->flags & MF_MONSTERZ) && (b.mz_portc & 1) && !(v & 1))   // (the sound CPU's IRQ0, HOLD)
			b.snd_int = 1;
		b.mz_portc = v;
		return;
	}
	if ((intptr_t)u == 0) {
		if (s_m->flags & MF_MOONWAR)
			b.mw_sel = (uint8_t)BIT(~v, 4);
		return;
	}
	if (s_m->flags & MF_PROT) {   // theend_protection_w
		b.prot_state = (uint16_t)((b.prot_state << 4) | (v & 0x0f));
		const int num1 = (b.prot_state >> 8) & 0x0f, num2 = (b.prot_state >> 4) & 0x0f, op = b.prot_state & 0x0f;
		switch (op) {
		case 0x6: b.prot_result ^= 0x80; break;
		case 0x9: b.prot_result = (uint8_t)((num1 + 1 < 0xf ? num1 + 1 : 0xf) << 4); break;
		case 0xb: b.prot_result = (uint8_t)((num2 - num1 > 0 ? num2 - num1 : 0) << 4); break;
		case 0xa: b.prot_result = 0x00; break;
		case 0xf: b.prot_result = (uint8_t)((num1 - num2 > 0 ? num1 - num2 : 0) << 4); break;
		}
	}
}

// ---- the latches and the video controls
static void irq_enable_w(uint8_t v)
{
	b.irq_enabled = v & 1;
	if (!b.irq_enabled) {   // (CLEAR held low: the interrupt line cleared)
		b.nmi_line = 0;
		b.int_line = 0;
	}
}

static int cur_line(void);
static void stars_update_origin(void);

static void stars_enable_w(uint8_t v)
{
	if (!b.stars_enabled && (v & 1)) {   // (the shift registers' CLR released: the origin of this frame)
		b.star_origin = (uint32_t)(STAR_P - cur_line() * 512);
		b.star_origin_frame = b.frame;
	}
	b.stars_enabled = v & 1;
}

static void flip_x_w(uint8_t v)
{
	if (b.flipx != (v & 1)) {
		stars_update_origin();
		b.flipx = v & 1;
	}
}

static void konami_sound_control_w(uint8_t data)
{
	const uint8_t old = b.snd_ctrl;
	b.snd_ctrl = data;
	if ((old & 0x08) && !(data & 0x08))   // (the inverse of bit 3 clocks the INT flip-flop: HOLD_LINE)
		b.snd_int = 1;
	b.snd_mute = (data >> 4) & 1;         // (bit 4: the sound disable)
}

static void konami_filter_w(uint32_t offset)
{
	// AV0-AV5: the second AY (3C), AV6-AV11: the first (3D); two bits a channel: 0.22 uF, 0.047 uF
	for (int w = 0; w < 2; w++)
		for (int ch = 0; ch < 3; ch++) {
			const int d = (int)(offset >> (2 * ch + 6 * (1 - w))) & 3;
			const double c = ((d & 1) ? 0.22e-6 : 0) + ((d & 2) ? 0.047e-6 : 0);
			filter_rc_lowpass_3r(&b.flt[w][ch], 1000, 5100, 0, c, (double)s_snd_clock / 8.0);
		}
}

static void set_banks(void);

static void gfxbank_w(int offset, uint8_t v) { b.gfxbank[offset] = v; }

static uint8_t konami_sound_timer(void)
{
	// the LS393 / LS93 / LS90 chain clocked at the sound clock x 8 (16*16*2*8*5*2 = 40960 clocks)
	uint32_t cycles = (uint32_t)(s_snd.cyc % 5120u) * 8u;
	uint8_t hibit = 0;
	if (cycles >= 16 * 16 * 2 * 8 * 5) {
		hibit = 1;
		cycles -= 16 * 16 * 2 * 8 * 5;
	}
	return (uint8_t)(hibit << 7 | BIT(cycles, 14) << 6 | BIT(cycles, 13) << 5 | BIT(cycles, 11) << 4 | 0x0e);
}
static uint8_t frogger_sound_timer(void) { return bitswap8(konami_sound_timer(), 7, 6, 3, 4, 5, 2, 1, 0); }

// ---- the handlers
static uint8_t ppi_r(int n, int o) { return i8255_read(&s_ppi[n], o & 3); }
static void ppi_w(int n, int o, uint8_t v) { i8255_write(&s_ppi[n], o & 3, v); }

static uint8_t fn_read(uint32_t fn, uint32_t o)
{
	uint8_t r = 0xff;
	switch (fn) {
	case F_PPI0: return ppi_r(0, (int)o);
	case F_PPI1: return ppi_r(1, (int)o);
	case F_THEEND_PPI:
		if (o & 0x0100) r &= ppi_r(0, (int)o);
		if (o & 0x0200) r &= ppi_r(1, (int)o);
		return r;
	case F_FROGGER_PPI:
		if (o & 0x1000) r &= ppi_r(1, (int)(o >> 1));
		if (o & 0x2000) r &= ppi_r(0, (int)(o >> 1));
		return r;
	case F_FROGF_PPI:
		if (o & 0x1000) r &= ppi_r(0, (int)(o >> 3));
		if (o & 0x2000) r &= ppi_r(1, (int)(o >> 3));
		return r;
	case F_TURTLES_PPI0: return ppi_r(0, (int)(o >> 4));
	case F_TURTLES_PPI1: return ppi_r(1, (int)(o >> 4));
	case F_FROGGERAM_PPI:
		if (o & 0x0100) r &= ppi_r(0, (int)o);
		if (o & 0x0200) r &= ppi_r(1, (int)o);
		return bitswap8(r, 0, 1, 2, 3, 4, 5, 6, 7);
	case F_AY0_DATA: return ay8910_data_r(&s_ay[0]);
	case F_AY1_DATA: return ay8910_data_r(&s_ay[1]);
	case F_JUMPBUG_PROT:
		switch (o) {
		case 0x0114: return 0x4f;
		case 0x0118: return 0xd3;
		case 0x0214: return 0xcf;
		case 0x0235: return 0x02;
		default: return 0xff;
		}
	case F_CHECKMAJ_PROT:
		switch (s_main.pc) {
		case 0x0f15: return 0xf5;
		case 0x0f8f: case 0x10b3: return 0x7c;
		case 0x10e0: return 0x00;
		case 0x10f1: case 0x1402: return 0xaa;
		default: return 0;
		}
	case F_DINGO_3000: return 0xaa;
	case F_DINGO_3035: return 0x8c;
	case F_DINGOE_3001: return 0xaa;
	case F_MONSTERZ_PROT: {   // (bitswap<8>(shift ^ 0x40050, 7, 18, 0, 1, 4, 6, 5, 3))
		const uint32_t x = b.mz_shift ^ 0x40050;
		return (uint8_t)(BIT(x, 7) << 7 | BIT(x, 18) << 6 | BIT(x, 0) << 5 | BIT(x, 1) << 4 | BIT(x, 4) << 3 |
		                 BIT(x, 6) << 2 | BIT(x, 5) << 1 | BIT(x, 3));
	}
	default: return s_unmap;
	}
}

static void checkman_command(uint8_t v)
{
	b.soundlatch = v;
	z80_gen_nmi(&s_snd);   // (pulse_input_line NMI)
}

static void fn_write(uint32_t fn, uint32_t o, uint8_t v)
{
	switch (fn) {
	case F_PPI0: ppi_w(0, (int)o, v); break;
	case F_PPI1: ppi_w(1, (int)o, v); break;
	case F_THEEND_PPI:
		if (o & 0x0100) ppi_w(0, (int)o, v);
		if (o & 0x0200) ppi_w(1, (int)o, v);
		break;
	case F_FROGGER_PPI:
		if (o & 0x1000) ppi_w(1, (int)(o >> 1), v);
		if (o & 0x2000) ppi_w(0, (int)(o >> 1), v);
		break;
	case F_FROGF_PPI:
		if (o & 0x1000) ppi_w(0, (int)(o >> 3), v);
		if (o & 0x2000) ppi_w(1, (int)(o >> 3), v);
		break;
	case F_TURTLES_PPI0: ppi_w(0, (int)(o >> 4), v); break;
	case F_TURTLES_PPI1: ppi_w(1, (int)(o >> 4), v); break;
	case F_FROGGERAM_PPI:
		v = bitswap8(v, 0, 1, 2, 3, 4, 5, 6, 7);
		if (o & 0x0100) ppi_w(0, (int)o, v);
		if (o & 0x0200) ppi_w(1, (int)o, v);
		break;
	case F_AY0_ADDR: ay8910_address_w(&s_ay[0], v); break;
	case F_AY0_DATA: ay8910_data_w(&s_ay[0], v); break;
	case F_AY1_ADDR: ay8910_address_w(&s_ay[1], v); break;
	case F_AY1_DATA: ay8910_data_w(&s_ay[1], v); break;
	case F_IRQEN: irq_enable_w(v); break;
	case F_STARS: stars_enable_w(v); break;
	case F_FLIPX: flip_x_w(v); break;
	case F_FLIPY: b.flipy = v & 1; break;
	case F_FLIPXY: flip_x_w(v); b.flipy = v & 1; break;
	case F_GFXBANK: gfxbank_w((int)o, v); break;
	case F_BG_EN: b.bg_en = v & 1; break;
	case F_BG_R: b.bg_r = v & 1; break;
	case F_BG_G: b.bg_g = v & 1; break;
	case F_BG_B: b.bg_b = v & 1; break;
	case F_LFO: galsnd_lfo_w(&s_galsnd, (int)o, v); break;
	case F_SOUND: galsnd_sound_w(&s_galsnd, (int)o, v); break;
	case F_PITCH: galsnd_pitch_w(&s_galsnd, v); break;
	case F_SOUNDLATCH:
		b.soundlatch = v;
		b.latch_pending = 1;
		break;
	case F_KSND_CTRL: konami_sound_control_w(v); break;
	case F_EXPLORER_SCTRL: b.snd_int = 1; break;      // (ASSERT_LINE, cleared by the latch read)
	case F_FROGGERMC_SCTRL: b.snd_int = 1; break;     // (cleared on the acknowledge: HOLD)
	case F_ZZ_AY:
		switch (o & 0x300) {
		case 0x000:   // (bit 0 = WRITE, bit 1 = C/D)
			if (o & 1) {
				if (o & 2)
					ay8910_address_w(&s_ay[0], b.zz_latch);
				else
					ay8910_data_w(&s_ay[0], b.zz_latch);
			}
			break;
		case 0x100: b.zz_latch = (uint8_t)o; break;
		}
		break;
	case F_ZZ_BANK:
		b.bank[0] = v & 1;
		b.bank[1] = (~v) & 1;
		set_banks();
		break;
	case F_FOURPLAY_BANK:   // (gfxbank[4] the CPU bank, gfxbank[0] the graphics bank)
		b.gfxbank[4] = (uint8_t)((b.gfxbank[4] & (2 - o)) | (v << o));
		b.gfxbank[0] = b.gfxbank[4] == 3;
		b.bank[0] = b.gfxbank[4] & 3;
		set_banks();
		break;
	case F_VIDEIGHT_BANK: {
		static const uint8_t gfxbanks[8] = { 0, 10, 2, 8, 1, 9, 4, 11 };
		if (o == 2)
			stars_enable_w(v);
		else {
			b.gfxbank[4] = (uint8_t)((b.gfxbank[4] & (6 - o)) | (v << ((o + 1) >> 1)));
			gfxbank_w(0, gfxbanks[b.gfxbank[4] & 7]);
			b.bank[0] = b.gfxbank[4] & 7;
			set_banks();
		}
		break;
	}
	case F_VIDEIGHT_GFX:
		if (v < 2 && b.gfxbank[4] == 3) {   // (Moon Cresta)
			static const uint8_t gfxbanks[8] = { 8, 12, 8, 14, 8, 13, 8, 15 };
			if (o == 0) b.gfxbank[3] = (uint8_t)((b.gfxbank[3] & 6) | v);
			if (o == 1) b.gfxbank[3] = (uint8_t)((b.gfxbank[3] & 5) | (v << 1));
			if (o == 2) b.gfxbank[3] = (uint8_t)((b.gfxbank[3] & 3) | (v << 2));
			gfxbank_w(0, gfxbanks[b.gfxbank[3] & 7]);
		}
		if (v < 2 && b.gfxbank[4] == 2 && o == 2)   // (Uniwar S)
			gfxbank_w(0, (uint8_t)(v + 2));
		if (v < 2 && b.gfxbank[4] == 6 && o == 2)   // (Pisces)
			gfxbank_w(0, (uint8_t)(v + 4));
		break;
	case F_GUTT_BANK:
		b.bank[0] = v & 1;
		set_banks();
		break;
	case F_KB_SOUND1: b.kb_sound = (uint8_t)((b.kb_sound & ~1) | (v & 1)); break;
	case F_KB_SOUND2:
		b.kb_sound = (uint8_t)((b.kb_sound & ~2) | ((v & 1) << 1));
		b.soundlatch = (uint8_t)(b.kb_sound | 0xf0);
		break;
	case F_KB_SPEECH: b.kb_speech = v; break;
	case F_MSH_CS: b.msh_cs = v & 1; break;
	case F_CC_TRIG:   // (start on the rising edge)
		if (v && !b.cc_trig)
			b.cc_addr = (uint32_t)b.cc_start * 64;
		b.cc_trig = v != 0;
		break;
	case F_CC_RATE: b.cc_rate = v; break;
	case F_CC_VOL: b.cc_vol = v & 0x1f; break;
	case F_CHECKMAN_CMD: checkman_command(v); break;
	default: break;
	}
}

// ---- the main CPU's bus: the table resolved into pages
static int ent_match(const me_t *e, uint32_t a, uint32_t *off)
{
	const uint32_t m = a & ~(uint32_t)e->mirror;
	if (m < e->lo || m > e->hi)
		return 0;
	*off = m - e->lo;
	return 1;
}

static int resolve(uint32_t a, int write, uint32_t *off)
{
	for (int i = s_nmap - 1; i >= 0; i--) {
		const me_t *e = &s_map[i];
		if ((write ? e->wk : e->rk) == K_NONE)
			continue;
		if (ent_match(e, a, off))
			return i;
	}
	return -1;
}

static const uint8_t *bank_ptr(int bank) { return s_bank_base[bank] + (uint32_t)b.bank[bank] * s_bank_stride[bank]; }

static uint8_t *ent_ptr(const me_t *e, uint32_t off, int write)
{
	switch (write ? e->wk : e->rk) {
	case K_ROM: return s_rom + e->ra + off;
	case K_RAM: return b.pool + (write ? e->wa : e->ra) + off;
	case K_BANK: return (uint8_t *)bank_ptr((int)e->ra) + off;
	default: return NULL;
	}
}

static void build_pages(void)
{
	s_ent_used = 0;
	for (int p = 0; p < 256; p++) {
		for (int w = 0; w < 2; w++) {
			uint32_t off0;
			const int e0 = resolve((uint32_t)p << 8, w, &off0);
			int direct = e0 >= 0;
			uint8_t *base = NULL;
			if (direct) {
				const me_t *e = &s_map[e0];
				const int k = w ? e->wk : e->rk;
				direct = k == K_ROM || k == K_RAM || k == K_BANK;
				if (direct && w && k != K_RAM)
					direct = 0;
				if (direct) {
					base = ent_ptr(e, off0, w);
					for (int i = 1; i < 256 && direct; i++) {
						uint32_t off;
						const int ei = resolve((uint32_t)(p << 8 | i), w, &off);
						if (ei != e0 || ent_ptr(e, off, w) != base + i)
							direct = 0;
					}
				}
			}
			int16_t *pe = w ? &s_wpe[p] : &s_rpe[p];
			uint8_t **pt = w ? &s_went[p] : &s_rent[p];
			if (direct) {
				if (w) s_wmap[p] = base; else s_rmap[p] = (const uint8_t *)base;
				*pe = -1;
				*pt = NULL;
				continue;
			}
			if (w) s_wmap[p] = NULL; else s_rmap[p] = NULL;
			// a handled page: one entry, or each byte's
			int uniform = 1;
			for (int i = 1; i < 256 && uniform; i++) {
				uint32_t off;
				if (resolve((uint32_t)(p << 8 | i), w, &off) != e0)
					uniform = 0;
			}
			*pt = NULL;
			if (uniform) {
				*pe = (int16_t)e0;
				continue;
			}
			uint8_t *idx = s_ent_used < ENT_PAGES ? s_ent_mem[s_ent_used++] : NULL;
			if (!idx) {   // (not expected: the maps here have fewer mixed pages)
				*pe = -1;
				continue;
			}
			for (int i = 0; i < 256; i++) {
				uint32_t off;
				const int ei = resolve((uint32_t)(p << 8 | i), w, &off);
				idx[i] = (uint8_t)(ei < 0 ? 0xff : ei);
			}
			*pe = -2;
			*pt = idx;
		}
	}
}

static void set_banks(void)
{
	// the banked pages again (only the K_BANK entries' pages change)
	for (int i = 0; i < s_nmap; i++) {
		const me_t *e = &s_map[i];
		if (e->rk != K_BANK)
			continue;
		for (uint32_t a = e->lo; a <= e->hi; a += 256)
			if (s_rmap[a >> 8])
				s_rmap[a >> 8] = bank_ptr((int)e->ra) + (a - e->lo);
	}
}

static inline int page_ent(const int16_t *pe, uint8_t *const *pt, uint16_t a)
{
	const int e = pe[a >> 8];
	if (e != -2)
		return e;
	const int i = pt[a >> 8][a & 0xff];
	return i == 0xff ? -1 : i;
}

static uint8_t main_read(void *u, uint16_t a)
{
	(void)u;
	const uint8_t *d = s_rmap[a >> 8];
	if (d)
		return d[a & 0xff];
	const int ei = page_ent(s_rpe, s_rent, a);
	if (ei < 0)
		return s_unmap;
	const me_t *e = &s_map[ei];
	const uint32_t off = (a & ~(uint32_t)e->mirror) - e->lo;
	switch (e->rk) {
	case K_ROM: return s_rom[e->ra + off];
	case K_RAM: return b.pool[e->ra + off];
	case K_BANK: return bank_ptr((int)e->ra)[off];
	case K_PORT: return port_in((int)e->ra);
	case K_FN: return fn_read(e->ra, off);
	default: return s_unmap;
	}
}

static void main_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	uint8_t *p = s_wmap[a >> 8];
	if (p) {
		p[a & 0xff] = v;
		return;
	}
	const int ei = page_ent(s_wpe, s_went, a);
	if (ei < 0)
		return;
	const me_t *e = &s_map[ei];
	const uint32_t off = (a & ~(uint32_t)e->mirror) - e->lo;
	switch (e->wk) {
	case K_RAM: b.pool[e->wa + off] = v; break;
	case K_FN: fn_write(e->wa, off, v); break;
	default: break;
	}
}

static uint8_t main_in(z80 *z, uint8_t port)
{
	(void)z;
	switch (s_m->io) {
	case IO_BONGO:
		if (port == 0x02)
			return ay8910_data_r(&s_ay[0]);
		return 0;
	default:
		return s_m->io == IO_MSHUTTLE ? 0xff : 0;
	}
}

static void main_out(z80 *z, uint8_t port, uint8_t v)
{
	(void)z;
	if (s_i->patch == patch_checkman || s_i->patch == patch_dingoe) {   // (the sound command on any port)
		checkman_command(v);
		return;
	}
	switch (s_m->io) {
	case IO_BONGO:   // address_data_w: 0 address, 1 data
		if (port == 0x00) ay8910_address_w(&s_ay[0], v);
		else if (port == 0x01) ay8910_data_w(&s_ay[0], v);
		break;
	case IO_OZON1:   // data_address_w: 0 data, 1 address
		if (port == 0x00) ay8910_data_w(&s_ay[0], v);
		else if (port == 0x01) ay8910_address_w(&s_ay[0], v);
		break;
	case IO_JUNGSUB:
		b.soundlatch = v;
		if (!b.latch_pending) {   // (the data pending line on the sound CPU's NMI)
			b.latch_pending = 1;
			z80_gen_nmi(&s_snd);
		}
		break;
	case IO_MSHUTTLE:
		switch (port & 0x0f) {
		case 0x08: if (!b.msh_cs) ay8910_address_w(&s_ay[0], v); break;
		case 0x09: if (!b.msh_cs) ay8910_data_w(&s_ay[0], v); break;
		}
		break;
	default:
		break;
	}
}

static uint8_t main_in_msh(z80 *z, uint8_t port)
{
	(void)z;
	if ((port & 0x0f) == 0x0c)
		return b.msh_cs ? 0xff : ay8910_data_r(&s_ay[0]);
	return 0xff;
}

// ---- the sound CPUs
static uint8_t latch_read_clear(void)
{
	b.latch_pending = 0;
	return b.soundlatch;
}

static uint8_t ay_port_a(void *u)
{
	(void)u;
	switch (s_m->snd) {
	case SND_EXPLORER: return konami_sound_timer();
	case SND_TAKEOFF: b.snd_int = 0; return b.soundlatch;   // (explorer_sound_latch_r: the IRQ cleared)
	case SND_BONGO: return raw_port(P_DSW);
	default: return b.soundlatch;
	}
}
static uint8_t ay_port_b(void *u)
{
	(void)u;
	if (s_m->snd == SND_KONAMI1 || (s_m->flags & MF_FROGTMR))
		return frogger_sound_timer();
	return konami_sound_timer();
}
static void cc_port_a(void *u, uint8_t v) { (void)u; b.cc_start = v; }
static void sfx_ay1_a(void *u, uint8_t v) { (void)u; b.soundlatch2 = v; }
static void sfx_ay1_b(void *u, uint8_t v)   // (sample_control_w: the inverse of bit 0 clocks the sample CPU's INT)
{
	(void)u;
	if ((b.sample_ctrl & 1) && !(v & 1))
		b.smp_int = 1;
	b.sample_ctrl = v;
}
static void mz_ay0_b(void *u, uint8_t v)   // (monsterz: the main CPU's IRQ0, HOLD)
{
	(void)u;
	if ((b.mz_portb & 1) && !(v & 1))
		b.int_line = 1;
	b.mz_portb = v;
}
static void cc_port_b(void *u, uint8_t v) { (void)u; b.cc_loop = v; }

static uint8_t snd_read(void *u, uint16_t a)
{
	(void)u;
	switch (s_m->snd) {
	case SND_KONAMI2: case SND_AMIGO2: case SND_EXPLORER: case SND_TURPINS:
		if (s_m->flags & MF_MONSTERZ) {
			if (a < 0x5000)
				return a < s_srom_size ? s_srom[a] : 0;
			if (a == 0xd000) {   // (bitswap<8>(shift2 ^ 0x188, 14, 5, 8, 3, 7, 4, 2, 1))
				const uint32_t x = b.mz_shift2 ^ 0x188;
				return (uint8_t)(BIT(x, 14) << 7 | BIT(x, 5) << 6 | BIT(x, 8) << 5 | BIT(x, 3) << 4 | BIT(x, 7) << 3 |
				                 BIT(x, 4) << 2 | BIT(x, 2) << 1 | BIT(x, 1));
			}
		}
		if (a < 0x8000)
			return a < 0x2000 && a < s_srom_size ? s_srom[a] : 0;
		if (s_m->snd == SND_TURPINS && a == 0x9000)
			return konami_sound_timer();
		return (a & 0x1000) ? 0 : b.zram[a & 0x3ff];
	case SND_KONAMI1:
		a &= 0x7fff;
		if (a < 0x2000) return a < s_srom_size ? s_srom[a] : 0;
		if (a >= 0x4000 && a < 0x6000) return b.zram[a & 0x3ff];
		return 0;
	case SND_TAKEOFF:
		if (a < 0x1000) return a < s_srom_size ? s_srom[a] : 0;
		if (a >= 0x8000 && !(a & 0x1000)) return b.zram[a & 0x3ff];
		return 0;
	case SND_CHECKMAN: case SND_JUNGSUB:
		if (a < 0x1000) return a < s_srom_size ? s_srom[a] : 0xff;
		if (a >= 0x2000 && a < 0x2400) return b.zram[a & 0x3ff];
		return 0xff;
	case SND_CHECKMAJ:
		if (a < 0x1000) return a < s_srom_size ? s_srom[a] : 0xff;
		if (a >= 0x8000 && a < 0x8400) return b.zram[a & 0x3ff];
		if (a == 0xa002) return ay8910_data_r(&s_ay[0]);
		return 0xff;
	case SND_KINGBALL:
		a &= 0x3fff;
		return a < 0x2000 && a < s_srom_size ? s_srom[a] : 0xff;
	default:
		return 0;
	}
}

static void snd_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	switch (s_m->snd) {
	case SND_KONAMI2: case SND_AMIGO2: case SND_EXPLORER: case SND_TURPINS: case SND_TAKEOFF:
		if ((s_m->flags & MF_MONSTERZ) && a == 0xd000) {
			b.mz_shift = b.mz_shift << 8 | v;
			break;
		}
		if (a >= 0x8000) {
			if (!(a & 0x1000))
				b.zram[a & 0x3ff] = v;
			else if (s_m->snd != SND_TAKEOFF)
				konami_filter_w(a & 0xfff);
		}
		break;
	case SND_KONAMI1:
		a &= 0x7fff;
		if (a >= 0x4000 && a < 0x6000)
			b.zram[a & 0x3ff] = v;
		else if (a >= 0x6000)
			konami_filter_w(a & 0xfff);
		break;
	case SND_CHECKMAN: case SND_JUNGSUB:
		if (a >= 0x2000 && a < 0x2400)
			b.zram[a & 0x3ff] = v;
		break;
	case SND_CHECKMAJ:
		if (a >= 0x8000 && a < 0x8400)
			b.zram[a & 0x3ff] = v;
		else if (a == 0xa000)
			ay8910_address_w(&s_ay[0], v);
		else if (a == 0xa001)
			ay8910_data_w(&s_ay[0], v);
		break;
	default:
		break;
	}
}

static uint8_t snd_in(z80 *z, uint8_t port)
{
	(void)z;
	uint8_t r = 0xff;
	switch (s_m->snd) {
	case SND_KONAMI2: case SND_AMIGO2: case SND_EXPLORER: case SND_TURPINS:
		if (port & 0x20) r &= ay8910_data_r(&s_ay[1]);
		if (port & 0x80) r &= ay8910_data_r(&s_ay[0]);
		return r;
	case SND_KONAMI1:
		return (port & 0x40) ? ay8910_data_r(&s_ay[0]) : 0xff;
	case SND_TAKEOFF:
		if (port == 0x20) return konami_sound_timer();
		if (port == 0x80) return ay8910_data_r(&s_ay[0]);
		return 0xff;
	case SND_CHECKMAN: case SND_JUNGSUB:
		if (port == 0x03) return latch_read_clear();
		if (port == 0x06) return ay8910_data_r(&s_ay[0]);
		return 0xff;
	case SND_KINGBALL:
		return b.soundlatch;
	default:
		return 0xff;
	}
}

static void snd_out(z80 *z, uint8_t port, uint8_t v)
{
	(void)z;
	switch (s_m->snd) {
	case SND_KONAMI2: case SND_AMIGO2: case SND_EXPLORER: case SND_TURPINS:
		if ((s_m->flags & MF_MONSTERZ) && (port & 0x04))
			b.dac2 = v;
		if (port & 0x10) ay8910_address_w(&s_ay[1], v);
		else if (port & 0x20) ay8910_data_w(&s_ay[1], v);
		if (port & 0x40) ay8910_address_w(&s_ay[0], v);
		else if (port & 0x80) ay8910_data_w(&s_ay[0], v);
		break;
	case SND_KONAMI1:
		if (port & 0x40) ay8910_data_w(&s_ay[0], v);
		else if (port & 0x80) ay8910_address_w(&s_ay[0], v);
		break;
	case SND_TAKEOFF:
		if (port == 0x40) ay8910_address_w(&s_ay[0], v);
		else if (port == 0x80) ay8910_data_w(&s_ay[0], v);
		break;
	case SND_CHECKMAN: case SND_JUNGSUB:
		if (port == 0x04) ay8910_address_w(&s_ay[0], v);
		else if (port == 0x05) ay8910_data_w(&s_ay[0], v);
		break;
	case SND_KINGBALL:
		b.dac = v >> 4;
		break;
	default:
		break;
	}
}

// the SF-X sample board (sfx_sample_map / sfx_sample_portmap; monsterz_sample_map)
static uint8_t smp_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x6000) {
		if ((s_m->flags & MF_MONSTERZ) && a >= 0x4000)
			return 0;
		return a < s_arom_size ? s_arom[a] : 0;
	}
	if (a >= 0x8000 && !(a & 0x1000))
		return b.zram2[a & 0x3ff];
	return 0;
}
static void smp_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if ((s_m->flags & MF_MONSTERZ) && a == 0x4000)
		b.mz_shift2 = b.mz_shift2 << 8 | v;
	else if (a >= 0x8000 && !(a & 0x1000))
		b.zram2[a & 0x3ff] = v;
}
static uint8_t smp_in(z80 *z, uint8_t port)
{
	(void)z;
	return (port & 0x04) ? i8255_read(&s_ppi[2], port & 3) : 0xff;
}
static void smp_out(z80 *z, uint8_t port, uint8_t v)
{
	(void)z;
	if (port & 0x04) i8255_write(&s_ppi[2], port & 3, v);
	if (port & 0x10) b.dac = v;
}

static int has_snd_cpu(void)
{
	switch (s_m->snd) {
	case SND_KONAMI2: case SND_KONAMI1: case SND_AMIGO2: case SND_EXPLORER: case SND_TAKEOFF: case SND_TURPINS:
	case SND_CHECKMAN: case SND_CHECKMAJ: case SND_JUNGSUB: case SND_KINGBALL:
		return 1;
	default:
		return 0;
	}
}
static int n_ay(void)
{
	switch (s_m->snd) {
	case SND_KONAMI2: case SND_AMIGO2: case SND_EXPLORER: case SND_TURPINS: case SND_FANTASTC: return 2;
	case SND_NONE: case SND_KINGBALL: return 0;
	default: return 1;
	}
}
static int konami_filtered(void)
{
	return s_m->snd == SND_KONAMI2 || s_m->snd == SND_KONAMI1 || s_m->snd == SND_TURPINS;
}

static int s_line;   // (the line the CPUs run)
static int cur_line(void) { return s_line; }

// ---- the sound: three samples a line (48 kHz; Sidam 46.875 kHz), each after a third of the line of the sound CPU
static uint32_t s_snd_step, s_ay_step[2], s_cc_ticks;   // (16.16: the sound CPU's / the AYs' clocks a sample)
static int32_t s_ay_gain;
static uint32_t s_recip[64];

static int32_t ay_sample(int i, uint32_t clocks)
{
	int32_t buf[96];
	if (konami_filtered()) {
		int16_t ch[3 * 48];
		const int n = ay8910_render_ch(&s_ay[i], clocks, ch, 48);
		if (n <= 0)
			return 0;
		int32_t sum = 0;
		for (int k = 0; k < n; k++)
			sum += filter_rc_step(&b.flt[i][0], ch[3 * k]) + filter_rc_step(&b.flt[i][1], ch[3 * k + 1]) +
			       filter_rc_step(&b.flt[i][2], ch[3 * k + 2]);
		return (int32_t)(((int64_t)sum * s_recip[n < 64 ? n : 63]) >> 16) * s_ay_gain >> 8;
	}
	const int max = ay8910_out_max(&s_ay[i], clocks);
	if (max > 96)
		return 0;
	memset(buf, 0, (size_t)max * sizeof(buf[0]));
	const int n = ay8910_render(&s_ay[i], clocks, buf, max);
	if (n <= 0)
		return 0;
	int32_t sum = 0;
	for (int k = 0; k < n; k++)
		sum += buf[k];
	return (int32_t)(((int64_t)sum * s_recip[n < 64 ? n : 63]) >> 16);
}

static int32_t dc_block(int i, int32_t v)   // (a one-pole high pass at about 7 Hz)
{
	if (!b.dc_init) {   // (from the first sample's level: no thump at the start)
		b.dc_init = 1;
		b.dc[i] = v * 256;
	}
	b.dc[i] += (v * 256 - b.dc[i]) >> 10;
	return v - (b.dc[i] >> 8);
}

static void cc_tick(void)   // (cclimber_audio sample_tick)
{
	const uint8_t data = s_samples ? s_samples[(b.cc_addr >> 1) & 0x1fff] : 0x70;
	b.cc_dac = (b.cc_addr & 1) ? (data & 0x0f) : (data >> 4);
	if (b.cc_trig)
		b.cc_addr++;
	if (data == 0x70)   // (the end marker: from the loop point)
		b.cc_addr = (uint32_t)b.cc_loop * 64;
}

static void sound_slice(void)
{
	if (has_snd_cpu()) {
		rbx_prof_set(GX_PA_SOUND_CPU);
		b.snd_frac += s_snd_step;
		b.stgt += (uint32_t)(b.snd_frac >> 16);
		b.snd_frac &= 0xffff;
		const int hold = s_m->snd != SND_EXPLORER && s_m->snd != SND_TAKEOFF;
		z80_run(&s_snd, (unsigned long)b.stgt, &b.snd_int, hold);
	}
	if (s_m->flags & MF_SFX) {
		b.smp_frac += s_snd_step;
		b.stgt2 += (uint32_t)(b.smp_frac >> 16);
		b.smp_frac &= 0xffff;
		z80_run(&s_smp, (unsigned long)b.stgt2, &b.smp_int, 1);
	}
	rbx_prof_set(GX_PA_SOUND);
	int32_t v = 0;
	const int nay = n_ay();
	for (int i = 0; i < nay; i++) {
		b.ay_frac[i] += s_ay_step[i];
		const uint32_t clk = (uint32_t)(b.ay_frac[i] >> 16);
		b.ay_frac[i] &= 0xffff;
		v += ay_sample(i, clk);
	}
	if (s_m->snd == SND_KINGBALL)   // (DAC_4BIT_R2R at 0.53)
		v += (int32_t)b.dac * (32767 * 53 / 100 / 15);
	if (s_m->snd == SND_CCLIMBER) {
		for (uint32_t t = s_cc_ticks; t > 0; t--)   // (the 768 kHz clock: ticks every 256 - rate)
			if (++b.cc_cnt >= (uint32_t)(256 - b.cc_rate)) {
				b.cc_cnt = 0;
				cc_tick();
			}
		v += (int32_t)b.cc_dac * (int32_t)b.cc_vol * (32767 / 2 / 15 / 31);
	}
	if (s_m->flags & MF_SFX)   // (DAC_8BIT_R2R at 1.0, monsterz a second)
		v += ((int32_t)b.dac + ((s_m->flags & MF_MONSTERZ) ? (int32_t)b.dac2 : 0)) * 64;
	if (nay || s_m->snd == SND_KINGBALL)   // (the AYs' and DACs' DC level out: the amplifier's coupling capacitor)
		v = dc_block(0, v);
	if (s_m->disc != DISC_NONE)
		galsnd_render(&s_galsnd, &v, 1);
	if (b.snd_mute)
		v = 0;
	rbx_audio_put(v);
}

// ---- the drawing
static int s_bul, s_bg, s_tx, s_sx, s_frogadj, s_bul_base, s_nspr_gens, s_has_bullets;
static uint32_t s_ncolors_mask;
static int s_sfxadj, s_vis_x0, s_vis_w;   // (SF-X: the tilemap in columns; the visible part of the 256 pixels)
static uint16_t *s_fb;               // (SF-X: the full 256-pixel lines, the visible part copied out)   // (the gfx elements' colour count - 1: MAME takes the colour modulo it)

static void stars_update_origin(void)
{
	if (b.frame == b.star_origin_frame)
		return;
	// the period is 2^17 - 1; each frame the register clocks 2^17 times flipped (one extra), 2^17 - 2 not flipped
	const uint32_t n = (b.frame - b.star_origin_frame) % STAR_P;
	if (b.flipx)
		b.star_origin = (b.star_origin + n) % STAR_P;
	else
		b.star_origin = (b.star_origin + STAR_P - n) % STAR_P;
	b.star_origin_frame = b.frame;
}

static void tile_info(const snap_t *sn, uint32_t *code, uint32_t *color, uint8_t attrib, int x, int y)
{
	const uint8_t *gb = sn->gfxbank;
	switch (s_tx) {
	case TX_UPPER: *code += 0x100; break;
	case TX_FROGGER: *color = ((*color >> 1) & 3) | ((*color << 2) & 4); break;
	case TX_BAGMANMC: *code |= (uint32_t)gb[0] << 9; *color |= (uint32_t)gb[0] << 3; break;
	case TX_PISCES: *code |= (uint32_t)gb[0] << 8; break;
	case TX_MIMONKEY: *code |= (uint32_t)gb[0] << 8 | (uint32_t)gb[2] << 9; break;
	case TX_BATMAN2: if (*code & 0x80) *code |= (uint32_t)gb[0] << 8; break;
	case TX_MOONCRST:
		if (gb[2] && (*code & 0xc0) == 0x80)
			*code = (*code & 0x3f) | (uint32_t)gb[0] << 6 | (uint32_t)gb[1] << 7 | 0x0100;
		break;
	case TX_MOONQSR: *code |= (uint32_t)(attrib & 0x20) << 3; break;
	case TX_MSHUTTLE: *code |= (uint32_t)(attrib & 0x30) << 4; break;
	case TX_JUMPBUG:
		if ((*code & 0xc0) == 0x80 && (gb[2] & 1))
			*code += 128 + ((gb[0] & 1) << 6) + ((gb[1] & 1) << 7) + ((~gb[4] & 1) << 8);
		break;
	case TX_NAMENAYO:
		if ((attrib & 1) == 0) {   // (the main game display)
			const int attr = sn->ext[y & 0x1f];
			*code += (uint32_t)((attr & 0x38) >> 3) * 0x100;
			*color = (uint32_t)attr & 7;
		} else if ((attrib & 0xfe) == 0x20)   // (the course map)
			*code += 0x400;
		break;
	case TX_VIDEIGHT: *code |= (uint32_t)gb[0] << 8; *color |= (uint32_t)gb[4] << 3; break;
	default: break;
	}
	(void)x;
}

static void sprite_info(const snap_t *sn, const uint8_t *base, uint32_t *code, uint32_t *color, int *fx, int *fy)
{
	const uint8_t *gb = sn->gfxbank;
	switch (s_sx) {
	case SX_UPPER: *code += 0x40; break;
	case SX_FROGGER: *color = ((*color >> 1) & 3) | ((*color << 2) & 4); break;
	case SX_BAGMANMC: *code |= (uint32_t)gb[0] << 7 | 0x40; *color |= (uint32_t)gb[0] << 3; break;
	case SX_PISCES: *code |= (uint32_t)gb[0] << 6; break;
	case SX_MIMONKEY: *code |= (uint32_t)gb[0] << 6 | (uint32_t)gb[2] << 7; break;
	case SX_MOONCRST:
		if (gb[2] && (*code & 0x30) == 0x20)
			*code = (*code & 0x0f) | (uint32_t)gb[0] << 4 | (uint32_t)gb[1] << 5 | 0x40;
		break;
	case SX_MOONQSR: *code |= (uint32_t)(base[2] & 0x20) << 1; break;
	case SX_MSHUTTLE: *code |= (uint32_t)(base[2] & 0x30) << 2; break;
	case SX_CALIPSO: *code = base[1]; *fx = 0; *fy = 0; break;
	case SX_JUMPBUG:
		if ((*code & 0x30) == 0x20 && (gb[2] & 1))
			*code += 32 + ((gb[0] & 1) << 4) + ((gb[1] & 1) << 5) + ((~gb[4] & 1) << 6);
		break;
	case SX_NAMENAYO: if (base[2] & 0x08) *code += 0x40; *color += 8; break;
	case SX_GUTTANGT: *code |= 0x40; break;
	case SX_VIDEIGHT: *code |= (uint32_t)gb[0] << 6; *color |= (uint32_t)gb[4] << 3; break;
	default: break;
	}
}

static void update_tiles(const snap_t *sn)
{
	for (int t = 0; t < 1024; t++) {
		const int x = t & 31, y = t >> 5;
		uint32_t code = sn->vram[t];
		const uint8_t attrib = sn->obj[x * 2 + 1];
		uint32_t color = attrib & 7;
		tile_info(sn, &code, &color, attrib, x, y);
		code = rbx_wrap(code, s_nchr);
		color &= s_ncolors_mask;
		const uint32_t key = code | color << 16;
		if (s_tile_key[t] == key)
			continue;
		s_tile_key[t] = key;
		const uint8_t *src = s_chr + code * 64;
		uint8_t *d = s_sfxadj ? s_tiles + (x * 8) * 256 + y * 8 : s_tiles + (y * 8) * 256 + x * 8;
		const uint8_t base = (uint8_t)(color << 2);
		for (int j = 0; j < 8; j++, d += 256, src += 8)
			for (int i = 0; i < 8; i++)
				d[i] = (uint8_t)(base | src[i]);
	}
}

static void fill_row(uint16_t *d, int x0, int x1, uint16_t pen)
{
	for (int x = x0; x < x1; x++)
		d[x] = pen;
}

static void draw_background(const snap_t *sn, uint16_t *bm, int pitch)
{
	for (int y = 0; y < VIS_H; y++) {
		uint16_t *d = bm + y * pitch;
		switch (s_bg) {
		case BG_SCR: case BG_JUMPBUG:
			fill_row(d, 0, VIS_W, sn->bg_en ? PEN_BLUE : PEN_BLACK);
			break;
		case BG_ANTEATER: {   // (the blue split, 56 / 256 - 56)
			const uint16_t c = sn->bg_en ? PEN_BLUE : PEN_BLACK;
			if (sn->flipx) {
				fill_row(d, 0, 256 - 56, PEN_BLACK);
				fill_row(d, 256 - 56, VIS_W, c);
			} else {
				fill_row(d, 0, 56, c);
				fill_row(d, 56, VIS_W, PEN_BLACK);
			}
			break;
		}
		case BG_FROGGER:   // (the river, 128 / 128)
			if (sn->flipx) {
				fill_row(d, 0, 128, PEN_BLACK);
				fill_row(d, 128, VIS_W, PEN_FROGBLUE);
			} else {
				fill_row(d, 0, 128, PEN_FROGBLUE);
				fill_row(d, 128, VIS_W, PEN_BLACK);
			}
			break;
		case BG_TURTLES: case BG_SFX:
			fill_row(d, 0, VIS_W, (uint16_t)(PEN_TURTLES + (sn->bg_r << 2 | sn->bg_g << 1 | sn->bg_b)));
			break;
		default:
			fill_row(d, 0, VIS_W, PEN_BLACK);
			break;
		}
	}
	// the stars (galaxian_draw_stars / scramble_draw_stars / jumpbug's)
	if (!sn->stars_enabled || (s_bg != BG_GAL && s_bg != BG_SCR && s_bg != BG_ANTEATER && s_bg != BG_JUMPBUG && s_bg != BG_SFX))
		return;
	const int scr = s_bg == BG_SCR || s_bg == BG_ANTEATER || s_bg == BG_SFX;
	const uint32_t origin = scr ? 0 : sn->star_origin;
	const int maxx = s_bg == BG_JUMPBUG ? 232 : 256;
	static const uint8_t colormask[4] = { 0x20, 0x08, 0xff, 0xff };
	const int blink = sn->stars_blink & 3;
	const uint8_t mask = scr ? colormask[blink] : 0xff;
	for (int i = 0; i < s_nstars; i++) {
		const uint32_t idx = s_stars[i] >> 8;
		const uint8_t col = (uint8_t)s_stars[i];
		uint32_t rel = idx >= origin ? idx - origin : idx + STAR_P - origin;
		const int y = (int)(rel >> 9);
		if (y < VIS_Y0 || y >= VIS_Y0 + VIS_H)
			continue;
		const int x = (int)((rel & 511) >> 1);
		if (x >= maxx || !((y ^ (x >> 3)) & 1))
			continue;
		if (scr && blink == 2 && !(y & 2))
			continue;
		if (!((col | 0x80) & mask))
			continue;
		bm[(y - VIS_Y0) * pitch + x] = (uint16_t)(PEN_STAR + col);
	}
}

static void draw_tilemap(const snap_t *sn, uint16_t *bm, int pitch, int opaque_x0, int opaque_x1)
{
	uint8_t scroll[32];
	for (int c = 0; c < 32; c++) {
		uint8_t s = sn->obj[c * 2];
		if (s_frogadj)   // (Frogger: the top and bottom 4 bits swapped entering the adder)
			s = (uint8_t)(s >> 4 | s << 4);
		scroll[c] = s;
	}
	const int fx = sn->flipx, fy = sn->flipy;
	if (s_sfxadj) {   // (a scroll a row: set_scrollx(row, data); flipped, about the visible area's centre, 8 more)
		for (int y = 0; y < VIS_H; y++) {
			const int ty = fy ? 255 - (y + VIS_Y0) : y + VIS_Y0;
			const int sc = scroll[ty >> 3];
			const uint8_t *src = s_tiles + ty * 256;
			uint16_t *d = bm + y * pitch;
			for (int x = 0; x < 256; x++) {
				const uint8_t p = src[fx ? 255 - ((x - 8 - sc) & 255) : (x + sc) & 255];
				if (p & 3)
					d[x] = p;
			}
		}
		return;
	}
	for (int y = 0; y < VIS_H; y++) {
		const int sy = y + VIS_Y0;
		uint16_t *d = bm + y * pitch;
		for (int c = 0; c < 32; c++) {
			const int tc = fx ? 31 - c : c;
			const int ty = fy ? (255 - sy + scroll[tc]) & 255 : (sy + scroll[tc]) & 255;
			const uint8_t *src = s_tiles + ty * 256 + tc * 8;
			uint16_t *o = d + c * 8;
			for (int i = 0; i < 8; i++) {
				const uint8_t p = fx ? src[7 - i] : src[i];
				if (p & 3)
					o[i] = p;
				else if (c * 8 + i >= opaque_x0 && c * 8 + i < opaque_x1)   // (namenayo's opaque part)
					o[i] = p;
			}
		}
	}
}

static void draw_sprites(const snap_t *sn, uint16_t *bm, int pitch, const uint8_t *spritebase, int y0, int y1)
{
	int cx0 = 0, cx1 = VIS_W;   // (sprites_clip: 16 of the line buffer's 256 pixels, plus the sprite offset's 1)
	if (s_bg != BG_NAMENAYO) {
		if (sn->flipx)
			cx1 = 256 - (16 + 1);
		else
			cx0 = 16 + 1;
	}
	for (int sprnum = 7; sprnum >= 0; sprnum--) {
		const uint8_t *base = spritebase + sprnum * 4;
		const uint8_t base0 = s_frogadj ? (uint8_t)(base[0] >> 4 | base[0] << 4) : base[0];
		uint8_t sy = (uint8_t)(240 - (base0 - (s_sfxadj ? sprnum >= 3 : sprnum < 3)));   // (the first three match y - 1)
		uint32_t code = base[1] & 0x3f;
		int flipx = (base[1] & 0x40) != 0, flipy = (base[1] & 0x80) != 0;
		uint32_t color = base[2] & 7;
		uint8_t sx = (uint8_t)(base[3] + 1);   // (the +1 of the sprites against the tiles)
		sprite_info(sn, base, &code, &color, &flipx, &flipy);
		if (sn->flipx) {
			sx = (uint8_t)(240 - sx);
			flipx = !flipx;
		}
		if (sn->flipy) {
			sy = (uint8_t)(240 - sy);
			flipy = !flipy;
		}
		code = rbx_wrap(code, s_nspr);
		color &= s_ncolors_mask;
		const uint8_t *src = s_spr + code * 256;
		const uint16_t pbase = (uint16_t)(color * 4);
		for (int j = 0; j < 16; j++) {
			const int yy = sy + j;
			if (yy < y0 || yy >= y1)
				continue;
			const uint8_t *s = src + (flipy ? 15 - j : j) * 16;
			uint16_t *d = bm + (yy - VIS_Y0) * pitch;
			for (int i = 0; i < 16; i++) {
				const int xx = sx + i;
				if (xx < cx0 || xx >= cx1)
					continue;
				const uint8_t p = s[flipx ? 15 - i : i];
				if (p)
					d[xx] = (uint16_t)(pbase + p);
			}
		}
	}
}

static inline void bullet_px(uint16_t *d, int x, uint16_t pen)
{
	if (x >= 0 && x < VIS_W)
		d[x] = pen;
}

static void draw_bullet(uint16_t *d, int offs, int x)
{
	switch (s_bul) {
	case BUL_GAL: case BUL_THEEND: {
		const uint16_t pen = (uint16_t)((s_bul == BUL_THEEND ? PEN_BUL_TE : PEN_BUL) + offs);
		x -= 4;
		for (int k = 0; k < 4; k++)
			bullet_px(d, x + k, pen);
		break;
	}
	case BUL_SFX:   // (sfx_draw_bullet: Scramble's 16 pixels on)
		x += 16;
		x -= 4;
		bullet_px(d, x - 1, PEN_YELLOW);
		bullet_px(d, x - 2, PEN_YELLOW);
		break;
	case BUL_SCR:
		x -= 4;
		bullet_px(d, x - 1, PEN_YELLOW);
		bullet_px(d, x - 2, PEN_YELLOW);
		break;
	case BUL_MSH:
		for (int k = 0; k < 4; k++) {
			--x;
			bullet_px(d, x, (uint16_t)(((x & 0x40) == 0) ? PEN_MSH + ((x >> 2) & 7) : PEN_PURPLE));
		}
		break;
	default:
		break;
	}
}

static void draw_bullets(const snap_t *sn, uint16_t *bm, int pitch)
{
	const uint8_t *base = sn->obj + s_bul_base;
	for (int y = VIS_Y0; y < VIS_Y0 + VIS_H; y++) {
		int shell = -1, missile = -1;
		uint8_t effy = (uint8_t)(sn->flipy ? ((y - 1) ^ 255) : (y - 1));   // (the first three match y - 1)
		for (int w = 0; w < 3; w++)
			if ((uint8_t)(base[w * 4 + 1] + effy) == 0xff)
				shell = w;
		effy = (uint8_t)(sn->flipy ? (y ^ 255) : y);
		for (int w = 3; w < 8; w++)
			if ((uint8_t)(base[w * 4 + 1] + effy) == 0xff) {
				if (w != 7)
					shell = w;
				else
					missile = w;
			}
		uint16_t *d = bm + (y - VIS_Y0) * pitch;
		if (shell >= 0)
			draw_bullet(d, shell, 255 - base[shell * 4 + 3]);
		if (missile >= 0)
			draw_bullet(d, missile, 255 - base[missile * 4 + 3]);
	}
}

static void gx_render(void)
{
	const snap_t *sn = s_snap;
	memcpy(rbx_video.palette, s_pal, sizeof(s_pal));
	uint16_t *bm = s_fb ? s_fb : rbx_video.bitmap;
	const int pitch = s_fb ? VIS_W : rbx_video.pitch;
	update_tiles(sn);
	draw_background(sn, bm, pitch);
	int ox0 = 0, ox1 = 0;
	if (s_bg == BG_NAMENAYO) {
		if (sn->flipx)
			ox0 = 72, ox1 = VIS_W;
		else
			ox0 = 0, ox1 = 184;
	}
	draw_tilemap(sn, bm, pitch, ox0, ox1);
	if (s_m->flags & MF_TIMEFGTR) {   // (the sprite RAM's base for each 64-line part of the screen)
		static const int bands[5] = { 16, 48, 112, 176, 240 };
		static const int bases[4] = { 0x40, 0x140, 0x240, 0x340 };
		for (int k = 0; k < 4; k++)
			for (int g = 0; g < s_nspr_gens; g++)
				draw_sprites(sn, bm, pitch, sn->obj + bases[k] + g * 0x20, bands[k], bands[k + 1]);
	} else
		for (int g = 0; g < s_nspr_gens; g++)
			draw_sprites(sn, bm, pitch, sn->obj + 0x40 + g * 0x20, VIS_Y0, VIS_Y0 + VIS_H);
	if (s_has_bullets)
		draw_bullets(sn, bm, pitch);
	if (s_fb)   // (sfx, monsterz: ORIENTATION_FLIP_X, the visible part mirrored)
		for (int y = 0; y < VIS_H; y++) {
			const uint16_t *src = s_fb + y * VIS_W + s_vis_x0 + s_vis_w - 1;
			uint16_t *d = rbx_video.bitmap + y * rbx_video.pitch;
			for (int x = 0; x < s_vis_w; x++)
				d[x] = src[-x];
		}
}

static void vblank_capture(void)
{
	const unsigned pa = rbx_prof_enter(GX_PA_VBLANK);
	rbx_video_wait_idle();
	stars_update_origin();
	snap_t *sn = s_snap;
	memcpy(sn->vram, b.pool + PO_VRAM, sizeof(sn->vram));
	memcpy(sn->obj, b.pool + PO_OBJ, sizeof(sn->obj));
	memcpy(sn->ext, b.pool + PO_EXT, sizeof(sn->ext));
	sn->flipx = b.flipx;
	sn->flipy = b.flipy;
	sn->stars_enabled = b.stars_enabled;
	sn->stars_blink = b.stars_blink;
	sn->bg_en = b.bg_en;
	sn->bg_r = b.bg_r;
	sn->bg_g = b.bg_g;
	sn->bg_b = b.bg_b;
	memcpy(sn->gfxbank, b.gfxbank, sizeof(sn->gfxbank));
	sn->star_origin = b.star_origin;
	rbx_video_frame_done();
	rbx_prof_leave(pa);
}

// ---- frame
static uint32_t s_blink_period;   // (the 555's period in lines x 16)

static void gm_game_changed(int sel)   // (gmgalax: the bank and the graphics bank, the stars off, the CPU reset)
{
	b.gm_sel = (uint8_t)sel;
	b.bank[0] = (uint8_t)sel;
	set_banks();
	gfxbank_w(0, (uint8_t)sel);
	stars_enable_w(0);
	z80 c = s_main;
	z80_init(&s_main);
	s_main.read_byte = c.read_byte;
	s_main.write_byte = c.write_byte;
	s_main.port_in = c.port_in;
	s_main.port_out = c.port_out;
	s_main.read_map = c.read_map;
	s_main.op_rom = c.op_rom;
	s_main.int_data = 0xff;
	s_main.cyc = c.cyc;
}

static void gx_frame(void)
{
	rbx_audio.frame_mhz = s_frame_mhz;
	if (s_custom == CU_GMGALAX) {
		const int sel = raw_port(P_GAMESEL) & 1;
		if (sel != b.gm_sel)
			gm_game_changed(sel);
	}
	for (int line = 0; line < LINES; line++) {
		s_line = line;
		if (line == VIS_Y0 + VIS_H) {   // (VBSTART: the screen update, the interrupt)
			vblank_capture();
			b.frame++;
			if (b.irq_enabled) {
				if (s_m->flags & MF_IRQ0)
					b.int_line = 1;
				else if (!b.nmi_line) {
					b.nmi_line = 1;
					z80_gen_nmi(&s_main);
				}
			}
			if (s_m->snd == SND_CHECKMAN || s_m->snd == SND_JUNGSUB)   // (the sound CPU's vblank IRQ0, HOLD)
				b.snd_int = 1;
		}
		if (s_m->snd == SND_CHECKMAJ && (line & 7) == 0)   // (checkmaj_irq0_gen: every 8 lines)
			b.snd_int = 1;
		if (s_m->flags & MF_STARTMR) {
			b.blink_acc += 16;
			if (b.blink_acc >= s_blink_period) {
				b.blink_acc -= s_blink_period;
				b.stars_blink++;
			}
		}
		rbx_prof_set(GX_PA_MAIN);
		b.tgt += 192;
		z80_run(&s_main, (unsigned long)b.tgt, &b.int_line, (s_m->flags & MF_MONSTERZ) ? 1 : 0);
		for (int k = 0; k < 3; k++)
			sound_slice();
	}
	rbx_prof_set(RBX_PA_DRIVER);
}

// ---- the decryptions (galaxian.cpp's helpers and driver inits)
static void decode_mooncrst(const uint8_t *rom, uint8_t *dest, int length)
{
	for (int offs = 0; offs < length; offs++) {
		const uint8_t data = rom[offs];
		uint8_t res = data;
		if (BIT(data, 1)) res ^= 0x40;
		if (BIT(data, 5)) res ^= 0x04;
		if (!BIT(offs, 0)) res = bitswap8(res, 7, 2, 5, 4, 3, 6, 1, 0);
		dest[offs] = res;
	}
}

static void swap_blocks(uint8_t *rom, uint32_t len)   // (each 0x100 block's halves mirrored: j <-> j ^ 0xff)
{
	for (uint32_t i = 0; i < len; i += 0x100)
		for (int j = 0; j < 0x80; j++) {
			const uint8_t t = rom[i | j];
			rom[i | j] = rom[i | (j ^ 0xff)];
			rom[i | (j ^ 0xff)] = t;
		}
}

static void mshuttle_decode(const uint8_t convtable[8][16])
{
	for (int addr = 0; addr < 0x8000; addr++) {
		const uint8_t src = s_rom[addr];
		const int i = (addr & 1) | (src & 0x02) | ((src & 0x80) >> 5);
		const int j = (src & 0x01) | ((src & 0x04) >> 1) | ((src & 0x10) >> 2) | ((src & 0x40) >> 3);
		s_opcodes[addr] = (uint8_t)((src & 0xaa) | convtable[i][j]);
	}
}

static void decrypt(int dec, uint8_t *gfx, uint32_t gfx_size)
{
	uint8_t *rom = s_rom;
	switch (dec) {
	case DEC_HIGHROLL:
		for (int i = 0; i < 0x4000; i++) {
			uint8_t x = rom[i];
			switch (i & 3) {
			case 0: x = bitswap8(x, 1, 6, 7, 4, 5, 2, 3, 0); break;
			case 1: x = bitswap8(x, 5, 6, 3, 4, 1, 2, 7, 0); break;
			case 2: x = bitswap8(x, 3, 6, 1, 4, 7, 2, 5, 0); break;
			case 3: x = bitswap8(x, 1, 6, 3, 4, 5, 2, 7, 0); break;
			}
			s_opcodes[i] = x;
		}
		for (int i = 0; i < 0x4000; i++)
			rom[i] = (i & 1) ? bitswap8(rom[i], 1, 6, 7, 4, 3, 2, 5, 0) : bitswap8(rom[i], 3, 6, 1, 4, 5, 2, 7, 0);
		break;
	case DEC_WAROFBUGG: {
		static uint8_t buf[0x4000];
		memcpy(buf, rom, 0x4000);
		for (int i = 0; i < 0x40; i++)
			for (int j = 0; j < 0x100; j++)
				rom[i * 256 + j] = buf[i * 256 + (j ^ 0xff)];
		break;
	}
	case DEC_LADYBUGG2:
		swap_blocks(rom, 0x10000);
		memcpy(s_opcodes, rom, 0x10000);   // (ladybugg2_opcodes_map: 0000-0fff from 4000, 1000-3fff from 1000)
		memcpy(s_opcodes, rom + 0x4000, 0x1000);
		break;
	case DEC_JUMPBUGBC:
		swap_blocks(rom, 0x10000);
		break;
	case DEC_GUTTANGTS3:
		swap_blocks(rom, 0x4800);
		break;
	case DEC_VICTORYC:
		for (uint32_t i = 0; i < s_rom_size; i++) {
			if (i & 0x80) rom[i] ^= 0x80;
			if (i & 0x20) rom[i] ^= 0x04;
			if (i & 0x04) rom[i] ^= 0x40;
			if (i & 0x01) rom[i] ^= 0x08;
			rom[i] = bitswap8(rom[i], 6, 3, 5, 4, 2, 7, 1, 0);
		}
		break;
	case DEC_BIGKONGGX:
		for (uint32_t i = 0; i + 256 <= s_rom_size; i += 256)
			for (int j = 0; j < 128; j++) {
				const uint8_t t = rom[i + j];
				rom[i + j] = rom[i + 255 - j];
				rom[i + 255 - j] = t;
			}
		break;
	case DEC_CKONGGX: {
		static const uint16_t remap[88] = {
			0x5000, 0x0d00, 0x0e00, 0x0f00, 0x0c00, 0x0100, 0x0200, 0x0300, 0x0400, 0x0500, 0x0600, 0x0700, 0x0800, 0x0900, 0x0a00, 0x0b00,
			0x1400, 0x1500, 0x1600, 0x1700, 0x1800, 0x1900, 0x1a00, 0x1b00, 0x1c00, 0x1d00, 0x1e00, 0x1f00, 0x1000, 0x1100, 0x1200, 0x1300,
			0x2400, 0x2500, 0x2600, 0x2700, 0x2800, 0x2900, 0x2a00, 0x2b00, 0x2c00, 0x2d00, 0x2e00, 0x2f00, 0x2000, 0x2100, 0x2200, 0x2300,
			0x3400, 0x3500, 0x3600, 0x3700, 0x3800, 0x3900, 0x3a00, 0x3b00, 0x3c00, 0x3d00, 0x3e00, 0x3f00, 0x3000, 0x3100, 0x3200, 0x3300,
			0x4400, 0x4500, 0x4600, 0x4700, 0x4800, 0x4900, 0x4a00, 0x4b00, 0x4c00, 0x4d00, 0x4e00, 0x4f00, 0x4000, 0x4100, 0x4200, 0x4300,
			0x0000, 0x5100, 0x5200, 0x5300, 0x0000, 0x5100, 0x5200, 0x5300 };
		static uint8_t buf[0x5800];
		for (int i = 0; i < 88; i++)
			memcpy(buf + i * 0x100, rom + remap[i], 0x100);
		memcpy(rom, buf, 0x5800);
		break;
	}
	case DEC_MOONCRST: case DEC_JUNGSUB:
		decode_mooncrst(rom, rom, 0x4000);
		break;
	case DEC_MOONQSR:
		decode_mooncrst(rom, s_opcodes, 0x4000);
		break;
	case DEC_DEVILFSHG: {
		const uint32_t len = s_rom_size;
		uint8_t *buf = (uint8_t *)malloc(len);
		if (!buf)
			break;
		memcpy(buf, rom, len);
		for (uint32_t i = 0; i < len; i++)   // (bitswap<5>(i, 1, 0, 3, 4, 2))
			rom[i] = buf[(i & ~0x1fu) | (BIT(i, 1) << 4 | BIT(i, 0) << 3 | BIT(i, 3) << 2 | BIT(i, 4) << 1 | BIT(i, 2))];
		free(buf);
		break;
	}
	case DEC_CHECKMAN: {
		static const uint8_t xortable[8][4] = { { 6,0,6,0 }, { 5,1,5,1 }, { 4,2,6,1 }, { 2,4,5,0 },
		                                        { 4,6,1,5 }, { 0,6,2,5 }, { 0,2,0,2 }, { 1,4,1,4 } };
		for (uint32_t offs = 0; offs < s_rom_size; offs++) {
			uint8_t data = rom[offs];
			const uint32_t line = offs & 7;
			data ^= (uint8_t)((BIT(data, xortable[line][0]) << xortable[line][1]) |
			                  (BIT(data, xortable[line][2]) << xortable[line][3]));
			rom[offs] = data;
		}
		break;
	}
	case DEC_DINGOE:
		for (uint32_t offs = 0; offs < s_rom_size; offs++) {
			uint8_t data = rom[offs];
			data ^= (uint8_t)(BIT(data, 2) << 4);
			data ^= (uint8_t)(BIT(data, 5) << 0);
			data ^= 0x02;
			if (offs & 0x02)
				data = bitswap8(data, 7, 6, 5, 0, 3, 2, 1, 4);
			rom[offs] = data;
		}
		break;
	case DEC_MSHUTTLE: {
		static const uint8_t t[8][16] = {
			{ 0x40,0x41,0x44,0x15,0x05,0x51,0x54,0x55,0x50,0x00,0x01,0x04,0xff,0x10,0x11,0x14 },
			{ 0x45,0x51,0x55,0x44,0x40,0x11,0x05,0x41,0x10,0x14,0x54,0x50,0x15,0x04,0x00,0x01 },
			{ 0x11,0x14,0x10,0x00,0x44,0x05,0xff,0x04,0x45,0x15,0x55,0x50,0xff,0x01,0x54,0x51 },
			{ 0x14,0x01,0x11,0x10,0x50,0x15,0x00,0x40,0x04,0x51,0x45,0x05,0x55,0x54,0xff,0x44 },
			{ 0x04,0x10,0xff,0x40,0x15,0x41,0x50,0x50,0x11,0xff,0x14,0x00,0x51,0x45,0x55,0x01 },
			{ 0x44,0x45,0x00,0x51,0xff,0xff,0x15,0x11,0x01,0x10,0x04,0x55,0x05,0x40,0x50,0x41 },
			{ 0x51,0x00,0x01,0x05,0x04,0x55,0x54,0x50,0x41,0xff,0x11,0x15,0x14,0x10,0x44,0x40 },
			{ 0x05,0x04,0x51,0x01,0xff,0xff,0x55,0xff,0x00,0x50,0x15,0x14,0x44,0x41,0x40,0x54 } };
		mshuttle_decode(t);
		break;
	}
	case DEC_MSHUTTLJ: {
		static const uint8_t t[8][16] = {
			{ 0x41,0x54,0x51,0x14,0x05,0x10,0x01,0x55,0x44,0x11,0x00,0x50,0x15,0x40,0x04,0x45 },
			{ 0x50,0x11,0x40,0x55,0x51,0x14,0x45,0x04,0x54,0x15,0x10,0x05,0x44,0x01,0x00,0x41 },
			{ 0x44,0x11,0x00,0x50,0x41,0x54,0x04,0x14,0x15,0x40,0x51,0x55,0x05,0x10,0x01,0x45 },
			{ 0x10,0x50,0x54,0x55,0x01,0x44,0x40,0x04,0x14,0x11,0x00,0x41,0x45,0x15,0x51,0x05 },
			{ 0x14,0x41,0x01,0x44,0x04,0x50,0x51,0x45,0x11,0x40,0x54,0x15,0x10,0x00,0x55,0x05 },
			{ 0x01,0x05,0x41,0x45,0x54,0x50,0x55,0x10,0x11,0x15,0x51,0x14,0x44,0x40,0x04,0x00 },
			{ 0x05,0x55,0x00,0x50,0x11,0x40,0x54,0x14,0x45,0x51,0x10,0x04,0x44,0x01,0x41,0x15 },
			{ 0x55,0x50,0x15,0x10,0x01,0x04,0x41,0x44,0x45,0x40,0x05,0x00,0x11,0x14,0x51,0x54 } };
		mshuttle_decode(t);
		break;
	}
	case DEC_FANTASTC: {
		static const uint16_t lut[32] = { 0, 2, 4, 6, 7, 3, 5, 1, 6, 0, 2, 4, 1, 5, 3, 0,
		                                  2, 4, 6, 3, 5, 6, 0, 2, 4, 1, 1, 5, 3, 7, 7, 7 };
		static uint8_t buf[0x8000];
		memcpy(buf, rom, 0x8000);
		for (int i = 0; i < 32; i++)
			memcpy(rom + i * 0x400, buf + lut[i] * 0x1000 + (i & 3) * 0x400, 0x400);
		break;
	}
	case DEC_SCOBRAE:
		for (int offs = 0; offs < 0x6000; offs++) {
			int i = offs & 0x7f;
			int x = rom[offs];
			if (offs & 0x80) i ^= 0x7f;
			if (i & 0x01) x ^= 0x49;
			if (i & 0x02) x ^= 0x21;
			if (i & 0x04) x ^= 0x18;
			if (i & 0x08) x ^= 0x12;
			if (i & 0x10) x ^= 0x84;
			if (i & 0x20) x ^= 0x24;
			if (i & 0x40) x ^= 0x40;
			rom[offs] = (uint8_t)(x ^ 0xff);
		}
		break;
	case DEC_LOSTTOMB: case DEC_ANTEATER: {
		uint8_t *buf = (uint8_t *)malloc(gfx_size);
		if (!buf)
			break;
		memcpy(buf, gfx, gfx_size);
		for (uint32_t offs = 0; offs < gfx_size; offs++) {
			uint32_t src;
			if (dec == DEC_ANTEATER) {
				src = offs & 0x9bf;
				src |= (BIT(offs, 4) ^ BIT(offs, 9) ^ (BIT(offs, 2) & BIT(offs, 10))) << 6;
				src |= (BIT(offs, 2) ^ BIT(offs, 10)) << 9;
				src |= (BIT(offs, 0) ^ BIT(offs, 6) ^ 1) << 10;
			} else {
				src = offs & 0xa7f;
				src |= ((BIT(offs, 1) & BIT(offs, 8)) | ((1 ^ BIT(offs, 1)) & BIT(offs, 10))) << 7;
				src |= (BIT(offs, 7) ^ (BIT(offs, 1) & (BIT(offs, 7) ^ BIT(offs, 10)))) << 8;
				src |= ((BIT(offs, 1) & BIT(offs, 7)) | ((1 ^ BIT(offs, 1)) & BIT(offs, 8))) << 10;
			}
			gfx[offs] = src < gfx_size ? buf[src] : 0;
		}
		free(buf);
		break;
	}
	case DEC_FROGGER: case DEC_FROGGERS: case DEC_FROGGERMC: {
		// the first ROM of the sound CPU with D0 and D1 swapped (froggermc: a bigger one); Frogger's 2nd gfx ROM too
		const uint32_t n = dec == DEC_FROGGERMC ? 0x1000 : 0x800;
		for (uint32_t offs = 0; offs < n && offs < s_srom_size; offs++)
			s_srom[offs] = bitswap8(s_srom[offs], 7, 6, 5, 4, 3, 2, 0, 1);
		if (dec == DEC_FROGGER)
			for (uint32_t offs = 0x800; offs < 0x1000 && offs < gfx_size; offs++)
				gfx[offs] = bitswap8(gfx[offs], 7, 6, 5, 4, 3, 2, 0, 1);
		break;
	}
	case DEC_SUPERBON:
		for (int i = 0; i < 0x1000; i++)
			switch (i & 0x280) {
			case 0x000: rom[i] ^= 0x92; break;
			case 0x080: rom[i] ^= 0x82; break;
			case 0x200: rom[i] ^= 0x12; break;
			case 0x280: rom[i] ^= 0x10; break;
			}
		break;
	case DEC_CRAZYM:
		for (int i = 0; i < 0x4000; i++) {
			const int odd = i & 1;
			switch (rom[i] & 0x38) {
			case 0x00: rom[i] ^= odd ? 0x30 : 0x20; break;
			case 0x08: rom[i] ^= odd ? 0x30 : 0x08; break;
			case 0x10: rom[i] ^= odd ? 0x30 : 0x28; break;
			case 0x18: rom[i] ^= odd ? 0x18 : 0x08; break;
			case 0x20: rom[i] ^= odd ? 0x30 : 0x10; break;
			case 0x28: rom[i] ^= odd ? 0x30 : 0x20; break;
			case 0x30: rom[i] ^= 0x18; break;
			case 0x38: rom[i] ^= odd ? 0x30 : 0x20; break;
			}
		}
		break;
	case DEC_MIMONKEY: {
		static const uint8_t xt[8][16] = {
			{ 0x03,0x03,0x05,0x07,0x85,0x00,0x85,0x85,0x80,0x80,0x06,0x03,0x03,0x00,0x00,0x81 },
			{ 0x83,0x87,0x03,0x87,0x06,0x00,0x06,0x04,0x02,0x00,0x84,0x84,0x04,0x00,0x01,0x83 },
			{ 0x82,0x82,0x84,0x02,0x04,0x00,0x00,0x03,0x82,0x00,0x06,0x80,0x03,0x00,0x81,0x07 },
			{ 0x06,0x06,0x82,0x81,0x85,0x00,0x04,0x07,0x81,0x05,0x04,0x00,0x03,0x00,0x82,0x84 },
			{ 0x07,0x07,0x80,0x07,0x07,0x00,0x85,0x86,0x00,0x07,0x06,0x04,0x85,0x00,0x86,0x85 },
			{ 0x81,0x83,0x02,0x02,0x87,0x00,0x86,0x03,0x04,0x06,0x80,0x05,0x87,0x00,0x81,0x81 },
			{ 0x01,0x01,0x00,0x07,0x07,0x00,0x01,0x01,0x07,0x07,0x06,0x00,0x06,0x00,0x07,0x07 },
			{ 0x80,0x87,0x81,0x87,0x83,0x00,0x84,0x01,0x01,0x86,0x86,0x80,0x86,0x00,0x86,0x86 } };
		for (int a = 0; a < 0x4000; a++) {   // (MAME's 16-line table is the 8 lines twice)
			const int line = (a & 0x07) | ((a & 0x200) >> 6);
			const int col = ((rom[a] & 0x80) >> 4) | (rom[a] & 0x07);
			rom[a] ^= xt[line & 7][col];
		}
		break;
	}
	default:
		break;
	}
}

// ---- the palette (galaxian_palette: compute_resistor_weights(0, 224, -1, ...) with 470 ohm pulldowns)
static void res_weights(const int *res, int n, int pd, double *w, double *sum)
{
	*sum = 0;
	for (int k = 0; k < n; k++) {
		double r0 = pd ? 1.0 / pd : 1.0 / 1e12, r1 = 1.0 / 1e12;
		for (int j = 0; j < n; j++) {
			if (j == k)
				r1 += 1.0 / res[j];
			else
				r0 += 1.0 / res[j];
		}
		r0 = 1.0 / r0;
		r1 = 1.0 / r1;
		double v = 224.0 * r0 / (r1 + r0);
		if (v > 224.0) v = 224.0;
		w[k] = v;
		*sum += v;
	}
}

static int combine_w(const double *w, int n, int v)
{
	double s = 0;
	for (int k = 0; k < n; k++)
		s += w[k] * ((v >> k) & 1);
	return (int)(s + 0.5);
}

static void build_palette(void)
{
	static const int res[3] = { 1000, 470, 220 };
	double w3[3], w2[2], s3, s2;
	res_weights(res, 3, 470, w3, &s3);
	res_weights(res + 1, 2, 470, w2, &s2);
	const double scale = 224.0 / (s3 > s2 ? s3 : s2);
	for (int k = 0; k < 3; k++)
		w3[k] *= scale;
	for (int k = 0; k < 2; k++)
		w2[k] *= scale;
	memset(s_pal, 0, sizeof(s_pal));
	const uint32_t len = s_proms_size < 256 ? s_proms_size : 256;
	for (uint32_t i = 0; i < len; i++) {
		const uint8_t v = s_proms[i];
		uint32_t r = (uint32_t)combine_w(w3, 3, v), g = (uint32_t)combine_w(w3, 3, v >> 3),
		         bb = (uint32_t)combine_w(w2, 2, v >> 6);
		if (s_m->flags & MF_EAGLE) {   // (RGB -> GBR)
			const uint32_t t = r;
			r = g;
			g = bb;
			bb = t;
		}
		s_pal[i] = r << 16 | g << 8 | bb;
	}
	int blank = 1;   // (the PROM not dumped (kong, timefgtr): MAME draws them black; here a plain palette instead)
	for (uint32_t i = 0; i < len; i++)
		if (s_proms[i])
			blank = 0;
	if (blank) {
		static const uint32_t hue[8] = { 0xff4040, 0x40ff40, 0x4080ff, 0xffff40, 0x40ffff, 0xff40ff, 0xffffff, 0xff9020 };
		for (int c = 0; c < 64; c++) {
			s_pal[c * 4 + 0] = 0;
			s_pal[c * 4 + 1] = hue[c & 7];
			s_pal[c * 4 + 2] = hue[(c + 3) & 7];
			s_pal[c * 4 + 3] = 0xffffff;
		}
	}
	// the stars: 150 / 100 ohm per gun, compressed into 194..255
	const int minval = 224 * 130 / 150, midval = 224 * 130 / 100, maxval = 224 * 130 / 60;
	const uint8_t starmap[4] = { 0, (uint8_t)minval,
	                             (uint8_t)(minval + (255 - minval) * (midval - minval) / (maxval - minval)), 255 };
	for (int i = 0; i < 64; i++) {
		const uint32_t r = starmap[(BIT(i, 4) << 1) | BIT(i, 5)], g = starmap[(BIT(i, 2) << 1) | BIT(i, 3)],
		               bb = starmap[(BIT(i, 0) << 1) | BIT(i, 1)];
		s_pal[PEN_STAR + i] = r << 16 | g << 8 | bb;
	}
	s_pal[PEN_BLACK] = 0;
	for (int i = 0; i < 8; i++) {
		uint32_t c = i < 7 ? 0xffffff : 0xffff00;
		if (i == 7 && (s_m->flags & MF_MOONWAR))
			c = 0xefef97;
		s_pal[PEN_BUL + i] = c;
		s_pal[PEN_BUL_TE + i] = (c & 0xff0000) | (c & 0xff) << 8 | ((c >> 8) & 0xff);
	}
	s_pal[PEN_YELLOW] = 0xffff00;
	static const uint32_t msh[8] = { 0xffffff, 0xffff00, 0x00ffff, 0x00ff00, 0xff00ff, 0xff0000, 0x0000ff, 0x000000 };
	for (int i = 0; i < 8; i++)
		s_pal[PEN_MSH + i] = msh[i];
	s_pal[PEN_PURPLE] = 0xff00ff;
	s_pal[PEN_BLUE] = 0x000056;
	s_pal[PEN_FROGBLUE] = 0x000047;
	for (int i = 0; i < 8; i++)   // (red 390, green 470, blue 390 ohm)
		s_pal[PEN_TURTLES + i] = (uint32_t)(((i >> 2) & 1) * 0x55) << 16 | (uint32_t)(((i >> 1) & 1) * 0x47) << 8 |
		                         (uint32_t)((i & 1) * 0x55);
}

static void stars_init(void)
{
	uint32_t shiftreg = 0;
	s_nstars = 0;
	for (int i = 0; i < STAR_P; i++) {
		// stars where the upper 8 bits are 1 and the low bit 0; the colour from the 6 bits below the top 8
		const int enabled = (shiftreg & 0x1fe01) == 0x1fe00;
		const int color = (int)((~shiftreg & 0x1f8) >> 3);
		if (enabled && s_nstars < 1024)
			s_stars[s_nstars++] = (uint32_t)i << 8 | (uint32_t)color;
		shiftreg = (shiftreg >> 1) | ((((shiftreg >> 12) ^ ~shiftreg) & 1) << 16);
	}
}

// ---- the graphics (galaxian_charlayout / galaxian_spritelayout: RGN_FRAC(1,2) planes)
static inline int gbit(const uint8_t *rom, uint32_t o) { return (rom[o >> 3] >> (7 - (o & 7))) & 1; }

static void decode_gfx(const uint8_t *g1, uint32_t s1, const uint8_t *g2, uint32_t s2)
{
	const uint32_t h1 = s1 / 2 * 8, h2 = s2 / 2 * 8;
	for (uint32_t t = 0; t < s_nchr; t++)
		for (int y = 0; y < 8; y++)
			for (int x = 0; x < 8; x++) {
				const uint32_t o = t * 64 + (uint32_t)(y * 8 + x);
				s_chr[t * 64 + (uint32_t)(y * 8 + x)] = (uint8_t)(gbit(g1, o) << 1 | gbit(g1, h1 + o));
			}
	for (uint32_t t = 0; t < s_nspr; t++)
		for (int y = 0; y < 16; y++)
			for (int x = 0; x < 16; x++) {
				const uint32_t o = t * 256 + (uint32_t)((x & 7) + (x >> 3) * 64 + (y & 7) * 8 + (y >> 3) * 128);
				s_spr[t * 256 + (uint32_t)(y * 16 + x)] = (uint8_t)(gbit(g2, o) << 1 | gbit(g2, h2 + o));
			}
}

// ---- init / reset / exit
static void gx_exit(void);

static int find_config(const char *machine, int *init)
{
	char name[32];
	const char *colon = strchr(machine, ':');
	const size_t n = colon ? (size_t)(colon - machine) : strlen(machine);
	if (n >= sizeof(name))
		return -1;
	memcpy(name, machine, n);
	name[n] = 0;
	const char *in = colon ? colon + 1 : "galaxian";
	int ii = -1;
	for (int i = 0; i < (int)(sizeof(s_inits) / sizeof(s_inits[0])); i++)
		if (!strcmp(in, s_inits[i].name))
			ii = i;
	if (init)
		*init = ii;
	if (ii < 0)
		return -1;
	for (int i = 0; i < (int)(sizeof(s_machines) / sizeof(s_machines[0])); i++)
		if (!strcmp(name, s_machines[i].name))
			return i;
	return -1;
}

static void cpu_setup(void)
{
	z80_init(&s_main);
	s_main.read_byte = main_read;
	s_main.write_byte = main_write;
	s_main.port_in = s_m->io == IO_MSHUTTLE ? main_in_msh : main_in;
	s_main.port_out = main_out;
	s_main.read_map = s_rmap;
	s_main.op_rom = s_opcodes;
	s_main.int_data = 0xff;
	z80_init(&s_snd);
	s_snd.read_byte = snd_read;
	s_snd.write_byte = snd_write;
	s_snd.port_in = snd_in;
	s_snd.port_out = snd_out;
	s_snd.int_data = 0xff;
	// the sound CPU's read map: its ROM and RAM pages (the rest through snd_read)
	memset(s_zmap, 0, sizeof(s_zmap));
	for (int p = 0; p < 256; p++) {
		const uint32_t a = (uint32_t)p << 8;
		uint32_t rom_a = 0xffffffff;
		int ram = 0;
		switch (s_m->snd) {
		case SND_KONAMI2: case SND_AMIGO2: case SND_EXPLORER: case SND_TURPINS: case SND_TAKEOFF:
			if (a < (s_m->snd == SND_TAKEOFF ? 0x1000u : 0x2000u)) rom_a = a;
			else if (a >= 0x8000 && !(a & 0x1000)) ram = 1;
			break;
		case SND_KONAMI1:
			if ((a & 0x7fff) < 0x2000) rom_a = a & 0x7fff;
			else if ((a & 0x7fff) >= 0x4000 && (a & 0x7fff) < 0x6000) ram = 1;
			break;
		case SND_CHECKMAN: case SND_JUNGSUB:
			if (a < 0x1000) rom_a = a;
			else if (a >= 0x2000 && a < 0x2400) ram = 1;
			break;
		case SND_CHECKMAJ:
			if (a < 0x1000) rom_a = a;
			else if (a >= 0x8000 && a < 0x8400) ram = 1;
			break;
		case SND_KINGBALL:
			if ((a & 0x3fff) < 0x2000) rom_a = a & 0x3fff;
			break;
		default:
			break;
		}
		if (rom_a != 0xffffffff && s_srom && rom_a + 256 <= s_srom_size)
			s_zmap[p] = s_srom + rom_a;
		else if (ram)
			s_zmap[p] = b.zram + (p & 3) * 256;
	}
	s_snd.read_map = s_zmap;
	z80_init(&s_smp);
	s_smp.read_byte = smp_read;
	s_smp.write_byte = smp_write;
	s_smp.port_in = smp_in;
	s_smp.port_out = smp_out;
	s_smp.int_data = 0xff;
	memset(s_smap, 0, sizeof(s_smap));
	for (int p = 0; p < 256; p++) {
		const uint32_t a = (uint32_t)p << 8;
		if (a < ((s_m->flags & MF_MONSTERZ) ? 0x4000u : 0x6000u) && s_arom && a + 256 <= s_arom_size)
			s_smap[p] = s_arom + a;
		else if (a >= 0x8000 && !(a & 0x1000))
			s_smap[p] = b.zram2 + (p & 3) * 256;
	}
	s_smp.read_map = s_smap;
}

static void chips_callbacks(void)
{
	for (int i = 0; i < 3; i++) {
		s_ppi[i].user = (void *)(intptr_t)i;
		s_ppi[i].in_pa = ppi_in_a;
		s_ppi[i].in_pb = ppi_in_b;
		s_ppi[i].in_pc = ppi_in_c;
		s_ppi[i].out_pa = ppi_out_a;
		s_ppi[i].out_pb = ppi_out_b;
		s_ppi[i].out_pc = ppi_out_c;
		s_ppi[i].tri_pa = s_ppi[i].tri_pb = s_ppi[i].tri_pc = 0xff;
	}
	if (s_m->flags & MF_SFX) {
		s_ay[1].port_a_write = sfx_ay1_a;
		s_ay[1].port_b_write = sfx_ay1_b;
		if (s_m->flags & MF_MONSTERZ)
			s_ay[0].port_b_write = mz_ay0_b;
	}
	if (s_m->snd == SND_CCLIMBER) {
		s_ay[0].port_a_write = cc_port_a;
		s_ay[0].port_b_write = cc_port_b;
	} else if (s_m->snd != SND_ZIGZAG && s_m->snd != SND_FANTASTC && s_m->snd != SND_JUMPBUG && s_m->snd != SND_OZON1 &&
	           s_m->snd != SND_CHECKMAN && s_m->snd != SND_JUNGSUB) {
		s_ay[0].port_a_read = ay_port_a;
		if (s_m->snd != SND_CHECKMAJ && s_m->snd != SND_BONGO && s_m->snd != SND_TAKEOFF && s_m->snd != SND_EXPLORER)
			s_ay[0].port_b_read = ay_port_b;
	}
}

static void gx_reset(void)
{
	memset(&b, 0, sizeof(b));
	b.lfsr = 0x1234567;
	b.cc_vol = 0;
	if (s_m->map == map_zigzag) {   // (bankswap_w(0))
		b.bank[0] = 0;
		b.bank[1] = 1;
	}
	uint32_t nvs = 0;
	const uint8_t *nv = rbx_region_opt("nvram", &nvs);
	if (nv)
		memcpy(b.pool + PO_RA, nv, nvs < 0x400 ? nvs : 0x400);
	for (int i = 0; i < 2; i++)
		for (int c = 0; c < 3; c++)
			b.flt[i][c].k = 0x10000;
	build_pages();
	cpu_setup();
	for (int i = 0; i < 3; i++)
		i8255_reset(&s_ppi[i]);
	b.mz_portb = b.mz_portc = 0xff;
	for (int i = 0; i < 2; i++)
		ay8910_reset(&s_ay[i]);
	chips_callbacks();
	galsnd_reset(&s_galsnd);
	if (s_custom == CU_GMGALAX)
		gm_game_changed(raw_port(P_GAMESEL) & 1);
	if (s_tile_key)
		memset(s_tile_key, 0xff, 1024 * sizeof(s_tile_key[0]));
}

static int gx_init(const rbx_game *g)
{
	int ii;
	const int mi = find_config(g->machine, &ii);
	if (mi < 0 || ii < 0) {
		rbx_fail("%s: machine %s not emulated", g->set, g->machine);
		return -1;
	}
	s_m = &s_machines[mi];
	s_i = &s_inits[ii];
	uint32_t rsz = 0, g1s = 0, g2s = 0;
	uint8_t *rom = rbx_region("maincpu", &rsz);
	uint8_t *gfx1 = rbx_region("gfx1", &g1s);
	s_proms = rbx_region("proms", &s_proms_size);
	uint8_t *gfx2 = (s_m->flags & MF_GFX2) ? rbx_region("gfx2", &g2s) : NULL;
	if (!rom || !gfx1 || !s_proms || ((s_m->flags & MF_GFX2) && !gfx2)) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	s_srom = rbx_region_opt("audiocpu", &s_srom_size);
	if (has_snd_cpu() && !s_srom) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	s_arom = rbx_region_opt("audio2", &s_arom_size);
	if ((s_m->flags & MF_SFX) && !s_arom) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	uint32_t smp = 0;
	s_samples = rbx_region_opt("cclimber_audio:samples", &smp);
	if (s_samples && smp < 0x2000)
		s_samples = NULL;
	// (a full 64 KB view at least: the maps' ROM ranges past a smaller region read 0)
	s_rom_size = rsz;
	const uint32_t view = rsz > 0x10000 ? rsz : 0x10000;
	s_rom = (uint8_t *)calloc(1, view + 0x100);
	const int need_op = s_i->dec == DEC_HIGHROLL || s_i->dec == DEC_LADYBUGG2 || s_i->dec == DEC_MOONQSR ||
	                    s_i->dec == DEC_MSHUTTLE || s_i->dec == DEC_MSHUTTLJ;
	if (need_op)
		s_opcodes = (uint8_t *)calloc(1, 0x10000);
	s_snap = (snap_t *)calloc(1, sizeof(snap_t));
	s_tiles = (uint8_t *)calloc(1, 256 * 256);
	s_tile_key = (uint32_t *)malloc(1024 * sizeof(uint32_t));
	s_stars = (uint32_t *)malloc(1024 * sizeof(uint32_t));
	if (!s_rom || (need_op && !s_opcodes) || !s_snap || !s_tiles || !s_tile_key || !s_stars) {
		gx_exit();
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	memcpy(s_rom, rom, rsz);
	if (need_op)
		memcpy(s_opcodes, s_rom, 0x10000);
	decrypt(s_i->dec, gfx1, g1s);
	// graphics
	if (!gfx2) {
		gfx2 = gfx1;
		g2s = g1s;
	}
	s_nchr = g1s / 16;
	s_nspr = g2s / 64;
	if (!s_nchr || !s_nspr) {
		gx_exit();
		rbx_fail("%s: unexpected graphics size", g->set);
		return -1;
	}
	s_chr = (uint8_t *)malloc((size_t)s_nchr * 64);
	s_spr = (uint8_t *)malloc((size_t)s_nspr * 256);
	if (!s_chr || !s_spr) {
		gx_exit();
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	decode_gfx(gfx1, g1s, gfx2, g2s);
	// the map: the machine's, then the init's changes
	s_nmap = 0;
	s_ents[0] = s_m->map;
	s_ents[1] = s_i->patch;
	for (int k = 0; k < 2; k++)
		for (const me_t *e = s_ents[k]; e && e->rk != 0xff; e++)
			if (s_nmap < (int)(sizeof(s_map) / sizeof(s_map[0])))
				s_map[s_nmap++] = *e;
	// (unmap_value_high: the maps declaring it; the bootleg maps without read 0)
	s_unmap = 0xff;
	if (s_m->map == map_ozon1 || s_m->map == map_mimonkey || s_m->map == map_mimonscr || s_m->map == map_ckongs ||
	    s_m->map == map_amigo2 || s_m->map == map_turpins || s_m->map == map_fourplay || s_m->map == map_videight ||
	    s_m->map == map_guttangt || s_m->map == map_guttangts3 || s_m->map == map_ckongg || s_m->map == map_bigkonggx ||
	    s_m->map == map_ckongmc || s_m->map == map_bagmanmc)
		s_unmap = 0;
	// banks
	s_bank_base[0] = s_bank_base[1] = s_rom;
	s_bank_stride[0] = s_bank_stride[1] = 0x4000;
	if (s_m->map == map_zigzag) {
		s_bank_base[0] = s_bank_base[1] = s_rom + 0x2000;
		s_bank_stride[0] = s_bank_stride[1] = 0x1000;
	} else if (s_m->map == map_guttangt) {
		s_bank_base[0] = s_rom + 0x2000;
		s_bank_stride[0] = 0x2000;
	}
	// video configuration
	s_bul = s_i->bul;
	s_bg = s_i->bg;
	s_tx = s_i->tx;
	s_sx = s_i->sx;
	s_frogadj = (s_i->flags & IF_FROGADJ) != 0;
	s_bul_base = (s_m->flags & MF_BUL_C0) ? 0xc0 : 0x60;
	s_nspr_gens = (s_m->flags & MF_2SPR) ? 2 : 1;
	s_has_bullets = s_bul != BUL_NONE;
	s_sfxadj = (s_i->flags & IF_SFXADJ) != 0;
	s_vis_x0 = (s_m->flags & MF_SFX) ? 3 * 8 : 0;   // (sfx: the screen's edges cut, 24 to 239)
	s_vis_w = (s_m->flags & MF_SFX) ? VIS_W - 5 * 8 : VIS_W;
	s_fb = NULL;
	if (s_vis_x0 && !(s_fb = (uint16_t *)calloc(1, VIS_W * VIS_H * sizeof(uint16_t)))) {
		gx_exit();
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	s_ncolors_mask = s_m->map == map_videight ? 255 : (s_m->map == map_gmgalax || s_m->map == map_fourplay) ? 15 : 7;
	// the ports and their custom bits
	for (int p = 0; p < P_COUNT; p++)
		s_port[p] = rbx_port_find(s_port_tags[p]);
	s_custom = CU_NONE;
	if (!strcmp(s_i->name, "azurian")) s_custom = CU_AZURIAN;
	else if (s_m->snd == SND_KINGBALL) s_custom = CU_KINGBALL;
	else if (s_m->flags & MF_MOONWAR) s_custom = CU_MOONWAR;
	else if (s_m->map == map_gmgalax) s_custom = CU_GMGALAX;
	else if (s_port[P_COINAGE] >= 0)
		s_custom = s_m->map == map_ckongs ? CU_CKONGS : s_m->map == map_ckongmc ? CU_CKONGMC : CU_CKONGG;
	else if (s_m->flags & MF_PROT) {
		// (the theend / scramble ports: IN2 bits 5 and 7 from the protection)
		for (const rbx_ipt_port *pp = g->ports; pp && pp->tag; pp++)
			if (!strcmp(pp->tag, "IN2"))
				for (int f = 0; f < pp->count; f++)
					if (g->fields[pp->first + f].type == RBX_IPT_CUSTOM && (g->fields[pp->first + f].mask & 0x20))
						s_custom = CU_THEEND;
	}
	// clocks
	const int sidam = (s_m->flags & MF_SIDAM) != 0;
	s_frame_mhz = sidam ? 59185 : 60606;
	s_rate = sidam ? 46875 : 48000;
	s_cpu_clock = sidam ? 3000000 : 3072000;
	uint32_t ay_clk = 1789772;
	int ay_gain = 128;
	s_snd_clock = 0;
	switch (s_m->snd) {
	case SND_KONAMI2: case SND_KONAMI1: case SND_TURPINS: s_snd_clock = 1789772; ay_clk = 1789772; ay_gain = 220; break;
	case SND_AMIGO2: case SND_EXPLORER: case SND_TAKEOFF: s_snd_clock = 1500000; ay_clk = 1500000; ay_gain = 64; break;
	case SND_CHECKMAN: s_snd_clock = 1620000; ay_clk = 1789750; break;
	case SND_CHECKMAJ: s_snd_clock = 1620000; ay_clk = 1620000; ay_gain = 512; break;
	case SND_JUNGSUB: s_snd_clock = 3072000; ay_clk = 1536000; break;
	case SND_KINGBALL: s_snd_clock = 2500000; break;
	case SND_BONGO: case SND_JUMPBUG: case SND_OZON1: case SND_CCLIMBER: ay_clk = 1536000; break;
	case SND_ZIGZAG: ay_clk = 3072000; break;
	case SND_FANTASTC: ay_clk = 3072000; ay_gain = 64; break;
	default: break;
	}
	s_ay_gain = ay_gain;
	s_snd_step = (uint32_t)((uint64_t)(uint32_t)s_snd_clock * 65536 / s_rate);
	for (int i = 0; i < 2; i++) {
		ay8910_init(&s_ay[i], AY8910_TYPE_AY8910, ay_clk, AY8910_LEGACY_OUTPUT, ay_gain, 1, 0);
		s_ay_step[i] = (uint32_t)((uint64_t)ay_clk * 65536 / s_rate);
	}
	s_cc_ticks = 768000 / 48000;
	s_recip[0] = 0;
	for (int n = 1; n < 64; n++)
		s_recip[n] = 65536u / (uint32_t)n;
	s_blink_period = (uint32_t)(0.693 * (100000 + 2 * 10000) * 0.00001 * (s_cpu_clock / 192.0) * 16.0 + 0.5);
	const int disc_type = s_m->disc == DISC_MCR ? GALSND_MOONCRST : s_m->disc == DISC_BONGO ? GALSND_BONGO : GALSND_GALAXIAN;
	galsnd_init(&s_galsnd, disc_type, (double)s_rate, 14000.0f);
	build_palette();
	stars_init();
	if (rbx_audio_setup(s_rate) || rbx_video_setup(s_vis_w, VIS_H, NPENS)) {
		gx_exit();
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	rbx_prof_name(RBX_PA_DRV, s_pa_names, GX_PA_END - RBX_PA_DRV);
	gx_reset();
	return 0;
}

static void gx_exit(void)
{
	rbx_video_wait_idle();
	free(s_rom);
	free(s_opcodes);
	free(s_chr);
	free(s_spr);
	free(s_snap);
	free(s_tiles);
	free(s_tile_key);
	free(s_stars);
	free(s_fb);
	s_fb = NULL;
	s_rom = s_opcodes = s_chr = s_spr = s_tiles = NULL;
	s_snap = NULL;
	s_tile_key = s_stars = NULL;
}

// ---- save states
static void z_fix(z80 *z, const z80 *cur)
{
	z->read_byte = cur->read_byte;
	z->write_byte = cur->write_byte;
	z->port_in = cur->port_in;
	z->port_out = cur->port_out;
	z->userdata = NULL;
	z->read_map = cur->read_map;
	z->op_rom = cur->op_rom;
}

static int gx_state(void *ctx, int load)
{
	int rc = rbx_state_io(ctx, "gx", &b, sizeof(b));
	z80 m = s_main, z = s_snd, z2 = s_smp;
	rc |= rbx_state_io(ctx, "gx_main", &m, sizeof(m));
	rc |= rbx_state_io(ctx, "gx_snd", &z, sizeof(z));
	if (s_m->flags & MF_SFX) {
		rc |= rbx_state_io(ctx, "gx_smp", &z2, sizeof(z2));
		rc |= rbx_state_io(ctx, "gx_ppi2", (uint8_t *)&s_ppi[2] + I8255_STATE_OFFSET, (uint32_t)I8255_STATE_SIZE);
	}
	for (int i = 0; i < 2; i++) {
		rc |= rbx_state_io(ctx, i ? "gx_ppi1" : "gx_ppi0", (uint8_t *)&s_ppi[i] + I8255_STATE_OFFSET,
		                   (uint32_t)I8255_STATE_SIZE);
		rc |= rbx_state_io(ctx, i ? "gx_ay1" : "gx_ay0", &s_ay[i], ay8910_state_size());
	}
	rc |= rbx_state_io(ctx, "gx_disc", &s_galsnd.s, GALSND_STATE_SIZE);
	if (!rc && load) {
		z_fix(&m, &s_main);
		z_fix(&z, &s_snd);
		z_fix(&z2, &s_smp);
		s_main = m;
		s_snd = z;
		s_smp = z2;
		set_banks();
		if (s_tile_key)
			memset(s_tile_key, 0xff, 1024 * sizeof(s_tile_key[0]));
	}
	return rc ? -1 : 0;
}

static int gx_save(void *ctx) { return gx_state(ctx, 0); }
static int gx_load(void *ctx) { return gx_state(ctx, 1); }
static int gx_supported(const char *machine) { return find_config(machine, NULL) >= 0; }

const rbx_driver rbx_drv_galaxian = { "Namco Galaxian", gx_init, gx_exit, gx_reset, gx_frame, gx_save, gx_load,
                                      gx_supported, gx_render, NULL };
