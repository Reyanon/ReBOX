/*  Taito F2 System, ported from MAME src/mame/taito/taito_f2.cpp and taito_f2_v.cpp (BSD-3-Clause,
    copyright-holders: David Graves, Bryan McPhail, Brad Oliver, Andrew Prime, Brian Troha, Nicola Salmoria;
    thanks to Richard Bush). Memory maps, machine configurations, interrupts (IRQ5 at vblank, IRQ6 500 cycles
    later), the TC0200OBJ / TC0540OBN sprite generator with its buffering variants and sprite banks, and the
    screen mixers (TC0360PRI) follow MAME. I/O chips: TC0220IOC / TC0510NIO (src/mame/taito/taitoio.cpp,
    BSD-3-Clause) and TE7750 (src/devices/machine/te7750.cpp, BSD-3-Clause, AJR); palette TC0110PCR
    (src/mame/taito/tc0110pcr.cpp, BSD-3-Clause, Nicola Salmoria).
    Tilemaps: video/tc0100scn.c, tc0280grd.c (TC0430GRW), tc0480scp.c. Sound board: machine/taito_sound.c.
    Drive Out (bootleg) has its own Z80 + OKI M6295 sound board here. Mega Blast: C-Chip (machine/taitocchip.c).
    Not emulated: Football Champ bootleg (different hardware registers, not working in MAME either).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m68k_bus.h"
#include "../../cpu/m68k/m68k.h"
#include "../machine/taito_sound.h"
#include "../machine/taitocchip.h"
#include "../video/tc0100scn.h"
#include "../video/tc0280grd.h"
#include "../video/tc0480scp.h"
#include "../video/tc0360pri.h"
#include "../../cpu/z80/z80.h"
#include "../../sound/okim6295.h"

#include <stdlib.h>
#include <string.h>

// ---- main CPU memory maps ----
enum { D_END, D_ROM, D_EXTRA, D_RAM, D_PAL, D_SPR, D_SCN, D_SCN2, D_GRD, D_SCP, D_EXT,      // direct
       D_SCNC, D_SCN2C, D_GRDC, D_SCPC, D_PRI, D_IOC, D_NIO, D_NIOSW, D_TE, D_SYT, D_PCR, D_PORT,
       D_SPRBANK, D_KOSBANK, D_PADDLE, D_DIAL, D_MJDSW, D_MJIN, D_MJSEL, D_MJBANK, D_RTC, D_DOSND, D_CCHIP };
enum { LANE_WORD, LANE_HI, LANE_LO };
typedef struct { uint32_t start, end; uint8_t dev, lane; const char *tag; } range_t;

#define R(s, e, d) { s, e, d, LANE_WORD, 0 }
#define RL(s, e, d, l) { s, e, d, l, 0 }
#define RP(s, e, t) { s, e, D_PORT, LANE_WORD, t }
#define END { 0, 0, D_END, 0, 0 }

static const range_t map_finalb[] = {
	R(0x000000, 0x03ffff, D_ROM), R(0x100000, 0x10ffff, D_RAM), R(0x200000, 0x200007, D_PCR),
	RL(0x300000, 0x30000f, D_IOC, LANE_LO), RL(0x320000, 0x320003, D_SYT, LANE_LO), R(0x800000, 0x80ffff, D_SCN),
	R(0x820000, 0x82000f, D_SCNC), R(0x900000, 0x90ffff, D_SPR), END };
static const range_t map_dondokod[] = {
	R(0x000000, 0x07ffff, D_ROM), R(0x100000, 0x10ffff, D_RAM), R(0x200000, 0x201fff, D_PAL),
	RL(0x300000, 0x30000f, D_IOC, LANE_LO), RL(0x320000, 0x320003, D_SYT, LANE_HI), R(0x800000, 0x80ffff, D_SCN),
	R(0x820000, 0x82000f, D_SCNC), R(0x900000, 0x90ffff, D_SPR), R(0xa00000, 0xa01fff, D_GRD),
	R(0xa02000, 0xa0200f, D_GRDC), RL(0xb00000, 0xb0001f, D_PRI, LANE_LO), END };
static const range_t map_thundfox[] = {
	R(0x000000, 0x07ffff, D_ROM), R(0x100000, 0x101fff, D_PAL), RL(0x200000, 0x20000f, D_IOC, LANE_LO),
	RL(0x220000, 0x220003, D_SYT, LANE_HI), R(0x300000, 0x30ffff, D_RAM), R(0x400000, 0x40ffff, D_SCN),
	R(0x420000, 0x42000f, D_SCNC), R(0x500000, 0x50ffff, D_SCN2), R(0x520000, 0x52000f, D_SCN2C),
	R(0x600000, 0x60ffff, D_SPR), RL(0x800000, 0x80001f, D_PRI, LANE_HI), END };
static const range_t map_cameltry[] = {
	R(0x000000, 0x03ffff, D_ROM), R(0x100000, 0x10ffff, D_RAM), R(0x200000, 0x201fff, D_PAL),
	RL(0x300000, 0x30000f, D_IOC, LANE_LO), R(0x300018, 0x30001f, D_PADDLE), RL(0x320000, 0x320003, D_SYT, LANE_HI),
	R(0x800000, 0x813fff, D_SCN), R(0x820000, 0x82000f, D_SCNC), R(0x900000, 0x90ffff, D_SPR),
	R(0xa00000, 0xa01fff, D_GRD), R(0xa02000, 0xa0200f, D_GRDC), RL(0xd00000, 0xd0001f, D_PRI, LANE_LO), END };
static const range_t map_driftoutct[] = {
	R(0x000000, 0x07ffff, D_ROM), R(0x100000, 0x10ffff, D_RAM), R(0x200000, 0x201fff, D_PAL),
	RL(0x300000, 0x30000f, D_IOC, LANE_LO), { 0x300018, 0x300019, D_DIAL, LANE_WORD, "PADDLE1" },
	{ 0x30001a, 0x30001b, D_DIAL, LANE_WORD, "PADDLE2" }, RL(0x320000, 0x320003, D_SYT, LANE_HI),
	R(0x800000, 0x813fff, D_SCN), R(0x820000, 0x82000f, D_SCNC), R(0x900000, 0x90ffff, D_SPR),
	R(0xa00000, 0xa01fff, D_GRD), R(0xa02000, 0xa0200f, D_GRDC), RL(0xd00000, 0xd0001f, D_PRI, LANE_LO), END };
static const range_t map_qtorimon[] = {
	R(0x000000, 0x07ffff, D_ROM), R(0x100000, 0x10ffff, D_RAM), R(0x200000, 0x200007, D_PCR),
	RL(0x500000, 0x50000f, D_IOC, LANE_LO), RL(0x600000, 0x600003, D_SYT, LANE_HI), R(0x800000, 0x80ffff, D_SCN),
	R(0x820000, 0x82000f, D_SCNC), R(0x900000, 0x90ffff, D_SPR), END };
static const range_t map_megab[] = {
	R(0x000000, 0x07ffff, D_ROM), RL(0x100000, 0x100003, D_SYT, LANE_HI), RL(0x120000, 0x12000f, D_IOC, LANE_LO),
	RL(0x180000, 0x180fff, D_CCHIP, LANE_LO), R(0x200000, 0x20ffff, D_RAM), R(0x300000, 0x301fff, D_PAL),
	RL(0x400000, 0x40001f, D_PRI, LANE_LO), R(0x600000, 0x60ffff, D_SCN), R(0x610000, 0x61ffff, D_RAM),
	R(0x620000, 0x62000f, D_SCNC), R(0x800000, 0x80ffff, D_SPR), END };
static const range_t map_liquidk[] = {
	R(0x000000, 0x07ffff, D_ROM), R(0x100000, 0x10ffff, D_RAM), R(0x200000, 0x201fff, D_PAL),
	RL(0x300000, 0x30000f, D_IOC, LANE_LO), RL(0x320000, 0x320003, D_SYT, LANE_LO), R(0x800000, 0x80ffff, D_SCN),
	R(0x820000, 0x82000f, D_SCNC), R(0x900000, 0x90ffff, D_SPR), RL(0xb00000, 0xb0001f, D_PRI, LANE_LO), END };
static const range_t map_quizhq[] = {
	R(0x000000, 0x0bffff, D_ROM), R(0x100000, 0x10ffff, D_RAM), R(0x200000, 0x200007, D_PCR),
	RP(0x500000, 0x500001, "DSWB"), RP(0x500002, 0x500003, "IN0"), RP(0x580000, 0x580001, "DSWA"),
	RP(0x580002, 0x580003, "IN1"), RP(0x580004, 0x580005, "IN2"), RL(0x600000, 0x600003, D_SYT, LANE_LO),
	R(0x800000, 0x80ffff, D_SCN), R(0x820000, 0x82000f, D_SCNC), R(0x900000, 0x90ffff, D_SPR), END };
static const range_t map_ssi[] = {
	R(0x000000, 0x07ffff, D_ROM), R(0x100000, 0x10000f, D_NIO), R(0x200000, 0x20ffff, D_RAM),
	R(0x300000, 0x301fff, D_PAL), RL(0x400000, 0x400003, D_SYT, LANE_HI), R(0x600000, 0x60ffff, D_SCN),
	R(0x620000, 0x62000f, D_SCNC), R(0x800000, 0x80ffff, D_SPR), END };
static const range_t map_gunfront[] = {
	R(0x000000, 0x0bffff, D_ROM), R(0x100000, 0x10ffff, D_RAM), R(0x200000, 0x201fff, D_PAL),
	R(0x300000, 0x30000f, D_NIOSW), RL(0x320000, 0x320003, D_SYT, LANE_HI), R(0x800000, 0x80ffff, D_SCN),
	R(0x820000, 0x82000f, D_SCNC), R(0x900000, 0x90ffff, D_SPR), RL(0xb00000, 0xb0001f, D_PRI, LANE_LO), END };
static const range_t map_growl[] = {
	R(0x000000, 0x0fffff, D_ROM), R(0x100000, 0x10ffff, D_RAM), R(0x200000, 0x201fff, D_PAL),
	RP(0x300000, 0x300001, "DSWA"), RP(0x300002, 0x300003, "DSWB"), RP(0x320000, 0x320001, "IN0"),
	RP(0x320002, 0x320003, "IN1"), RP(0x320004, 0x320005, "IN2"), RL(0x400000, 0x400003, D_SYT, LANE_HI),
	R(0x500000, 0x50000f, D_SPRBANK), RP(0x508000, 0x50800f, "IN3"), RP(0x50c000, 0x50c00f, "IN4"),
	R(0x800000, 0x80ffff, D_SCN), R(0x820000, 0x82000f, D_SCNC), R(0x900000, 0x90ffff, D_SPR),
	RL(0xb00000, 0xb0001f, D_PRI, LANE_LO), END };
static const range_t map_mjnquest[] = {
	R(0x000000, 0x0fffff, D_ROM), R(0x110000, 0x11ffff, D_RAM), R(0x120000, 0x12ffff, D_RAM),
	R(0x200000, 0x200007, D_PCR), R(0x300000, 0x30000f, D_MJDSW), R(0x310000, 0x310001, D_MJIN),
	R(0x320000, 0x320001, D_MJSEL), RL(0x360000, 0x360003, D_SYT, LANE_HI), RL(0x380000, 0x380001, D_MJBANK, LANE_LO),
	R(0x400000, 0x40ffff, D_SCN), R(0x420000, 0x42000f, D_SCNC), R(0x500000, 0x50ffff, D_SPR), END };
static const range_t map_footchmp[] = {
	R(0x000000, 0x07ffff, D_ROM), R(0x100000, 0x10ffff, D_RAM), R(0x200000, 0x20ffff, D_SPR),
	R(0x300000, 0x30000f, D_SPRBANK), R(0x400000, 0x40ffff, D_SCP), R(0x430000, 0x43002f, D_SCPC),
	RL(0x500000, 0x50001f, D_PRI, LANE_LO), R(0x600000, 0x601fff, D_PAL), RL(0x700000, 0x70001f, D_TE, LANE_LO),
	RL(0xa00000, 0xa00003, D_SYT, LANE_LO), END };
static const range_t map_koshien[] = {
	R(0x000000, 0x0fffff, D_ROM), R(0x100000, 0x10ffff, D_RAM), R(0x200000, 0x201fff, D_PAL),
	R(0x300000, 0x30000f, D_NIO), RL(0x320000, 0x320003, D_SYT, LANE_HI), R(0x800000, 0x80ffff, D_SCN),
	R(0x820000, 0x82000f, D_SCNC), R(0x900000, 0x90ffff, D_SPR), R(0xa20000, 0xa20001, D_KOSBANK),
	RL(0xb00000, 0xb0001f, D_PRI, LANE_HI), END };
static const range_t map_yuyugogo[] = {
	R(0x000000, 0x03ffff, D_ROM), R(0x200000, 0x20000f, D_NIO), RL(0x400000, 0x400003, D_SYT, LANE_HI),
	R(0x800000, 0x80ffff, D_SCN), R(0x820000, 0x82000f, D_SCNC), R(0x900000, 0x90ffff, D_SPR),
	R(0xa00000, 0xa01fff, D_PAL), R(0xb00000, 0xb10fff, D_RAM), R(0xc00000, 0xc01fff, D_EXT),
	R(0xd00000, 0xdfffff, D_EXTRA), END };
static const range_t map_ninjak[] = {
	R(0x000000, 0x07ffff, D_ROM), R(0x100000, 0x10ffff, D_RAM), R(0x200000, 0x201fff, D_PAL),
	RL(0x300000, 0x30001f, D_TE, LANE_HI), RL(0x400000, 0x400003, D_SYT, LANE_HI), R(0x600000, 0x60000f, D_SPRBANK),
	R(0x800000, 0x80ffff, D_SCN), R(0x820000, 0x82000f, D_SCNC), R(0x900000, 0x90ffff, D_SPR),
	RL(0xb00000, 0xb0001f, D_PRI, LANE_LO), END };
static const range_t map_solfigtr[] = {
	R(0x000000, 0x07ffff, D_ROM), R(0x100000, 0x10ffff, D_RAM), R(0x200000, 0x201fff, D_PAL),
	RP(0x300000, 0x300001, "DSWA"), RP(0x300002, 0x300003, "DSWB"), RP(0x320000, 0x320001, "IN0"),
	RP(0x320002, 0x320003, "IN1"), RP(0x320004, 0x320005, "IN2"), RL(0x400000, 0x400003, D_SYT, LANE_HI),
	R(0x500000, 0x50000f, D_SPRBANK), R(0x800000, 0x80ffff, D_SCN), R(0x820000, 0x82000f, D_SCNC),
	R(0x900000, 0x90ffff, D_SPR), RL(0xb00000, 0xb0001f, D_PRI, LANE_LO), END };
static const range_t map_qzquest[] = {   // also qzchikyu
	R(0x000000, 0x17ffff, D_ROM), R(0x200000, 0x20000f, D_NIO), RL(0x300000, 0x300003, D_SYT, LANE_LO),
	R(0x400000, 0x401fff, D_PAL), R(0x500000, 0x50ffff, D_RAM), R(0x600000, 0x60ffff, D_SPR),
	R(0x700000, 0x70ffff, D_SCN), R(0x720000, 0x72000f, D_SCNC), END };
static const range_t map_pulirula[] = {
	R(0x000000, 0x0bffff, D_ROM), RL(0x200000, 0x200003, D_SYT, LANE_HI), R(0x300000, 0x30ffff, D_RAM),
	R(0x400000, 0x401fff, D_GRD), R(0x402000, 0x40200f, D_GRDC), R(0x600000, 0x603fff, D_EXT),
	R(0x700000, 0x701fff, D_PAL), R(0x800000, 0x80ffff, D_SCN), R(0x820000, 0x82000f, D_SCNC),
	R(0x900000, 0x90ffff, D_SPR), RL(0xa00000, 0xa0001f, D_PRI, LANE_HI), R(0xb00000, 0xb0000f, D_NIO), END };
static const range_t map_metalb[] = {
	R(0x000000, 0x0bffff, D_ROM), R(0x100000, 0x10ffff, D_RAM), R(0x300000, 0x30ffff, D_SPR),
	R(0x500000, 0x50ffff, D_SCP), R(0x530000, 0x53002f, D_SCPC), RL(0x600000, 0x60001f, D_PRI, LANE_LO),
	R(0x700000, 0x703fff, D_PAL), R(0x800000, 0x80000f, D_NIOSW), RL(0x900000, 0x900003, D_SYT, LANE_HI), END };
static const range_t map_yesnoj[] = {
	R(0x000000, 0x07ffff, D_ROM), R(0x200000, 0x20ffff, D_RAM), R(0x400000, 0x40ffff, D_SPR),
	R(0x500000, 0x50ffff, D_SCN), R(0x520000, 0x52000f, D_SCNC), R(0x600000, 0x601fff, D_PAL),
	RL(0x700000, 0x70001f, D_RTC, LANE_LO), RL(0x800000, 0x800003, D_SYT, LANE_HI), RP(0xa00000, 0xa00001, "IN0"),
	RP(0xa00002, 0xa00003, "IN1"), RP(0xa00004, 0xa00005, "IN2"), RP(0xb00000, 0xb00001, "DSWA"), END };
static const range_t map_deadconx[] = {
	R(0x000000, 0x0fffff, D_ROM), R(0x100000, 0x10ffff, D_RAM), R(0x200000, 0x20ffff, D_SPR),
	R(0x300000, 0x30000f, D_SPRBANK), R(0x400000, 0x40ffff, D_SCP), R(0x430000, 0x43002f, D_SCPC),
	RL(0x500000, 0x50001f, D_PRI, LANE_LO), R(0x600000, 0x601fff, D_PAL), RL(0x700000, 0x70001f, D_TE, LANE_LO),
	RL(0xa00000, 0xa00003, D_SYT, LANE_HI), END };
static const range_t map_dinorex[] = {
	R(0x000000, 0x2fffff, D_ROM), R(0x300000, 0x30000f, D_NIO), R(0x400000, 0x400fff, D_EXT),
	R(0x500000, 0x501fff, D_PAL), R(0x600000, 0x60ffff, D_RAM), RL(0x700000, 0x70001f, D_PRI, LANE_LO),
	R(0x800000, 0x80ffff, D_SPR), R(0x900000, 0x90ffff, D_SCN), R(0x920000, 0x92000f, D_SCNC),
	RL(0xa00000, 0xa00003, D_SYT, LANE_HI), END };
static const range_t map_qjinsei[] = {
	R(0x000000, 0x1fffff, D_ROM), RL(0x200000, 0x200003, D_SYT, LANE_HI), R(0x300000, 0x30ffff, D_RAM),
	R(0x600000, 0x603fff, D_EXT), R(0x700000, 0x701fff, D_PAL), R(0x800000, 0x80ffff, D_SCN),
	R(0x820000, 0x82000f, D_SCNC), R(0x900000, 0x90ffff, D_SPR), RL(0xa00000, 0xa0001f, D_PRI, LANE_LO),
	R(0xb00000, 0xb0000f, D_NIO), END };
static const range_t map_qcrayon[] = {
	R(0x000000, 0x07ffff, D_ROM), R(0x100000, 0x10ffff, D_RAM), R(0x300000, 0x3fffff, D_EXTRA),
	RL(0x500000, 0x500003, D_SYT, LANE_HI), R(0x600000, 0x603fff, D_EXT), R(0x700000, 0x701fff, D_PAL),
	R(0x800000, 0x80ffff, D_SPR), R(0x900000, 0x90ffff, D_SCN), R(0x920000, 0x92000f, D_SCNC),
	R(0xa00000, 0xa0000f, D_NIO), RL(0xb00000, 0xb0001f, D_PRI, LANE_LO), END };
static const range_t map_qcrayon2[] = {
	R(0x000000, 0x07ffff, D_ROM), R(0x200000, 0x20ffff, D_RAM), R(0x300000, 0x301fff, D_PAL),
	R(0x400000, 0x40ffff, D_SPR), R(0x500000, 0x50ffff, D_SCN), R(0x520000, 0x52000f, D_SCNC),
	R(0x600000, 0x67ffff, D_EXTRA), R(0x700000, 0x70000f, D_NIO), RL(0x900000, 0x90001f, D_PRI, LANE_LO),
	RL(0xa00000, 0xa00003, D_SYT, LANE_HI), R(0xb00000, 0xb017ff, D_EXT), END };
static const range_t map_driftout[] = {
	R(0x000000, 0x0fffff, D_ROM), RL(0x200000, 0x200003, D_SYT, LANE_HI), R(0x300000, 0x30ffff, D_RAM),
	R(0x400000, 0x401fff, D_GRD), R(0x402000, 0x40200f, D_GRDC), R(0x700000, 0x701fff, D_PAL),
	R(0x800000, 0x80ffff, D_SCN), R(0x820000, 0x82000f, D_SCNC), R(0x900000, 0x90ffff, D_SPR),
	RL(0xa00000, 0xa0001f, D_PRI, LANE_HI), R(0xb00000, 0xb0000f, D_NIO),
	{ 0xb00018, 0xb00019, D_DIAL, LANE_WORD, "PADDLE1" }, { 0xb0001a, 0xb0001b, D_DIAL, LANE_WORD, "PADDLE2" }, END };
static const range_t map_driveout[] = {   // driftout with the bootleg sound latch at 0x200000
	R(0x000000, 0x0fffff, D_ROM), RL(0x200000, 0x200003, D_DOSND, LANE_HI), R(0x300000, 0x30ffff, D_RAM),
	R(0x400000, 0x401fff, D_GRD), R(0x402000, 0x40200f, D_GRDC), R(0x700000, 0x701fff, D_PAL),
	R(0x800000, 0x80ffff, D_SCN), R(0x820000, 0x82000f, D_SCNC), R(0x900000, 0x90ffff, D_SPR),
	RL(0xa00000, 0xa0001f, D_PRI, LANE_HI), R(0xb00000, 0xb0000f, D_NIO),
	{ 0xb00018, 0xb00019, D_DIAL, LANE_WORD, "PADDLE1" }, { 0xb0001a, 0xb0001b, D_DIAL, LANE_WORD, "PADDLE2" }, END };

// ---- machine configurations ----
enum { IO_NONE, IO_TC0220IOC, IO_TC0510NIO, IO_TE7750, IO_TE7750_NINJAK };
enum { PAL_RGBX444, PAL_XRGB555, PAL_RGB5X, PAL_PCR };
enum { VB_NO_BUFFER, VB_PARTIAL, VB_PARTIAL_THUNDFOX, VB_FULL, VB_PARTIAL_QZCHIKYU };
enum { SCR_DEFAULT, SCR_PRI, SCR_PRI_ROZ, SCR_THUNDFOX, SCR_SSI, SCR_YESNOJ, SCR_METALB, SCR_DEADCONX };
enum { ROZ_NONE, ROZ_GRD, ROZ_GRW };
enum { SPR_NORMAL, SPR_FINALB, SPR_MJNQUEST };
enum { SND_YM2610, SND_YM2203, SND_DRIVEOUT };

typedef struct {
	const char *name;
	const range_t *map;
	int io, pal, pal_size, vblank, screen, roz, sprgfx, snd;
	int sprite_type, hide, flip_hide, footchmp;
	uint32_t cpu_clock;             // 0 = 12 MHz
	int hidden;                     // emulated but not offered (controls the core does not have yet)
	int cchip;                      // C-Chip (megab): pulsed at vblank, ports not connected
	int scn_layout, scn_x, scn_y, scn_flipx, scn_fliptx, scn_fliptx_y, two_scn;
	int scp_x, scp_y, scp_tx, scp_fx, scp_colbase;
	int pivot_x, pivot_y;
} machine_t;

static const machine_t s_machines[] = {
	{ .name = "finalb", .map = map_finalb, .io = IO_TC0220IOC, .pal = PAL_PCR, .vblank = VB_PARTIAL, .screen = SCR_DEFAULT,
	  .sprgfx = SPR_FINALB, .hide = 1, .flip_hide = 1, .scn_x = 1 },
	{ .name = "dondokod", .map = map_dondokod, .io = IO_TC0220IOC, .pal = PAL_RGBX444, .vblank = VB_PARTIAL,
	  .screen = SCR_PRI_ROZ, .roz = ROZ_GRD, .hide = 3, .flip_hide = 3, .scn_x = 3, .pivot_x = -16 },
	{ .name = "thundfox", .map = map_thundfox, .io = IO_TC0220IOC, .pal = PAL_RGBX444, .vblank = VB_PARTIAL_THUNDFOX,
	  .screen = SCR_THUNDFOX, .hide = 3, .flip_hide = -3, .scn_x = 3, .scn_flipx = 5, .scn_fliptx = 4, .scn_fliptx_y = 1,
	  .two_scn = 1 },
	{ .name = "cameltry", .map = map_cameltry, .io = IO_TC0220IOC, .pal = PAL_RGBX444, .vblank = VB_PARTIAL,
	  .screen = SCR_PRI_ROZ, .roz = ROZ_GRD, .hide = 3, .flip_hide = 3, .scn_x = 3, .pivot_x = -16 },
	{ .name = "cameltrya", .map = map_cameltry, .io = IO_TC0220IOC, .pal = PAL_RGBX444, .vblank = VB_PARTIAL,
	  .screen = SCR_PRI_ROZ, .roz = ROZ_GRD, .snd = SND_YM2203, .hide = 3, .flip_hide = 3, .scn_x = 3, .pivot_x = -16 },
	{ .name = "driftoutct", .map = map_driftoutct, .io = IO_TC0220IOC, .pal = PAL_RGBX444, .vblank = VB_PARTIAL,
	  .screen = SCR_PRI_ROZ, .roz = ROZ_GRD, .hide = 3, .flip_hide = 3, .scn_x = 3, .pivot_x = -16 },
	{ .name = "qtorimon", .map = map_qtorimon, .io = IO_TC0220IOC, .pal = PAL_PCR, .vblank = VB_PARTIAL,
	  .screen = SCR_DEFAULT, .scn_layout = TC0100SCN_1BPP },
	{ .name = "megab", .map = map_megab, .io = IO_TC0220IOC, .pal = PAL_RGBX444, .vblank = VB_PARTIAL,
	  .screen = SCR_PRI, .hide = 3, .flip_hide = 3, .scn_x = 3, .cchip = 1 },
	{ .name = "liquidk", .map = map_liquidk, .io = IO_TC0220IOC, .pal = PAL_RGBX444, .vblank = VB_PARTIAL,
	  .screen = SCR_PRI, .hide = 3, .flip_hide = 3, .scn_x = 3 },
	{ .name = "quizhq", .map = map_quizhq, .io = IO_NONE, .pal = PAL_PCR, .vblank = VB_PARTIAL, .screen = SCR_DEFAULT,
	  .scn_layout = TC0100SCN_1BPP },
	{ .name = "ssi", .map = map_ssi, .io = IO_TC0510NIO, .pal = PAL_RGBX444, .vblank = VB_PARTIAL_THUNDFOX,
	  .screen = SCR_SSI, .hide = 3, .flip_hide = 3, .scn_x = 3 },
	{ .name = "gunfront", .map = map_gunfront, .io = IO_TC0510NIO, .pal = PAL_RGBX444, .vblank = VB_PARTIAL,
	  .screen = SCR_PRI, .hide = 3, .flip_hide = 3, .scn_x = 3 },
	{ .name = "growl", .map = map_growl, .io = IO_NONE, .pal = PAL_RGB5X, .vblank = VB_NO_BUFFER, .screen = SCR_PRI,
	  .hide = 3, .flip_hide = 3, .scn_x = 3 },
	{ .name = "mjnquest", .map = map_mjnquest, .io = IO_NONE, .pal = PAL_PCR, .vblank = VB_NO_BUFFER,
	  .screen = SCR_DEFAULT, .sprgfx = SPR_MJNQUEST, .hidden = 1 },   // mahjong panel: no mahjong keys in the core yet
	{ .name = "footchmp", .map = map_footchmp, .io = IO_TE7750, .pal = PAL_RGBX444, .vblank = VB_FULL,
	  .screen = SCR_DEADCONX, .hide = 3, .flip_hide = 3, .footchmp = 1, .scp_x = 0x1d + 3, .scp_y = 0x08, .scp_tx = -1,
	  .scp_fx = -1 },
	{ .name = "hthero", .map = map_footchmp, .io = IO_TE7750, .pal = PAL_RGBX444, .vblank = VB_FULL,
	  .screen = SCR_DEADCONX, .hide = 3, .flip_hide = 3, .footchmp = 1, .scp_x = 0x33 + 3, .scp_y = -0x04, .scp_tx = -1,
	  .scp_fx = -1 },
	{ .name = "koshien", .map = map_koshien, .io = IO_TC0510NIO, .pal = PAL_RGB5X, .vblank = VB_NO_BUFFER,
	  .screen = SCR_PRI, .hide = 1, .flip_hide = -1, .scn_x = 1, .scn_flipx = 2 },
	{ .name = "yuyugogo", .map = map_yuyugogo, .io = IO_TC0510NIO, .pal = PAL_RGBX444, .vblank = VB_NO_BUFFER,
	  .screen = SCR_YESNOJ, .sprite_type = 1, .hide = 3, .flip_hide = 3, .scn_layout = TC0100SCN_1BPP, .scn_x = 3 },
	{ .name = "ninjak", .map = map_ninjak, .io = IO_TE7750_NINJAK, .pal = PAL_RGBX444, .vblank = VB_NO_BUFFER,
	  .screen = SCR_PRI, .scn_x = 1 },
	{ .name = "solfigtr", .map = map_solfigtr, .io = IO_NONE, .pal = PAL_RGBX444, .vblank = VB_NO_BUFFER,
	  .screen = SCR_PRI, .hide = 3, .flip_hide = -3, .scn_x = 3, .scn_flipx = 6, .scn_fliptx = 6 },
	{ .name = "qzquest", .map = map_qzquest, .io = IO_TC0510NIO, .pal = PAL_XRGB555, .vblank = VB_PARTIAL,
	  .screen = SCR_DEFAULT },
	{ .name = "pulirula", .map = map_pulirula, .io = IO_TC0510NIO, .pal = PAL_XRGB555, .vblank = VB_NO_BUFFER,
	  .screen = SCR_PRI_ROZ, .roz = ROZ_GRW, .sprite_type = 2, .hide = 3, .flip_hide = 3, .scn_x = 3, .pivot_x = -10,
	  .pivot_y = 16 },
	{ .name = "metalb", .map = map_metalb, .io = IO_TC0510NIO, .pal = PAL_RGB5X, .pal_size = 8192,
	  .vblank = VB_NO_BUFFER, .screen = SCR_METALB, .hide = 3, .flip_hide = 3, .scp_x = 0x32 + 3, .scp_y = -0x04,
	  .scp_tx = 1, .scp_fx = -1, .scp_colbase = 4096 },
	{ .name = "qzchikyu", .map = map_qzquest, .io = IO_TC0510NIO, .pal = PAL_XRGB555, .vblank = VB_PARTIAL_QZCHIKYU,
	  .screen = SCR_DEFAULT, .flip_hide = 4, .scn_flipx = -4, .scn_fliptx = -11 },
	{ .name = "yesnoj", .map = map_yesnoj, .io = IO_NONE, .pal = PAL_RGB5X, .vblank = VB_NO_BUFFER, .screen = SCR_YESNOJ,
	  .hide = 3, .flip_hide = 3, .scn_layout = TC0100SCN_1BPP, .scn_x = 3 },
	{ .name = "deadconx", .map = map_deadconx, .io = IO_TE7750, .pal = PAL_RGB5X, .vblank = VB_NO_BUFFER,
	  .screen = SCR_DEADCONX, .hide = 3, .flip_hide = 3, .scp_x = 0x1e + 3, .scp_y = 0x08, .scp_tx = -1, .scp_fx = -1 },
	{ .name = "deadconxj", .map = map_deadconx, .io = IO_TE7750, .pal = PAL_RGB5X, .vblank = VB_NO_BUFFER,
	  .screen = SCR_DEADCONX, .hide = 3, .flip_hide = 3, .scp_x = 0x34 + 3, .scp_y = -0x05, .scp_tx = -1, .scp_fx = -1 },
	{ .name = "dinorex", .map = map_dinorex, .io = IO_TC0510NIO, .pal = PAL_RGB5X, .vblank = VB_NO_BUFFER,
	  .screen = SCR_PRI, .sprite_type = 3, .hide = 3, .flip_hide = 3, .scn_x = 3 },
	{ .name = "qjinsei", .map = map_qjinsei, .io = IO_TC0510NIO, .pal = PAL_RGB5X, .vblank = VB_NO_BUFFER,
	  .screen = SCR_PRI, .sprite_type = 3, .hide = 3, .flip_hide = 3, .scn_x = 3 },
	{ .name = "qcrayon", .map = map_qcrayon, .io = IO_TC0510NIO, .pal = PAL_RGB5X, .vblank = VB_NO_BUFFER,
	  .screen = SCR_PRI, .sprite_type = 3, .hide = 3, .flip_hide = 3, .scn_x = 3 },
	{ .name = "qcrayon2", .map = map_qcrayon2, .io = IO_TC0510NIO, .pal = PAL_RGB5X, .vblank = VB_NO_BUFFER,
	  .screen = SCR_PRI, .sprite_type = 3, .hide = 3, .flip_hide = 3, .scn_x = 3 },
	{ .name = "driftout", .map = map_driftout, .io = IO_TC0510NIO, .pal = PAL_XRGB555, .vblank = VB_NO_BUFFER,
	  .screen = SCR_PRI_ROZ, .roz = ROZ_GRW, .hide = 3, .flip_hide = 3, .scn_x = 3, .pivot_x = -16, .pivot_y = 16 },
	{ .name = "driveout", .map = map_driveout, .io = IO_TC0510NIO, .pal = PAL_XRGB555, .vblank = VB_NO_BUFFER,
	  .screen = SCR_PRI_ROZ, .roz = ROZ_GRW, .snd = SND_DRIVEOUT, .hide = 3, .flip_hide = 3, .scn_x = 3, .pivot_x = -16,
	  .pivot_y = 16, .cpu_clock = 14000000 },
};

#define LINES 256
#define VBLANK_LINE 240
#define SCREEN_W 320
#define VIS_Y0 16
#define VIS_H 224
#define FOOTCHMP_GAME 1

// ---- board state ----
static const machine_t *s_m;
static uint8_t *s_rom, *s_extra;
static uint32_t s_rom_size, s_extra_size;
#define MAX_RAM 4
static uint8_t *s_ram[MAX_RAM];
static uint32_t s_ram_size[MAX_RAM];
static int s_nram;
static uint16_t *s_pal;               // palette RAM (or TC0110PCR RAM)
static uint32_t s_pal_bytes;
static uint16_t s_pcr_addr;
static uint16_t (*s_spr_hp)[0x8000];         // sprite RAM 0x10000 bytes
#define s_spr (*s_spr_hp)
static uint16_t s_spr_buf[0x8000], s_spr_delayed[0x8000];
static uint16_t (*s_ext_hp)[0x2000];         // sprite extension (0x4000 bytes mapped; 0x800 words used)
#define s_ext (*s_ext_hp)
static tc0100scn_t *s_scn[2];
static tc0280grd_t *s_grd;
static tc0480scp_t *s_scp;
static tc0360pri_t s_pri;
static uint8_t s_io_regs[8];
static uint64_t s_cc_frac;
static uint8_t s_te_latch[9], s_te_dir[9];
static int s_te_port[9];
static int s_port_io[8];
static int s_range_port[32];
static int s_port_key[5], s_port_mjin[2], s_port_mjdsw[2];
static int s_mj_input, s_mj_gfxbank;
static int s_dial[2], s_dial_last[2], s_port_dial[2];
static uint64_t s_cpu_frac;

// ---- Drive Out bootleg sound: Z80 + OKI M6295 behind a nibble command latch (MAME driveout_state) ----
static z80 s_dz;
static uint8_t s_dz_ram[0x800];
static uint8_t *s_dz_rom, *s_dz_oki;   // OKI space per bank: 0x00000-0x1ffff banked, 0x20000-0x3ffff fixed (0x80000)
static uint32_t s_dz_rom_size;
static okim6295_t s_doki;
static int s_dz_latch, s_dz_nibble, s_dz_irq;
static uint64_t s_dz_frac, s_doki_frac;

static uint8_t dz_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x8000) return a < s_dz_rom_size ? s_dz_rom[a] : 0xff;
	if (a < 0x8800) return s_dz_ram[a & 0x7ff];
	if (a == 0x9800) return oki_read(&s_doki);
	if (a == 0xa000) { s_dz_irq = 0; return (uint8_t)s_dz_latch; }
	return 0xff;
}

static void dz_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0x8000 && a < 0x8800) s_dz_ram[a & 0x7ff] = v;
	else if (a == 0x9000) { if (v & 4) s_doki.bank = (uint32_t)(v & 3) * 0x40000; }
	else if (a == 0x9800) oki_write(&s_doki, v);
}

static uint8_t dz_in(z80 *z, uint8_t p) { (void)z; (void)p; return 0xff; }
static void dz_out(z80 *z, uint8_t p, uint8_t v) { (void)z; (void)p; (void)v; }

static void dz_start(void)
{
	z80_init(&s_dz);
	s_dz.read_byte = dz_read;
	s_dz.write_byte = dz_write;
	s_dz.port_in = dz_in;
	s_dz.port_out = dz_out;
}

static void dz_command_w(int offset, uint8_t data)
{
	if (offset == 0)
		s_dz_nibble = data & 1;
	else if (!s_dz_nibble)
		s_dz_latch = (data & 0x0f) | (s_dz_latch & 0xf0);
	else {
		s_dz_latch = ((data << 4) & 0xf0) | (s_dz_latch & 0x0f);
		s_dz_irq = 1;
	}
}

static int dz_init(void)
{
	uint32_t osize;
	uint8_t *oki = rbx_region("oki", &osize);
	s_dz_rom = rbx_region("audiocpu", &s_dz_rom_size);
	if (!oki || !s_dz_rom)
		return -1;
	s_dz_oki = (uint8_t *)calloc(4, 0x40000);
	if (!s_dz_oki)
		return -1;
	for (uint32_t b = 0; b < 4; b++)
		for (uint32_t i = 0; i < 0x40000; i++) {
			uint32_t src = i < 0x20000 ? b * 0x20000 + i : 0x80000 + (i - 0x20000);
			s_dz_oki[b * 0x40000 + i] = src < osize ? oki[src] : 0;
		}
	oki_init(&s_doki, 1000000, 0, s_dz_oki, 4 * 0x40000, 256);
	return rbx_audio_setup(s_doki.rate);
}

static void dz_reset(void)
{
	memset(s_dz_ram, 0, sizeof(s_dz_ram));
	dz_start();
	oki_reset(&s_doki);
	s_doki.bank = 0;
	s_dz_latch = s_dz_nibble = s_dz_irq = 0;
	s_dz_frac = s_doki_frac = 0;
}

static void dz_line(void)
{
	unsigned long end = s_dz.cyc + (unsigned long)rbx_line_cycles(&s_dz_frac, 4000000, 60000, 256);
	while ((long)(s_dz.cyc - end) < 0) {
		s_dz.int_pending = s_dz_irq ? 1 : 0;
		z80_step(&s_dz);
	}
	int n = rbx_line_cycles(&s_doki_frac, s_doki.rate, 60000, 256);
	for (int i = 0; i < n; i++)
		rbx_audio_put(oki_sample(&s_doki));
}
static uint8_t *s_spr_tiles, *s_spr_used;
static uint32_t s_spr_ntiles, s_spr_colors, s_spr_gran;

// sprite chip state (MAME taitof2_state)
static uint16_t s_spritebank[8], s_spritebank_buffered[8];
static int s_sprites_disabled, s_sprites_active_area, s_master_scrollx, s_master_scrolly, s_prepare_sprites;

// drawing snapshot (rbx_video_frame_done) and the drawing side's own state
static struct {
	uint16_t spr[0x8000];
	uint16_t ext[0x800];
	uint16_t bank[8];
	int disabled, area, msx, msy;
	tc0360pri_t pri;
	uint16_t pal[0x2000];
} *s_snap;
static tc0100scn_t *s_snap_scn[2];
static tc0280grd_t *s_snap_grd;
static tc0480scp_t *s_snap_scp;
static int s_r_flipscreen;             // MAME m_sprites_flipscreen, kept across frames by draw_sprites
static uint8_t *s_pribuf;              // MAME screen.priority()
static uint8_t s_tilepri[6], s_spritepri[6], s_blendmode;

static inline int sext12(int v) { return (v & 0x800) ? (v | ~0xfff) : (v & 0xfff); }

// ---- I/O chips ----
static uint8_t ioc_read(int offset)
{
	switch (offset) {
	case 0: case 1: case 2: case 3: case 7: return (uint8_t)rbx_port_read(s_port_io[offset]);
	case 4: return s_io_regs[4];
	}
	return 0xff;
}

static void ioc_write(int offset, uint8_t data) { s_io_regs[offset & 7] = data; }   // 0 watchdog, 4 coin lockout / counters

static uint8_t te_read(int offset)
{
	if (offset < 9) {
		uint8_t data = s_te_latch[offset], dir = s_te_dir[offset];
		if (dir) {
			uint8_t in = s_te_port[offset] >= 0 ? (uint8_t)rbx_port_read(s_te_port[offset]) : 0xff;
			data = (uint8_t)((data & ~dir) | (in & dir));
		}
		return data;
	}
	return 0xff;
}

static void te_write(int offset, uint8_t data)
{
	if (offset < 9)
		s_te_latch[offset] = data;   // outputs: coin counters / lockout
	else if (offset == 9)
		s_te_dir[7] = data;
	else if (offset == 10) {         // soft mode CR1
		s_te_dir[0] = (data & 0x10) ? 0xff : 0x00;
		s_te_dir[1] = (uint8_t)(((data & 0x08) ? 0xf0 : 0x00) | ((data & 0x01) ? 0x0f : 0x00));
		s_te_dir[2] = (data & 0x02) ? 0xff : 0x00;
	} else if (offset == 11) {
		s_te_dir[3] = (data & 0x10) ? 0xff : 0x00;
		s_te_dir[4] = (uint8_t)(((data & 0x08) ? 0xf0 : 0x00) | ((data & 0x01) ? 0x0f : 0x00));
		s_te_dir[5] = (data & 0x02) ? 0xff : 0x00;
	} else if (offset == 12) {
		s_te_dir[6] = (data & 0x10) ? 0xff : 0x00;
		s_te_dir[8] = (data & 0x02) ? 0xff : 0x00;
	}
}

// ---- sprite banks (MAME spritebank_w, koshien_spritebank_w) ----
static void spritebank_w(int offset, uint16_t data)
{
	if (offset < 2)
		return;
	if (offset < 4) {
		int j = (offset & 1) << 1;
		int i = data << 11;
		s_spritebank_buffered[j] = (uint16_t)i;
		s_spritebank_buffered[j + 1] = (uint16_t)(i + 0x400);
	} else {
		s_spritebank_buffered[offset] = (uint16_t)(data << 10);
	}
}

static void koshien_spritebank_w(uint16_t data)
{
	s_spritebank_buffered[0] = 0x0000;
	s_spritebank_buffered[1] = 0x0400;
	s_spritebank_buffered[2] = (uint16_t)(((data & 0x00f) + 1) * 0x800);
	s_spritebank_buffered[4] = (uint16_t)((((data & 0x0f0) >> 4) + 1) * 0x800);
	s_spritebank_buffered[6] = (uint16_t)((((data & 0xf00) >> 8) + 1) * 0x800);
	s_spritebank_buffered[3] = (uint16_t)(s_spritebank_buffered[2] + 0x400);
	s_spritebank_buffered[5] = (uint16_t)(s_spritebank_buffered[4] + 0x400);
	s_spritebank_buffered[7] = (uint16_t)(s_spritebank_buffered[6] + 0x400);
}

// ---- main CPU handlers ----
static const range_t *find_range(uint32_t a, int *idx)
{
	const range_t *r = s_m->map;
	for (int i = 0; r[i].dev; i++)
		if (a >= r[i].start && a <= r[i].end) {
			*idx = i;
			return &r[i];
		}
	return NULL;
}

static uint8_t dev8_read(const range_t *r, uint32_t off)
{
	switch (r->dev) {
	case D_IOC: return ioc_read((int)off);
	case D_TE: return te_read((int)off);
	case D_SYT: return off == 1 ? tc0140syt_master_comm_r(tsnd_syt()) : 0;
	case D_RTC: return 0;
	case D_CCHIP: return off < 0x400 ? cchip_mem68_r(off) : cchip_asic_r(off - 0x400);
	}
	return 0;
}

static void dev8_write(const range_t *r, uint32_t off, uint8_t v)
{
	switch (r->dev) {
	case D_IOC: ioc_write((int)off, v); break;
	case D_TE: te_write((int)off, v); break;
	case D_PRI: tc0360pri_w(&s_pri, (int)off, v); break;
	case D_SYT:
		if (off == 0) tc0140syt_master_port_w(tsnd_syt(), v);
		else if (off == 1) tc0140syt_master_comm_w(tsnd_syt(), v);
		break;
	case D_DOSND: dz_command_w((int)off, v); break;
	case D_CCHIP:
		if (off < 0x400)
			cchip_mem68_w(off, v);
		else
			cchip_asic68_w(off - 0x400, v);
		break;
	case D_MJBANK:
		if ((v ^ s_mj_gfxbank) & 1) {
			s_mj_gfxbank = v & 1;
			s_scn[0]->code_bank = (uint32_t)s_mj_gfxbank << 15;
		}
		break;
	}
}

static uint16_t dev16_read(const range_t *r, int idx, uint32_t a)
{
	uint32_t off = (a - r->start) >> 1;
	switch (r->dev) {
	case D_SCNC: return tc0100scn_ctrl_r(s_scn[0], (int)off);
	case D_SCN2C: return tc0100scn_ctrl_r(s_scn[1], (int)off);
	case D_SCPC: return tc0480scp_ctrl_r(s_scp, (int)off);
	case D_NIO: return ioc_read((int)off);
	case D_NIOSW: return ioc_read((int)(off ^ 1));
	case D_PCR: return off == 1 ? s_pal[s_pcr_addr] : 0xff;
	case D_PORT: return (uint16_t)rbx_port_read(s_range_port[idx]);
	case D_PADDLE: {   // cameltry: delta since the last read
		int p = off == 0 ? 0 : off == 2 ? 1 : -1;
		if (p < 0)
			return 0;
		int curr = s_dial[p], res = curr - s_dial_last[p];
		s_dial_last[p] = curr;
		return (uint16_t)res;
	}
	case D_DIAL: {
		int p = r->tag[6] == '2';
		return (uint16_t)((rbx_port_read(s_range_port[idx]) & 0xff00) | (s_dial[p] & 0xff));
	}
	case D_MJDSW:
		if (off == 0) return (uint16_t)((rbx_port_read(s_port_mjin[0]) & 0xff) << 8 | (rbx_port_read(s_port_mjdsw[0]) & 0xff));
		if (off == 1) return (uint16_t)((rbx_port_read(s_port_mjin[1]) & 0xff) << 8 | (rbx_port_read(s_port_mjdsw[1]) & 0xff));
		return 0xff;
	case D_MJIN: {
		uint16_t ret = 0x3f;
		for (int i = 0; i < 5; i++)
			if (s_mj_input & (1 << i))
				ret &= (uint16_t)rbx_port_read(s_port_key[i]);
		return (uint16_t)(0xffc0 | ret);
	}
	}
	return 0;
}

static void dev16_write(const range_t *r, uint32_t a, uint16_t v, uint16_t mask)
{
	uint32_t off = (a - r->start) >> 1;
	switch (r->dev) {
	case D_SCNC: tc0100scn_ctrl_w(s_scn[0], (int)off, v, mask); break;
	case D_SCN2C: tc0100scn_ctrl_w(s_scn[1], (int)off, v, mask); break;
	case D_GRDC: tc0280grd_ctrl_w(s_grd, (int)off, v, mask); break;
	case D_SCPC: tc0480scp_ctrl_w(s_scp, (int)off, v, mask); break;
	case D_NIO: case D_NIOSW: {
		int o = (int)(r->dev == D_NIOSW ? off ^ 1 : off);
		ioc_write(o, (mask & 0x00ff) ? (uint8_t)v : (uint8_t)(v >> 8));
		break;
	}
	case D_PCR:
		if (off == 0) s_pcr_addr = (uint16_t)((v >> 1) & 0xfff);
		else if (off == 1) s_pal[s_pcr_addr] = v;
		break;
	case D_SPRBANK: spritebank_w((int)off, v); break;
	case D_KOSBANK: koshien_spritebank_w(v); break;
	case D_MJSEL: s_mj_input = v >> 6; break;
	}
}

static uint16_t main_read16(uint32_t a)
{
	int idx;
	const range_t *r = find_range(a, &idx);
	if (!r)
		return 0;
	if (r->lane == LANE_WORD)
		return dev16_read(r, idx, a);
	uint8_t v = dev8_read(r, (a - r->start) >> 1);
	return r->lane == LANE_HI ? (uint16_t)(v << 8) : v;
}

static uint8_t main_read8(uint32_t a)
{
	int idx;
	const range_t *r = find_range(a, &idx);
	if (!r)
		return 0;
	if (r->lane == LANE_WORD)
		return (uint8_t)(dev16_read(r, idx, a & ~1u) >> ((a & 1) ? 0 : 8));
	if ((r->lane == LANE_HI) != !(a & 1))
		return 0;
	return dev8_read(r, (a - r->start) >> 1);
}

static void main_write16m(uint32_t a, uint16_t v, uint16_t mask)
{
	int idx;
	const range_t *r = find_range(a, &idx);
	if (!r)
		return;
	if (r->lane == LANE_WORD) {
		dev16_write(r, a, v, mask);
		return;
	}
	uint32_t o = (a - r->start) >> 1;
	if (r->lane == LANE_HI && (mask & 0xff00))
		dev8_write(r, o, (uint8_t)(v >> 8));
	else if (r->lane == LANE_LO && (mask & 0x00ff))
		dev8_write(r, o, (uint8_t)v);
}

static void main_write16(uint32_t a, uint16_t v) { main_write16m(a, v, 0xffff); }
static void main_write8(uint32_t a, uint8_t v)
{
	if (a & 1)
		main_write16m(a & ~1u, v, 0x00ff);
	else
		main_write16m(a, (uint16_t)(v << 8), 0xff00);
}

// ---- sprites: buffering at vblank (MAME taito_f2_v.cpp) ----
static void handle_sprite_buffering(void)
{
	if (s_prepare_sprites) {
		memcpy(s_spr_buf, s_spr, sizeof(s_spr));
		s_prepare_sprites = 0;
	}
}

static void update_sprites_active_area(void)
{
	memcpy(s_spritebank, s_spritebank_buffered, sizeof(s_spritebank));
	handle_sprite_buffering();
	const uint16_t *b = s_spr_buf;
	if (s_sprites_active_area == 0x8000 && b[(0x8000 + 6) / 2] == 0 && b[(0x8000 + 10) / 2] == 0)
		s_sprites_active_area = 0;
	for (int off = 0; off < 0x4000; off += 16) {
		int offs = off + s_sprites_active_area;
		if (b[(offs + 6) / 2] & 0x8000) {
			s_sprites_disabled = (b[(offs + 10) / 2] >> 12) & 1;
			if (s_m->footchmp)
				s_sprites_active_area = 0x8000 * (b[(offs + 6) / 2] & 1);
			else
				s_sprites_active_area = 0x8000 * (b[(offs + 10) / 2] & 1);
			continue;
		}
		if ((b[(offs + 4) / 2] & 0xf000) == 0xa000) {
			s_master_scrollx = sext12(b[(offs + 4) / 2]);
			s_master_scrolly = sext12(b[(offs + 6) / 2]);
		}
	}
}

static void screen_vblank(void)
{
	update_sprites_active_area();
	if (s_m->vblank == VB_NO_BUFFER) {
		s_prepare_sprites = 1;
		return;
	}
	s_prepare_sprites = 0;
	memcpy(s_spr_buf, s_spr_delayed, sizeof(s_spr_buf));
	switch (s_m->vblank) {
	case VB_FULL:
		memcpy(s_spr_buf, s_spr, sizeof(s_spr_buf));
		break;
	case VB_PARTIAL:
		for (int i = 0; i < 0x8000; i += 4)
			s_spr_buf[i] = s_spr[i];
		break;
	case VB_PARTIAL_THUNDFOX:
		for (int i = 0; i < 0x8000; i += 8) {
			s_spr_buf[i] = s_spr[i];
			s_spr_buf[i + 1] = s_spr[i + 1];
			s_spr_buf[i + 4] = s_spr[i + 4];
		}
		break;
	case VB_PARTIAL_QZCHIKYU:
		for (int i = 0; i < 0x8000; i += 8) {
			s_spr_buf[i] = s_spr[i];
			s_spr_buf[i + 1] = s_spr[i + 1];
			s_spr_buf[i + 4] = s_spr[i + 4];
			s_spr_buf[i + 5] = s_spr[i + 5];
			s_spr_buf[i + 6] = s_spr[i + 6];
			s_spr_buf[i + 7] = s_spr[i + 7];
		}
		break;
	}
	memcpy(s_spr_delayed, s_spr, sizeof(s_spr_delayed));
}

// ---- sprites: drawing (MAME draw_sprites, taito_f2_tc360_spritemixdraw) ----
typedef struct { uint32_t code, color; int flipx, flipy, x, y, zoomx, zoomy; uint64_t primask; } tempsprite_t;
static tempsprite_t (*s_list_hp)[0x400];
#define s_list (*s_list_hp)

static tempsprite_t *s_list_end;   // (the list draw_sprites made: s_list .. s_list_end)

// one sprite on the screen lines cy0..cy1 (bm: the line VIS_Y0)
static void sprite_mixdraw(uint16_t *bm, int pitch, const tempsprite_t *s, int use_mixer, int use_pri, int cy0, int cy1)
{
	uint32_t pal_base = s_spr_gran * (s->color % s_spr_colors);
	uint32_t code = s->code < s_spr_ntiles ? s->code : s->code % s_spr_ntiles;
	if (!s_spr_used[code])
		return;
	const uint8_t *source_base = s_spr_tiles + (size_t)code * 256;
	int scalex = s->zoomx, scaley = s->zoomy;
	if (!scalex || !scaley)
		return;
	int h = (scaley * 16 + 0x8000) >> 16, w = (scalex * 16 + 0x8000) >> 16;
	if (!w || !h)
		return;
	int dx = (16 << 16) / w, dy = (16 << 16) / h;
	int sx = s->x, sy = s->y, ex = sx + w, ey = sy + h;
	int x_index_base, y_index;
	if (s->flipx) { x_index_base = (w - 1) * dx; dx = -dx; } else x_index_base = 0;
	if (s->flipy) { y_index = (h - 1) * dy; dy = -dy; } else y_index = 0;
	const int min_x = 0, max_x = SCREEN_W - 1, min_y = cy0, max_y = cy1;
	if (sx < min_x) { int pixels = min_x - sx; sx += pixels; x_index_base += pixels * dx; }
	if (sy < min_y) { int pixels = min_y - sy; sy += pixels; y_index += pixels * dy; }
	if (ex > max_x + 1) ex = max_x + 1;
	if (ey > max_y + 1) ey = max_y + 1;
	if (ex <= sx)
		return;
	for (int y = sy; y < ey; y++, y_index += dy) {
		const uint8_t *source = source_base + (y_index >> 16) * 16;
		uint16_t *dest = bm + (y - VIS_Y0) * pitch;
		uint8_t *pri = s_pribuf + (y - VIS_Y0) * pitch;
		int x_index = x_index_base;
		if (!use_pri) {   // zoom_transpen: no priority bitmap
			for (int x = sx; x < ex; x++, x_index += dx) {
				uint8_t c = source[x_index >> 16];
				if (c)
					dest[x] = (uint16_t)(pal_base + c);
			}
			continue;
		}
		for (int x = sx; x < ex; x++, x_index += dx) {
			uint8_t c = source[x_index >> 16];
			if (!c || (pri[x] & 0x80))
				continue;
			if (use_mixer) {
				uint8_t p = pri[x], tilemap_priority = 0;
				if (p & 0x10) tilemap_priority = s_tilepri[4];
				else if (p & 0x08) tilemap_priority = s_tilepri[3];
				else if (p & 0x04) tilemap_priority = s_tilepri[2];
				else if (p & 0x02) tilemap_priority = s_tilepri[1];
				else if (p & 0x01) tilemap_priority = s_tilepri[0];
				uint8_t sprite_priority = s_spritepri[s->primask & 3];
				uint16_t pix = (uint16_t)(pal_base + c);
				if ((s_blendmode & 0xc0) == 0xc0 && sprite_priority == tilemap_priority - 1)
					dest[x] = (uint16_t)((pix & 0xfff0) | (dest[x] & 0xf));
				else if ((s_blendmode & 0xc0) == 0xc0 && sprite_priority == tilemap_priority + 1) {
					if (dest[x] & 0xf)
						dest[x] = (uint16_t)((dest[x] & 0xfff0) | (pix & 0xf));
					else
						dest[x] = pix;
				} else if ((s_blendmode & 0xc0) == 0x80 && sprite_priority == tilemap_priority - 1)
					dest[x] = (uint16_t)(dest[x] & 0xffef);
				else if ((s_blendmode & 0xc0) == 0x80 && sprite_priority == tilemap_priority + 1)
					dest[x] = (uint16_t)(pix & 0xffef);
				else if (sprite_priority > tilemap_priority)
					dest[x] = pix;
			} else {
				if (!((1ull << (pri[x] & 0x3f)) & s->primask))
					dest[x] = (uint16_t)(pal_base + c);
			}
			pri[x] |= 0x80;
		}
	}
}

// the listed sprites (primasks or uses_tc360_mixer) on the lines cy0..cy1, last listed first
static void draw_list(uint16_t *bm, int pitch, int uses_tc360_mixer, int cy0, int cy1)
{
	for (const tempsprite_t *sp = s_list_end; sp != s_list;) {
		sp--;
		sprite_mixdraw(bm, pitch, sp, uses_tc360_mixer, 1, cy0, cy1);
	}
}

// draw: 0 only lists them (with primasks or uses_tc360_mixer; draw_list draws them)
static void draw_sprites(uint16_t *bm, int pitch, const uint64_t *primasks, int uses_tc360_mixer, int draw)
{
	const uint16_t *b = s_snap->spr;
	int big_sprite = 0, last_continuation_tile = 0;
	int y_no = 0, x_no = 0, xlatch = 0, ylatch = 0;
	uint32_t zoomword, zoomx, zoomy, zx = 0, zy = 0, zoomxlatch = 0, zoomylatch = 0;
	int scrollx = 0, scrolly = 0;
	tempsprite_t *sp = s_list;
	int disabled = s_snap->disabled;
	int master_scrollx = s_snap->msx, master_scrolly = s_snap->msy;
	int area = s_snap->area;
	int scroll1x = 0, scroll1y = 0;
	int x = 0, y = 0, xcurrent = 0, ycurrent = 0;
	uint32_t color = 0;
	int f2_x_offset = s_m->hide;
	if (s_r_flipscreen)
		f2_x_offset = -s_m->flip_hide;
	if (area == 0x8000 && b[(0x8000 + 6) / 2] == 0 && b[(0x8000 + 10) / 2] == 0)
		area = 0;
	for (int off = 0; off < 0x4000; off += 16) {
		int offs = off + area;
		if (b[(offs + 6) / 2] & 0x8000) {
			disabled = (b[(offs + 10) / 2] >> 12) & 1;
			s_r_flipscreen = (b[(offs + 10) / 2] >> 13) & 1;
			f2_x_offset = s_m->hide;
			if (s_r_flipscreen)
				f2_x_offset = -s_m->flip_hide;
			if (s_m->footchmp)
				area = 0x8000 * (b[(offs + 6) / 2] & 1);
			else
				area = 0x8000 * (b[(offs + 10) / 2] & 1);
			continue;
		}
		if ((b[(offs + 4) / 2] & 0xf000) == 0xa000) {
			master_scrollx = sext12(b[(offs + 4) / 2]);
			master_scrolly = sext12(b[(offs + 6) / 2]);
		}
		if ((b[(offs + 4) / 2] & 0xf000) == 0x5000) {
			scroll1x = sext12(b[(offs + 4) / 2]);
			scroll1y = sext12(b[(offs + 6) / 2]);
		}
		if (disabled)
			continue;
		uint16_t spritedata = b[(offs + 8) / 2];
		int spritecont = (spritedata & 0xff00) >> 8;
		if (spritecont & 0x08) {
			if (!big_sprite) {
				xlatch = b[(offs + 4) / 2] & 0xfff;
				ylatch = b[(offs + 6) / 2] & 0xfff;
				x_no = y_no = 0;
				zoomword = b[(offs + 2) / 2];
				zoomylatch = (zoomword >> 8) & 0xff;
				zoomxlatch = zoomword & 0xff;
				big_sprite = 1;
			}
		} else if (big_sprite) {
			last_continuation_tile = 1;
		}
		if (!(spritecont & 0x04))
			color = spritedata & 0xff;
		if (!big_sprite || !(spritecont & 0xf0)) {
			x = b[(offs + 4) / 2];
			if (x & 0x8000) {
				scrollx = -f2_x_offset - 0x60;
				scrolly = 0;
			} else if (x & 0x4000) {
				scrollx = master_scrollx - f2_x_offset - 0x60;
				scrolly = master_scrolly;
			} else {
				scrollx = scroll1x + master_scrollx - f2_x_offset - 0x60;
				scrolly = scroll1y + master_scrolly;
			}
			x &= 0xfff;
			y = b[(offs + 6) / 2] & 0xfff;
			xcurrent = x;
			ycurrent = y;
		} else {
			if (!(spritecont & 0x10))
				y = ycurrent;
			else if (spritecont & 0x20) {
				y += 16;
				y_no++;
			}
			if (!(spritecont & 0x40))
				x = xcurrent;
			else if (spritecont & 0x80) {
				x += 16;
				y_no = 0;
				x_no++;
			}
		}
		if (big_sprite) {
			zoomx = zoomxlatch;
			zoomy = zoomylatch;
			zx = zy = 0x10;
			if (zoomx || zoomy) {
				x = xlatch + (int)((x_no * (0xff - zoomx) + 15) / 16);
				y = ylatch + (int)((y_no * (0xff - zoomy) + 15) / 16);
				zx = (uint32_t)(xlatch + (int)(((x_no + 1) * (0xff - zoomx) + 15) / 16) - x);
				zy = (uint32_t)(ylatch + (int)(((y_no + 1) * (0xff - zoomy) + 15) / 16) - y);
			}
		} else {
			zoomword = b[(offs + 2) / 2];
			zoomy = (zoomword >> 8) & 0xff;
			zoomx = zoomword & 0xff;
			zx = (0x100 - zoomx) / 16;
			zy = (0x100 - zoomy) / 16;
		}
		if (last_continuation_tile) {
			big_sprite = 0;
			last_continuation_tile = 0;
		}
		uint32_t code = 0;
		int extoffs = offs;
		if (extoffs >= 0x8000)
			extoffs -= 0x4000;
		switch (s_m->sprite_type) {
		case 0:
			code = b[offs / 2] & 0x1fff;
			code = s_snap->bank[(code & 0x1c00) >> 10] + (code & 0x3ff);
			break;
		case 1: code = (b[offs / 2] & 0x3ff) | ((s_snap->ext[extoffs >> 4] & 0x3f) << 10); break;
		case 2: code = (b[offs / 2] & 0xff) | (s_snap->ext[extoffs >> 4] & 0xff00); break;
		case 3: code = (b[offs / 2] & 0xff) | ((s_snap->ext[extoffs >> 4] & 0xff) << 8); break;
		}
		if (code == 0)
			continue;
		int flipx = spritecont & 1, flipy = (spritecont >> 1) & 1;
		int curx = sext12(x + scrollx), cury = sext12(y + scrolly);
		if (s_r_flipscreen) {
			curx = 320 - curx - (int)zx;
			cury = 256 - cury - (int)zy;
			flipx = !flipx;
			flipy = !flipy;
		}
		sp->code = code;
		sp->color = color;
		if (s_spr_gran == 64)
			sp->color /= 4;
		sp->flipx = flipx;
		sp->flipy = flipy;
		sp->x = curx;
		sp->y = cury;
		sp->zoomx = (int)(zx << 12);
		sp->zoomy = (int)(zy << 12);
		if (primasks || uses_tc360_mixer) {
			sp->primask = uses_tc360_mixer ? (color & 0xc0) >> 6 : primasks[(color & 0xc0) >> 6];
			sp++;
		} else {
			sprite_mixdraw(bm, pitch, sp, 0, 0, VIS_Y0, VIS_Y0 + VIS_H - 1);
		}
	}
	s_list_end = sp;
	if (draw)
		draw_list(bm, pitch, uses_tc360_mixer, VIS_Y0, VIS_Y0 + VIS_H - 1);
}

// ---- palette ----
static void palette_update(void)
{
	const uint16_t *p = s_snap->pal;
	int n = s_m->pal_size ? s_m->pal_size : 4096;
	for (int i = 0; i < n; i++) {
		uint16_t d = p[i];
		int r, g, b;
		switch (s_m->pal) {
		case PAL_RGBX444:
			r = (d >> 12) & 15; g = (d >> 8) & 15; b = (d >> 4) & 15;
			r |= r << 4; g |= g << 4; b |= b << 4;
			break;
		case PAL_XRGB555:
			r = (d >> 10) & 31; g = (d >> 5) & 31; b = d & 31;
			r = (r << 3) | (r >> 2); g = (g << 3) | (g >> 2); b = (b << 3) | (b >> 2);
			break;
		case PAL_RGB5X:
			r = ((d >> 11) & 0x1e) | ((d >> 3) & 1);
			g = ((d >> 7) & 0x1e) | ((d >> 2) & 1);
			b = ((d >> 3) & 0x1e) | ((d >> 1) & 1);
			r = (r << 3) | (r >> 2); g = (g << 3) | (g >> 2); b = (b << 3) | (b >> 2);
			break;
		default:   // TC0110PCR with color_xbgr555
			r = d & 31; g = (d >> 5) & 31; b = (d >> 10) & 31;
			r = (r << 3) | (r >> 2); g = (g << 3) | (g >> 2); b = (b << 3) | (b >> 2);
			break;
		}
		rbx_video.palette[i] = (uint32_t)(r << 16 | g << 8 | b);
	}
}

// ---- screen updates (MAME taito_f2_v.cpp) ----
static void read_tc360(int tilepri_layers[3])
{
	const tc0360pri_t *p = &s_snap->pri;
	int l0 = tilepri_layers[0], l1 = tilepri_layers[1], l2 = tilepri_layers[2];
	s_tilepri[l0] = tc0360pri_r(p, 5) & 0x0f;
	s_tilepri[l1] = tc0360pri_r(p, 5) >> 4;
	s_tilepri[l2] = tc0360pri_r(p, 4) >> 4;
}

static void read_spritepri(void)
{
	const tc0360pri_t *p = &s_snap->pri;
	s_spritepri[0] = tc0360pri_r(p, 6) & 0x0f;
	s_spritepri[1] = tc0360pri_r(p, 6) >> 4;
	s_spritepri[2] = tc0360pri_r(p, 7) & 0x0f;
	s_spritepri[3] = tc0360pri_r(p, 7) >> 4;
	s_blendmode = tc0360pri_r(p, 0) & 0xc0;
}

// TC0480SCP screens (metalb, deadconx): drawn in bands of lines, by the threads that help the render thread too
// (f2_help). Each line is its layers then its sprites, as the whole screen is: the bands are the same picture
#define BANDS 8
static int s_job_next, s_job_left, s_job_active, s_helpers;
static uint8_t s_band_layer[5];
static int s_band_mixer;

static void draw_band(int band)
{
	const int pitch = rbx_video.pitch, r0 = VIS_H * band / BANDS, n = VIS_H * (band + 1) / BANDS - r0;
	uint16_t *bm = rbx_video.bitmap;
	for (int y = r0; y < r0 + n; y++)
		memset(bm + y * pitch, 0, SCREEN_W * 2);
	memset(s_pribuf + (size_t)pitch * r0, 0, (size_t)pitch * n);
	for (int i = 0; i < 5; i++)
		tc0480scp_draw(s_snap_scp, s_band_layer[i], bm + r0 * pitch, s_pribuf + (size_t)pitch * r0, pitch, VIS_Y0 + r0, n,
		               (uint8_t)(1 << i));
	draw_list(bm, pitch, s_band_mixer, VIS_Y0 + r0, VIS_Y0 + r0 + n - 1);
}

// bands until none is left (one: a single band); the number drawn
static int draw_bands(int one)
{
	int n = 0;
	for (;;) {
		const int b = __atomic_fetch_add(&s_job_next, 1, __ATOMIC_ACQ_REL);
		if (b >= BANDS)
			return n;
		draw_band(b);
		__atomic_sub_fetch(&s_job_left, 1, __ATOMIC_ACQ_REL);
		n++;
		if (one)
			return n;
	}
}

static int f2_help(int helper)
{
	if (!__atomic_load_n(&s_job_active, __ATOMIC_SEQ_CST))
		return 0;
	__atomic_add_fetch(&s_helpers, 1, __ATOMIC_SEQ_CST);
	int did = 0;
	if (__atomic_load_n(&s_job_active, __ATOMIC_SEQ_CST))
		did = draw_bands(helper & 0x100) > 0;
	__atomic_sub_fetch(&s_helpers, 1, __ATOMIC_SEQ_CST);
	return did;
}

static void render_bands(const uint8_t layer[5], int mixer)
{
	memcpy(s_band_layer, layer, 5);
	s_band_mixer = mixer;
	s_job_next = 0;
	s_job_left = BANDS;
	__atomic_store_n(&s_job_active, 1, __ATOMIC_SEQ_CST);
	draw_bands(0);
	while (__atomic_load_n(&s_job_left, __ATOMIC_ACQUIRE)) {
	}
	__atomic_store_n(&s_job_active, 0, __ATOMIC_SEQ_CST);
	while (__atomic_load_n(&s_helpers, __ATOMIC_SEQ_CST)) {   // (none may still take a band of the next frame)
	}
}

static void f2_render(void)
{
	uint16_t *bm = rbx_video.bitmap;
	int pitch = rbx_video.pitch;
	palette_update();
	if (s_m->screen == SCR_METALB || s_m->screen == SCR_DEADCONX)
		goto scp;   // (cleared by the bands)
	for (int y = 0; y < VIS_H; y++)
		memset(bm + y * pitch, 0, SCREEN_W * 2);
	memset(s_pribuf, 0, (size_t)pitch * VIS_H);   // indexed with the bitmap pitch
scp:;
	tc0100scn_t *scn = s_snap_scn[0];
	switch (s_m->screen) {
	case SCR_SSI:
		draw_sprites(bm, pitch, NULL, 0, 1);
		break;
	case SCR_DEFAULT:
	case SCR_YESNOJ: {
		int bl = tc0100scn_bottomlayer(scn);
		if (s_m->screen == SCR_YESNOJ)
			draw_sprites(bm, pitch, NULL, 0, 1);
		tc0100scn_draw(scn, bl, bm, s_pribuf, pitch, VIS_Y0, VIS_H, 0);
		tc0100scn_draw(scn, bl ^ 1, bm, s_pribuf, pitch, VIS_Y0, VIS_H, 0);
		if (s_m->screen == SCR_DEFAULT)
			draw_sprites(bm, pitch, NULL, 0, 1);
		tc0100scn_draw(scn, 2, bm, s_pribuf, pitch, VIS_Y0, VIS_H, 0);
		break;
	}
	case SCR_PRI: {
		int layer[3];
		layer[0] = tc0100scn_bottomlayer(scn);
		layer[1] = layer[0] ^ 1;
		layer[2] = 2;
		read_tc360(layer);
		read_spritepri();
		tc0100scn_draw(scn, layer[0], bm, s_pribuf, pitch, VIS_Y0, VIS_H, 1);
		tc0100scn_draw(scn, layer[1], bm, s_pribuf, pitch, VIS_Y0, VIS_H, 2);
		tc0100scn_draw(scn, layer[2], bm, s_pribuf, pitch, VIS_Y0, VIS_H, 4);
		draw_sprites(bm, pitch, NULL, 1, 1);
		break;
	}
	case SCR_PRI_ROZ: {
		const tc0360pri_t *p = &s_snap->pri;
		int roz_base_color = (tc0360pri_r(p, 1) & 0x3f) << 2;
		int rozpri = (tc0360pri_r(p, 1) & 0xc0) >> 6;
		rozpri = (tc0360pri_r(p, 8 + rozpri / 2) >> 4 * (rozpri & 1)) & 0x0f;
		int layer[3], tilepri[3];
		layer[0] = tc0100scn_bottomlayer(scn);
		layer[1] = layer[0] ^ 1;
		layer[2] = 2;
		tilepri[layer[0]] = tc0360pri_r(p, 5) & 0x0f;
		tilepri[layer[1]] = tc0360pri_r(p, 5) >> 4;
		tilepri[layer[2]] = tc0360pri_r(p, 4) >> 4;
		read_spritepri();
		int drawn = 0;
		for (int i = 0; i < 16; i++) {
			if (rozpri == i) {
				tc0280grd_draw(s_snap_grd, bm, s_pribuf, pitch, SCREEN_W, VIS_Y0, VIS_H, s_m->pivot_x, s_m->pivot_y,
				               roz_base_color, (uint8_t)(1 << drawn));
				s_tilepri[drawn] = (uint8_t)i;
				drawn++;
			}
			for (int j = 0; j < 3; j++)
				if (tilepri[layer[j]] == i) {
					tc0100scn_draw(scn, layer[j], bm, s_pribuf, pitch, VIS_Y0, VIS_H, (uint8_t)(1 << drawn));
					s_tilepri[drawn] = (uint8_t)i;
					drawn++;
				}
		}
		draw_sprites(bm, pitch, NULL, 1, 1);
		break;
	}
	case SCR_THUNDFOX: {
		const tc0360pri_t *p = &s_snap->pri;
		int tilepri[2][3], spritepri[4], layer[2][3], drawn[2];
		layer[0][0] = tc0100scn_bottomlayer(s_snap_scn[0]);
		layer[0][1] = layer[0][0] ^ 1;
		layer[0][2] = 2;
		tilepri[0][layer[0][0]] = tc0360pri_r(p, 5) & 0x0f;
		tilepri[0][layer[0][1]] = tc0360pri_r(p, 5) >> 4;
		tilepri[0][layer[0][2]] = tc0360pri_r(p, 4) >> 4;
		layer[1][0] = tc0100scn_bottomlayer(s_snap_scn[1]);
		layer[1][1] = layer[1][0] ^ 1;
		layer[1][2] = 2;
		tilepri[1][layer[1][0]] = tc0360pri_r(p, 9) & 0x0f;
		tilepri[1][layer[1][1]] = tc0360pri_r(p, 9) >> 4;
		tilepri[1][layer[1][2]] = tc0360pri_r(p, 8) >> 4;
		spritepri[0] = tc0360pri_r(p, 6) & 0x0f;
		spritepri[1] = tc0360pri_r(p, 6) >> 4;
		spritepri[2] = tc0360pri_r(p, 7) & 0x0f;
		spritepri[3] = tc0360pri_r(p, 7) >> 4;
		drawn[0] = drawn[1] = 0;
		while (drawn[0] < 3 && drawn[1] < 3) {
			int pick = tilepri[0][drawn[0]] < tilepri[1][drawn[1]] ? 0 : 1;
			tc0100scn_draw(s_snap_scn[pick], layer[pick][drawn[pick]], bm, s_pribuf, pitch, VIS_Y0, VIS_H,
			               (uint8_t)(1 << (drawn[pick] + 3 * pick)));
			drawn[pick]++;
		}
		while (drawn[0] < 3) {
			tc0100scn_draw(s_snap_scn[0], layer[0][drawn[0]], bm, s_pribuf, pitch, VIS_Y0, VIS_H, (uint8_t)(1 << drawn[0]));
			drawn[0]++;
		}
		while (drawn[1] < 3) {
			tc0100scn_draw(s_snap_scn[1], layer[1][drawn[1]], bm, s_pribuf, pitch, VIS_Y0, VIS_H,
			               (uint8_t)(1 << (drawn[1] + 3)));
			drawn[1]++;
		}
		uint64_t primasks[4] = { 0, 0, 0, 0 };
		for (int i = 0; i < 4; i++) {
			if (spritepri[i] < tilepri[0][0]) primasks[i] |= 0xaaaaaaaaaaaaaaaaull;
			if (spritepri[i] < tilepri[0][1]) primasks[i] |= 0xccccccccccccccccull;
			if (spritepri[i] < tilepri[0][2]) primasks[i] |= 0xf0f0f0f0f0f0f0f0ull;
			if (spritepri[i] < tilepri[1][0]) primasks[i] |= 0xff00ff00ff00ff00ull;
			if (spritepri[i] < tilepri[1][1]) primasks[i] |= 0xffff0000ffff0000ull;
			if (spritepri[i] < tilepri[1][2]) primasks[i] |= 0xffffffff00000000ull;
		}
		draw_sprites(bm, pitch, primasks, 0, 1);
		break;
	}
	case SCR_METALB:
	case SCR_DEADCONX: {
		const tc0360pri_t *p = &s_snap->pri;
		const tc0480scp_t *scp = s_snap_scp;
		int priority = tc0480scp_bg_priority(scp);
		uint8_t layer[5];
		layer[0] = (uint8_t)((priority & 0xf000) >> 12);
		layer[1] = (uint8_t)((priority & 0x0f00) >> 8);
		layer[2] = (uint8_t)((priority & 0x00f0) >> 4);
		layer[3] = (uint8_t)(priority & 0x000f);
		layer[4] = 4;
		if (s_m->screen == SCR_METALB) {
			uint8_t invlayer[4];
			invlayer[layer[0]] = 0;
			invlayer[layer[1]] = 1;
			invlayer[layer[2]] = 2;
			invlayer[layer[3]] = 3;
			s_tilepri[invlayer[0]] = tc0360pri_r(p, 4) & 0x0f;
			s_tilepri[invlayer[1]] = tc0360pri_r(p, 4) >> 4;
			s_tilepri[invlayer[2]] = tc0360pri_r(p, 5) & 0x0f;
			s_tilepri[invlayer[3]] = tc0360pri_r(p, 5) >> 4;
			s_tilepri[4] = tc0360pri_r(p, 9) & 0x0f;
			read_spritepri();
			draw_sprites(bm, pitch, NULL, 1, 0);
			render_bands(layer, 1);
		} else {
			uint8_t tilepri[5], spritepri[4];
			tilepri[0] = tc0360pri_r(p, 4) >> 4;
			tilepri[1] = tc0360pri_r(p, 5) & 0x0f;
			tilepri[2] = tc0360pri_r(p, 5) >> 4;
			tilepri[3] = tc0360pri_r(p, 4) & 0x0f;
			tilepri[layer[4]] = tc0360pri_r(p, 9) >> 4;
			spritepri[0] = tc0360pri_r(p, 6) & 0x0f;
			spritepri[1] = tc0360pri_r(p, 6) >> 4;
			spritepri[2] = tc0360pri_r(p, 7) & 0x0f;
			spritepri[3] = tc0360pri_r(p, 7) >> 4;
			uint64_t primasks[4] = { 0, 0, 0, 0 };
			for (int i = 0; i < 4; i++) {
				if (spritepri[i] < tilepri[layer[0]]) primasks[i] |= 0xaaaaaaaaaaaaaaaaull;
				if (spritepri[i] < tilepri[layer[1]]) primasks[i] |= 0xccccccccccccccccull;
				if (spritepri[i] < tilepri[layer[2]]) primasks[i] |= 0xf0f0f0f0f0f0f0f0ull;
				if (spritepri[i] < tilepri[layer[3]]) primasks[i] |= 0xff00ff00ff00ff00ull;
				if (spritepri[i] < tilepri[layer[4]]) primasks[i] |= 0xffff0000ffff0000ull;
			}
			draw_sprites(bm, pitch, primasks, 0, 0);
			render_bands(layer, 0);
		}
		break;
	}
	}
}

// main side at vblank: wait for the previous drawing, buffer sprites as the screen update does, snapshot
static void vblank_capture(void)
{
	rbx_video_wait_idle();
	handle_sprite_buffering();
	memcpy(s_snap->spr, s_spr_buf, sizeof(s_snap->spr));
	memcpy(s_snap->ext, s_ext, sizeof(s_snap->ext));
	memcpy(s_snap->bank, s_spritebank, sizeof(s_snap->bank));
	s_snap->disabled = s_sprites_disabled;
	s_snap->area = s_sprites_active_area;
	s_snap->msx = s_master_scrollx;
	s_snap->msy = s_master_scrolly;
	s_snap->pri = s_pri;
	memcpy(s_snap->pal, s_pal, s_pal_bytes < sizeof(s_snap->pal) ? s_pal_bytes : sizeof(s_snap->pal));
	for (int i = 0; i < 2; i++)
		if (s_scn[i])
			tc0100scn_copy(s_snap_scn[i], s_scn[i]);
	if (s_grd)
		tc0280grd_copy(s_snap_grd, s_grd);
	if (s_scp)
		tc0480scp_copy(s_snap_scp, s_scp);
	rbx_video_frame_done();
}

// ---- sprite graphics ----
static int decode_sprites(void)
{
	uint32_t size;
	uint8_t *g = rbx_region("sprites", &size);
	if (!g)
		return -1;
	if (s_m->sprgfx == SPR_MJNQUEST)   // bytes of each word reversed and nibble swapped (MAME init_mjnquest)
		for (uint32_t i = 0; i + 1 < size; i += 2) {
			uint8_t t = g[i];
			g[i] = (uint8_t)((g[i + 1] >> 4) | (g[i + 1] << 4));
			g[i + 1] = (uint8_t)((t >> 4) | (t << 4));
		}
	uint32_t n = size / 128;
	uint32_t hsize = 0;
	uint8_t *hi = NULL;
	if (s_m->sprgfx == SPR_FINALB) {
		hi = rbx_region("sprites_hi", &hsize);
		if (!hi)
			return -1;
		if (hsize / 64 < n)
			n = hsize / 64;
	}
	if (!n)
		return -1;
	s_spr_tiles = (uint8_t *)malloc((size_t)n * 257);
	if (!s_spr_tiles)
		return -1;
	s_spr_used = s_spr_tiles + (size_t)n * 256;
	for (uint32_t i = 0; i < n; i++) {
		uint8_t *d = s_spr_tiles + (size_t)i * 256, any = 0;
		for (int j = 0; j < 256; j++) {   // gfx_16x16x4_packed_lsb
			uint8_t b = g[i * 128 + (j >> 1)];
			uint8_t p = (j & 1) ? (b >> 4) : (b & 15);
			if (hi) {   // finalb: top two bits from layout_6bpp_hi
				int y = j >> 4, x = j & 15;
				uint8_t hb = hi[i * 64 + y * 4 + (x >> 2)];
				p = (uint8_t)((p & 0xf) | ((((hb >> ((x & 3) * 2)) & 3) << 4) & 0x30));
			}
			d[j] = p;
			any |= p;
		}
		s_spr_used[i] = any != 0;
	}
	s_spr_ntiles = n;
	s_spr_gran = hi ? 64 : 16;
	s_spr_colors = hi ? 4096 / 64 : 256;
	return 0;
}

// ---- init / reset ----
static const machine_t *find_machine(const char *name)
{
	for (size_t i = 0; i < sizeof(s_machines) / sizeof(s_machines[0]); i++)
		if (!strcmp(s_machines[i].name, name))
			return &s_machines[i];
	return NULL;
}

static void map_main(void)
{
	m68k_bus_reset_map();
	int ram = 0;
	for (const range_t *r = s_m->map; r->dev; r++) {
		uint32_t len = r->end - r->start + 1;
		switch (r->dev) {
		case D_ROM: m68k_bus_map(r->start, r->end, s_rom, s_rom_size, 1, 0); break;
		case D_EXTRA: if (s_extra) m68k_bus_map(r->start, r->end, s_extra, s_extra_size, 1, 0); break;
		case D_RAM: m68k_bus_map(r->start, r->end, s_ram[ram], s_ram_size[ram], 1, 1); ram++; break;
		case D_PAL: m68k_bus_map(r->start, r->end, s_pal, s_pal_bytes, 1, 1); break;
		case D_SPR: m68k_bus_map(r->start, r->end, s_spr, sizeof(s_spr), 1, 1); break;
		case D_SCN: m68k_bus_map(r->start, r->end, s_scn[0]->ram, len, 1, 1); break;
		case D_SCN2: m68k_bus_map(r->start, r->end, s_scn[1]->ram, len, 1, 1); break;
		case D_GRD: m68k_bus_map(r->start, r->end, s_grd->ram, TC0280GRD_RAM_BYTES, 1, 1); break;
		case D_SCP: m68k_bus_map(r->start, r->end, s_scp->ram, TC0480SCP_RAM_BYTES, 1, 1); break;
		case D_EXT: m68k_bus_map(r->start, r->end, s_ext, sizeof(s_ext), 0, 1); break;
		}
	}
	m68k_bus.read8 = main_read8;
	m68k_bus.read16 = main_read16;
	m68k_bus.write8 = main_write8;
	m68k_bus.write16 = main_write16;
}

static void f2_reset(void)
{
	for (int i = 0; i < s_nram; i++)
		memset(s_ram[i], 0, s_ram_size[i]);
	memset(s_spr, 0, sizeof(s_spr));
	memset(s_spr_buf, 0, sizeof(s_spr_buf));
	memset(s_spr_delayed, 0, sizeof(s_spr_delayed));
	memset(s_ext, 0, sizeof(s_ext));
	memset(s_io_regs, 0, sizeof(s_io_regs));
	memset(s_te_latch, 0, sizeof(s_te_latch));
	memset(s_te_dir, 0xff, sizeof(s_te_dir));   // soft mode: all ports input after reset
	s_pcr_addr = 0;
	for (int i = 0; i < 2; i++)
		if (s_scn[i])
			tc0100scn_reset(s_scn[i]);
	if (s_grd)
		tc0280grd_reset(s_grd);
	if (s_scp)
		tc0480scp_reset(s_scp);
	tc0360pri_reset(&s_pri);
	for (int i = 0; i < 8; i++)
		s_spritebank[i] = s_spritebank_buffered[i] = (uint16_t)(0x400 * i);
	s_sprites_disabled = 1;
	s_sprites_active_area = 0;
	s_master_scrollx = s_master_scrolly = 0;
	s_prepare_sprites = 0;
	s_r_flipscreen = 0;
	s_mj_input = 0;
	s_mj_gfxbank = 0;
	if (s_scn[0])
		s_scn[0]->code_bank = 0;
	s_dial[0] = s_dial[1] = s_dial_last[0] = s_dial_last[1] = 0;
	if (s_m->cchip)
		cchip_reset();
	s_cc_frac = 0;
	m68k_bus_irq_reset();
	m68k_pulse_reset();
	if (s_m->snd == SND_DRIVEOUT)
		dz_reset();
	else
		tsnd_reset();
	s_cpu_frac = 0;
}

static void f2_exit(void);

static tc0100scn_t *make_scn(const char *region, int second)
{
	uint32_t size;
	uint8_t *g = rbx_region(region, &size);
	tc0100scn_t *c = (tc0100scn_t *)calloc(1, sizeof(tc0100scn_t));
	if (!c || tc0100scn_init(c, g, g ? size : 0, s_m->scn_layout)) {
		free(c);
		return NULL;
	}
	c->x_offset = s_m->scn_x;
	c->y_offset = s_m->scn_y;
	c->flip_xoffs = s_m->scn_flipx;
	c->flip_text_xoffs = s_m->scn_fliptx;
	c->flip_text_yoffs = s_m->scn_fliptx_y;
	if (second) {
		c->multiscrn_xoffs = TC0100SCN_SINGLE_VDU;
		c->multiscrn_hack = 1;
	}
	if (s_m->sprgfx == SPR_MJNQUEST)
		c->code_mask = 0x7fff;
	return c;
}

static int f2_init(const rbx_game *g)
{
	if (!s_ext_hp && !(s_ext_hp = calloc(1, sizeof(*s_ext_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	if (!s_list_hp && !(s_list_hp = calloc(1, sizeof(*s_list_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	if (!s_spr_hp && !(s_spr_hp = calloc(1, sizeof(*s_spr_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	s_m = find_machine(g->machine);
	if (!s_m) {
		rbx_fail("%s: machine %s is not emulated", g->set, g->machine);
		return -1;
	}
	s_rom = rbx_region("maincpu", &s_rom_size);
	if (!s_rom) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	m68k_bus_swap16(s_rom, s_rom_size);
	s_extra = rbx_region("extra", &s_extra_size);
	if (s_extra)
		m68k_bus_swap16(s_extra, s_extra_size);
	// work RAM blocks and palette
	s_nram = 0;
	s_pal_bytes = 0;
	int uses_scn = 0, uses_scn2 = 0, uses_grd = 0, uses_scp = 0;
	for (const range_t *r = s_m->map; r->dev; r++) {
		uint32_t len = r->end - r->start + 1;
		if (r->dev == D_RAM && s_nram < MAX_RAM) {
			s_ram[s_nram] = (uint8_t *)calloc(1, len);
			s_ram_size[s_nram++] = len;
		}
		if (r->dev == D_PAL)
			s_pal_bytes = len;
		uses_scn |= r->dev == D_SCN;
		uses_scn2 |= r->dev == D_SCN2;
		uses_grd |= r->dev == D_GRD;
		uses_scp |= r->dev == D_SCP;
	}
	if (s_m->pal == PAL_PCR)
		s_pal_bytes = 0x2000;
	s_pal = (uint16_t *)calloc(1, 0x4000);
	s_snap = calloc(1, sizeof(*s_snap));
	s_pribuf = (uint8_t *)calloc(SCREEN_W, VIS_H);
	if (!s_pal || !s_snap || !s_pribuf) {
		f2_exit();
		return -1;
	}
	for (int i = 0; i < s_nram; i++)
		if (!s_ram[i]) {
			f2_exit();
			return -1;
		}
	// video chips (the snapshot copies share the decoded tiles)
	if (uses_scn) {
		s_scn[0] = make_scn("tc0100scn_1", 0);
		s_snap_scn[0] = (tc0100scn_t *)malloc(sizeof(tc0100scn_t));
		if (!s_scn[0] || !s_snap_scn[0]) { f2_exit(); return -1; }
		*s_snap_scn[0] = *s_scn[0];
	}
	if (uses_scn2) {
		s_scn[1] = make_scn("tc0100scn_2", 1);
		s_snap_scn[1] = (tc0100scn_t *)malloc(sizeof(tc0100scn_t));
		if (!s_scn[1] || !s_snap_scn[1]) { f2_exit(); return -1; }
		*s_snap_scn[1] = *s_scn[1];
	}
	if (uses_grd) {
		uint32_t size;
		uint8_t *gg = rbx_region(s_m->roz == ROZ_GRW ? "tc0430grw" : "tc0280grd", &size);
		s_grd = (tc0280grd_t *)calloc(1, sizeof(tc0280grd_t));
		s_snap_grd = (tc0280grd_t *)malloc(sizeof(tc0280grd_t));
		if (!gg || !s_grd || !s_snap_grd || tc0280grd_init(s_grd, gg, size, s_m->roz == ROZ_GRW ? 1 : 2)) {
			f2_exit();
			return -1;
		}
		*s_snap_grd = *s_grd;
	}
	if (uses_scp) {
		uint32_t size;
		uint8_t *gg = rbx_region("tc0480scp", &size);
		s_scp = (tc0480scp_t *)calloc(1, sizeof(tc0480scp_t));
		s_snap_scp = (tc0480scp_t *)malloc(sizeof(tc0480scp_t));
		if (!gg || !s_scp || !s_snap_scp || tc0480scp_init(s_scp, gg, size)) {
			f2_exit();
			return -1;
		}
		s_scp->x_offset = s_m->scp_x;
		s_scp->y_offset = s_m->scp_y;
		s_scp->text_xoffs = s_m->scp_tx;
		s_scp->flip_xoffs = s_m->scp_fx;
		s_scp->col_base = s_m->scp_colbase;
		*s_snap_scp = *s_scp;
	}
	if (decode_sprites()) {
		rbx_fail("%s: sprite graphics missing", g->set);
		f2_exit();
		return -1;
	}
	if (s_m->cchip) {   // 12 MHz; the ports don't appear to hook up to anything
		cchip_config cc;
		memset(&cc, 0, sizeof(cc));
		cc.clock = 12000000;
		if (cchip_init(&cc)) {
			rbx_fail("%s: C-Chip ROMs missing", g->set);
			f2_exit();
			return -1;
		}
	}
	// inputs
	static const char *const io_tags[8] = { "DSWA", "DSWB", "IN0", "IN1", 0, 0, 0, "IN2" };
	for (int i = 0; i < 8; i++)
		s_port_io[i] = io_tags[i] ? rbx_port_find(io_tags[i]) : -1;
	static const char *const te_tags[9] = { "DSWA", "DSWB", "IN2", 0, 0, "IN0", "IN1", "IN3", "IN4" };
	static const char *const te_ninjak[9] = { "DSWA", "DSWB", "IN0", "IN1", "IN3", "IN4", "IN2", 0, 0 };
	for (int i = 0; i < 9; i++) {
		const char *t = s_m->io == IO_TE7750_NINJAK ? te_ninjak[i] : te_tags[i];
		s_te_port[i] = t ? rbx_port_find(t) : -1;
	}
	for (int i = 0; s_m->map[i].dev && i < 32; i++)
		s_range_port[i] = s_m->map[i].tag ? rbx_port_find(s_m->map[i].tag) : -1;
	static const char *const keys[5] = { "KEY0", "KEY1", "KEY2", "KEY3", "KEY4" };
	for (int i = 0; i < 5; i++)
		s_port_key[i] = rbx_port_find(keys[i]);
	s_port_mjin[0] = rbx_port_find("IN0");
	s_port_mjin[1] = rbx_port_find("IN1");
	s_port_mjdsw[0] = rbx_port_find("DSWA");
	s_port_mjdsw[1] = rbx_port_find("DSWB");
	s_port_dial[0] = rbx_port_find("PADDLE1");
	s_port_dial[1] = rbx_port_find("PADDLE2");
	// sound (MAME routes: YM2610 SSG 0.75 to both sides, FM 1.0; cameltrya YM2203 SSG 0.20, FM 0.60, OKI 0.10)
	tsnd_config sc;
	memset(&sc, 0, sizeof(sc));
	sc.frame_rate_mhz = 60000;
	sc.lines = LINES;
	if (s_m->snd == SND_YM2203) {
		sc.map = TSND_MAP_9000;
		sc.rom_flat = 1;
		sc.z80_clock = 6000000;
		sc.ym.type = RBX_YM2203;
		sc.ym.clock = 3000000;
		sc.ym.gain[0] = sc.ym.gain[1] = sc.ym.gain[2] = 51;
		sc.ym.gain[3] = 154;
		sc.oki_clock = 1056000;
		sc.oki_pin7_high = 1;
		sc.oki_gain = 26;
	} else {
		sc.map = TSND_MAP_E000;
		sc.bank_mask = 7;
		sc.z80_clock = 4000000;
		sc.ym.type = RBX_YM2610;
		sc.ym.clock = 8000000;
		sc.ym.gain[0] = 192;
		sc.ym.gain[1] = sc.ym.gain[2] = 128;   // (MAME's FM left and right at 1.0 each, averaged on one speaker)
	}
	if (s_m->snd == SND_DRIVEOUT ? dz_init() : tsnd_init(&sc)) {
		rbx_fail("%s: sound init failed", g->set);
		f2_exit();
		return -1;
	}
	if (rbx_video_setup(SCREEN_W, VIS_H, s_m->pal_size ? s_m->pal_size : 4096)) {
		f2_exit();
		return -1;
	}
	if (rbx_video.pitch != SCREEN_W) {   // the priority buffer is indexed with the bitmap pitch
		free(s_pribuf);
		s_pribuf = (uint8_t *)calloc((size_t)rbx_video.pitch, VIS_H);
		if (!s_pribuf) {
			f2_exit();
			return -1;
		}
	}
	m68k_init();
	m68k_set_cpu_type(M68K_CPU_TYPE_68000);
	map_main();
	f2_reset();
	return 0;
}

static void f2_exit(void)
{
	for (int i = 0; i < 2; i++) {
		if (s_scn[i])
			tc0100scn_exit(s_scn[i]);
		free(s_scn[i]);
		free(s_snap_scn[i]);
		s_scn[i] = s_snap_scn[i] = NULL;
	}
	if (s_grd)
		tc0280grd_exit(s_grd);
	free(s_grd);
	free(s_snap_grd);
	s_grd = s_snap_grd = NULL;
	if (s_scp)
		tc0480scp_exit(s_scp);
	free(s_scp);
	free(s_snap_scp);
	s_scp = s_snap_scp = NULL;
	for (int i = 0; i < MAX_RAM; i++) {
		free(s_ram[i]);
		s_ram[i] = NULL;
	}
	s_nram = 0;
	free(s_pal);
	s_pal = NULL;
	free(s_snap);
	s_snap = NULL;
	free(s_pribuf);
	s_pribuf = NULL;
	free(s_spr_tiles);
	s_spr_tiles = s_spr_used = NULL;
	if (s_m && s_m->snd == SND_DRIVEOUT) {
		free(s_dz_oki);
		s_dz_oki = NULL;
	} else
		tsnd_exit();
	free(s_ext_hp);
	s_ext_hp = NULL;
	free(s_list_hp);
	s_list_hp = NULL;
	free(s_spr_hp);
	s_spr_hp = NULL;
}

// ---- frame ----
static void f2_frame(void)
{
	for (int p = 0; p < 2; p++)
		if (s_port_dial[p] >= 0)
			s_dial[p] = (s_dial[p] + rbx_analog_dpad(p + 1, RBX_IPT_DIAL, 10)) & 0xff;   // (d-pad: about MAME's key deltas)
	for (int line = 0; line < LINES; line++) {
		int cycles = rbx_line_cycles(&s_cpu_frac, s_m->cpu_clock ? s_m->cpu_clock : 12000000, 60000, LINES);
		if (line == VBLANK_LINE) {
			vblank_capture();
			screen_vblank();
			m68k_bus_hold_irq(5);
			if (s_m->cchip) {   // megab_interrupt: C-Chip INT pulse (cleared by a zero-time timer)
				cchip_ext_interrupt(1);
				cchip_ext_interrupt(0);
			}
			int done = m68k_execute(500);   // MAME trigger_int6: 500 cycles after the vblank interrupt
			m68k_bus_hold_irq(6);
			if (cycles - done > 0)
				m68k_execute(cycles - done);
		} else {
			m68k_execute(cycles);
		}
		if (s_m->cchip)
			cchip_run_clocks(&s_cc_frac, 60000, LINES);
		if (s_m->snd == SND_DRIVEOUT)
			dz_line();
		else
			tsnd_line();
	}
	if (s_m->snd != SND_DRIVEOUT)
		tsnd_frame_end();
}

// ---- save states ----
static int f2_state(void *ctx, int load)
{
	uint8_t cpu[1024];
	uint32_t csize = m68k_context_size();
	if (csize > sizeof(cpu))
		return -1;
	if (!load)
		m68k_get_context(cpu);
	uint32_t irq = m68k_bus_irq_pending();
	int misc[16] = { s_sprites_disabled, s_sprites_active_area, s_master_scrollx, s_master_scrolly, s_prepare_sprites,
	                 s_r_flipscreen, s_pcr_addr, s_mj_input, s_mj_gfxbank, s_dial[0], s_dial[1], s_dial_last[0],
	                 s_dial_last[1], 0, 0, 0 };
	uint64_t fr = s_cpu_frac;
	int rc = 0;
	char tag[8] = "ram0";
	rc |= rbx_state_io(ctx, "m68k", cpu, csize);
	rc |= rbx_state_io(ctx, "irq", &irq, 4);
	for (int i = 0; i < s_nram; i++) {
		tag[3] = (char)('0' + i);
		rc |= rbx_state_io(ctx, tag, s_ram[i], s_ram_size[i]);
	}
	rc |= rbx_state_io(ctx, "pal", s_pal, 0x4000);
	rc |= rbx_state_io(ctx, "spr", s_spr, sizeof(s_spr));
	rc |= rbx_state_io(ctx, "sprbuf", s_spr_buf, sizeof(s_spr_buf));
	rc |= rbx_state_io(ctx, "sprdel", s_spr_delayed, sizeof(s_spr_delayed));
	rc |= rbx_state_io(ctx, "sprext", s_ext, sizeof(s_ext));
	rc |= rbx_state_io(ctx, "sprbank", s_spritebank, sizeof(s_spritebank));
	rc |= rbx_state_io(ctx, "sprbankb", s_spritebank_buffered, sizeof(s_spritebank_buffered));
	for (int i = 0; i < 2; i++)
		if (s_scn[i]) {
			rc |= rbx_state_io(ctx, i ? "scn1" : "scn0", s_scn[i]->ram, sizeof(s_scn[i]->ram));
			rc |= rbx_state_io(ctx, i ? "scn1c" : "scn0c", s_scn[i]->ctrl, sizeof(s_scn[i]->ctrl));
		}
	if (s_grd) {
		rc |= rbx_state_io(ctx, "grd", s_grd->ram, sizeof(s_grd->ram));
		rc |= rbx_state_io(ctx, "grdc", s_grd->ctrl, sizeof(s_grd->ctrl));
	}
	if (s_scp) {
		rc |= rbx_state_io(ctx, "scp", s_scp->ram, sizeof(s_scp->ram));
		rc |= rbx_state_io(ctx, "scpc", s_scp->ctrl, sizeof(s_scp->ctrl));
	}
	rc |= rbx_state_io(ctx, "pri", s_pri.regs, sizeof(s_pri.regs));
	rc |= rbx_state_io(ctx, "io", s_io_regs, sizeof(s_io_regs));
	rc |= rbx_state_io(ctx, "te", s_te_latch, sizeof(s_te_latch));
	rc |= rbx_state_io(ctx, "tedir", s_te_dir, sizeof(s_te_dir));
	rc |= rbx_state_io(ctx, "misc", misc, sizeof(misc));
	rc |= rbx_state_io(ctx, "frac", &fr, sizeof(fr));
	if (s_m->snd == SND_DRIVEOUT) {
		z80 z = s_dz;
		int dm[4] = { s_dz_latch, s_dz_nibble, s_dz_irq, (int)s_doki.bank };
		uint64_t df[2] = { s_dz_frac, s_doki_frac };
		rc |= rbx_state_io(ctx, "dz80", &z, sizeof(z));
		rc |= rbx_state_io(ctx, "dzram", s_dz_ram, sizeof(s_dz_ram));
		rc |= rbx_state_io(ctx, "dzmisc", dm, sizeof(dm));
		rc |= rbx_state_io(ctx, "dzfrac", df, sizeof(df));
		rc |= rbx_state_io(ctx, "doki", s_doki.v, sizeof(s_doki.v) + sizeof(int));
		if (!rc && load) {
			z.read_byte = dz_read;
			z.write_byte = dz_write;
			z.port_in = dz_in;
			z.port_out = dz_out;
			z.userdata = NULL;
			s_dz = z;
			s_dz_latch = dm[0]; s_dz_nibble = dm[1]; s_dz_irq = dm[2]; s_doki.bank = (uint32_t)dm[3];
			s_dz_frac = df[0]; s_doki_frac = df[1];
		}
	} else
		rc |= tsnd_state(ctx, load);
	if (s_m->cchip) {
		uint64_t cf = s_cc_frac;
		rc |= rbx_state_io(ctx, "ccfrac", &cf, sizeof(cf));
		rc |= cchip_state(ctx, load);
		if (!rc && load)
			s_cc_frac = cf;
	}
	if (rc)
		return -1;
	if (load) {
		m68k_set_context(cpu);
		m68k_bus_set_irq_pending(irq);
		s_sprites_disabled = misc[0]; s_sprites_active_area = misc[1]; s_master_scrollx = misc[2];
		s_master_scrolly = misc[3]; s_prepare_sprites = misc[4]; s_r_flipscreen = misc[5];
		s_pcr_addr = (uint16_t)misc[6]; s_mj_input = misc[7]; s_mj_gfxbank = misc[8];
		s_dial[0] = misc[9]; s_dial[1] = misc[10]; s_dial_last[0] = misc[11]; s_dial_last[1] = misc[12];
		if (s_scn[0])
			s_scn[0]->code_bank = s_m->sprgfx == SPR_MJNQUEST ? (uint32_t)s_mj_gfxbank << 15 : 0;
		if (s_scp)
			tc0480scp_post_load(s_scp);
		s_cpu_frac = fr;
	}
	return 0;
}

static int f2_save(void *ctx) { return f2_state(ctx, 0); }
static int f2_load(void *ctx) { return f2_state(ctx, 1); }
static int f2_supported(const char *machine)
{
	const machine_t *m = find_machine(machine);
	return m && !m->hidden;
}

const rbx_driver rbx_drv_taito_f2 = { "Taito F2 System", f2_init, f2_exit, f2_reset, f2_frame, f2_save, f2_load,
                                      f2_supported, f2_render, f2_help };
