/*  Seta Hardware, the machines with X1-012 tilemaps. Ported from MAME src/mame/seta/seta.cpp (BSD-3-Clause,
    copyright-holders: Luca Elia), with the uPD71054C of src/devices/machine/pit8253.cpp (BSD-3-Clause,
    copyright-holders: Brad Oliver, Aaron Giles), the ADC0834 of src/devices/machine/adc083x.cpp (BSD-3-Clause,
    copyright-holders: smf) and the hopper of src/devices/machine/ticket.cpp (BSD-3-Clause, copyright-holders:
    Aaron Giles).
    One driver, a table row per MAME machine: the address map (RAM and ROM on the bus's page table, the rest through one
    O(1) lookup), the screen, the gfx of the layers, the palette, the interrupts and the sound.
    Pens are MAME's: bitmap values are the palette device's pens (indirect ones included), the palette effect of Blandia
    needs them. The frame is captured at vblank and drawn from the copy (render may run on its own thread).
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
#include "../video/x1_012.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define LINES 256      // (set_size(64 * 8, 32 * 8))
#define X1_CLOCK 16000000u
#define X1_RATE (X1_CLOCK / 512)   // (31250 Hz)
#define PIT_CPU_CYCLES 16          // (the PIT counts 16000000 / 2 / 8, the CPUs that have one run at 16 MHz)

// ---- the machine table ----
enum {   // a word of the map
	K_NONE, K_PORT, K_DSW, K_FFFF, K_VREGS, K_ACK, K_SPRY, K_SPRC, K_VC, K_VCR, K_X1, K_PIT, K_OKI, K_OKIBANK, K_GUNW,
	K_GUNR, K_BGFLAG, K_LATCH, K_YM, K_OUTPUTS, K_ACKR,
	K_RTC, K_MUX, K_PAY, K_LED, K_RSPRC, K_RSPRY, K_RSPRCTRL
};
enum { PT_P1, PT_P2, PT_COINS, PT_EXTRA, PT_UNK, PT_INPUT, PT_DSW1A, PT_DSW1B, PT_DSW2A, PT_DSW2B, PT_COIN, PT_DOOR, PT_DSW3, PT_DSW, PT_COUNT };
static const char *const s_port_tags[PT_COUNT] = { "P1", "P2", "COINS", "EXTRA", "UNK", "INPUT", "DSW1-A", "DSW1-B", "DSW2-A",
                                                   "DSW2-B", "COIN", "DOOR", "DSW3", "DSW" };
enum { M_END, M_ROM, M_RAM, M_PAL, M_VRAM0, M_VRAM1, M_SPRCODE };
typedef struct { uint32_t s, e; uint8_t type; uint32_t off, size; } mem_t;   // (M_ROM: off in the region; size: mirror)
typedef struct { uint32_t s, e; uint8_t rk, wk, arg; } io_t;

#define ROM(s, e) { s, e, M_ROM, s, 0 }
#define ROMM(s, e, size) { s, e, M_ROM, s, size }
#define RAM(s, e) { s, e, M_RAM, 0, 0 }
#define RAMM(s, e, size) { s, e, M_RAM, 0, size }
#define PAL(s, e) { s, e, M_PAL, 0, 0 }
#define VRAM(n, s, e) { s, e, (uint8_t)(M_VRAM0 + (n)), 0, 0 }
#define SCODE(s, e) { s, e, M_SPRCODE, 0, 0 }
#define MEND { 0, 0, M_END, 0, 0 }

#define IO_R(s, e, k, a) { s, e, k, K_NONE, a }
#define IO_W(s, e, k, a) { s, e, K_NONE, k, a }
#define IO_RW(s, e, k, a) { s, e, k, k, a }
#define IO_PORT(s, p) IO_R(s, (s) + 1, K_PORT, p)
#define IO_PLAYERS(s) IO_PORT(s, PT_P1), IO_PORT((s) + 2, PT_P2), IO_PORT((s) + 4, PT_COINS)
#define IO_DSW(s) IO_R(s, (s) + 3, K_DSW, 0)
#define IO_SPRITES(s) IO_RW(s, (s) + 0x5ff, K_SPRY, 0), IO_RW((s) + 0x600, (s) + 0x607, K_SPRC, 0)
#define IO_VC(s0, s1) IO_RW(s0, (s0) + 5, K_VC, 0), IO_RW(s1, (s1) + 5, K_VC, 1)
#define IO_X1(s) IO_RW(s, (s) + 0x3fff, K_X1, 0)
#define IO_PIT(s) IO_RW(s, (s) + 7, K_PIT, 0)
#define IO_VREGS(s) IO_W(s, (s) + 1, K_VREGS, 0)
#define IO_ACK(s, lev) IO_W(s, (s) + 1, K_ACK, lev)
#define IOEND { 0, 0, K_NONE, K_NONE, 0 }

typedef struct { const mem_t *mem; const io_t *io; } map_t;
#define MAP(n) static const map_t n = { n##_mem, n##_io }

// zingzip_map; the others that follow it
#define MEM_COMMON_VRAM VRAM(0, 0x800000, 0x80ffff), VRAM(1, 0x880000, 0x88ffff)
static const mem_t zingzip_map_mem[] = {
	ROM(0, 0x1fffff), RAM(0x200000, 0x21ffff), RAM(0x300000, 0x30ffff), PAL(0x700000, 0x70ffff), MEM_COMMON_VRAM,
	SCODE(0xb00000, 0xb03fff), MEND
};
static const io_t zingzip_map_io[] = {
	IO_PLAYERS(0x400000), IO_VREGS(0x500002), IO_DSW(0x600000), IO_VC(0x900000, 0x980000), IO_SPRITES(0xa00000),
	IO_X1(0xc00000), IOEND
};
MAP(zingzip_map);
static const io_t wrofaero_map_io[] = {
	IO_PLAYERS(0x400000), IO_VREGS(0x500002), IO_DSW(0x600000), IO_VC(0x900000, 0x980000), IO_SPRITES(0xa00000),
	IO_X1(0xc00000), IO_W(0xd00000, 0xd00007, K_PIT, 0), IO_ACK(0xf00000, 4), IOEND
};
static const map_t wrofaero_map = { zingzip_map_mem, wrofaero_map_io };
static const io_t zombraid_map_io[] = {
	IO_PLAYERS(0x400000), IO_VREGS(0x500002), IO_DSW(0x600000), IO_VC(0x900000, 0x980000), IO_SPRITES(0xa00000),
	IO_X1(0xc00000), IO_W(0xf00000, 0xf00001, K_GUNW, 0), IO_R(0xf00002, 0xf00003, K_GUNR, 0), IOEND
};
static const map_t zombraid_map = { zingzip_map_mem, zombraid_map_io };
static const mem_t rezon_map_mem[] = {
	ROM(0, 0x1fffff), RAM(0x200000, 0x20ffff), PAL(0x700000, 0x70ffff), MEM_COMMON_VRAM, SCODE(0xb00000, 0xb03fff), MEND
};
static const io_t rezon_map_io[] = {
	IO_PLAYERS(0x400000), IO_VREGS(0x500002), IO_ACK(0x500004, 2), IO_DSW(0x600000), IO_VC(0x900000, 0x980000),
	IO_SPRITES(0xa00000), IO_X1(0xc00000), IOEND
};
MAP(rezon_map);
static const io_t blandiap_map_io[] = {
	IO_PLAYERS(0x400000), IO_VREGS(0x500002), IO_DSW(0x600000), IO_VC(0x900000, 0x980000), IO_SPRITES(0xa00000),
	IO_X1(0xc00000), IO_ACK(0xe00000, 2), IO_ACK(0xf00000, 4), IOEND
};
static const map_t blandiap_map = { zingzip_map_mem, blandiap_map_io };
static const mem_t blandia_map_mem[] = {
	ROM(0, 0x1fffff), RAM(0x200000, 0x21ffff), RAM(0x300000, 0x30ffff), PAL(0x700000, 0x70ffff),
	VRAM(0, 0xb00000, 0xb0ffff), VRAM(1, 0xb80000, 0xb8ffff), SCODE(0x900000, 0x903fff), MEND
};
static const io_t blandia_map_io[] = {
	IO_PLAYERS(0x400000), IO_VREGS(0x500002), IO_DSW(0x600000), IO_SPRITES(0x800000), IO_VC(0xa00000, 0xa80000),
	IO_X1(0xc00000), IO_ACK(0xe00000, 2), IO_ACK(0xf00000, 4), IOEND
};
MAP(blandia_map);
static const mem_t jjsquawb_map_mem[] = {
	ROM(0, 0x1fffff), RAM(0x200000, 0x21ffff), RAM(0x300000, 0x30ffff), PAL(0x700000, 0x70ffff),
	VRAM(0, 0x800000, 0x803fff), VRAM(1, 0x804000, 0x807fff), RAM(0x884000, 0x88ffff), SCODE(0xb0c000, 0xb0ffff), MEND
};
static const io_t jjsquawb_map_io[] = {
	IO_PLAYERS(0x400000), IO_VREGS(0x500002), IO_DSW(0x600000), IO_VC(0x908000, 0x909000), IO_SPRITES(0xa0a000),
	IO_X1(0xc00000), IOEND
};
MAP(jjsquawb_map);
static const mem_t daioh_map_mem[] = {
	ROM(0, 0x0fffff), RAM(0x100000, 0x10ffff), PAL(0x700000, 0x70ffff), MEM_COMMON_VRAM, SCODE(0xb00000, 0xb03fff),
	RAM(0xb04000, 0xb13fff), MEND
};
static const io_t daioh_map_io[] = {
	IO_PLAYERS(0x400000), IO_VREGS(0x500002), IO_PORT(0x500006, PT_EXTRA), IO_DSW(0x600000), IO_VC(0x900000, 0x980000),
	IO_SPRITES(0xa00000), IO_X1(0xc00000), IOEND
};
MAP(daioh_map);
static const mem_t daiohp_map_mem[] = {
	ROMM(0, 0x0fffff, 0x80000), ROMM(0x100000, 0x1fffff, 0x80000), RAM(0x200000, 0x20ffff), PAL(0x700000, 0x70ffff),
	MEM_COMMON_VRAM, SCODE(0xb00000, 0xb03fff), RAM(0xb04000, 0xb13fff), MEND
};
static const map_t daiohp_map = { daiohp_map_mem, daioh_map_io };
static const mem_t drgnunit_map_mem[] = {
	ROM(0, 0x0bffff), RAM(0xf00000, 0xf0ffff), RAM(0xffc000, 0xffffff), PAL(0x700000, 0x700fff),
	VRAM(0, 0x900000, 0x90ffff), SCODE(0xe00000, 0xe03fff), MEND
};
static const io_t drgnunit_map_io[] = {
	IO_X1(0x100000), IO_VREGS(0x500002), IO_DSW(0x600000), IO_RW(0x800000, 0x800005, K_VC, 0), IO_PLAYERS(0xb00000),
	IO_SPRITES(0xd00000), IOEND
};
MAP(drgnunit_map);
static const mem_t extdwnhl_map_mem[] = {
	ROM(0, 0x0fffff), RAM(0x200000, 0x23ffff), PAL(0x600000, 0x610fff), MEM_COMMON_VRAM, SCODE(0xb00000, 0xb03fff),
	RAM(0xb04000, 0xb13fff), MEND
};
static const io_t extdwnhl_map_io[] = {
	IO_PLAYERS(0x400000), IO_R(0x400008, 0x40000b, K_DSW, 0), IO_R(0x40000c, 0x40000d, K_FFFF, 0), IO_VREGS(0x500002),
	IO_VC(0x900000, 0x980000), IO_SPRITES(0xa00000), IO_X1(0xe00000), IOEND
};
MAP(extdwnhl_map);
static const mem_t kamenrid_map_mem[] = {
	ROM(0, 0x07ffff), RAM(0x200000, 0x20ffff), PAL(0x700000, 0x703fff), VRAM(0, 0x800000, 0x807fff),
	VRAM(1, 0x880000, 0x887fff), SCODE(0xb00000, 0xb03fff), RAM(0xb04000, 0xb07fff), MEND
};
static const io_t kamenrid_map_io[] = {
	IO_PORT(0x500000, PT_P1), IO_PORT(0x500002, PT_P2), IO_R(0x500004, 0x500007, K_DSW, 0), IO_PORT(0x500008, PT_COINS),
	IO_VREGS(0x600002), IO_ACK(0x600004, 2), IO_ACK(0x600006, 4), IO_VC(0x900000, 0x980000), IO_SPRITES(0xa00000),
	IO_PIT(0xc00000), IO_X1(0xd00000), IOEND
};
MAP(kamenrid_map);
static const mem_t madshark_map_mem[] = {
	ROM(0, 0x0fffff), RAM(0x200000, 0x20ffff), PAL(0x700000, 0x700fff), VRAM(0, 0x800000, 0x803fff),
	VRAM(1, 0x880000, 0x883fff), SCODE(0xb00000, 0xb03fff), MEND
};
static const io_t madshark_map_io[] = {
	IO_PLAYERS(0x500000), IO_DSW(0x500008), IO_VREGS(0x600002), IO_ACK(0x600004, 2), IO_ACK(0x600006, 4),
	IO_VC(0x900000, 0x980000), IO_SPRITES(0xa00000), IO_PIT(0xc00000), IO_X1(0xd00000), IOEND
};
MAP(madshark_map);
static const io_t madsharkbl_map_io[] = {
	IO_RW(0x100000, 0x100001, K_OKI, 0), IO_PLAYERS(0x500000), IO_DSW(0x500008), IO_W(0x50000c, 0x50000d, K_OKIBANK, 0),
	IO_VREGS(0x600002), IO_ACK(0x600004, 2), IO_ACK(0x600006, 4), IO_VC(0x900000, 0x980000), IO_SPRITES(0xa00000), IOEND
};
static const map_t madsharkbl_map = { madshark_map_mem, madsharkbl_map_io };
static const io_t magspeed_map_io[] = {
	IO_PLAYERS(0x500000), IO_DSW(0x500008), IO_VREGS(0x500014), IO_ACK(0x500018, 2), IO_ACK(0x50001c, 4),
	IO_VC(0x900000, 0x980000), IO_SPRITES(0xa00000), IO_PIT(0xc00000), IO_X1(0xd00000), IOEND
};
static const map_t magspeed_map = { kamenrid_map_mem, magspeed_map_io };
static const mem_t msgundam_map_mem[] = {
	ROM(0, 0x07ffff), ROM(0x100000, 0x1fffff), RAMM(0x200000, 0x27ffff, 0x10000), PAL(0x700000, 0x700fff),
	SCODE(0x900000, 0x903fff), VRAM(0, 0xa00000, 0xa03fff), VRAM(1, 0xa80000, 0xa83fff), MEND
};
static const io_t msgundam_map_io[] = {
	IO_PLAYERS(0x400000), IO_ACK(0x400000, 2), IO_ACK(0x400004, 4), IO_VREGS(0x500004), IO_DSW(0x600000),
	IO_SPRITES(0x800000), IO_VC(0xb00000, 0xb80000), IO_X1(0xc00000), IO_PIT(0xd00000), IOEND
};
MAP(msgundam_map);
static const mem_t msgundamb_map_mem[] = {
	ROM(0, 0x07ffff), ROM(0x100000, 0x1fffff), RAM(0x200000, 0x20ffff), PAL(0x700000, 0x700fff), VRAM(0, 0x800000, 0x803fff),
	VRAM(1, 0x880000, 0x883fff), SCODE(0xb00000, 0xb03fff), MEND
};
static const io_t msgundamb_map_io[] = {
	IO_PLAYERS(0x400000), IO_ACK(0x400000, 2), IO_ACK(0x400004, 4), IO_VREGS(0x500002), IO_DSW(0x600000),
	IO_VC(0x900000, 0x980000), IO_SPRITES(0xa00000), IO_X1(0xc00000), IO_PIT(0xd00000), IOEND
};
MAP(msgundamb_map);
static const mem_t oisipuzl_map_mem[] = {
	ROM(0, 0x07ffff), ROM(0x100000, 0x17ffff), RAM(0x200000, 0x20ffff), VRAM(0, 0x800000, 0x803fff),
	VRAM(1, 0x880000, 0x883fff), SCODE(0xb00000, 0xb03fff), PAL(0xc00000, 0xc00fff), MEND
};
static const io_t oisipuzl_map_io[] = {
	IO_DSW(0x300000), IO_PLAYERS(0x400000), IO_VREGS(0x500002), IO_X1(0x700000), IO_VC(0x900000, 0x980000),
	IO_SPRITES(0xa00000), IOEND
};
MAP(oisipuzl_map);
static const io_t triplfun_map_io[] = {
	IO_DSW(0x300000), IO_PLAYERS(0x400000), IO_VREGS(0x500002), IO_RW(0x500006, 0x500007, K_OKI, 0),
	IO_VC(0x900000, 0x980000), IO_SPRITES(0xa00000), IOEND
};
static const map_t triplfun_map = { oisipuzl_map_mem, triplfun_map_io };
static const mem_t utoukond_map_mem[] = {
	ROM(0, 0x0fffff), RAM(0x200000, 0x20ffff), PAL(0x700000, 0x700fff), VRAM(0, 0x800000, 0x803fff),
	VRAM(1, 0x880000, 0x883fff), SCODE(0xb00000, 0xb03fff), MEND
};
static const io_t utoukond_map_io[] = {
	IO_PLAYERS(0x400000), IO_VREGS(0x500002), IO_DSW(0x600000), IO_W(0x900000, 0x900005, K_VC, 0),
	IO_W(0x980000, 0x980005, K_VC, 1), IO_SPRITES(0xa00000), IO_W(0xc00000, 0xc00001, K_LATCH, 0), IOEND
};
MAP(utoukond_map);
static const mem_t crazyfgt_map_mem[] = {
	ROM(0, 0x07ffff), RAM(0x400000, 0x40ffff), PAL(0x640000, 0x640fff), VRAM(1, 0x800000, 0x803fff),
	VRAM(0, 0x880000, 0x883fff), SCODE(0xb00000, 0xb03fff), MEND
};
static const io_t crazyfgt_map_io[] = {
	IO_PORT(0x610000, PT_COINS), IO_PORT(0x610002, PT_UNK), IO_PORT(0x610004, PT_INPUT), IO_W(0x620002, 0x620003, K_OUTPUTS, 0),
	IO_DSW(0x630000), IO_W(0x650000, 0x650003, K_YM, 0), IO_W(0x658000, 0x658001, K_OKI, 0),
	IO_R(0x670000, 0x670001, K_ACKR, 1), IO_RW(0x900000, 0x900005, K_VC, 1), IO_RW(0x980000, 0x980005, K_VC, 0),
	IO_SPRITES(0xa00000), IO_W(0xa80000, 0xa80001, K_BGFLAG, 0), IOEND
};
MAP(crazyfgt_map);
static const mem_t zingzipbl_map_mem[] = {
	ROM(0, 0x07ffff), RAM(0x200000, 0x20ffff), PAL(0x700000, 0x700fff), VRAM(0, 0x800000, 0x803fff),
	VRAM(1, 0x880000, 0x883fff), SCODE(0xb00000, 0xb03fff), MEND
};
static const io_t zingzipbl_map_io[] = {
	IO_VREGS(0x500002), IO_RW(0x902000, 0x902001, K_OKI, 0), IO_W(0x902004, 0x902005, K_VCR, 0x11),
	IO_W(0x902006, 0x902007, K_VCR, 0x10), IO_W(0x902008, 0x902009, K_VCR, 0x01), IO_W(0x90200a, 0x90200b, K_VCR, 0x00),
	IO_W(0x900004, 0x900005, K_VCR, 0x02), IO_W(0x980004, 0x980005, K_VCR, 0x12), IO_R(0x902010, 0x902013, K_DSW, 0),
	IO_PORT(0x902014, PT_P1), IO_PORT(0x902016, PT_P2), IO_SPRITES(0xa00000), IOEND
};
MAP(zingzipbl_map);
static const mem_t setaroul_map_mem[] = {
	ROM(0, 0x01ffff), RAM(0xc00000, 0xc03fff), VRAM(0, 0xe00000, 0xe03fff), MEND
};
static const io_t setaroul_map_io[] = {
	IO_RW(0xcc0000, 0xcc001f, K_RTC, 0), IO_R(0xd40000, 0xd40001, K_PORT, PT_DSW1A), IO_W(0xd40000, 0xd40001, K_PAY, 0),
	IO_PORT(0xd40002, PT_DSW1B), IO_PORT(0xd40004, PT_DSW2A), IO_PORT(0xd40006, PT_DSW2B),
	IO_R(0xd40008, 0xd40009, K_PORT, PT_COIN), IO_W(0xd40008, 0xd40009, K_LED, 0), IO_PORT(0xd4000a, PT_DOOR),
	IO_RW(0xd40010, 0xd40011, K_MUX, 0), IO_PORT(0xd40018, PT_DSW3), IO_X1(0xdc0000), IO_RW(0xe40000, 0xe40005, K_VC, 0),
	IO_RW(0xf00000, 0xf03fff, K_RSPRC, 0), IO_W(0xf40000, 0xf40bff, K_RSPRY, 0), IO_W(0xf40c00, 0xf40c11, K_RSPRCTRL, 0), IOEND
};
MAP(setaroul_map);

enum { G_DRGNUNIT, G_SETAROUL, G_MSGUNDAM, G_ZINGZIP, G_BLANDIA, G_JJSQUAWK, G_JJSQUAWKB, G_ZZBL };
enum { P_DIRECT0, P_DIRECT, P_ZINGZIP, P_BLANDIA, P_GUNDHARA, P_JJSQUAWK, P_PROM };
enum { IRQ_VBLANK, IRQ_1_2, IRQ_2_4, IRQ_ROUL };
enum { VB_ASSERT, VB_HOLD };
enum { EOF_NONE, EOF_SETAC, EOF_SETAC_FIRST, EOF_TNZS };   // (FIRST: VIDEO_UPDATE_AFTER_VBLANK, the buffer flips before the draw)
enum { S_X1, S_X1_STEREO, S_OKI, S_X1_YM3438, S_YM3812_OKI };
enum { BANK_NONE, BANK_X1, BANK_ZOMBRAID };

typedef struct {
	const char *name;
	const map_t *map;
	uint32_t cpu, mhz;              // CPU clock, refresh in milli-Hz
	uint16_t vis_w;
	uint8_t vis_y0, vis_y1;
	int8_t fgx_flip, fgx_noflip, xo_flip, xo_noflip;
	uint8_t gfx, pal, irq, vb_level, vb_mode, pit, eof, tm_flip, snd, bank;
} machine_t;

#define C16 16000000u
#define C8 8000000u
#define Y1 8, 247
#define Y2 16, 239
static const machine_t s_machines[] = {
	//                       cpu  mhz    w   vis     fgx    xo       gfx          pal         irq      vb lev,mode  pit eof         flip snd          bank
	{ "msgundam", &msgundam_map, C16, 56660, 384, Y1, 0, 0, -2, -2, G_MSGUNDAM, P_DIRECT, IRQ_VBLANK, 2, VB_ASSERT, 1, EOF_SETAC, 0, S_X1, 0 },
	{ "msgundamb", &msgundamb_map, C16, 56660, 384, Y1, 0, 0, -2, -2, G_MSGUNDAM, P_DIRECT, IRQ_VBLANK, 2, VB_ASSERT, 1, EOF_SETAC, 0, S_X1, 0 },
	{ "kamenrid", &kamenrid_map, C16, 60000, 384, Y1, 0, 0, -2, -2, G_MSGUNDAM, P_DIRECT, IRQ_VBLANK, 2, VB_ASSERT, 1, EOF_NONE, 0, S_X1, 0 },
	{ "utoukond", &utoukond_map, C16, 60000, 384, Y2, 0, 0, 0, -2, G_MSGUNDAM, P_DIRECT, IRQ_1_2, 0, 0, 0, EOF_NONE, 0, S_X1_YM3438, 0 },
	{ "gundhara", &wrofaero_map, C16, 60000, 384, Y1, 0, 0, 0, 0, G_JJSQUAWK, P_GUNDHARA, IRQ_1_2, 0, 0, 1, EOF_NONE, 0, S_X1, 0 },
	{ "setaroul", &setaroul_map, C8, 60000, 384, Y1, 0, 7, 0, 5, G_SETAROUL, P_PROM, IRQ_ROUL, 2, 0, 0, EOF_TNZS, 0, S_X1_STEREO, 0 },
	{ "setaroulm", &setaroul_map, C8, 60000, 384, Y1, 0, 7, 0, 5, G_SETAROUL, P_PROM, IRQ_ROUL, 5, 0, 0, EOF_TNZS, 0, S_X1_STEREO, 0 },
	{ "drgnunit", &drgnunit_map, C8, 60000, 384, Y1, 2, 2, -2, -2, G_DRGNUNIT, P_DIRECT0, IRQ_1_2, 0, 0, 0, EOF_SETAC, 0, S_X1, 0 },
	{ "stg", &drgnunit_map, C8, 60000, 384, Y1, 0, 0, -2, -2, G_DRGNUNIT, P_DIRECT0, IRQ_1_2, 0, 0, 0, EOF_SETAC, 0, S_X1, 0 },
	{ "qzkklogy", &drgnunit_map, C8, 60000, 384, Y1, 1, 1, -1, -1, G_DRGNUNIT, P_DIRECT0, IRQ_1_2, 0, 0, 0, EOF_SETAC, 0, S_X1, 0 },
	{ "qzkklgy2", &drgnunit_map, C16, 60000, 384, Y1, 0, 0, -3, -1, G_DRGNUNIT, P_DIRECT0, IRQ_1_2, 0, 0, 0, EOF_SETAC, 0, S_X1, 0 },
	{ "rezon", &rezon_map, C16, 57420, 384, Y1, 0, 0, -2, -2, G_MSGUNDAM, P_DIRECT, IRQ_VBLANK, 2, VB_ASSERT, 0, EOF_NONE, 0, S_X1, 0 },
	{ "blandia", &blandia_map, C16, 60000, 384, Y1, 8, 0, 6, -2, G_BLANDIA, P_BLANDIA, IRQ_2_4, 0, 0, 0, EOF_SETAC_FIRST, 0, S_X1, BANK_X1 },
	{ "blandiap", &blandiap_map, C16, 60000, 384, Y1, 8, 0, 6, -2, G_BLANDIA, P_BLANDIA, IRQ_2_4, 0, 0, 0, EOF_SETAC_FIRST, 0, S_X1, BANK_X1 },
	{ "zingzip", &zingzip_map, C16, 57420, 384, Y1, 0, 0, -2, -1, G_ZINGZIP, P_ZINGZIP, IRQ_VBLANK, 3, VB_HOLD, 0, EOF_NONE, 0, S_X1, 0 },
	{ "zingzipbl", &zingzipbl_map, C16, 57420, 384, Y1, 0, 0, -2, -1, G_ZZBL, P_ZINGZIP, IRQ_VBLANK, 6, VB_HOLD, 0, EOF_NONE, 0, S_OKI, 0 },
	{ "daioh", &daioh_map, C16, 57420, 384, Y1, 0, 0, -2, -2, G_MSGUNDAM, P_DIRECT, IRQ_1_2, 0, 0, 0, EOF_NONE, 0, S_X1, 0 },
	{ "daiohp", &daiohp_map, C16, 57420, 384, Y1, 0, 0, -2, -2, G_MSGUNDAM, P_DIRECT, IRQ_1_2, 0, 0, 0, EOF_NONE, 0, S_X1, 0 },
	{ "wrofaero", &wrofaero_map, C16, 60000, 384, Y1, 0, 0, 0, 0, G_MSGUNDAM, P_DIRECT, IRQ_VBLANK, 2, VB_HOLD, 1, EOF_NONE, 0, S_X1, 0 },
	{ "jjsquawk", &zingzip_map, C16, 60000, 384, Y1, 1, 1, -1, -1, G_JJSQUAWK, P_JJSQUAWK, IRQ_1_2, 0, 0, 0, EOF_NONE, 0, S_X1, 0 },
	{ "jjsquawb", &jjsquawb_map, C16, 60000, 384, Y1, 1, 1, -1, -1, G_JJSQUAWKB, P_JJSQUAWK, IRQ_1_2, 0, 0, 0, EOF_NONE, 0, S_X1, 0 },
	{ "madshark", &madshark_map, C16, 60000, 384, Y2, 0, 0, 0, 0, G_JJSQUAWK, P_JJSQUAWK, IRQ_VBLANK, 2, VB_ASSERT, 1, EOF_NONE, 0, S_X1, 0 },
	{ "madsharkbl", &madsharkbl_map, C16, 60000, 384, Y2, 0, 0, 0, 0, G_JJSQUAWK, P_JJSQUAWK, IRQ_VBLANK, 2, VB_ASSERT, 0, EOF_NONE, 0, S_OKI, 0 },
	{ "oisipuzl", &oisipuzl_map, C16, 60000, 320, Y2, 1, 1, -1, -1, G_MSGUNDAM, P_DIRECT, IRQ_1_2, 0, 0, 0, EOF_NONE, 1, S_X1_STEREO, 0 },
	{ "triplfun", &triplfun_map, C16, 60000, 320, Y2, 1, 1, -1, -1, G_MSGUNDAM, P_DIRECT, IRQ_VBLANK, 3, VB_HOLD, 0, EOF_NONE, 1, S_OKI, 0 },
	{ "eightfrc", &zingzip_map, C16, 60000, 384, Y2, 4, 3, 0, 0, G_MSGUNDAM, P_DIRECT, IRQ_1_2, 0, 0, 0, EOF_NONE, 0, S_X1, BANK_X1 },
	{ "magspeed", &magspeed_map, C16, 60000, 384, Y1, 0, 0, 0, -2, G_MSGUNDAM, P_DIRECT, IRQ_VBLANK, 2, VB_ASSERT, 1, EOF_NONE, 0, S_X1, 0 },
	{ "extdwnhl", &extdwnhl_map, C16, 60000, 320, Y1, 0, 0, -2, -2, G_ZINGZIP, P_ZINGZIP, IRQ_1_2, 0, 0, 0, EOF_NONE, 0, S_X1_STEREO, 0 },
	{ "zombraid", &zombraid_map, C16, 60000, 384, Y1, 0, 0, -2, -2, G_JJSQUAWK, P_GUNDHARA, IRQ_1_2, 0, 0, 0, EOF_NONE, 0, S_X1, BANK_ZOMBRAID },
	{ "crazyfgt", &crazyfgt_map, C16, 59185, 384, Y2, 0, 0, 0, -2, G_BLANDIA, P_GUNDHARA, IRQ_VBLANK, 1, VB_ASSERT, 0, EOF_NONE, 0, S_YM3812_OKI, 0 },
};
#define NMACHINES ((int)(sizeof(s_machines) / sizeof(s_machines[0])))

// ---- state ----
#define IO_PAGES 32
static const machine_t *s_m;
static uint8_t *s_rom, *s_prom;
static uint32_t s_rom_size;
static uint16_t *s_ram;
static uint32_t s_ram_size;
static uint16_t *s_pal;              // (0x20000 bytes, on the bus at the map's PAL)
static uint16_t s_snap_pal[0x1000 / 2 + 0x600];
static uint16_t s_vram[2][0x8000];
static x1_012_t *s_layer;            // [2]
static int s_nlayers, s_pal_entries, s_npens, s_pal_first, s_pal2;
static uint16_t *s_indirect;         // pen -> indirect color (NULL: pens are the colors)
static uint32_t *s_rgb;              // indirect colors
static x1_001_t *s_x1, *s_snap;
static x1_010_t s_x1s;
static okim6295_t s_oki;
static uint8_t *s_x1_rom, *s_x1_buf, *s_oki_rom, *s_oki_buf;
static uint32_t s_x1_size, s_oki_size;
static int s_port[PT_COUNT], s_port_bet[26], s_port_coin1;
static uint8_t s_io_page[4096];
static uint16_t s_io_pool[IO_PAGES][2048];   // (read entry << 8 | write entry) + 1 based, per word
static int s_io_pages;
static int s_vis_h;

// the machine's state that is saved
typedef struct {
	uint64_t cpu_done, cpu_target, cpu_frac, snd_frac, ym_frac, oki_acc, z_frac;
	uint64_t pit_next;       // (CPU cycles)
	uint32_t pit_period;     // (PIT clocks), 0 = stopped
	uint16_t pit_count, pit_low, pit_latch;
	uint8_t pit_ctrl, pit_wmsb, pit_rmsb, pit_out, pit_latched;
	int32_t oki_hold;
	uint8_t vregs, led, pay, mux, latch, latch_pending, z_irq, oki_bank;
	int8_t x1_bank;          // (MAME m_samples_bank, -1 until the first write)
	// the hopper (ticket_dispenser_device, 150 ms)
	uint8_t hop_status, hop_power;
	uint64_t hop_next;
	uint64_t coin_start;     // (setaroul, CPU cycles + 1; 0: none)
	uint8_t coin_last;
	// ADC0834
	uint8_t adc_cs, adc_clk, adc_di, adc_do, adc_state, adc_sgl, adc_odd, adc_sel1, adc_output, adc_sars;
	int8_t adc_bit;
	uint8_t gun[4];
	uint32_t rtc_frames;
} state_t;
static state_t s;
static z80 s_z;
static const uint8_t *s_zmap[256];
static uint8_t s_zram[0x1000];
static int32_t *s_mix, *s_ymbuf;
static int s_mix_n, s_ym_n, s_mix_cap, s_ym_cap;
static uint8_t s_snap_vregs, s_snap_led;
static int s_force_pal;

// ---- uPD71054C (pit8253.cpp), counter 0 in modes 2 and 3 ----
static inline uint64_t now_cycles(void) { return s.cpu_done + (uint64_t)m68k_cycles_run(); }

static void pit_load(uint32_t count)
{
	const int mode = (s.pit_ctrl >> 1) & ((s.pit_ctrl & 4) ? 3 : 7);
	if (count == 1)
		count = mode == 2 ? 2 : 0;   // (load_count: illegal in these modes)
	s.pit_count = (uint16_t)count;
	if (!s.pit_period && (mode == 2 || mode == 3)) {
		// the first rising edge: one clock to load the counter, then n clocks (from the clock edge after this write)
		const uint32_t n = count ? count : 0x10000;
		const uint64_t t = now_cycles();
		s.pit_period = n;
		s.pit_next = (t + PIT_CPU_CYCLES - 1) / PIT_CPU_CYCLES * PIT_CPU_CYCLES + (uint64_t)PIT_CPU_CYCLES * (n + 1);
		m68k_end_timeslice();
	}
}

static void pit_write(int offset, uint8_t data)
{
	if (offset == 3) {
		if (!(data & 0xc0) && (data & 0x30)) {   // (counter 0)
			const int was_high = s.pit_out;
			s.pit_ctrl = data & 0x3f;
			s.pit_wmsb = s.pit_rmsb = 0;
			s.pit_period = 0;
			s.pit_out = 1;
			if (!was_high)
				m68k_bus_set_line(4, 1);   // (pit_out0: the rising edge of the mode write)
		} else if (!(data & 0xf0)) {             // (counter 0 latch)
			s.pit_latched = 1;
		}
		return;
	}
	if (offset)
		return;
	switch ((s.pit_ctrl >> 4) & 3) {
	case 1:
		pit_load(data);
		break;
	case 2:
		pit_load((uint32_t)data << 8);
		break;
	case 3:
		if (s.pit_wmsb)
			pit_load((uint32_t)s.pit_low | (uint32_t)data << 8);
		else
			s.pit_low = data;
		s.pit_wmsb ^= 1;
		break;
	}
}

static uint8_t pit_read(int offset)
{
	if (offset)
		return 0xff;
	uint32_t v = 0;
	if (s.pit_period) {
		const uint64_t left = s.pit_next > now_cycles() ? (s.pit_next - now_cycles()) / PIT_CPU_CYCLES : 0;
		v = (uint32_t)left;
	}
	if (s.pit_latched == 1) {       // (the latch command: the count of its first read)
		s.pit_latch = (uint16_t)v;
		s.pit_latched = 2;
	}
	if (s.pit_latched)
		v = s.pit_latch;
	const int hi = ((s.pit_ctrl >> 4) & 3) == 3 ? s.pit_rmsb : ((s.pit_ctrl >> 4) & 3) == 2;
	s.pit_rmsb ^= ((s.pit_ctrl >> 4) & 3) == 3;
	if (!s.pit_rmsb)
		s.pit_latched = 0;
	return (uint8_t)(hi ? v >> 8 : v);
}

static void pit_catch_up(void)
{
	while (s.pit_period && s.pit_next <= s.cpu_done) {
		m68k_bus_set_line(4, 1);   // (pit_out0: rising edge)
		const uint32_t n = s.pit_count ? s.pit_count : 0x10000;
		s.pit_period = n;
		s.pit_next += (uint64_t)PIT_CPU_CYCLES * n;
	}
}

// ---- the X1-010's banked window (m_x1_bank) ----
static void x1_bank_apply(int entry)
{
	if (s_m->bank == BANK_X1)
		memcpy(s_x1_buf + 0xc0000, s_x1_rom + (uint32_t)entry * 0x40000, 0x40000);
	else if (s_m->bank == BANK_ZOMBRAID) {
		const uint32_t o = entry ? 0x80000 + (uint32_t)(entry - 1) * 0x80000 : 0x80000;
		memcpy(s_x1_buf + 0x80000, s_x1_rom + o, 0x80000);
	}
}

static void oki_bank_apply(int entry)
{
	memcpy(s_oki_buf + 0x20000, s_oki_rom + (uint32_t)entry * 0x20000, 0x20000);
}

// ---- ADC0834 (adc083x.cpp) with the light guns ----
enum { AD_IDLE, AD_WAIT_START, AD_SHIFT_MUX, AD_SETTLE, AD_MSB, AD_WAIT_SE, AD_LSB, AD_DONE };

static void adc_cs_w(int state)
{
	if (s.adc_cs == 0 && state) {
		s.adc_state = AD_IDLE;
		s.adc_sars = 1;
		s.adc_do = 1;
	}
	if (s.adc_cs && !state) {
		s.adc_state = AD_WAIT_START;
		s.adc_sars = 1;
		s.adc_do = 1;
	}
	s.adc_cs = (uint8_t)state;
}

static int adc_conversion(void)
{
	int pos = s.adc_odd + s.adc_sel1 * 2;
	int p = s.gun[pos], n = s.adc_sgl ? 0 : s.gun[pos ^ 1];
	int r = p - n;
	return r < 0 ? 0 : r > 255 ? 255 : r;
}

static void adc_clk_w(int state)
{
	if (s.adc_cs == 0) {
		if (s.adc_clk == 0 && state) {
			switch (s.adc_state) {
			case AD_WAIT_START:
				if (s.adc_di) {
					s.adc_state = AD_SHIFT_MUX;
					s.adc_sars = 0;
					s.adc_sgl = s.adc_odd = s.adc_sel1 = 0;
					s.adc_bit = 0;
				}
				break;
			case AD_SHIFT_MUX:
				if (s.adc_bit == 0 && s.adc_di) s.adc_sgl = 1;
				if (s.adc_bit == 1 && s.adc_di) s.adc_odd = 1;
				if (s.adc_bit == 2 && s.adc_di) s.adc_sel1 = 1;
				if (++s.adc_bit == 3)   // (m_mux_bits of the ADC0834)
					s.adc_state = AD_SETTLE;
				break;
			case AD_WAIT_SE:
				s.adc_sars = 0;
				s.adc_state = AD_LSB;
				s.adc_bit = 1;
				break;
			}
		}
		if (s.adc_clk && !state) {
			switch (s.adc_state) {
			case AD_SETTLE:
				s.adc_output = (uint8_t)adc_conversion();
				s.adc_state = AD_MSB;
				s.adc_bit = 7;
				s.adc_sars = 1;
				s.adc_do = 0;
				break;
			case AD_MSB:
				s.adc_do = (s.adc_output >> s.adc_bit) & 1;
				if (--s.adc_bit < 0)
					s.adc_state = AD_WAIT_SE;
				break;
			case AD_LSB:
				s.adc_do = (s.adc_output >> s.adc_bit) & 1;
				if (++s.adc_bit == 8)
					s.adc_state = AD_DONE;
				break;
			case AD_DONE:
				s.adc_state = AD_IDLE;
				s.adc_do = 0;
				break;
			}
		}
	}
	s.adc_clk = (uint8_t)state;
}

// ---- setaroul's RTC (a fixed date, the seconds of the frames) ----
static uint16_t rtc_r(int offset)
{
	if (offset >= 7)
		offset++;
	if (offset / 2 >= 7)
		return 0;
	const uint32_t sec = s.rtc_frames / 60;
	const int ss = (int)(sec % 60), mm = (int)((sec / 60) % 60), hh = (int)((12 + sec / 3600) % 24);
	static const uint8_t date[7] = { 0, 0, 0, 0, 0x01, 0x01, 0x94 };
	uint8_t v = date[offset / 2];
	if (offset / 2 == 0) v = (uint8_t)((ss / 10) << 4 | (ss % 10));
	if (offset / 2 == 1) v = (uint8_t)((mm / 10) << 4 | (mm % 10));
	if (offset / 2 == 2) v = (uint8_t)((hh / 10) << 4 | (hh % 10));
	return (uint16_t)((v >> ((offset & 1) * 4)) & 0xf);
}

static uint16_t coin_r(void)
{
	// (setaroul_state::coin_sensors_r: the coin passing the sensors)
	uint32_t data = 3;
	if (s.coin_start) {
		const uint64_t d = (now_cycles() + 1 - s.coin_start) / (s_m->cpu / 1000);   // (ms)
		if (d <= 16 * 10)
			data &= ~2u;
		if (d > 16 * 5 && d < 16 * 15)
			data &= ~1u;
		if (d > 16 * 15)
			s.coin_start = 0;
	}
	rbx_port_custom(s_port[PT_COIN], 0xc0, data << 6);
	rbx_port_custom(s_port[PT_COIN], 0x01, s.hop_status);
	return (uint16_t)rbx_port_read(s_port[PT_COIN]);
}

// ---- the bus ----
static inline const io_t *io_find(uint32_t a, int shift)
{
	const int p = s_io_page[a >> 12];
	if (!p)
		return NULL;
	const int e = (s_io_pool[p - 1][(a & 0xfff) >> 1] >> shift) & 0xff;
	return e ? &s_m->map->io[e - 1] : NULL;
}

static uint16_t main_read16(uint32_t a)
{
	a &= 0xfffffe;
	const io_t *io = io_find(a, 8);
	if (!io)
		return 0;
	const uint32_t o = (a - io->s) >> 1;
	switch (io->rk) {
	case K_PORT:
		if (io->arg == PT_COIN)
			return coin_r();
		return (uint16_t)rbx_port_read(s_port[io->arg]);
	case K_DSW: {
		const uint32_t d = rbx_port_read(s_port[PT_DSW]);
		return (uint16_t)((o ? d : d >> 8) & 0xff);
	}
	case K_FFFF: return 0xffff;
	case K_SPRY: return s_x1->ylow[o];
	case K_SPRC: return s_x1->ctrl[o];
	case K_VC: return s_layer[io->arg].vctrl[o];
	case K_X1: return x1_010_word_r(&s_x1s, o);
	case K_PIT: return pit_read((int)o);
	case K_OKI: return oki_read(&s_oki);
	case K_GUNR: return s.adc_do;
	case K_ACKR:
		m68k_bus_set_line(io->arg, 0);
		return 0;
	case K_RTC: return rtc_r((int)o);
	case K_MUX:
		return s.mux < 26 ? (uint16_t)rbx_port_read(s_port_bet[s.mux]) : 0xff;
	case K_RSPRC:
		return (uint16_t)(((o & 1) ? s_x1->code[o >> 1] & 0xff : s_x1->code[o >> 1] >> 8) << 8);
	}
	return 0;
}

static uint8_t main_read8(uint32_t a)
{
	return (uint8_t)(main_read16(a & ~1u) >> ((a & 1) ? 0 : 8));
}

static void main_write16m(uint32_t a, uint16_t v, uint16_t mask)
{
	const io_t *io = io_find(a, 0);
	if (!io)
		return;
	const uint32_t o = (a - io->s) >> 1;
	switch (io->wk) {
	case K_VREGS:
		if (mask & 0xff) {
			s.vregs = (uint8_t)v;
			const int bank = (v >> 3) & 7;
			if (bank != s.x1_bank) {
				s.x1_bank = (int8_t)bank;
				if (s_m->bank)
					x1_bank_apply(bank);
			}
		}
		break;
	case K_ACK:
		m68k_bus_set_line(io->arg, 0);
		break;
	case K_SPRY:
		if (mask & 0xff)
			s_x1->ylow[o] = (uint8_t)v;
		break;
	case K_SPRC:
		if (mask & 0xff)
			s_x1->ctrl[o] = (uint8_t)v;
		break;
	case K_VC: {
		uint16_t *r = &s_layer[io->arg].vctrl[o];
		*r = (uint16_t)((*r & ~mask) | (v & mask));
		break;
	}
	case K_VCR: {
		uint16_t *r = &s_layer[io->arg >> 4].vctrl[io->arg & 15];
		*r = (uint16_t)((*r & ~mask) | (v & mask));
		break;
	}
	case K_X1:
		x1_010_word_w(&s_x1s, o, v);
		break;
	case K_PIT:
		if (mask & 0xff)
			pit_write((int)o, (uint8_t)v);
		break;
	case K_OKI:
		if (mask & 0xff)
			oki_write(&s_oki, (uint8_t)v);
		break;
	case K_OKIBANK:
		if (mask & 0xff) {
			const int b = ((v >> 3) & 1) << 1 | ((v >> 2) & 1);   // (bitswap<2>(data, 3, 2))
			if (b != s.oki_bank) {
				s.oki_bank = (uint8_t)b;
				oki_bank_apply(b);
			}
		}
		break;
	case K_GUNW:
		adc_cs_w((v >> 2) & 1);
		s.adc_di = (v >> 1) & 1;
		adc_clk_w(v & 1);
		break;
	case K_OUTPUTS:   // (crazyfgt_state::outputs_w: the DS2430A's data line, driven low by the board)
		if (mask & 0xff)
			rbx_port_custom(s_port[PT_COINS], 0x80, !((v >> 5) & 1));
		break;
	case K_BGFLAG:
		if (mask & 0xff00)
			s_x1->bgflag = (uint8_t)(v >> 8);
		break;
	case K_LATCH:
		if (mask & 0xff) {
			s.latch = (uint8_t)v;
			s.latch_pending = 1;
			s.z_irq = 1;
		}
		break;
	case K_YM:
		if (mask & 0xff)
			rbx_ym_write(0, o, (uint8_t)v);
		break;
	case K_PAY: {
		// (setaroul_state::pay_w: the hopper's motor)
		s.pay = (uint8_t)v;
		const int on = !(v & 0x40) && (v & 0x80);
		if (on && !s.hop_power) {
			s.hop_next = now_cycles() + s_m->cpu / 1000 * 150;
			s.hop_power = 1;
			s.hop_status = 0;
		} else if (!on && s.hop_power) {
			s.hop_next = 0;
			s.hop_power = 0;
		}
		break;
	}
	case K_LED:
		s.led = (uint8_t)v;
		break;
	case K_MUX:
		s.mux = (uint8_t)v;
		break;
	case K_RSPRC: {
		const uint8_t d = (uint8_t)(v >> 8);
		uint16_t *c = &s_x1->code[o >> 1];
		*c = (o & 1) ? (uint16_t)((*c & 0xff00) | d) : (uint16_t)((*c & 0x00ff) | d << 8);
		break;
	}
	case K_RSPRY:
		if (!(o & 1))
			s_x1->ylow[o >> 1] = (uint8_t)(v >> 8);
		break;
	case K_RSPRCTRL:
		if (!(o & 1) && (o >> 1) < 4)
			s_x1->ctrl[o >> 1] = (uint8_t)(v >> 8);
		break;
	}
}

static void main_write16(uint32_t a, uint16_t v) { main_write16m(a & 0xfffffe, v, 0xffff); }
static void main_write8(uint32_t a, uint8_t v)
{
	if (a & 1)
		main_write16m(a & 0xfffffe, v, 0x00ff);
	else
		main_write16m(a & 0xfffffe, (uint16_t)(v << 8), 0xff00);
}

// ---- utoukond's sound CPU ----
static uint8_t z_read(void *ctx, uint16_t a)
{
	(void)ctx;
	if (a < 0xe000)
		return s_zmap[a >> 8][a & 0xff];
	if (a < 0xf000)
		return s_zram[a & 0xfff];
	return x1_010_read(&s_x1s, a & 0xfff);
}
static void z_write(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	if (a >= 0xf000)
		x1_010_write(&s_x1s, a & 0xfff, v);
	else if (a >= 0xe000)
		s_zram[a & 0xfff] = v;
}
static uint8_t z_in(z80 *z, uint8_t p)
{
	(void)z;
	if (p < 4)
		return rbx_ym_read(0, p);
	if (p == 0xc0)
		return s.latch;
	return 0xff;
}
static void z_out(z80 *z, uint8_t p, uint8_t v)
{
	(void)z;
	if (p < 4)
		rbx_ym_write(0, p, v);
	else if (p == 0x80 && !(v & 0x40)) {   // (utoukond_sound_control_w: the latch's acknowledge)
		s.latch_pending = 0;
		s.z_irq = 0;
	}
}
static int s_nmi_line;
static void ym_irq_nmi(int state)
{
	if (state && !s_nmi_line)
		z80_gen_nmi(&s_z);
	s_nmi_line = state;
}
static void ym_irq_68k(int state) { m68k_bus_set_line(2, state); }

// ---- video ----
static void build_indirect(void)
{
	s_indirect = NULL;
	if (s_m->pal == P_DIRECT0 || s_m->pal == P_DIRECT || s_m->pal == P_PROM)
		return;
	s_indirect = (uint16_t *)malloc((size_t)s_npens * 2);
	if (!s_indirect)
		return;
	const int indirect = s_m->pal == P_BLANDIA ? 0xc00 : 0x600;
	for (int i = 0; i < s_npens; i++)
		s_indirect[i] = (uint16_t)(i % indirect);   // (the pens a palette init leaves alone)
#define PEN(p, v) s_indirect[p] = (uint16_t)(v)
	for (int color = 0; color < 0x20; color++)
		for (int pen = 0; pen < 0x40; pen++) {
			const int c6 = color << 6 | pen, c3 = ((color & ~3) << 4) + pen, c4 = (color << 4) + pen;
			switch (s_m->pal) {
			case P_BLANDIA:
				PEN(0x0200 + c6, 0x200 + (color << 4 | (pen & 0x0f)));
				PEN(0x1200 + c6, 0x200 + pen);
				PEN(0x0a00 + c6, 0x400 + (color << 4 | (pen & 0x0f)));
				PEN(0x1a00 + c6, 0x400 + pen);
				break;
			case P_GUNDHARA:
				PEN(0x0200 + c6, 0x400 + (c3 & 0x1ff));
				PEN(0x1200 + c6, 0x400 + (c3 & 0x1ff));
				PEN(0x0a00 + c6, 0x200 + (c3 & 0x1ff));
				PEN(0x1a00 + c6, 0x200 + (c3 & 0x1ff));
				break;
			case P_JJSQUAWK:
				PEN(0x0200 + c6, 0x400 + (c4 & 0x1ff));
				PEN(0x1200 + c6, 0x400 + (c4 & 0x1ff));
				PEN(0x0a00 + c6, 0x200 + (c4 & 0x1ff));
				PEN(0x1a00 + c6, 0x200 + (c4 & 0x1ff));
				break;
			case P_ZINGZIP:
				PEN(0x400 + c6, 0x400 + (c3 & 0x1ff));
				PEN(0xc00 + c6, 0x400 + (c3 & 0x1ff));
				break;
			}
		}
#undef PEN
	if (s_m->pal == P_BLANDIA)
		for (int i = 0; i < 0x2200; i++)
			s_indirect[0x2200 + i] = (uint16_t)(0x600 + (i & 0x1ff));   // (the effect's pens)
}

static inline uint32_t pal_rgb(uint32_t d)
{
	uint32_t r = (d >> 10) & 31, g = (d >> 5) & 31, b = d & 31;
	r = (r << 3) | (r >> 2);
	g = (g << 3) | (g >> 2);
	b = (b << 3) | (b >> 2);
	return r << 16 | g << 8 | b;
}

static void set_pens(void)   // (set_pens)
{
	const int n = s_pal_entries;
	if (s_m->pal == P_PROM)
		return;
	static uint16_t last[sizeof(s_snap_pal) / 2];
	if (!s_force_pal && !memcmp(last, s_snap_pal, (size_t)n * 2))
		return;
	memcpy(last, s_snap_pal, (size_t)n * 2);
	s_force_pal = 0;
	if (!s_indirect) {
		for (int i = 0; i < n; i++)
			rbx_video.palette[i] = pal_rgb(s_snap_pal[i]);
		return;
	}
	for (int i = 0; i < n; i++)
		s_rgb[i] = pal_rgb(s_snap_pal[i]);
	for (int i = 0; i < s_npens; i++)
		rbx_video.palette[i] = s_rgb[s_indirect[i]];
}

static void draw_layer(int n, int flip, int opaque)
{
	x1_012_draw(&s_layer[n], rbx_video.bitmap, rbx_video.pitch, rbx_video.width, rbx_video.height, s_m->vis_y0, s_vis_h, flip, opaque);
}

static void draw_sprites(int bank_size)
{
	x1_draw(s_snap, rbx_video.bitmap, rbx_video.pitch, rbx_video.width, rbx_video.height, 0, s_m->vis_y0, LINES, s_m->vis_y1, bank_size);
}

static void draw_effect_or_layer1(int flip)
{
	if ((s_snap_vregs & 4) && s_pal2)
		x1_012_draw_effect(&s_layer[1], rbx_video.bitmap, rbx_video.pitch, rbx_video.width, rbx_video.height, s_m->vis_y0, s_vis_h, flip, s_npens / 2);
	else if (s_nlayers > 1)
		draw_layer(1, flip, 0);
}

static void tm_render(void)   // (seta_layers_update, screen_update_seta)
{
	set_pens();
	const int bank_size = s_m->bank == BANK_NONE && s_m->eof == EOF_TNZS ? 0x800 : 0x1000;
	const int flip = ((s_snap->ctrl[0] >> 6) & 1) ^ s_m->tm_flip;
	const int order = s_nlayers > 1 ? s_snap_vregs : 0;
	if (s_m->eof == EOF_TNZS && !(s_snap_led & 0x80)) {
		memset(rbx_video.bitmap, 0, (size_t)rbx_video.pitch * rbx_video.height * 2);
		return;
	}
	if (order & 1) {   // (swap the layers?)
		draw_layer(1, flip, 1);
		if (order & 2) {   // (layer-sprite priority?)
			draw_sprites(bank_size);
			draw_layer(0, flip, 0);
		} else {
			draw_layer(0, flip, 0);
			draw_sprites(bank_size);
		}
	} else {
		draw_layer(0, flip, 1);
		if (order & 2) {
			draw_sprites(bank_size);
			draw_effect_or_layer1(flip);
		} else {
			draw_effect_or_layer1(flip);
			draw_sprites(bank_size);
		}
	}
}

static void capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap_pal, (const uint8_t *)s_pal + s_pal_first, (size_t)(s_pal_entries > 0x600 ? 0x600 : s_pal_entries) * 2);
	if (s_pal2)
		memcpy(s_snap_pal + 0x600, (const uint8_t *)s_pal + s_pal2, 0x600 * 2);
	memcpy(s_snap->ctrl, s_x1->ctrl, sizeof(s_x1->ctrl));
	memcpy(s_snap->ylow, s_x1->ylow, sizeof(s_x1->ylow));
	memcpy(s_snap->code, s_x1->code, sizeof(s_x1->code));
	s_snap->bgflag = s_x1->bgflag;
	for (int i = 0; i < s_nlayers; i++)
		x1_012_capture(&s_layer[i]);
	s_snap_vregs = s.vregs;
	s_snap_led = s.led;
}

// ---- init ----
static void tm_exit(void);

static int build_io(void)
{
	memset(s_io_page, 0, sizeof(s_io_page));
	memset(s_io_pool, 0, sizeof(s_io_pool));
	s_io_pages = 0;
	const io_t *io = s_m->map->io;
	for (int i = 0; io[i].e; i++)
		for (uint32_t a = io[i].s & ~1u; a <= io[i].e; a += 2) {
			if (!s_io_page[a >> 12]) {
				if (s_io_pages == IO_PAGES)
					return -1;
				s_io_page[a >> 12] = (uint8_t)++s_io_pages;
			}
			uint16_t *w = &s_io_pool[s_io_page[a >> 12] - 1][(a & 0xfff) >> 1];
			if (io[i].rk)
				*w = (uint16_t)((*w & 0x00ff) | (uint16_t)(i + 1) << 8);
			if (io[i].wk)
				*w = (uint16_t)((*w & 0xff00) | (uint16_t)(i + 1));
		}
	return 0;
}

static void build_map(void)
{
	m68k_bus_reset_map();
	uint32_t used = 0;
	const mem_t *m = s_m->map->mem;
	for (int i = 0; m[i].type != M_END; i++) {
		const uint32_t len = m[i].e - m[i].s + 1;
		switch (m[i].type) {
		case M_ROM:
			m68k_bus_map(m[i].s, m[i].e, s_rom + m[i].off, m[i].size ? m[i].size : len, 1, 0);
			break;
		case M_RAM: {
			const uint32_t size = m[i].size ? m[i].size : len;
			m68k_bus_map(m[i].s, m[i].e, (uint8_t *)s_ram + used, size, 1, 1);
			used += size;
			break;
		}
		case M_PAL:
			m68k_bus_map(m[i].s, m[i].e, s_pal, len, 1, 1);
			break;
		case M_VRAM0:
		case M_VRAM1:
			m68k_bus_map(m[i].s, m[i].e, s_vram[m[i].type - M_VRAM0], len, 1, 1);
			break;
		case M_SPRCODE:
			m68k_bus_map(m[i].s, m[i].e, s_x1->code, sizeof(s_x1->code), 1, 1);
			break;
		}
	}
	m68k_bus.read8 = main_read8;
	m68k_bus.read16 = main_read16;
	m68k_bus.write8 = main_write8;
	m68k_bus.write16 = main_write16;
}

static uint32_t ram_needed(void)
{
	uint32_t used = 0;
	const mem_t *m = s_m->map->mem;
	for (int i = 0; m[i].type != M_END; i++)
		if (m[i].type == M_RAM)
			used += m[i].size ? m[i].size : m[i].e - m[i].s + 1;
	return used;
}

// (MAME gfx layouts of the layers; STEPn written out by loops)
static void step(uint32_t *d, int n, uint32_t start, uint32_t inc)
{
	for (int i = 0; i < n; i++)
		d[i] = start + (uint32_t)i * inc;
}

static int decode_layer(x1_012_gfx *g, const char *region, int kind, int base, int gran)
{
	uint32_t size;
	const uint8_t *rom = rbx_region(region, &size);
	if (!rom)
		return -1;
	uint32_t planes[8], xo[16], yo[16];
	rbx_gfx_layout l;
	memset(&l, 0, sizeof(l));
	l.width = l.height = 16;
	l.planeoffs = planes;
	l.xoffs = xo;
	l.yoffs = yo;
	uint32_t dec_size = size;
	if (kind == 4 || kind == 6 || kind == 8)   // (layout_tilemap, _6bpp, _8bpp)
		return x1_012_decode_packed(g, kind, rom, size, base, gran);
	if (kind == 60) {   // layout_tilemap_6bpp_jjsquawkb
		static const uint32_t x[16] = { 256 + 128, 256 + 129, 256 + 130, 256 + 131, 256 + 0, 256 + 1, 256 + 2, 256 + 3, 128, 129, 130, 131, 0, 1, 2, 3 };
		const uint32_t half = size * 4;
		l.planes = 6;
		l.charincrement = 16 * 16 * 4;
		planes[0] = half + 0;
		planes[1] = half + 4;
		planes[2] = 8;
		planes[3] = 12;
		planes[4] = 0;
		planes[5] = 4;
		memcpy(xo, x, sizeof(x));
		step(yo, 8, 0, 16);
		step(yo + 8, 8, 32 * 16, 16);
		dec_size = size / 2;
	} else if (kind == 40) {   // layout_zzbl
		const uint32_t half = size * 4;
		l.planes = 4;
		l.charincrement = 16 * 16 * 2;
		planes[0] = half + 8;
		planes[1] = half;
		planes[2] = 8;
		planes[3] = 0;
		step(xo, 8, 0, 1);
		step(xo + 8, 8, 8 * 2 * 16, 1);
		step(yo, 16, 0, 8 * 2);
		dec_size = size / 2;
	} else {                   // layout_zzbl_6bpp
		const uint32_t third = size / 3 * 8;
		l.planes = 6;
		l.charincrement = 16 * 16 * 2;
		planes[0] = third * 2 + 8;
		planes[1] = third * 2;
		planes[2] = third + 8;
		planes[3] = third;
		planes[4] = 8;
		planes[5] = 0;
		step(xo, 8, 8 * 2 * 16, 1);
		step(xo + 8, 8, 0, 1);
		step(yo, 16, 0, 8 * 2);
		dec_size = size / 3;
	}
	return x1_012_decode(g, &l, rom, dec_size, base, gran);
}

static int setup_layers(void)
{
	x1_012_t *a = &s_layer[0], *b = &s_layer[1];
	int rc = 0;
	s_nlayers = 2;
	switch (s_m->gfx) {
	case G_DRGNUNIT:
		rc = decode_layer(&a->gfx[0], "gfx2", 4, 0, 16);
		a->ngfx = 1;
		s_nlayers = 1;
		break;
	case G_SETAROUL:
		rc = decode_layer(&a->gfx[0], "gfx2", 8, 0, 16);
		a->ngfx = 1;
		s_nlayers = 1;
		break;
	case G_MSGUNDAM:
		rc = decode_layer(&a->gfx[0], "gfx2", 4, 0x400, 16) || decode_layer(&b->gfx[0], "gfx3", 4, 0x200, 16);
		a->ngfx = b->ngfx = 1;
		break;
	case G_ZINGZIP:
		rc = decode_layer(&a->gfx[0], "gfx2", 6, 0x400, 64) || decode_layer(&b->gfx[0], "gfx3", 4, 0x200, 16);
		x1_012_share(&a->gfx[1], &a->gfx[0], 0xc00);
		a->ngfx = 2;
		b->ngfx = 1;
		break;
	case G_BLANDIA:
		rc = decode_layer(&a->gfx[0], "gfx2", 6, 0xa00, 64) || decode_layer(&b->gfx[0], "gfx3", 6, 0x200, 64);
		x1_012_share(&a->gfx[1], &a->gfx[0], 0x1a00);
		x1_012_share(&b->gfx[1], &b->gfx[0], 0x1200);
		a->ngfx = b->ngfx = 2;
		break;
	case G_JJSQUAWK:
	case G_JJSQUAWKB: {
		const int k = s_m->gfx == G_JJSQUAWK ? 6 : 60;
		rc = decode_layer(&a->gfx[0], "gfx2", k, 0x200, 64) || decode_layer(&b->gfx[0], "gfx3", k, 0xa00, 64);
		x1_012_share(&a->gfx[1], &a->gfx[0], 0x1200);
		x1_012_share(&b->gfx[1], &b->gfx[0], 0x1a00);
		a->ngfx = b->ngfx = 2;
		break;
	}
	case G_ZZBL:
		rc = decode_layer(&a->gfx[0], "gfx2", 61, 0x400, 64) || decode_layer(&b->gfx[0], "gfx3", 40, 0x200, 16);
		x1_012_share(&a->gfx[1], &a->gfx[0], 0xc00);
		a->ngfx = 2;
		b->ngfx = 1;
		break;
	}
	for (int i = 0; i < 2; i++) {
		s_layer[i].vram = s_vram[i];
		s_layer[i].xoffs[0] = s_m->xo_noflip;
		s_layer[i].xoffs[1] = s_m->xo_flip;
	}
	return rc;
}

// layout_sprites without a bit test per pixel (x1_init_layout gives the same: gundhara has 64K of them): 8 pixels of a
// plane at a time, 16 x 8 rows of two bytes in each half of the region
static int sprites_decode(const uint8_t *gfx, uint32_t gsize, uint32_t total)
{
	x1_001_t *x = s_x1;
	memset(x, 0, sizeof(*x));
	x->spritelimit = 0x1ff;
	x->colors = s_m->gfx == G_SETAROUL ? 16 : 32;
	x->ntiles = total;
	x->tiles = (uint8_t *)malloc((size_t)total * 256);
	x->opaque = (uint8_t *)malloc(total ? total : 1);
	if (!x->tiles || !x->opaque || !total)
		return -1;
	uint64_t lut[256];
	for (int b = 0; b < 256; b++) {
		uint64_t v = 0;
		for (int k = 0; k < 8; k++)
			v |= (uint64_t)((b >> (7 - k)) & 1) << (k * 8);
		lut[b] = v;
	}
	const uint8_t *lo = gfx, *hi = gfx + gsize / 2;
	for (uint32_t t = 0; t < total; t++, lo += 64, hi += 64) {
		uint8_t *d = x->tiles + (size_t)t * 256;
		uint64_t any = 0, all = ~(uint64_t)0;
		for (int y = 0; y < 16; y++)
			for (int q = 0; q < 2; q++) {
				const int o = (y >= 8 ? 32 : 0) + q * 16 + (y & 7) * 2;
				uint64_t v = lut[hi[o + 1]] << 3 | lut[hi[o]] << 2 | lut[lo[o + 1]] << 1 | lut[lo[o]];
				memcpy(d + y * 16 + q * 8, &v, 8);   // (little-endian hosts)
				any |= v;
				all &= (v | v >> 1 | v >> 2 | v >> 3) & 0x0101010101010101ull;
				all |= ~(uint64_t)0x0101010101010101ull;
			}
		x->opaque[t] = !any ? 0 : all == ~(uint64_t)0 ? 2 : 1;
	}
	x1_reset(x);
	return 0;
}

static int setup_sprites(const char *set)
{
	uint32_t gsize;
	uint8_t *gfx = rbx_region("gfx1", &gsize);
	if (!gfx) {
		rbx_fail("%s: ROM regions missing", set);
		return -1;
	}
	if (s_m->map == &oisipuzl_map || s_m->map == &utoukond_map)
		for (uint32_t i = 0; i < gsize; i++)   // (ROMREGION_INVERT)
			gfx[i] ^= 0xff;
	s_x1 = (x1_001_t *)calloc(1, sizeof(x1_001_t));
	s_snap = (x1_001_t *)calloc(1, sizeof(x1_001_t));
	if (!s_x1 || !s_snap)
		return -1;
	x1_layout l;
	memset(&l, 0, sizeof(l));
	const uint32_t half = gsize * 4;
	l.total = gsize / 2 / 64;
	if (l.total > 0x10000)
		l.total = 0x10000;   // (the bank callback reaches no further)
	l.planes[0] = half + 8;
	l.planes[1] = half;
	l.planes[2] = 8;
	l.planes[3] = 0;
	if (s_m->gfx == G_ZZBL) {   // (layout_zzbl)
		step(l.xoffs, 8, 0, 1);
		step(l.xoffs + 8, 8, 8 * 2 * 16, 1);
		step(l.yoffs, 16, 0, 8 * 2);
	} else {                    // (layout_sprites)
		step(l.xoffs, 8, 0, 1);
		step(l.xoffs + 8, 8, 8 * 2 * 8, 1);
		step(l.yoffs, 8, 0, 8 * 2);
		step(l.yoffs + 8, 8, 8 * 2 * 8 * 2, 8 * 2);
	}
	l.charincrement = 512;
	if (s_m->gfx == G_ZZBL) {
		if (x1_init_layout(s_x1, gfx, gsize, 32, &l))
			return -1;
	} else if (sprites_decode(gfx, gsize, l.total))
		return -1;
	s_x1->setac_gfxbank = 1;
	s_x1->fg_flipxoffs = s_m->fgx_flip;
	s_x1->fg_noflipxoffs = s_m->fgx_noflip;
	s_x1->fg_flipyoffs = -0x12;
	s_x1->fg_noflipyoffs = 0x0e;
	s_x1->bg_flipyoffs = 0x1;
	s_x1->bg_noflipyoffs = -0x1;
	if (s_m->gfx == G_SETAROUL) {
		s_x1->gfx_colorbase = 0x10;
		s_x1->bg_flipyoffs = 0;
		s_x1->bg_noflipxoffs = 2;
	}
	*s_snap = *s_x1;   // (configuration and the decoded tiles; RAM is copied at each vblank)
	return 0;
}

static int setup_sound(const char *set)
{
	uint32_t size;
	if (s_m->snd == S_X1 || s_m->snd == S_X1_STEREO || s_m->snd == S_X1_YM3438) {
		s_x1_rom = rbx_region("x1snd", &s_x1_size);
		if (!s_x1_rom) {
			rbx_fail("%s: ROM regions missing", set);
			return -1;
		}
		if (s_m->bank) {
			s_x1_buf = (uint8_t *)malloc(0x100000);
			if (!s_x1_buf)
				return -1;
			memcpy(s_x1_buf, s_x1_rom, 0x100000);
			x1_010_init(&s_x1s, s_x1_buf, 0x100000);
		} else
			x1_010_init(&s_x1s, s_x1_rom, s_x1_size);
	}
	if (s_m->snd == S_OKI || s_m->snd == S_YM3812_OKI) {
		s_oki_rom = rbx_region("oki", &s_oki_size);
		if (!s_oki_rom) {
			rbx_fail("%s: ROM regions missing", set);
			return -1;
		}
		const uint8_t *rom = s_oki_rom;
		size = s_oki_size;
		if (s_m->map == &madsharkbl_map) {
			s_oki_buf = (uint8_t *)malloc(0x40000);
			if (!s_oki_buf)
				return -1;
			memcpy(s_oki_buf, s_oki_rom, 0x40000);
			rom = s_oki_buf;
			size = 0x40000;
		}
		if (s_m->snd == S_OKI)
			oki_init(&s_oki, !strcmp(s_m->name, "triplfun") ? 792000 : 1000000, 1, rom, size, 256);
		else
			oki_init(&s_oki, 4433619 / 4, 1, rom, size, 128);
	}
	if (s_m->snd == S_X1_YM3438 || s_m->snd == S_YM3812_OKI) {
		rbx_ym_config yc;
		memset(&yc, 0, sizeof(yc));
		if (s_m->snd == S_X1_YM3438) {
			yc.type = RBX_YM3438;
			yc.clock = 8000000;
			yc.gain[0] = yc.gain[1] = (int)(0.30 * 256 + 0.5);
			yc.irq = ym_irq_nmi;
		} else {
			yc.type = RBX_YM3812;
			yc.clock = 4000000;
			yc.gain[0] = 128;
			yc.irq = ym_irq_68k;
		}
		if (rbx_ym_init(0, &yc) || rbx_ym_set_async(0, 1))
			return -1;
	}
	s_mix_cap = 4096;
	s_ym_cap = 4096;
	s_mix = (int32_t *)calloc((size_t)s_mix_cap, 4);
	s_ymbuf = (int32_t *)calloc((size_t)s_ym_cap, 4);
	if (!s_mix || !s_ymbuf)
		return -1;
	int rc;
	switch (s_m->snd) {
	case S_X1_STEREO:
		rc = rbx_audio_setup_stereo(X1_RATE);
		break;
	case S_OKI:
		rc = rbx_audio_setup(s_oki.rate);
		break;
	case S_YM3812_OKI:
		rc = rbx_audio_setup(rbx_ym_rate(0));
		break;
	default:
		rc = rbx_audio_setup(X1_RATE);
		break;
	}
	if (rc)
		return -1;
	rbx_audio.frame_mhz = s_m->mhz;
	return 0;
}

static void tm_reset(void)
{
	rbx_video_wait_idle();
	memset(s_pal, 0, 0x20000);
	memset(s_ram, 0, s_ram_size);
	memset(s_vram, 0, sizeof(s_vram));
	memset(s_zram, 0, sizeof(s_zram));
	if (s_m->map == &zombraid_map) {   // (the default NVRAM)
		uint32_t size;
		const uint8_t *nv = rbx_region("nvram", &size);
		uint32_t off = 0;
		const mem_t *m = s_m->map->mem;
		for (int i = 0; m[i].type != M_END; i++) {
			if (m[i].type == M_RAM && m[i].s == 0x300000)
				break;
			if (m[i].type == M_RAM)
				off += m[i].e - m[i].s + 1;
		}
		if (nv)
			for (uint32_t i = 0; i < size && i < 0x10000; i += 2)
				((uint16_t *)((uint8_t *)s_ram + off))[i / 2] = (uint16_t)(nv[i] << 8 | nv[i + 1]);
	}
	for (int i = 0; i < 2; i++)
		memset(s_layer[i].vctrl, 0, sizeof(s_layer[i].vctrl));
	x1_reset(s_x1);
	memset(&s, 0, sizeof(s));
	s.x1_bank = -1;
	s.adc_cs = 0;
	s.adc_sars = 1;
	s.adc_do = 1;
	s.gun[0] = s.gun[1] = s.gun[2] = s.gun[3] = 0x80;
	s_nmi_line = 0;
	if (s_m->map == &crazyfgt_map)
		rbx_port_custom(s_port[PT_COINS], 0x80, 1);   // (the DS2430A's data line idles high)
	if (s_x1_rom) {
		x1_010_reset(&s_x1s);
		if (s_m->bank)
			x1_bank_apply(0);
	}
	if (s_oki_rom) {
		oki_reset(&s_oki);
		if (s_oki_buf)
			oki_bank_apply(0);
	}
	if (s_m->snd == S_X1_YM3438 || s_m->snd == S_YM3812_OKI)
		rbx_ym_reset(0);
	if (s_m->snd == S_X1_YM3438) {
		z80_init(&s_z);
		s_z.read_byte = z_read;
		s_z.write_byte = z_write;
		s_z.port_in = z_in;
		s_z.port_out = z_out;
		s_z.int_data = 0xff;
		s_z.read_map = s_zmap;
	}
	s_mix_n = s_ym_n = 0;
	s_force_pal = 1;
	m68k_bus_irq_reset();
	m68k_pulse_reset();
}

static int tm_init(const rbx_game *g)
{
	s_m = NULL;
	for (int i = 0; i < NMACHINES; i++)
		if (!strcmp(g->machine, s_machines[i].name))
			s_m = &s_machines[i];
	if (!s_m) {
		rbx_fail("%s: machine not emulated", g->set);
		return -1;
	}
	s_rom = rbx_region("maincpu", &s_rom_size);
	if (!s_rom) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	m68k_bus_swap16(s_rom, s_rom_size);
	s_vis_h = s_m->vis_y1 - s_m->vis_y0 + 1;
	s_ram_size = ram_needed();
	s_ram = (uint16_t *)malloc(s_ram_size ? s_ram_size : 2);
	s_pal = (uint16_t *)malloc(0x20000);
	s_layer = (x1_012_t *)calloc(2, sizeof(x1_012_t));
	s_x1_rom = s_oki_rom = NULL;
	if (!s_ram || !s_pal || !s_layer) {
		rbx_fail("%s: out of memory", g->set);
		tm_exit();
		return -1;
	}
	if (setup_sprites(g->set) || setup_layers()) {
		rbx_fail("%s: gfx setup failed", g->set);
		tm_exit();
		return -1;
	}
	if (build_io()) {
		rbx_fail("%s: too many I/O pages", g->set);
		tm_exit();
		return -1;
	}
	// the pens (PALETTE entries; the 2nd palette RAM of Blandia)
	s_pal_first = 0x400;
	s_pal_entries = 0x600;
	s_pal2 = 0;
	switch (s_m->pal) {
	case P_DIRECT0:
		s_pal_first = 0;
		s_pal_entries = 0x200;
		s_npens = 512;
		break;
	case P_DIRECT: s_npens = 512 * 3; break;
	case P_ZINGZIP: s_npens = 16 * 32 + 16 * 32 + 64 * 32 * 2; break;
	case P_BLANDIA:
		s_npens = (16 * 32 + 64 * 32 * 4) * 2;
		s_pal2 = 0x3c00;
		s_pal_entries = 0xc00;
		break;
	case P_GUNDHARA:
	case P_JJSQUAWK: s_npens = 16 * 32 + 64 * 32 * 4; break;
	case P_PROM:
		s_npens = 512;
		s_pal_entries = 0;
		break;
	}
	if (s_m->map == &jjsquawb_map)
		s_pal_first = 0xb400;
	build_indirect();
	s_rgb = (uint32_t *)malloc(0xc00 * 4);
	if ((s_npens > 0x600 && !s_indirect && s_m->pal != P_DIRECT) || !s_rgb) {
		rbx_fail("%s: out of memory", g->set);
		tm_exit();
		return -1;
	}
	if (setup_sound(g->set) || rbx_video_setup(s_m->vis_w, s_vis_h, s_npens)) {
		rbx_fail("%s: out of memory", g->set);
		tm_exit();
		return -1;
	}
	rbx_video.palette_used = s_npens;
	if (s_m->pal == P_PROM) {
		uint32_t psize;
		s_prom = rbx_region("proms", &psize);
		if (s_prom)
			for (int i = 0; i < 0x200; i++)
				rbx_video.palette[i] = pal_rgb((uint32_t)s_prom[i * 2] << 8 | s_prom[i * 2 + 1]);
		char tag[8];
		for (int i = 0; i < 26; i++) {
			snprintf(tag, sizeof(tag), "BET.%02X", i);
			s_port_bet[i] = rbx_port_find(tag);
		}
		s_port_coin1 = rbx_port_find("COIN1");
	}
	for (int i = 0; i < PT_COUNT; i++)
		s_port[i] = rbx_port_find(s_port_tags[i]);
	if (s_m->snd == S_X1_YM3438) {
		uint32_t zsize;
		const uint8_t *z = rbx_region("audiocpu", &zsize);
		if (!z) {
			rbx_fail("%s: ROM regions missing", g->set);
			tm_exit();
			return -1;
		}
		memset(s_zmap, 0, sizeof(s_zmap));
		for (int i = 0; i < 0xe0; i++)
			s_zmap[i] = z + i * 256;
	}
	m68k_init();
	m68k_set_cpu_type(M68K_CPU_TYPE_68000);
	build_map();
	tm_reset();
	return 0;
}

static void tm_exit(void)
{
	rbx_video_wait_idle();
	rbx_audio.frame_mhz = 0;   // (the next game's board may not set it)
	if (s_m && (s_m->snd == S_X1_YM3438 || s_m->snd == S_YM3812_OKI))
		rbx_ym_exit(0);
	if (s_x1)
		x1_exit(s_x1);
	if (s_layer) {
		x1_012_free(&s_layer[0]);
		x1_012_free(&s_layer[1]);
	}
	free(s_x1);
	free(s_snap);
	free(s_layer);
	free(s_ram);
	free(s_pal);
	free(s_indirect);
	free(s_rgb);
	free(s_x1_buf);
	free(s_oki_buf);
	free(s_mix);
	free(s_ymbuf);
	s_x1 = s_snap = NULL;
	s_layer = NULL;
	s_ram = s_pal = NULL;
	s_indirect = NULL;
	s_rgb = NULL;
	s_x1_buf = s_oki_buf = NULL;
	s_mix = s_ymbuf = NULL;
}

// ---- frame ----
static void run_cpu(void)
{
	while ((int64_t)(s.cpu_target - s.cpu_done) > 0) {
		uint64_t to = s.cpu_target;
		if (s.pit_period && s.pit_next < to)
			to = s.pit_next > s.cpu_done ? s.pit_next : s.cpu_done + 1;
		s.cpu_done += (uint64_t)m68k_execute((int)(to - s.cpu_done));
		pit_catch_up();
	}
}

static void vblank(void)
{
	if (s_m->eof == EOF_SETAC_FIRST)
		x1_setac_eof(s_x1);
	capture();
	rbx_video_frame_done();
	if (s_m->eof == EOF_SETAC)
		x1_setac_eof(s_x1);
	else if (s_m->eof == EOF_TNZS)
		x1_tnzs_eof(s_x1);
	if (s_m->irq == IRQ_VBLANK) {
		if (s_m->vb_mode == VB_HOLD)
			m68k_bus_hold_irq(s_m->vb_level);
		else
			m68k_bus_set_line(s_m->vb_level, 1);
	}
}

static void scan_irq(int line)
{
	switch (s_m->irq) {
	case IRQ_1_2:
		if (line == 240) m68k_bus_hold_irq(1);
		if (line == 112) m68k_bus_hold_irq(2);
		break;
	case IRQ_2_4:
		if (line == 240) m68k_bus_set_line(2, 1);
		if (line == 112) m68k_bus_set_line(4, 1);
		break;
	case IRQ_ROUL:
		if (!(line % 32)) m68k_bus_hold_irq(s_m->vb_level);
		if (line == 248) m68k_bus_hold_irq(4);
		break;
	}
}

static void sound_line(void)
{
	const int n = rbx_line_cycles(&s.snd_frac, s_m->snd == S_OKI ? s_oki.rate : X1_RATE, s_m->mhz, LINES);
	int32_t l[16], r[16];
	switch (s_m->snd) {
	case S_X1:
	case S_X1_STEREO: {
		const int m = n > 16 ? 16 : n;
		memset(l, 0, sizeof(int32_t) * (size_t)m);
		memset(r, 0, sizeof(int32_t) * (size_t)m);
		x1_010_render(&s_x1s, l, r, m);
		for (int i = 0; i < m; i++) {
			if (s_m->snd == S_X1)
				rbx_audio_put((l[i] + r[i]) / 2);   // (both sides at 0.5 to the mono speaker)
			else
				rbx_audio_put_stereo(l[i], r[i]);
		}
		break;
	}
	case S_OKI:
		for (int i = 0; i < n; i++)
			rbx_audio_put(oki_sample(&s_oki));
		break;
	case S_X1_YM3438: {
		const int m = n > 16 ? 16 : n;
		memset(l, 0, sizeof(int32_t) * (size_t)m);
		memset(r, 0, sizeof(int32_t) * (size_t)m);
		x1_010_render(&s_x1s, l, r, m);
		for (int i = 0; i < m && s_mix_n < s_mix_cap; i++)
			s_mix[s_mix_n++] = (l[i] + r[i]) / 2;
		s_ym_n += rbx_ym_advance(0, (uint32_t)rbx_line_cycles(&s.ym_frac, 8000000, s_m->mhz, LINES), s_ymbuf + s_ym_n, s_ym_cap - s_ym_n);
		break;
	}
	case S_YM3812_OKI: {
		const int k = rbx_ym_advance(0, (uint32_t)rbx_line_cycles(&s.ym_frac, 4000000, s_m->mhz, LINES), s_mix + s_mix_n, s_mix_cap - s_mix_n);
		const uint32_t yr = rbx_ym_rate(0);
		for (int i = 0; i < k; i++) {
			s.oki_acc += s_oki.rate;
			while (s.oki_acc >= yr) {
				s.oki_acc -= yr;
				s.oki_hold = oki_sample(&s_oki);
			}
			s_mix[s_mix_n + i] += s.oki_hold;
		}
		s_mix_n += k;
		break;
	}
	}
}

static void sound_frame_end(void)
{
	if (s_m->snd == S_X1_YM3438) {
		rbx_ym_async_take(0, s_ymbuf, s_ym_n);
		for (int i = 0; i < s_mix_n; i++) {   // (the FM's 55.5 kHz averaged to the X1-010's 31.25 kHz)
			const int a = (int)((int64_t)i * s_ym_n / s_mix_n), b = (int)((int64_t)(i + 1) * s_ym_n / s_mix_n);
			int32_t sum = 0;
			for (int k = a; k < b; k++)
				sum += s_ymbuf[k];
			rbx_audio_put(s_mix[i] + (b > a ? sum / (b - a) : 0));
		}
		memset(s_ymbuf, 0, sizeof(int32_t) * (size_t)s_ym_n);
		s_mix_n = s_ym_n = 0;
	} else if (s_m->snd == S_YM3812_OKI) {
		rbx_ym_async_take(0, s_mix, s_mix_n);
		for (int i = 0; i < s_mix_n; i++)
			rbx_audio_put(s_mix[i]);
		memset(s_mix, 0, sizeof(int32_t) * (size_t)s_mix_n);
		s_mix_n = 0;
	}
}

static void frame_inputs(void)
{
	if (s_port[PT_COIN] >= 0 && s_port_coin1 >= 0) {   // (setaroul: coin_drop_start)
		const uint8_t c = rbx_port_read(s_port_coin1) & 1;
		if (c && !s.coin_last && !s.coin_start)
			s.coin_start = s.cpu_done + 1;
		s.coin_last = c;
	}
	if (s_m->map == &zombraid_map)
		for (int i = 0; i < 4; i++) {
			// (the d-pad or a pointer moves the gun, PORT_REVERSE on the x axes)
			const int type = (i & 1) ? RBX_IPT_LIGHTGUN_Y : RBX_IPT_LIGHTGUN_X;
			const int d = rbx_analog_dpad(i / 2 + 1, type, 3);
			int v = s.gun[i] + ((i & 1) ? d : -d);
			s.gun[i] = (uint8_t)(v < 0 ? 0 : v > 255 ? 255 : v);
		}
}

static void tm_frame(void)
{
	frame_inputs();
	const int hopper = s_m->irq == IRQ_ROUL;
	for (int line = 0; line < LINES; line++) {
		if (line == s_m->vis_y1 + 1)
			vblank();
		scan_irq(line);
		const int cyc = rbx_line_cycles(&s.cpu_frac, s_m->cpu, s_m->mhz, LINES);
		s.cpu_target += (uint64_t)cyc;
		run_cpu();
		if (s_m->snd == S_X1_YM3438) {
			const unsigned long end = s_z.cyc + (unsigned long)rbx_line_cycles(&s.z_frac, 4000000, s_m->mhz, LINES);
			int irq = s.z_irq;
			z80_run(&s_z, end, &irq, 0);
		}
		if (hopper && s.hop_next && s.hop_next <= s.cpu_done) {
			// (update_output_state: the medal passes, on and off every period)
			s.hop_status ^= 1;
			s.hop_next += s_m->cpu / 1000 * 150;
		}
		sound_line();
	}
	sound_frame_end();
	s.rtc_frames++;
}

// ---- state ----
static int tm_state(void *ctx, int load)
{
	uint8_t cpu[1024];
	const uint32_t csize = m68k_context_size();
	if (csize > sizeof(cpu))
		return -1;
	if (!load)
		m68k_get_context(cpu);
	uint32_t irq = m68k_bus_irq_pending();
	uint32_t lines = m68k_bus.irq_asserted, nmi = (uint32_t)s_nmi_line;
	int rc = 0;
	rc |= rbx_state_io(ctx, "m68k", cpu, csize);
	rc |= rbx_state_io(ctx, "irq", &irq, 4);
	rc |= rbx_state_io(ctx, "lines", &lines, 4);
	rc |= rbx_state_io(ctx, "ram", s_ram, s_ram_size);
	rc |= rbx_state_io(ctx, "pal", s_pal, 0x20000);
	rc |= rbx_state_io(ctx, "vram", s_vram, sizeof(s_vram));
	rc |= rbx_state_io(ctx, "vc0", s_layer[0].vctrl, sizeof(s_layer[0].vctrl));
	rc |= rbx_state_io(ctx, "vc1", s_layer[1].vctrl, sizeof(s_layer[1].vctrl));
	rc |= rbx_state_io(ctx, "x1ctrl", s_x1->ctrl, sizeof(s_x1->ctrl));
	rc |= rbx_state_io(ctx, "x1ylow", s_x1->ylow, sizeof(s_x1->ylow));
	rc |= rbx_state_io(ctx, "x1code", s_x1->code, sizeof(s_x1->code));
	rc |= rbx_state_io(ctx, "x1bg", &s_x1->bgflag, 1);
	if (s_x1_rom)
		rc |= rbx_state_io(ctx, "x1010", &s_x1s, X1_010_STATE_SIZE);
	if (s_oki_rom)
		rc |= rbx_state_io(ctx, "oki", s_oki.v, (uint32_t)(sizeof(s_oki.v) + sizeof(s_oki.command)));
	rc |= rbx_state_io(ctx, "seta", &s, sizeof(s));
	if (s_m->snd == S_X1_YM3438) {
		z80 z = s_z;
		rc |= rbx_state_io(ctx, "z80", &z, sizeof(z));
		rc |= rbx_state_io(ctx, "zram", s_zram, sizeof(s_zram));
		rc |= rbx_state_io(ctx, "nmi", &nmi, 4);
		if (load && !rc) {
			z.read_byte = z_read;
			z.write_byte = z_write;
			z.port_in = z_in;
			z.port_out = z_out;
			z.read_map = s_zmap;
			z.op_rom = NULL;
			s_z = z;
		}
	}
	if (s_m->snd == S_X1_YM3438 || s_m->snd == S_YM3812_OKI)
		rc |= rbx_ym_state(0, ctx, load);
	if (rc)
		return -1;
	if (load) {
		s_nmi_line = (int)nmi;
		m68k_set_context(cpu);
		m68k_bus_set_irq_pending(irq);
		m68k_bus.irq_asserted = lines;
		m68k_bus_update_irq();
		if (s_m->bank && s.x1_bank >= 0)
			x1_bank_apply(s.x1_bank);
		else if (s_m->bank)
			x1_bank_apply(0);
		if (s_oki_buf)
			oki_bank_apply(s.oki_bank);
		s_force_pal = 1;
	}
	return 0;
}

static int tm_save(void *ctx) { return tm_state(ctx, 0); }
static int tm_load(void *ctx) { return tm_state(ctx, 1); }

static int tm_supported(const char *machine)
{
	for (int i = 0; i < NMACHINES; i++)
		if (!strcmp(machine, s_machines[i].name))
			return 1;
	return 0;
}

const rbx_driver rbx_drv_seta_tm = { "Seta Hardware", tm_init, tm_exit, tm_reset, tm_frame, tm_save, tm_load,
                                     tm_supported, tm_render, NULL };
