/*  Cave 68000 hardware. Ported from MAME src/mame/atlus/cave.cpp, cave_v.cpp, sprite013.cpp (BSD-3-Clause,
    copyright-holders: Luca Elia), src/devices/video/tmap038.cpp (BSD-3-Clause, copyright-holders: Luca Elia, Paul
    Priest, David Haywood) and src/devices/machine/nmk112.cpp (BSD-3-Clause, copyright-holders: Alex W. Jackson).
    A 68000 at 16 MHz; 1 to 4 038 tilemap chips (512x512, 8x8 or 16x16 tiles, row-scroll / row-select), the 013
    sprite chip (1024 zooming sprites drawn into a frame buffer at the vblank IRQ), a 15-bit palette. Sound: a YMZ280B,
    or two OKIs with the NMK112 (donpachi), or a Z80 with a YM2203 / YM2151 and OKIs (hotdogst, mazinger, metmqstr,
    pwrinst2, sailormn, agallet). 57.55 Hz (15625 / 271.5).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m68k_bus.h"
#include "../../cpu/m68k/m68k.h"
#include "../../cpu/z80/z80.h"
#include "../../machine/eeprom93c46.h"
#include "../../sound/okim6295.h"
#include "../../sound/rbx_ym.h"
#include "../../sound/ymz280b.h"

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CPU_CLOCK 16000000
#define FRAME_MHZ 57551          // 15625 / 271.5 Hz
#define FRAME_US 17376           // (the driver's frame time)
#define LINES 272                // scheduling slices of a frame (~64 us)
#define SPIN_CYC 800             // sound_cmd_w: spin_until_time(50 us)

enum { M_DFEVERON, M_DDONPACH, M_DONPACHI, M_ESPRADE, M_GUWANGE, M_GAIA, M_HOTDOGST, M_KOROKORO, M_CRUSHERM,
       M_MAZINGER, M_METMQSTR, M_PWRINST2, M_AGALLET, M_SAILORMN, M_UOPOKO, M_COUNT };
static const char *const s_mnames[M_COUNT] = { "dfeveron", "ddonpach", "donpachi", "esprade", "guwange", "gaia",
	"hotdogst", "korokoro", "crusherm", "mazinger", "metmqstr", "pwrinst2", "agallet", "sailormn", "uopoko" };

// tile layouts (cave.cpp gfx decodes)
enum { G_4MSB, G_8, G_6, G_6_2 };
// sprite layouts
enum { S_LSB4, S_MSB4, S_8 };
// sound
enum { SND_YMZ, SND_OKI2, SND_Z80 };

typedef struct {
	int layers;
	int sw, sh, vx0, vw, vh;             // screen size, visible area
	int spr_layout, spr_gran, spr_colors, spr_base, spr_alt, spr_delay;
	int spr_xoff, spr_yoff, spr_xoff_f, spr_yoff_f;
	int gfx[4], gbase[4], ggran[4];
	int snd;
	int pwrinst2_pri;
} machine_t;

static const machine_t s_machines[M_COUNT] = {
	//   L  sw   sh  vx0  vw   vh   spr layout gran colors base alt delay  offsets            tile layouts             bases                              granularities     sound
	[M_DFEVERON] = { 2, 320, 240, 0, 320, 240, S_LSB4, 16, 0x40, 0, 0, 0, 0, 0, 320, 240, { G_4MSB, G_4MSB }, { 0x400, 0x400 }, { 16, 16 }, SND_YMZ, 0 },
	[M_DDONPACH] = { 3, 320, 240, 0, 320, 240, S_MSB4, 256, 0x40, 0, 1, 0, 0, 0, 320, 240, { G_4MSB, G_4MSB, G_8 }, { 0x4000, 0x4000, 0x4000 }, { 256, 256, 256 }, SND_YMZ, 0 },
	[M_DONPACHI] = { 3, 320, 240, 0, 320, 240, S_MSB4, 16, 0x40, 0, 1, 0, 0, 0, 320, 240, { G_4MSB, G_4MSB, G_4MSB }, { 0x400, 0x400, 0x400 }, { 16, 16, 16 }, SND_OKI2, 0 },
	[M_ESPRADE]  = { 3, 320, 240, 0, 320, 240, S_8, 256, 0x40, 0, 0, 0, 0, 0, 320, 240, { G_8, G_8, G_8 }, { 0x4000, 0x4000, 0x4000 }, { 256, 256, 256 }, SND_YMZ, 0 },
	[M_GUWANGE]  = { 3, 320, 240, 0, 320, 240, S_8, 256, 0x40, 0, 0, 0, 0, 0, 320, 240, { G_8, G_8, G_8 }, { 0x4000, 0x4000, 0x4000 }, { 256, 256, 256 }, SND_YMZ, 0 },
	[M_GAIA]     = { 3, 320, 240, 0, 320, 224, S_LSB4, 256, 0x40, 0, 0, 0, 0, 0, 320, 240, { G_8, G_8, G_8 }, { 0x4000, 0x4000, 0x4000 }, { 256, 256, 256 }, SND_YMZ, 0 },
	[M_HOTDOGST] = { 3, 384, 240, 0, 384, 240, S_LSB4, 16, 0x40, 0, 0, 0, 0, 0, 384, 240, { G_4MSB, G_4MSB, G_4MSB }, { 0, 0, 0 }, { 16, 16, 16 }, SND_Z80, 0 },
	[M_KOROKORO] = { 1, 320, 240, 0, 318, 239, S_LSB4, 16, 0x40, 0x3c00, 0, 0, 0, 0, 320, 240, { G_4MSB }, { 0x400 }, { 16 }, SND_YMZ, 0 },
	[M_CRUSHERM] = { 1, 320, 240, 0, 318, 239, S_LSB4, 16, 0x40, 0x3c00, 0, 0, 0, 0, 320, 240, { G_4MSB }, { 0x400 }, { 16 }, SND_YMZ, 0 },
	[M_MAZINGER] = { 2, 384, 240, 0, 384, 240, S_LSB4, 16, 0x40, 0, 0, 1, 0, 0, 384, 240, { G_4MSB, G_6 }, { 0, 0x400 }, { 16, 64 }, SND_Z80, 0 },
	[M_METMQSTR] = { 3, 0x200, 240, 0x7d, 0x180, 240, S_LSB4, 16, 0x40, 0, 0, 1, 0, 0, 0x200, 240, { G_4MSB, G_4MSB, G_4MSB }, { 0x400, 0x400, 0x400 }, { 16, 16, 16 }, SND_Z80, 0 },
	[M_PWRINST2] = { 4, 0x200, 240, 0x70, 0x140, 240, S_MSB4, 16, 0x80, 0, 1, 0, 0, 1, 0x200, 240, { G_4MSB, G_4MSB, G_4MSB, G_4MSB }, { 0x800, 0x1000, 0x1800, 0x2000 }, { 16, 16, 16, 16 }, SND_Z80, 1 },
	[M_AGALLET]  = { 3, 321, 240, 1, 320, 240, S_LSB4, 16, 0x40, 0, 0, 0, 0, 0, 321, 240, { G_4MSB, G_4MSB, G_6_2 }, { 0x400, 0x800, 0xc00 }, { 16, 16, 64 }, SND_Z80, 0 },
	[M_SAILORMN] = { 3, 321, 240, 1, 320, 240, S_LSB4, 16, 0x40, 0, 0, 0, 0, 0, 321, 240, { G_4MSB, G_4MSB, G_6_2 }, { 0x400, 0x800, 0xc00 }, { 16, 16, 64 }, SND_Z80, 0 },
	[M_UOPOKO]   = { 1, 320, 240, 0, 320, 240, S_LSB4, 256, 0x40, 0, 0, 0, 0, 0, 320, 240, { G_8 }, { 0x4000 }, { 256 }, SND_YMZ, 0 },
};

// ---- the main CPU's handlers (cave.cpp memory maps) ----
enum { IO_VREGS, IO_VREGS_DP, IO_TMREG, IO_TMREG_PW, IO_YMZ, IO_OKI, IO_NMK112, IO_IN0, IO_IN1, IO_DSW, IO_EEP,
       IO_EEP_G, IO_EEP_H, IO_EEP_K, IO_EEP_M, IO_EEP_S, IO_EEP_PWR, IO_SNDCMD, IO_SNDACK, IO_SNDACKCMD, IO_SNDFLAGS,
       IO_WDOG, IO_NOP };
typedef struct { uint32_t start, end; uint8_t kind, arg; } io_range;
#define IO_END { 0, 0, 0xff, 0 }

static const io_range s_io_dfeveron[] = { { 0x300000, 0x300003, IO_YMZ, 0 }, { 0x800000, 0x80007f, IO_VREGS, 0 },
	{ 0x900000, 0x900005, IO_TMREG, 0 }, { 0xa00000, 0xa00005, IO_TMREG, 1 }, { 0xb00000, 0xb00001, IO_IN0, 0 },
	{ 0xb00002, 0xb00003, IO_IN1, 0 }, { 0xc00000, 0xc00001, IO_EEP, 0 }, IO_END };
static const io_range s_io_ddonpach[] = { { 0x300000, 0x300003, IO_YMZ, 0 }, { 0x800000, 0x80007f, IO_VREGS, 0 },
	{ 0x900000, 0x900005, IO_TMREG, 0 }, { 0xa00000, 0xa00005, IO_TMREG, 1 }, { 0xb00000, 0xb00005, IO_TMREG, 2 },
	{ 0xd00000, 0xd00001, IO_IN0, 0 }, { 0xd00002, 0xd00003, IO_IN1, 0 }, { 0xe00000, 0xe00001, IO_EEP, 0 }, IO_END };
static const io_range s_io_donpachi[] = { { 0x600000, 0x600005, IO_TMREG, 1 }, { 0x700000, 0x700005, IO_TMREG, 0 },
	{ 0x800000, 0x800005, IO_TMREG, 2 }, { 0x900000, 0x90007f, IO_VREGS_DP, 0 }, { 0xb00000, 0xb00003, IO_OKI, 0 },
	{ 0xb00010, 0xb00013, IO_OKI, 1 }, { 0xb00020, 0xb0002f, IO_NMK112, 0 }, { 0xc00000, 0xc00001, IO_IN0, 0 },
	{ 0xc00002, 0xc00003, IO_IN1, 0 }, { 0xd00000, 0xd00001, IO_EEP, 0 }, IO_END };
static const io_range s_io_gaia[] = { { 0x300000, 0x300003, IO_YMZ, 0 }, { 0x800000, 0x80007f, IO_VREGS, 0 },
	{ 0x900000, 0x900005, IO_TMREG, 0 }, { 0xa00000, 0xa00005, IO_TMREG, 1 }, { 0xb00000, 0xb00005, IO_TMREG, 2 },
	{ 0xd00010, 0xd00011, IO_IN0, 0 }, { 0xd00012, 0xd00013, IO_IN1, 0 }, { 0xd00014, 0xd00015, IO_DSW, 0 }, IO_END };
static const io_range s_io_guwange[] = { { 0x300000, 0x30007f, IO_VREGS, 0 }, { 0x800000, 0x800003, IO_YMZ, 0 },
	{ 0x900000, 0x900005, IO_TMREG, 0 }, { 0xa00000, 0xa00005, IO_TMREG, 1 }, { 0xb00000, 0xb00005, IO_TMREG, 2 },
	{ 0xd00010, 0xd00011, IO_EEP_G, 0 }, { 0xd00012, 0xd00013, IO_IN1, 0 }, IO_END };
static const io_range s_io_hotdogst[] = { { 0xa8006e, 0xa8006f, IO_SNDCMD, 0 }, { 0xa80000, 0xa8007f, IO_VREGS, 0 },
	{ 0xb00000, 0xb00005, IO_TMREG, 0 }, { 0xb80000, 0xb80005, IO_TMREG, 1 }, { 0xc00000, 0xc00005, IO_TMREG, 2 },
	{ 0xc80000, 0xc80001, IO_IN0, 0 }, { 0xc80002, 0xc80003, IO_IN1, 0 }, { 0xd00000, 0xd00001, IO_EEP_H, 0 },
	{ 0xd00002, 0xd00003, IO_NOP, 0 }, IO_END };
static const io_range s_io_korokoro[] = { { 0x140000, 0x140005, IO_TMREG, 0 }, { 0x1c0000, 0x1c007f, IO_VREGS, 0 },
	{ 0x240000, 0x240003, IO_YMZ, 1 }, { 0x280000, 0x280001, IO_IN0, 0 }, { 0x280002, 0x280003, IO_IN1, 0 },
	{ 0x280008, 0x280009, IO_NOP, 0 }, { 0x28000a, 0x28000b, IO_EEP_K, 0 }, { 0x28000c, 0x28000d, IO_NOP, 0 }, IO_END };
static const io_range s_io_crusherm[] = { { 0x140000, 0x140005, IO_TMREG, 0 }, { 0x240000, 0x240003, IO_YMZ, 1 },
	{ 0x280000, 0x280001, IO_IN0, 0 }, { 0x280002, 0x280003, IO_IN1, 0 }, { 0x280008, 0x280009, IO_NOP, 0 },
	{ 0x28000a, 0x28000b, IO_EEP_K, 0 }, { 0x28000c, 0x28000d, IO_NOP, 0 }, { 0x300000, 0x30007f, IO_VREGS, 0 }, IO_END };
static const io_range s_io_mazinger[] = { { 0x300068, 0x300069, IO_WDOG, 0 }, { 0x30006e, 0x30006f, IO_SNDACKCMD, 0 }, { 0x300000, 0x30007f, IO_VREGS, 0 },
	{ 0x600000, 0x600005, IO_TMREG, 1 }, { 0x700000, 0x700005, IO_TMREG, 0 }, { 0x800000, 0x800001, IO_IN0, 0 },
	{ 0x800002, 0x800003, IO_IN1, 0 }, { 0x900000, 0x900001, IO_EEP, 0 }, IO_END };
static const io_range s_io_metmqstr[] = { { 0x600000, 0x600001, IO_WDOG, 0 }, { 0xa80068, 0xa80069, IO_WDOG, 0 }, { 0xa8006c, 0xa8006d, IO_SNDFLAGS, 0 },
	{ 0xa8006e, 0xa8006f, IO_SNDACKCMD, 0 }, { 0xa80000, 0xa8007f, IO_VREGS, 0 }, { 0xb00000, 0xb00005, IO_TMREG, 2 },
	{ 0xb80000, 0xb80005, IO_TMREG, 1 }, { 0xc00000, 0xc00005, IO_TMREG, 0 }, { 0xc80000, 0xc80001, IO_IN0, 0 },
	{ 0xc80002, 0xc80003, IO_IN1, 0 }, { 0xd00000, 0xd00001, IO_EEP_M, 0 }, IO_END };
static const io_range s_io_pwrinst2[] = { { 0x500000, 0x500001, IO_IN0, 0 }, { 0x500002, 0x500003, IO_IN1, 0 },
	{ 0x700000, 0x700001, IO_EEP, 0 }, { 0xa80000, 0xa8007f, IO_VREGS_DP, 0 }, { 0xb00000, 0xb00005, IO_TMREG_PW, 2 },
	{ 0xb80000, 0xb80005, IO_TMREG_PW, 0 }, { 0xc00000, 0xc00005, IO_TMREG_PW, 1 }, { 0xc80000, 0xc80005, IO_TMREG_PW, 3 },
	{ 0xd80000, 0xd80001, IO_SNDACK, 0 }, { 0xe00000, 0xe00001, IO_SNDCMD, 0 }, { 0xe80000, 0xe80001, IO_EEP_PWR, 0 },
	IO_END };
static const io_range s_io_sailormn[] = { { 0x600000, 0x600001, IO_IN0, 0 }, { 0x600002, 0x600003, IO_IN1, 0 },
	{ 0x700000, 0x700001, IO_EEP_S, 0 }, { 0xa00000, 0xa00005, IO_TMREG, 0 }, { 0xa80000, 0xa80005, IO_TMREG, 1 },
	{ 0xb00000, 0xb00005, IO_TMREG, 2 }, { 0xb8006c, 0xb8006d, IO_SNDFLAGS, 0 }, { 0xb8006e, 0xb8006f, IO_SNDACKCMD, 0 },
	{ 0xb80000, 0xb8007f, IO_VREGS, 0 }, IO_END };
static const io_range s_io_uopoko[] = { { 0x300000, 0x300003, IO_YMZ, 0 }, { 0x600000, 0x60007f, IO_VREGS, 0 },
	{ 0x700000, 0x700005, IO_TMREG, 0 }, { 0x900000, 0x900001, IO_IN0, 0 }, { 0x900002, 0x900003, IO_IN1, 0 },
	{ 0xa00000, 0xa00001, IO_EEP, 0 }, IO_END };

// ---- state ----
static int s_m;                        // machine
static const machine_t *s_mc;
static const io_range *s_io;
static uint8_t *s_rom, *s_user1, *s_zrom;
static uint32_t s_rom_size, s_user1_size, s_zrom_size;
static int s_port_in0, s_port_in1, s_port_dsw;
static uint32_t s_in1_eep;   // IN1's IPT_CUSTOM bits (the EEPROM's data out; gaia has none, its coins there)
static char s_nv_path[256];

typedef struct {
	uint16_t vram[0x4000];   // 0x0000 16x16, 0x0800 line RAM, 0x0c00 scratch, 0x2000 8x8 (words)
} tmap_ram_t;
typedef struct {
	const uint8_t *data;     // 4 bpp packed (high nibble first) or 1 byte a pixel
	uint32_t count;          // tiles
	int bpp8;
	uint8_t *empty;          // a tile all transparent
} tgfx_t;

static struct cave_state {   // (on the heap while the game runs, ADR-040)
	uint16_t ram[0x8000];        // work RAM (64 KB)
	uint16_t ram2[0x8000];       // more RAM where the map has some (pwrinst2 0xa10000, gaia / metmqstr layer RAM)
	uint16_t misc[0x4000];       // small RAM (sailormn / agallet 0x400000 .., 0x40c000 ..)
	uint16_t tiny[8];
	uint16_t spr[0x8000];        // sprite RAM (4 banks of 0x4000 bytes)
	uint16_t pal[0x8000];
	tmap_ram_t tm[4];
	uint16_t tmregs[4][3];
	uint16_t vregs[0x40];        // the 013's video registers
	// irq
	uint8_t vblank_irq, sound_irq, unknown_irq, agallet_vblank_irq;
	uint8_t spr_bank_delay;
	// z80 boards
	uint16_t soundlatch;
	uint8_t latch_pending;
	uint8_t soundbuf_wptr, soundbuf_rptr, soundbuf_empty;
	uint8_t soundbuf_data[32];
	uint8_t z80_bank, oki_lo[2], oki_hi[2];
	uint8_t nmk_bank[2][4];
	uint8_t sailormn_tilebank;
	uint8_t pad;
	// schedule
	int32_t vend_line;           // -1: none
	int32_t startup_line;        // sailormn: the main CPU reset 1 ms after the machine's
	int32_t spin;                // 68000 cycles still spinning (sound_cmd_w)
	int32_t wdog;                // frames since the watchdog was last fed
	uint32_t frame;
	uint64_t cpu_frac, z80_frac, ym_frac, out_frac;
	uint32_t oki_acc[2];
	int32_t oki_hold[2];
	uint8_t state_end;
} *s_bp;
#define b (*s_bp)

static uint32_t s_spr_bytes, s_pal_bytes;
static int s_irq_level, s_tv_us, s_kludge, s_offs_x, s_offs_y, s_row_n, s_row_f;
static uint16_t s_bg_pen;
static int s_only8[4];
static int s_vstart_line, s_vend_lines;
static int s_wdog_frames;              // the watchdog's time (0: none)
static tgfx_t s_tgfx[4];
static const uint8_t *s_sgfx;          // sprite pixels: 4 bpp packed (low nibble first) or bytes
static uint32_t s_sgfx_pixels, s_sgfx_mask;
static uint32_t s_max_spr_clk;
static uint8_t *s_tbuf[4];             // decoded tiles we own (NULL: the region itself)

// ---- sound ----
static ymz280b_t s_ymz;
static int32_t *s_ymz_l, *s_ymz_r;
static int s_ymz_cap, s_ymz_n;
static uint32_t s_ymz_line0, s_ymz_line_n;
static int32_t s_line_cyc;
static okim6295_t s_oki[2];
static const uint8_t *s_oki_rom[2];
static uint32_t s_oki_size[2];
static int s_noki;
static z80 s_z80;
static uint8_t s_zram[0x2000];
static const uint8_t *s_zmap[256];
static int s_ym_irq;
static uint32_t s_zclock, s_ymclock;
static int32_t *s_mix;
static int s_mix_n, s_mix_cap;
static uint32_t s_out_rate;
static int s_nmk_paged[2];
static eeprom93c46_t s_eeprom;

// ---- video snapshot (the drawing side) ----
static struct {
	uint16_t spr[0x2000];          // the bank shown (0x4000 bytes), taken at the vblank IRQ
	uint16_t svregs[6];            // the 013's registers then
	uint16_t tm[4][0x4000];
	uint16_t tmregs[4][3];
	uint16_t vregs01[2];
	uint16_t pal[0x8000];
	uint8_t tilebank;
	int pal_dirty;
} *s_snap;
static uint16_t s_spr_pending[0x2000], s_svregs_pending[6];
static uint16_t *s_sbm;                // sprite frame buffer (visible area)
static uint16_t (*s_pal_conv_hp)[0x8000];    // (the palette last converted)
#define s_pal_conv (*s_pal_conv_hp)
static int s_pal_conv_valid;

static void update_irq(void) { m68k_bus_set_line(s_irq_level, (b.vblank_irq || b.sound_irq || b.unknown_irq) ? 1 : 0); }

// ---- YMZ280B ----
static void ymz_to(int n)
{
	if (n > s_ymz_cap)
		n = s_ymz_cap;
	if (n > s_ymz_n) {
		ymz280b_render(&s_ymz, s_ymz_l + s_ymz_n, s_ymz_r + s_ymz_n, n - s_ymz_n);
		s_ymz_n = n;
	}
}
static void ymz_sync(void)
{
	const int d = m68k_cycles_run();
	ymz_to((int)(s_ymz_line0 + (s_line_cyc > 0 ? (uint32_t)((uint64_t)(d < s_line_cyc ? d : s_line_cyc) * s_ymz_line_n / (uint32_t)s_line_cyc) : 0)));
}
static void ymz_irq(void *user, int state)
{
	(void)user;
	b.sound_irq = state != 0;
	update_irq();
}

// ---- OKI banking: NMK112 (donpachi, pwrinst2) or the sound CPU's latches ----
static uint8_t oki_nmk_read(void *user, uint32_t a)
{
	const int c = (int)(intptr_t)user;
	const uint32_t pages = s_oki_size[c] >> 16;
	if (!pages)
		return 0;
	uint32_t off;
	if (s_nmk_paged[c] && a < 0x400) {
		const int tb = (int)(a >> 8);
		off = (uint32_t)(b.nmk_bank[c][tb] % pages) << 16 | (uint32_t)tb << 8 | (a & 0xff);
	} else
		off = (uint32_t)(b.nmk_bank[c][a >> 16] % pages) << 16 | (a & 0xffff);
	return s_oki_rom[c][off];
}
static uint8_t oki_latch_read(void *user, uint32_t a)
{
	const int c = (int)(intptr_t)user;
	const uint32_t banks = s_oki_size[c] / 0x20000;
	if (!banks)
		return 0;
	const uint32_t bank = (a < 0x20000 ? b.oki_lo[c] : b.oki_hi[c]) % banks;
	return s_oki_rom[c][bank * 0x20000 + (a & 0x1ffff)];
}
static void nmk112_w(int offset, uint8_t data)
{
	const int chip = (offset >> 2) & 1;
	b.nmk_bank[chip][offset & 3] = data;
}

// ---- EEPROM and the sound CPU's latch ----
static void eeprom_lines(uint8_t data, int di, int cs, int clk)
{
	eeprom_write_lines(&s_eeprom, (data >> cs) & 1, (data >> clk) & 1, (data >> di) & 1);
}
static void sound_cmd_w(uint16_t data)
{
	b.soundlatch = data;
	if (!b.latch_pending) {   // (data_pending: the NMI line asserted)
		b.latch_pending = 1;
		z80_gen_nmi(&s_z80);
	}
	b.spin = -1;   // (spin_until_time(50 us): the frame loop lets the sound CPU reply)
	m68k_end_timeslice();
}
static uint16_t soundlatch_ack_r(void)
{
	if (!b.soundbuf_empty) {
		const uint8_t d = b.soundbuf_data[b.soundbuf_rptr];
		b.soundbuf_rptr = (b.soundbuf_rptr + 1) & 0x1f;
		b.soundbuf_empty = b.soundbuf_rptr == b.soundbuf_wptr;
		return d;
	}
	return 0xff;
}
static void soundlatch_ack_w(uint8_t data)
{
	if (b.soundbuf_empty || b.soundbuf_wptr != b.soundbuf_rptr) {
		b.soundbuf_data[b.soundbuf_wptr] = data;
		b.soundbuf_wptr = (b.soundbuf_wptr + 1) & 0x1f;
		b.soundbuf_empty = 0;
	}
}

static uint16_t irq_cause_r(int offset)
{
	uint16_t result = 3;
	if (b.vblank_irq)
		result ^= 1;
	if (b.unknown_irq)
		result ^= 2;
	if (offset == 2)
		b.vblank_irq = 0;
	if (offset == 3)
		b.unknown_irq = 0;
	update_irq();
	if (offset == 0) {
		result &= ~4;
		result |= b.agallet_vblank_irq ? 0 : 4;
	}
	return result;
}

static uint16_t in1_r(void)
{
	if (s_port_in1 < 0)
		return 0xffff;
	if (s_in1_eep)
		rbx_port_custom(s_port_in1, s_in1_eep, eeprom_do(&s_eeprom) ? s_in1_eep : 0);
	return (uint16_t)rbx_port_read(s_port_in1);
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
	case IO_VREGS:
	case IO_VREGS_DP:
		return off < 4 ? irq_cause_r(off) : 0;
	case IO_TMREG:
	case IO_TMREG_PW:
		return off < 3 ? b.tmregs[r->arg][off] : 0;
	case IO_YMZ:
		if (r->arg)
			return 0;   // (korokoro: write only)
		ymz_sync();
		return ymz280b_read(&s_ymz, off & 1);
	case IO_OKI:
		return oki_read(&s_oki[r->arg]);
	case IO_IN0:
		return (uint16_t)rbx_port_read(s_port_in0);
	case IO_IN1:
		return in1_r();
	case IO_EEP_G:   // (guwange: the inputs at the word, the EEPROM at its low byte)
		return (uint16_t)rbx_port_read(s_port_in0);
	case IO_DSW:
		return (uint16_t)rbx_port_read(s_port_dsw);
	case IO_SNDACK:
	case IO_SNDACKCMD:
		return soundlatch_ack_r();
	case IO_SNDFLAGS:
		return b.soundbuf_empty ? 2 : 0;
	case IO_EEP_PWR:
		return (uint16_t)(eeprom_do(&s_eeprom) ? 0xffff : 0xfff7);
	case IO_WDOG:   // (reset16_r)
		b.wdog = 0;
		return 0;
	default:
		return 0;
	}
}
static uint8_t main_r8(uint32_t a)
{
	const uint16_t w = main_r16(a & ~1u);
	return (uint8_t)((a & 1) ? w : w >> 8);
}

static void tmreg_w(int chip, int off, uint16_t v, uint16_t mask, int pw)
{
	if (off >= 3)
		return;
	if (pw && off == 2) {   // pwrinst2_vctrl_w: the layer priority bits re-coded
		switch (v & 0xf) {
		case 1: v = (uint16_t)(v & ~0xf); break;
		case 2: v = (uint16_t)((v & ~0xf) | 1); break;
		case 4: v = (uint16_t)((v & ~0xf) | 2); break;
		default: v = (uint16_t)((v & ~0xf) | 3); break;
		}
	}
	b.tmregs[chip][off] = (uint16_t)((v & mask) | (b.tmregs[chip][off] & ~mask));
}

static void main_w16m(uint32_t a, uint16_t v, uint16_t mask)
{
	a &= 0xfffffe;
	const io_range *r = io_find(a);
	if (!r)
		return;
	const int off = (int)((a - r->start) >> 1);
	const uint8_t lo = (uint8_t)v, hi = (uint8_t)(v >> 8);
	switch (r->kind) {
	case IO_VREGS:
	case IO_VREGS_DP:
		b.vregs[off & 0x3f] = (uint16_t)((v & mask) | (b.vregs[off & 0x3f] & ~mask));
		break;
	case IO_TMREG:
		tmreg_w(r->arg, off, v, mask, 0);
		break;
	case IO_TMREG_PW:
		tmreg_w(r->arg, off, v, mask, 1);
		break;
	case IO_YMZ:   // (umask 0x00ff)
		if (mask & 0xff) {
			ymz_sync();
			ymz280b_write(&s_ymz, off & 1, lo);
		}
		break;
	case IO_OKI:
		if (mask & 0xff)
			oki_write(&s_oki[r->arg], lo);
		break;
	case IO_NMK112:
		if (mask & 0xff)
			nmk112_w(off, lo);
		break;
	case IO_EEP:   // eeprom_w at the even byte: DI bit 3, CS bit 1, CLK bit 2 (coin counters / lockouts: none here)
		if (mask & 0xff00)
			eeprom_lines(hi, 3, 1, 2);
		break;
	case IO_EEP_H:
		if (mask & 0xff00)
			eeprom_lines(hi, 3, 1, 2);
		break;
	case IO_EEP_M:   // metmqstr: only with bit 0 low
		if ((mask & 0xff00) && !(hi & 1))
			eeprom_lines(hi, 3, 1, 2);
		break;
	case IO_EEP_S:   // sailormn: bit 0 the tile bank of layer 2
		if (mask & 0xff00) {
			b.sailormn_tilebank = hi & 1;
			eeprom_lines(hi & ~1, 3, 1, 2);
		}
		break;
	case IO_EEP_G:   // guwange at the odd byte: DI bit 7, CS bit 5, CLK bit 6
		if (mask & 0x00ff)
			eeprom_lines(lo, 7, 5, 6);
		break;
	case IO_EEP_K:   // korokoro: DI bit 14, CS bit 12, CLK bit 13 (the hopper motor bit 8)
		if (mask & 0xff00)
			eeprom_write_lines(&s_eeprom, (v >> 12) & 1, (v >> 13) & 1, (v >> 14) & 1);
		break;
	case IO_WDOG:
		b.wdog = 0;
		break;
	case IO_SNDCMD:
	case IO_SNDACKCMD:
		sound_cmd_w((uint16_t)((v & mask) | (b.soundlatch & ~mask)));
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

// ---- the sound CPU (cave.cpp sound maps) ----
static uint8_t latch_lo_r(void) { b.latch_pending = 0; return (uint8_t)b.soundlatch; }
static uint8_t latch_hi_r(void) { b.latch_pending = 0; return (uint8_t)(b.soundlatch >> 8); }

static void z80_set_bank(uint8_t data, int mask)
{
	b.z80_bank = (uint8_t)(data & mask);
	const int at = s_m == M_PWRINST2 ? 0x80 : 0x40;   // (the bank window: 0x8000 .. 0xbfff or 0x4000 .. 0x7fff)
	for (int p = 0; p < 0x40; p++) {
		const uint32_t o = (uint32_t)b.z80_bank * 0x4000 + (uint32_t)p * 256;
		s_zmap[at + p] = o + 256 <= s_zrom_size ? s_zrom + o : NULL;
	}
}
static void oki_bank_w(int chip, uint8_t data, int mask)
{
	b.oki_lo[chip] = (uint8_t)(data & mask);
	b.oki_hi[chip] = (uint8_t)((data >> 4) & mask);
}

static uint8_t z80_read(void *u, uint16_t a)
{
	(void)u;
	const uint8_t *p = s_zmap[a >> 8];
	if (p)
		return p[a & 0xff];
	switch (s_m) {
	case M_HOTDOGST:
	case M_METMQSTR:
	case M_PWRINST2:
		if (a >= 0xe000)
			return s_zram[a - 0xe000];
		break;
	case M_MAZINGER:
		if (a >= 0xc000 && a < 0xc800)
			return s_zram[a - 0xc000];
		if (a >= 0xf800)
			return s_zram[0x800 + (a - 0xf800)];
		break;
	default:   // sailormn / agallet: 0xc000 .. 0xdfff mirrored to 0xffff
		if (a >= 0xc000)
			return s_zram[a & 0x1fff];
		break;
	}
	return 0xff;
}
static void z80_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	switch (s_m) {
	case M_HOTDOGST:
	case M_METMQSTR:
	case M_PWRINST2:
		if (a >= 0xe000)
			s_zram[a - 0xe000] = v;
		break;
	case M_MAZINGER:
		if (a >= 0xc000 && a < 0xc800)
			s_zram[a - 0xc000] = v;
		else if (a >= 0xf800)
			s_zram[0x800 + (a - 0xf800)] = v;
		break;
	default:
		if (a >= 0xc000)
			s_zram[a & 0x1fff] = v;
		break;
	}
}
static uint8_t z80_in(z80 *z, uint8_t port)
{
	(void)z;
	switch (s_m) {
	case M_HOTDOGST:
		switch (port) {
		case 0x30: return latch_lo_r();
		case 0x40: return latch_hi_r();
		case 0x50: case 0x51: return rbx_ym_read(0, port & 1);
		case 0x60: return oki_read(&s_oki[0]);
		}
		break;
	case M_MAZINGER:
		switch (port) {
		case 0x30: return latch_lo_r();
		case 0x52: case 0x53: return rbx_ym_read(0, port & 1);
		}
		break;
	case M_METMQSTR:
		switch (port) {
		case 0x20: return 0;   // soundflags_r
		case 0x30: return latch_lo_r();
		case 0x40: return latch_hi_r();
		case 0x50: case 0x51: return rbx_ym_read(0, port & 1);
		}
		break;
	case M_PWRINST2:
		switch (port) {
		case 0x00: return oki_read(&s_oki[0]);
		case 0x08: return oki_read(&s_oki[1]);
		case 0x40: case 0x41: return rbx_ym_read(0, port & 1);
		case 0x60: return latch_hi_r();
		case 0x70: return latch_lo_r();
		}
		break;
	default:   // sailormn / agallet
		switch (port) {
		case 0x20: return 0;
		case 0x30: return latch_lo_r();
		case 0x40: return latch_hi_r();
		case 0x50: case 0x51: return rbx_ym_read(0, port & 1);
		case 0x60: return oki_read(&s_oki[0]);
		case 0x80: return oki_read(&s_oki[1]);
		}
		break;
	}
	return 0xff;
}
static void z80_out(z80 *z, uint8_t port, uint8_t v)
{
	(void)z;
	switch (s_m) {
	case M_HOTDOGST:
		switch (port) {
		case 0x00: z80_set_bank(v, 0x0f); break;
		case 0x50: case 0x51: rbx_ym_write(0, port & 1, v); break;
		case 0x60: oki_write(&s_oki[0], v); break;
		case 0x70: oki_bank_w(0, v, 3); break;
		}
		break;
	case M_MAZINGER:
		switch (port) {
		case 0x00: z80_set_bank(v, 0x07); break;
		case 0x10: soundlatch_ack_w(v); break;
		case 0x50: case 0x51: rbx_ym_write(0, port & 1, v); break;
		case 0x70: oki_write(&s_oki[0], v); break;
		case 0x74: oki_bank_w(0, v, 3); break;
		}
		break;
	case M_METMQSTR:
		switch (port) {
		case 0x00: z80_set_bank(v, 0x0f); break;
		case 0x50: case 0x51: rbx_ym_write(0, port & 1, v); break;
		case 0x60: oki_write(&s_oki[0], v); break;
		case 0x70: oki_bank_w(0, v, 7); break;
		case 0x80: oki_write(&s_oki[1], v); break;
		case 0x90: oki_bank_w(1, v, 7); break;
		}
		break;
	case M_PWRINST2:
		if (port >= 0x10 && port <= 0x17) {
			nmk112_w(port & 7, v);
			break;
		}
		switch (port) {
		case 0x00: oki_write(&s_oki[0], v); break;
		case 0x08: oki_write(&s_oki[1], v); break;
		case 0x40: case 0x41: rbx_ym_write(0, port & 1, v); break;
		case 0x50: soundlatch_ack_w(v); break;
		case 0x80: z80_set_bank(v, 0x07); break;
		}
		break;
	default:
		switch (port) {
		case 0x00: z80_set_bank(v, 0x1f); break;
		case 0x10: soundlatch_ack_w(v); break;
		case 0x50: case 0x51: rbx_ym_write(0, port & 1, v); break;
		case 0x60: oki_write(&s_oki[0], v); break;
		case 0x70: oki_bank_w(0, v, 0xf); break;
		case 0x80: oki_write(&s_oki[1], v); break;
		case 0xc0: oki_bank_w(1, v, 0xf); break;
		}
		break;
	}
}
static void ym_irq(int state) { s_ym_irq = state; }

static void z80_start(void)
{
	z80_init(&s_z80);
	s_z80.read_byte = z80_read;
	s_z80.write_byte = z80_write;
	s_z80.port_in = z80_in;
	s_z80.port_out = z80_out;
	s_z80.read_map = s_zmap;
}

// ---- graphics decoding (cave.cpp gfx layouts) ----
static uint8_t nib_hi(uint8_t v) { return v >> 4; }
static uint8_t nib_lo(uint8_t v) { return v & 15; }

static uint32_t bitswap24(uint32_t v, const int *bits)   // bits[0] the new bit 23 ... bits[23] the new bit 0
{
	uint32_t r = 0;
	for (int i = 0; i < 24; i++)
		r |= ((v >> bits[i]) & 1u) << (23 - i);
	return r;
}

static int decrypt_sprites(uint8_t *src, uint32_t len)
{
	uint8_t *buf = (uint8_t *)malloc(len);
	if (!buf)
		return -1;
	if (s_m == M_MAZINGER) {
		static const int bs[24] = { 23,22,21,20,19,9,7,3,15,4,17,14,18,2,16,5,11,8,6,13,1,10,12,0 };
		for (uint32_t i = 0; i < len; i++) {
			const uint32_t s = (i & ~0xffffffu) | bitswap24(i, bs);
			buf[(i ^ 0xdf88) % len] = s < len ? src[s] : 0;
		}
	} else if (s_m == M_PWRINST2) {
		static const int bs[24] = { 23,22,21,20,19,18,17,16,15,14,13,12,11,10,9,8,7,2,4,6,1,5,3,0 };
		for (uint32_t i = 0; i < len; i++) {
			uint32_t j = (i & ~0xffffffu) | bitswap24(i, bs);
			if ((j & 6) == 0 || (j & 6) == 6)
				j ^= 6;
			if ((j ^ 7) < len)
				buf[j ^ 7] = src[i];
		}
	} else {   // sailormn
		static const int bs[24] = { 23,22,21,20,15,10,12,6,11,1,13,3,16,17,2,5,14,7,18,8,4,19,9,0 };
		for (uint32_t i = 0; i < len; i++) {
			const uint32_t s = (i & ~0xffffffu) | bitswap24(i, bs);
			buf[(i ^ 0x950c4) % len] = s < len ? src[s] : 0;
		}
	}
	memcpy(src, buf, len);
	free(buf);
	return 0;
}

// the sprite ROM to pixels: 4 bpp with the even pixel in the low nibble (in place), or 8 bpp bytes (in place)
static void decode_sprites(uint8_t *rom, uint32_t len)
{
	if (s_mc->spr_layout == S_MSB4) {   // layout_sprites_msb: the even pixel in the high nibble
		for (uint32_t i = 0; i < len; i++)
			rom[i] = (uint8_t)(rom[i] << 4 | rom[i] >> 4);
	} else if (s_mc->spr_layout == S_8) {   // layout_sprites_8bpp: 4 pixels from 4 bytes
		for (uint32_t i = 0; i + 4 <= len; i += 4) {
			const uint8_t b0 = rom[i], b1 = rom[i + 1], b2 = rom[i + 2], b3 = rom[i + 3];
			rom[i + 0] = (uint8_t)(nib_lo(b1) << 4 | nib_lo(b3));
			rom[i + 1] = (uint8_t)(nib_hi(b1) << 4 | nib_hi(b3));
			rom[i + 2] = (uint8_t)(nib_lo(b0) << 4 | nib_lo(b2));
			rom[i + 3] = (uint8_t)(nib_hi(b0) << 4 | nib_hi(b2));
		}
	}
	s_sgfx = rom;
	s_sgfx_pixels = s_mc->spr_layout == S_8 ? len : len * 2;
	uint32_t m = 1;
	while (m < s_sgfx_pixels)
		m <<= 1;
	s_sgfx_mask = m - 1;
}

// a tile layer's ROM to one byte a pixel (8x8 tiles of 64 bytes), or kept as 4 bpp packed
static int decode_tiles(int chip, uint8_t *rom, uint32_t len)
{
	tgfx_t *t = &s_tgfx[chip];
	t->data = rom;
	t->bpp8 = 1;
	switch (s_mc->gfx[chip]) {
	case G_4MSB:   // gfx_8x8x4_packed_msb
		t->bpp8 = 0;
		t->count = len / 32;
		break;
	case G_8:   // layout_8x8x8: pixel p of a row from bytes 2 (p / 2) and 2 (p / 2) + 1
		for (uint32_t i = 0; i + 8 <= len; i += 8) {
			uint8_t r[8];
			for (int p = 0; p < 8; p++) {
				const uint8_t lo = rom[i + (p >> 1) * 2], hi = rom[i + (p >> 1) * 2 + 1];
				r[p] = (p & 1) ? (uint8_t)(nib_lo(hi) << 4 | nib_lo(lo)) : (uint8_t)(nib_hi(hi) << 4 | nib_hi(lo));
			}
			memcpy(rom + i, r, 8);
		}
		t->count = len / 64;
		break;
	case G_6:   // layout_8x8x6: the high 2 bits from bits 8 and 9 of the pair
		for (uint32_t i = 0; i + 8 <= len; i += 8) {
			uint8_t r[8];
			for (int p = 0; p < 8; p++) {
				const uint8_t lo = rom[i + (p >> 1) * 2], hi = rom[i + (p >> 1) * 2 + 1];
				r[p] = (p & 1) ? (uint8_t)(((hi >> 2) & 3) << 4 | nib_lo(lo)) : (uint8_t)(((hi >> 6) & 3) << 4 | nib_hi(lo));
			}
			memcpy(rom + i, r, 8);
		}
		t->count = len / 64;
		break;
	case G_6_2: {   // sailormn_unpack_tiles, then layout_8x8x6_2 (4 bits in the first half, 2 in the second)
		{
			uint8_t *src = rom + (len / 4) * 3 - 1;
			uint8_t *dst = rom + (len / 4) * 4 - 2;
			while (src <= dst) {
				const uint8_t d = src[0];
				dst[0] = (uint8_t)(((d & 0x03) << 4) + ((d & 0x0c) >> 2));
				dst[1] = (uint8_t)(((d & 0x30) >> 0) + ((d & 0xc0) >> 6));
				src -= 1;
				dst -= 2;
			}
		}
		const uint32_t half = len / 2, tiles = half / 32;
		uint8_t *out = (uint8_t *)malloc((size_t)tiles * 64);
		if (!out)
			return -1;
		for (uint32_t tl = 0; tl < tiles; tl++)
			for (int y = 0; y < 8; y++)
				for (int x = 0; x < 8; x++) {
					const uint32_t o = tl * 32 + (uint32_t)y * 4 + (uint32_t)(x >> 1);
					const uint8_t a = rom[o], c = rom[half + o];
					const uint8_t l4 = (x & 1) ? nib_lo(a) : nib_hi(a), h2 = (uint8_t)(((x & 1) ? nib_lo(c) : nib_hi(c)) & 3);
					out[tl * 64 + (uint32_t)y * 8 + (uint32_t)x] = (uint8_t)(h2 << 4 | l4);
				}
		s_tbuf[chip] = out;
		t->data = out;
		t->count = tiles;
		break;
	}
	}
	return 0;
}

// which tiles are all pen 0
static int tiles_empty(int chip)
{
	tgfx_t *t = &s_tgfx[chip];
	t->empty = (uint8_t *)malloc(t->count ? t->count : 1);
	if (!t->empty)
		return -1;
	const uint32_t bytes = t->bpp8 ? 64 : 32;
	for (uint32_t i = 0; i < t->count; i++) {
		const uint8_t *q = t->data + i * bytes;
		uint8_t any = 0;
		for (uint32_t k = 0; k < bytes; k++)
			any |= q[k];
		t->empty[i] = !any;
	}
	return 0;
}

// ---- drawing (cave_v.cpp screen_update, sprite013 draw_sprites), in bands of lines ----
#define BANDS 8

static inline uint8_t spr_pix(uint32_t o)
{
	o &= s_sgfx_mask;
	if (o >= s_sgfx_pixels)
		return 0;
	if (s_mc->spr_layout == S_8)
		return s_sgfx[o];
	return (uint8_t)((s_sgfx[o >> 1] >> ((o & 1) << 2)) & 15);
}

// (miny, maxy: the band; the clipping against it the same as against the screen, line by line)
static void spr_nozoom(uint32_t code, uint16_t colpri, int x, int y, int flipx, int flipy, int width, int height,
                       int miny, int maxy)
{
	const int minx = s_mc->vx0, maxx = s_mc->vx0 + s_mc->vw - 1;
	int drawx_start = x, drawy_start = y;
	int srcx_start, srcx_end, srcx_inc, srcy_start, srcy_end, srcy_inc;
	if (flipx) { srcx_start = width - 1; srcx_end = -1; srcx_inc = -1; }
	else { srcx_start = 0; srcx_end = width; srcx_inc = 1; }
	if (flipy) { srcy_start = height - 1; srcy_end = -1; srcy_inc = -1; }
	else { srcy_start = 0; srcy_end = height; srcy_inc = 1; }
	int srcx_base = srcx_start, srcy = srcy_start;
	(void)srcx_end;
	if (drawx_start < minx) {
		const int diff = minx - drawx_start;
		srcx_base += srcx_inc * diff;
		if ((flipx && srcx_base < 0) || (!flipx && srcx_base >= width))
			return;
		drawx_start += diff;
	}
	if (drawy_start < miny) {
		const int diff = miny - drawy_start;
		srcy += srcy_inc * diff;
		if ((flipy && srcy < 0) || (!flipy && srcy >= height))
			return;
		drawy_start += diff;
	}
	if (drawx_start > maxx || drawy_start > maxy)
		return;
	code <<= 8;
	const int left = flipx ? srcx_base + 1 : width - srcx_base;   // (source pixels to the sprite's edge)
	const int n = left < maxx - drawx_start + 1 ? left : maxx - drawx_start + 1;
	const int s8 = s_mc->spr_layout == S_8;
	for (int drawy = drawy_start; srcy != srcy_end && drawy <= maxy; drawy++, srcy += srcy_inc) {
		uint16_t *dst = s_sbm + drawy * s_mc->vw - s_mc->vx0 + drawx_start;
		uint32_t o = code + (uint32_t)(srcy * width) + (uint32_t)srcx_base;
		if (s8 && !flipx && (o & s_sgfx_mask) == o && o + (uint32_t)n <= s_sgfx_pixels) {
			const uint8_t *src = s_sgfx + o;
			for (int i = 0; i < n; i++)
				if (src[i])
					dst[i] = (uint16_t)(src[i] | colpri);
			continue;
		}
		for (int i = 0; i < n; i++, o += (uint32_t)srcx_inc) {
			const uint8_t pix = spr_pix(o);
			if (pix)
				dst[i] = (uint16_t)(pix | colpri);
		}
	}
}

static void spr_zoom(uint32_t code, uint16_t colpri, int x, int y, int flipx, int flipy, int width, int height, int zoomx,
                     int zoomy, int miny, int maxy)
{
	const int cminx = s_mc->vx0 << 8, cmaxx = (s_mc->vx0 + s_mc->vw) << 8, cminy = miny << 8, cmaxy = (maxy + 1) << 8;
	int drawx_start = x << 8, drawy_start = y << 8;
	int srcx_start, srcx_end, srcx_inc, srcy_start, srcy_end, srcy_inc;
	if (flipx) { srcx_start = width - 1; srcx_end = -1; srcx_inc = -1; }
	else { srcx_start = 0; srcx_end = width; srcx_inc = 1; }
	if (flipy) { srcy_start = height - 1; srcy_end = -1; srcy_inc = -1; }
	else { srcy_start = 0; srcy_end = height; srcy_inc = 1; }
	int srcx_base = srcx_start, srcy = srcy_start;
	if (zoomx <= 0 || zoomy <= 0)
		return;
	while (drawx_start < cminx) {
		srcx_base += srcx_inc;
		if (srcx_base == srcx_end)
			return;
		drawx_start += zoomx;
	}
	while (drawy_start < cminy) {
		srcy += srcy_inc;
		if (srcy == srcy_end)
			return;
		drawy_start += zoomy;
	}
	if (drawx_start >= cmaxx || drawy_start >= cmaxy)
		return;
	code <<= 8;
	for (int drawy = drawy_start; srcy != srcy_end && drawy < cmaxy; drawy += zoomy, srcy += srcy_inc) {
		uint16_t *dst = s_sbm + (drawy >> 8) * s_mc->vw - s_mc->vx0;
		const uint32_t srcoffs = code + (uint32_t)(srcy * width);
		for (int drawx = drawx_start, srcx = srcx_base; srcx != srcx_end && drawx < cmaxx; drawx += zoomx, srcx += srcx_inc) {
			const uint8_t pix = spr_pix(srcoffs + (uint32_t)srcx);
			if (pix)
				dst[drawx >> 8] = (uint16_t)(pix | colpri);
		}
	}
}

static inline int sext10(int v) { v &= 0x3ff; return (v & 0x200) ? v - 0x400 : v; }

// the sprites of lines y0 .. y1 - 1 (all of the list: the clock limit counts the same in every band)
static void draw_sprites(const uint16_t *source, const uint16_t *vr, int y0, int y1)
{
	memset(s_sbm + y0 * s_mc->vw, 0, sizeof(uint16_t) * (size_t)s_mc->vw * (size_t)(y1 - y0));
	const int miny = y0, maxy = y1 - 1;
	const int glob_flipx = (vr[0] & 0x8000) != 0, glob_flipy = (vr[1] & 0x8000) != 0;
	const int max_x = s_mc->sw, max_y = s_mc->sh;
	const uint16_t *finish = source + 0x2000;
	uint32_t clk = 0;
	for (; source < finish; source += 8) {
		clk += 32;
		if (clk > s_max_spr_clk)
			break;
		if (s_mc->spr_alt) {
			const uint16_t attr = source[0];
			const uint32_t code = source[1] + ((uint32_t)(attr & 3) << 16);
			int x = source[2] & 0x3ff, y = source[3] & 0x3ff;
			const uint16_t size = source[4];
			const int tw = ((size >> 8) & 0x1f) * 16, th = (size & 0x1f) * 16;
			x = sext10(x + s_mc->spr_xoff);
			y = sext10(y + s_mc->spr_yoff);
			if (!tw || !th || x + tw <= 0 || x >= max_x || y + th <= 0 || y >= max_y)
				continue;
			clk += (uint32_t)(tw * th);
			if (clk > s_max_spr_clk)
				break;
			int flipx = (attr >> 3) & 1, flipy = (attr >> 2) & 1;
			const uint16_t colpri = (uint16_t)(((attr & 0x30) << 10) | (attr & 0x3f00));
			if (glob_flipx) { x = s_mc->spr_xoff_f - x - tw; flipx = !flipx; }
			if (glob_flipy) { y = s_mc->spr_yoff_f - y - th; flipy = !flipy; }
			if (y + th > miny && y <= maxy)
				spr_nozoom(code, colpri, x, y, flipx, flipy, tw, th, miny, maxy);
		} else {
			int x, y;
			const int fixed = (vr[5] & 0x3000) != 0;
			if (!fixed) {
				x = (source[0] & 0x3ff) << 8;
				y = (source[1] & 0x3ff) << 8;
			} else {
				x = (int)((uint32_t)source[0] << 2);
				y = (int)((uint32_t)source[1] << 2);
			}
			const uint16_t attr = source[2];
			const uint32_t code = source[3] + ((uint32_t)(attr & 3) << 16);
			const int zoomx = source[4], zoomy = source[5];
			const uint16_t size = source[6];
			const int tw = ((size >> 8) & 0x1f) * 16, th = (size & 0x1f) * 16;
			if (!tw || !th)
				continue;
			clk += (uint32_t)(tw * th);
			if (clk > s_max_spr_clk)
				break;
			int flipx = (attr >> 3) & 1, flipy = (attr >> 2) & 1;
			const int twf = tw * zoomx, thf = th * zoomy;
			const int totw = twf / 0x100, toth = thf / 0x100;
			if (totw <= 1)
				x -= 0x80;
			if (toth <= 1)
				y -= 0x80;
			if (!fixed) {
				x >>= 8;
				y >>= 8;
				if (flipx && zoomx != 0x100) x += tw - totw;
				if (flipy && zoomy != 0x100) y += th - toth;
			} else {
				if (flipx && zoomx != 0x100) x += (tw << 8) - twf - 0x80;
				if (flipy && zoomy != 0x100) y += (th << 8) - thf - 0x80;
				x >>= 8;
				y >>= 8;
			}
			x = sext10(x + s_mc->spr_xoff);
			y = sext10(y + s_mc->spr_yoff);
			if (x + totw <= 0 || x >= max_x || y + toth <= 0 || y >= max_y)
				continue;
			const uint16_t colpri = (uint16_t)(((attr & 0x30) << 10) | (attr & 0x3f00));
			if (glob_flipx) { x = s_mc->spr_xoff_f - x - totw; flipx = !flipx; }
			if (glob_flipy) { y = s_mc->spr_yoff_f - y - toth; flipy = !flipy; }
			if (zoomx == 0x100 && zoomy == 0x100) {
				if (y + th > miny && y <= maxy)
					spr_nozoom(code, colpri, x, y, flipx, flipy, tw, th, miny, maxy);
			} else if (y + toth + 2 > miny && y <= maxy)   // (spr_zoom clips to the band itself)
				spr_zoom(code, colpri, x, y, flipx, flipy, tw, th, zoomx, zoomy, miny, maxy);
		}
	}
}

// one layer into the line: key = tile priority << 4 | layer priority << 2 | layer (the drawing order), the greatest
// kept; by runs of a tile's pixels
static void draw_layer_line(int chip, int y, int8_t *key, uint16_t *col)
{
	const uint16_t *vr = s_snap->tmregs[chip];
	if (vr[2] & 0x10)   // (disabled)
		return;
	const tgfx_t *g = &s_tgfx[chip];
	if (!g->count)
		return;
	const uint16_t *vram = s_snap->tm[chip];
	const int only8 = s_only8[chip];
	const int tiledim = only8 ? 0 : (vr[1] >> 13) & 1;   // (prepare(): 16x16 when set)
	const int flipx = !(vr[0] & 0x8000), flipy = !(vr[1] & 0x8000);
	const int rowscroll = !only8 && (vr[0] & 0x4000), rowselect = !only8 && (vr[1] & 0x4000);
	const int lpri = vr[2] & 3;
	const int offs_x = s_offs_x - (tiledim ? 1 + chip : 1 + chip + 8);
	const int offs_y = s_offs_y;
	const int offs_row = flipy ? s_row_f : s_row_n;
	const int sx = (vr[0] & 0x1ff) - s_snap->vregs01[0] + (flipx ? offs_x + 2 : -offs_x);
	const int sy = (vr[1] & 0x1ff) - s_snap->vregs01[1] + (flipy ? offs_y + 2 : -offs_y);
	const int l = flipy ? s_mc->sh - 1 - y : y;
	const uint16_t *lineram = vram + 0x800;
	const int ly = (rowselect ? lineram[((sy + offs_row + l) & 0x1ff) * 2 + 1] : l + sy) & 511;
	const int xs = sx + (rowscroll ? lineram[((sy + offs_row + l) & 0x1ff) * 2] : 0);
	const int base = s_mc->gbase[chip], gran = s_mc->ggran[chip];
	const int cmask = (s_mc->gfx[chip] == G_6 || s_mc->gfx[chip] == G_6_2) ? 0x0f : 0x3f;   // (color % the gfx's colors)
	const int bank = s_m == M_SAILORMN && chip == 2 && s_snap->tilebank;
	const int vx0 = s_mc->vx0, vw = s_mc->vw, ry = ly & 7;
	const int step = flipx ? -1 : 1;
	const int lk = lpri << 2 | chip;
	const uint8_t *empty = g->empty;
	for (int x = 0; x < vw;) {
		const int X = vx0 + x;
		const int lx = (flipx ? s_mc->sw - 1 - X + xs : X + xs) & 511;
		int n = flipx ? (lx & 7) + 1 : 8 - (lx & 7);
		if (n > vw - x)
			n = vw - x;
		const int tc = lx >> 3;
		uint32_t tile, code;
		if (tiledim) {
			const int idx = (ly >> 4) * 32 + (lx >> 4);
			tile = (uint32_t)vram[idx * 2] << 16 | vram[idx * 2 + 1];
			code = (tile & 0xffff) * 4;
			code ^= (uint32_t)(tc & 1);
			code ^= (uint32_t)(((ly >> 3) & 1) * 2);
		} else {
			const int idx = (ly >> 3) * 64 + tc;
			tile = (uint32_t)vram[0x2000 + idx * 2] << 16 | vram[0x2000 + idx * 2 + 1];
			code = tile & 0x3ffff;
			if (bank && code < 0x10000)
				code += 0x40000;
		}
		code = rbx_wrap(code, g->count);
		if (empty[code]) {
			x += n;
			continue;
		}
		const int k = (int)(tile >> 30) << 4 | lk;
		const int cbase = base + (int)((tile >> 24) & (uint32_t)cmask) * gran;
		int px = lx & 7;
		int8_t *kp = key + x;
		uint16_t *cp = col + x;
		if (g->bpp8) {
			const uint8_t *row = g->data + code * 64 + (uint32_t)(ry * 8);
			for (int i = 0; i < n; i++, px += step) {
				const uint8_t pen = row[px];
				if (pen && k > kp[i]) {
					kp[i] = (int8_t)k;
					cp[i] = (uint16_t)(cbase + pen);
				}
			}
		} else {
			const uint8_t *row = g->data + code * 32 + (uint32_t)(ry * 4);
			for (int i = 0; i < n; i++, px += step) {
				const uint8_t v = row[px >> 1];
				const uint8_t pen = (px & 1) ? (v & 15) : (v >> 4);
				if (pen && k > kp[i]) {
					kp[i] = (int8_t)k;
					cp[i] = (uint16_t)(cbase + pen);
				}
			}
		}
		x += n;
	}
}

static void run_band(int band)
{
	const int vw = s_mc->vw, vh = s_mc->vh;
	const int y0 = vh * band / BANDS, y1 = vh * (band + 1) / BANDS;
	draw_sprites(s_snap->spr, s_snap->svregs, y0, y1);
	int8_t key[512];
	uint16_t col[512];
	const int sbase = s_mc->spr_base, sgran = s_mc->spr_gran, scolors = s_mc->spr_colors;
	for (int y = y0; y < y1; y++) {
		memset(key, -1, (size_t)vw);
		for (int x = 0; x < vw; x++)
			col[x] = s_bg_pen;
		for (int chip = 0; chip < s_mc->layers; chip++)
			draw_layer_line(chip, y, key, col);
		const uint16_t *sl = s_sbm + y * vw;
		for (int x = 0; x < vw; x++) {
			const uint16_t pixel = sl[x];
			if (!pixel)
				continue;
			const int pen = pixel & 0xff, cp = (pixel >> 8) & 0xff;
			const int dstpri = key[x] < 0 ? 0 : (key[x] >> 4) + 1;
			int color;
			if (s_mc->pwrinst2_pri) {
				if (dstpri > ((cp >> 6) & 1) + 2)
					continue;
				color = (cp & 0x3f) | ((cp & 0x80) >> 1);
			} else {
				if (dstpri > ((cp >> 6) & 3))
					continue;
				color = cp & 0x3f;
			}
			col[x] = (uint16_t)(sbase + pen + (color % scolors) * sgran);
		}
		memcpy(rbx_video.bitmap + y * rbx_video.pitch, col, sizeof(uint16_t) * (size_t)vw);
	}
}

static int s_band_next, s_band_left, s_band_active, s_band_helpers;

static void cave_render(void)
{
	// the palette: xGRB_555
	if (s_snap->pal_dirty || !s_pal_conv_valid) {
		const uint32_t n = s_pal_bytes / 2;
		for (uint32_t i = 0; i < n; i++) {
			const uint16_t p = s_snap->pal[i];
			if (s_pal_conv_valid && s_pal_conv[i] == p)
				continue;
			s_pal_conv[i] = p;
			const int g = (p >> 10) & 31, r = (p >> 5) & 31, bb = p & 31;
			rbx_video.palette[i] = (uint32_t)((r << 3 | r >> 2) << 16 | (g << 3 | g >> 2) << 8 | (bb << 3 | bb >> 2));
		}
		s_pal_conv_valid = 1;
		s_snap->pal_dirty = 0;
	}
	// the frame in bands: the render thread takes them, the threads waiting for it help (cave_render_help)
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

static int cave_render_help(int helper)
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

// ---- the frame ----
static void vblank_start(void)
{
	b.vblank_irq = 1;
	update_irq();
	// get_sprite_info: the bank shown (delayed a frame: mazinger, metmqstr)
	const int bankmax = (int)(s_spr_bytes / 0x4000);
	const int bank = (b.vregs[4] & 3) % (bankmax ? bankmax : 1);
	int show = bank;
	if (s_mc->spr_delay) {
		show = b.spr_bank_delay;
		b.spr_bank_delay = (uint8_t)bank;
	}
	memcpy(s_spr_pending, b.spr + show * 0x2000, sizeof(s_spr_pending));
	for (int i = 0; i < 6; i++)
		s_svregs_pending[i] = b.vregs[i];
	b.agallet_vblank_irq = 1;
	b.vend_line = s_vstart_line + s_vend_lines;   // (2000 us later: may be in the next frame)
}
static void vblank_end(void)
{
	if (s_kludge == 3) {
		b.unknown_irq = 1;
		update_irq();
	}
	b.agallet_vblank_irq = 0;
}

static void frame_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap->spr, s_spr_pending, sizeof(s_spr_pending));
	memcpy(s_snap->svregs, s_svregs_pending, sizeof(s_svregs_pending));
	for (int c = 0; c < s_mc->layers; c++) {
		memcpy(s_snap->tm[c], b.tm[c].vram, sizeof(b.tm[c].vram));
		memcpy(s_snap->tmregs[c], b.tmregs[c], sizeof(b.tmregs[c]));
	}
	s_snap->vregs01[0] = b.vregs[0];
	s_snap->vregs01[1] = b.vregs[1];
	if (memcmp(s_snap->pal, b.pal, s_pal_bytes)) {
		memcpy(s_snap->pal, b.pal, s_pal_bytes);
		s_snap->pal_dirty = 1;
	}
	s_snap->tilebank = b.sailormn_tilebank;
	rbx_video_frame_done();
}

// the sound chips over a line, mixed into s_mix at the output rate
static void sound_line(void)
{
	if (s_mc->snd == SND_Z80) {
		const uint32_t clocks = (uint32_t)rbx_line_cycles(&b.ym_frac, s_ymclock, FRAME_MHZ, LINES);
		const int n = rbx_ym_advance(0, clocks, s_mix + s_mix_n, s_mix_cap - s_mix_n);
		for (int i = 0; i < n; i++) {
			int32_t s = 0;
			for (int c = 0; c < s_noki; c++) {
				b.oki_acc[c] += s_oki[c].rate;
				while (b.oki_acc[c] >= s_out_rate) {
					b.oki_acc[c] -= s_out_rate;
					b.oki_hold[c] = oki_sample(&s_oki[c]);
				}
				s += b.oki_hold[c];
			}
			s_mix[s_mix_n + i] += s;
		}
		s_mix_n += n;
	} else if (s_mc->snd == SND_OKI2) {
		int n = rbx_line_cycles(&b.out_frac, s_out_rate, FRAME_MHZ, LINES);
		if (n > s_mix_cap - s_mix_n)
			n = s_mix_cap - s_mix_n;
		for (int i = 0; i < n; i++) {
			int32_t s = 0;
			for (int c = 0; c < 2; c++) {
				b.oki_acc[c] += s_oki[c].rate;
				while (b.oki_acc[c] >= s_out_rate) {
					b.oki_acc[c] -= s_out_rate;
					b.oki_hold[c] = oki_sample(&s_oki[c]);
				}
				s += b.oki_hold[c];
			}
			s_mix[s_mix_n + i] = s;
		}
		s_mix_n += n;
	}
}

static void z80_to(uint64_t target)
{
	if (target > s_z80.cyc)
		z80_run(&s_z80, (unsigned long)target, &s_ym_irq, 0);
}

static void run_line(void)
{
	const int cyc = rbx_line_cycles(&b.cpu_frac, CPU_CLOCK, FRAME_MHZ, LINES);
	s_line_cyc = cyc;
	if (s_mc->snd == SND_YMZ) {
		const uint32_t rate = ymz280b_rate(&s_ymz);
		s_ymz_line0 = (uint32_t)s_ymz_n;
		s_ymz_line_n = (uint32_t)rbx_line_cycles(&b.out_frac, rate, FRAME_MHZ, LINES);
		m68k_execute(cyc);
		ymz_to((int)(s_ymz_line0 + s_ymz_line_n));
		return;
	}
	if (s_mc->snd == SND_OKI2) {
		m68k_execute(cyc);
		sound_line();
		return;
	}
	// with the sound CPU: the 68000 spins 50 us after a command while the Z80 runs
	const uint64_t z0 = s_z80.cyc;
	const int zcyc = rbx_line_cycles(&b.z80_frac, s_zclock, FRAME_MHZ, LINES);
#define ZAT(d) (z0 + (uint64_t)zcyc * (uint32_t)((d) < cyc ? (d) : cyc) / (uint32_t)cyc)
	int done = 0;
	if (b.spin > 0) {   // (the rest of a spin: the sound CPU runs it first)
		const int k = b.spin < cyc ? b.spin : cyc;
		b.spin -= k;
		done += k;
		z80_to(ZAT(done));
	}
	while (done < cyc) {
		done += m68k_execute(cyc - done);
		if (b.spin < 0) {   // (a command: the sound CPU catches up, then runs the 50 us the 68000 spins)
			z80_to(ZAT(done));
			b.spin = SPIN_CYC;
			const int k = b.spin < cyc - done ? b.spin : (cyc - done > 0 ? cyc - done : 0);
			b.spin -= k;
			done += k;
			z80_to(ZAT(done));
		}
	}
#undef ZAT
	z80_to(z0 + (uint64_t)zcyc);
	sound_line();
}

static void soft_reset(void);

static void cave_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	for (int line = 0; line < LINES; line++) {
		if (line == s_vstart_line)
			vblank_start();
		if (b.vend_line >= 0) {
			if (b.vend_line == line) {
				b.vend_line = -1;
				vblank_end();
			}
		}
		if (b.startup_line >= 0 && line == b.startup_line) {   // (sailormn_startup: the main CPU reset)
			b.startup_line = -1;
			m68k_pulse_reset();
		}
		run_line();
	}
	if (b.vend_line >= LINES)
		b.vend_line -= LINES;
	frame_capture();
	// sound out
	if (s_mc->snd == SND_YMZ) {
		for (int i = 0; i < s_ymz_n; i++)
			rbx_audio_put(s_ymz_l[i] + s_ymz_r[i]);
		memset(s_ymz_l, 0, sizeof(int32_t) * (size_t)s_ymz_n);
		memset(s_ymz_r, 0, sizeof(int32_t) * (size_t)s_ymz_n);
		s_ymz_n = 0;
	} else {
		if (s_mc->snd == SND_Z80)
			rbx_ym_async_take(0, s_mix, s_mix_n);
		for (int i = 0; i < s_mix_n; i++)
			rbx_audio_put(s_mix[i]);
		memset(s_mix, 0, sizeof(int32_t) * (size_t)s_mix_n);
		s_mix_n = 0;
	}
	b.frame++;
	if (s_wdog_frames && ++b.wdog >= s_wdog_frames)
		soft_reset();
}

// ---- init / reset ----
static void cave_exit(void);

// the machine's reset (devices and CPUs; the memory kept: the watchdog's reset, which mazinger waits for)
static void soft_reset(void)
{
	b.vblank_irq = b.sound_irq = b.unknown_irq = b.agallet_vblank_irq = 0;
	b.soundbuf_wptr = b.soundbuf_rptr = 0;
	memset(b.soundbuf_data, 0, sizeof(b.soundbuf_data));
	b.soundbuf_empty = 1;
	b.latch_pending = 0;
	b.spin = 0;
	b.wdog = 0;
	memset(b.nmk_bank, 0, sizeof(b.nmk_bank));
	b.vend_line = -1;
	b.startup_line = (s_m == M_SAILORMN || s_m == M_AGALLET) ? 1000 * LINES / FRAME_US : -1;
	s_ym_irq = 0;
	if (s_mc->snd == SND_YMZ)
		ymz280b_reset(&s_ymz);
	for (int c = 0; c < s_noki; c++)
		oki_reset(&s_oki[c]);
	if (s_mc->snd == SND_Z80) {
		z80_start();
		rbx_ym_reset(0);
	}
	m68k_bus_irq_reset();
	m68k_pulse_reset();
}

static void cave_reset(void)
{
	memset(&b, 0, offsetof(__typeof__(b), state_end));
	if (s_mc->snd == SND_Z80) {
		memset(s_zram, 0, sizeof(s_zram));
		memset(s_zmap, 0, sizeof(s_zmap));
		const uint32_t fixed = s_m == M_PWRINST2 ? 0x8000 : 0x4000;
		for (uint32_t p = 0; p < fixed / 256 && p * 256 + 256 <= s_zrom_size; p++)
			s_zmap[p] = s_zrom + p * 256;
		z80_set_bank(0, 0xff);
	}
	s_ymz_n = 0;
	s_mix_n = 0;
	s_pal_conv_valid = 0;
	soft_reset();
}

static void rom_map(uint32_t s, uint32_t e)   // the main CPU's region at the same offsets
{
	if (s >= s_rom_size)
		return;
	const uint32_t n = e - s + 1 <= s_rom_size - s ? e - s + 1 : s_rom_size - s;
	m68k_bus_map(s, s + n - 1, s_rom + s, n, 1, 0);
}

static int map_tmap(int chip, uint32_t start, int only8, uint32_t end8)
{
	s_only8[chip] = only8;
	if (only8)   // vram_8x8_map (mirrored up to end8): the 8x8 part only
		m68k_bus_map(start, end8, b.tm[chip].vram + 0x2000, 0x4000, 1, 1);
	else
		m68k_bus_map(start, start + 0x7fff, b.tm[chip].vram, 0x8000, 1, 1);
	return 0;
}

static int cave_init(const rbx_game *g)
{
	if (!s_pal_conv_hp && !(s_pal_conv_hp = calloc(1, sizeof(*s_pal_conv_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	if (!s_bp && !(s_bp = (struct cave_state *)calloc(1, sizeof(*s_bp)))) {
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
	s_mc = &s_machines[s_m];
	memset(s_tbuf, 0, sizeof(s_tbuf));
	memset(s_tgfx, 0, sizeof(s_tgfx));
	s_rom = rbx_region("maincpu", &s_rom_size);
	uint32_t ssize = 0;
	uint8_t *spr = rbx_region("sprites0", &ssize);
	if (!s_rom || !spr) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	m68k_bus_swap16(s_rom, s_rom_size);
	s_user1 = rbx_region("user1", &s_user1_size);
	if (s_user1)
		m68k_bus_swap16(s_user1, s_user1_size);
	if (!strcmp(g->set, "pwrinst2a") && s_rom_size > 0xd46c)   // init_pwrinst2a: a move's fix
		((uint16_t *)s_rom)[0xd46c / 2] = 0xd482;
	if ((s_m == M_MAZINGER || s_m == M_PWRINST2 || s_m == M_SAILORMN) && decrypt_sprites(spr, ssize)) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	decode_sprites(spr, ssize);
	for (int c = 0; c < s_mc->layers; c++) {
		char name[8];
		snprintf(name, sizeof(name), "layer%d", c);
		uint32_t ts = 0;
		uint8_t *t = rbx_region(name, &ts);
		if (!t || decode_tiles(c, t, ts) || tiles_empty(c)) {
			rbx_fail("%s: %s missing", g->set, name);
			cave_exit();
			return -1;
		}
	}
	// init_* (time of the vblank IRQ before the frame's end, IRQ level, kludges)
	s_kludge = 0;
	s_tv_us = 100;
	s_irq_level = 1;
	switch (s_m) {
	case M_DFEVERON: s_kludge = 2; break;
	case M_DONPACHI: case M_DDONPACH: s_tv_us = 90; break;
	case M_ESPRADE: case M_GUWANGE: case M_GAIA: case M_HOTDOGST: s_tv_us = 2000; break;
	case M_UOPOKO: s_tv_us = 2000; s_kludge = 2; break;
	case M_KOROKORO: case M_CRUSHERM: s_tv_us = 2000; s_irq_level = 2; break;
	case M_MAZINGER: s_tv_us = 2100; s_kludge = 3; break;
	case M_METMQSTR: s_tv_us = 17376; s_kludge = 3; break;
	case M_PWRINST2: s_tv_us = 2000; s_kludge = 4; break;
	case M_SAILORMN: s_tv_us = 2000; s_kludge = 1; break;
	default: break;
	}
	if (!strcmp(g->set, "feversos") || !strcmp(g->set, "dfeveron"))
		s_kludge = 2;
	s_wdog_frames = (s_m == M_MAZINGER || s_m == M_METMQSTR) ? 3 * FRAME_MHZ / 1000 : 0;   // (3 s, "a guess")
	s_vstart_line = (FRAME_US - s_tv_us) * LINES / FRAME_US;
	s_vend_lines = 2000 * LINES / FRAME_US;
	// video_start
	s_offs_x = 0x13;
	s_offs_y = -0x12;
	s_row_n = -1;
	s_row_f = 1;
	s_bg_pen = (uint16_t)(s_mc->gbase[0] + 63 * s_mc->ggran[0]);
	if (s_kludge == 1)
		s_row_f = -1;
	if (s_kludge == 2 || s_kludge == 4)
		s_bg_pen = (uint16_t)(s_mc->gbase[0] - s_mc->ggran[0]);
	if (s_kludge == 4)
		s_offs_y++;
	s_max_spr_clk = (uint32_t)((s_mc->vw > 360 ? 512 : 448) * 272 * 2);
	// memory
	memset(&b, 0, sizeof(b));
	s_spr_bytes = (s_m == M_KOROKORO || s_m == M_CRUSHERM) ? 0x8000 : 0x10000;
	m68k_init();
	m68k_set_cpu_type(M68K_CPU_TYPE_68000);
	m68k_bus_reset_map();
	m68k_bus.read8 = main_r8;
	m68k_bus.read16 = main_r16;
	m68k_bus.write8 = main_w8;
	m68k_bus.write16 = main_w16;
#define ROM(s, e) rom_map(s, e)
#define RAM(s, e, p) m68k_bus_map(s, e, p, (e) - (s) + 1, 1, 1)
	switch (s_m) {
	case M_DFEVERON:
		ROM(0, 0xfffff); RAM(0x100000, 0x10ffff, b.ram); RAM(0x400000, 0x40ffff, b.spr);
		map_tmap(0, 0x500000, 0, 0); map_tmap(1, 0x600000, 0, 0);
		RAM(0x708000, 0x708fff, b.pal); RAM(0x710000, 0x710fff, b.ram2);
		s_io = s_io_dfeveron; s_pal_bytes = 0x1000;
		break;
	case M_DDONPACH:
		ROM(0, 0xfffff); RAM(0x100000, 0x10ffff, b.ram); RAM(0x400000, 0x40ffff, b.spr);
		map_tmap(0, 0x500000, 0, 0); map_tmap(1, 0x600000, 0, 0); map_tmap(2, 0x700000, 1, 0x70ffff);
		RAM(0xc00000, 0xc0ffff, b.pal);
		s_io = s_io_ddonpach; s_pal_bytes = 0x10000;
		break;
	case M_DONPACHI:
		ROM(0, 0x7ffff); RAM(0x100000, 0x10ffff, b.ram);
		map_tmap(1, 0x200000, 0, 0); map_tmap(0, 0x300000, 0, 0); map_tmap(2, 0x400000, 1, 0x407fff);
		RAM(0x500000, 0x50ffff, b.spr); RAM(0xa08000, 0xa08fff, b.pal);
		s_io = s_io_donpachi; s_pal_bytes = 0x1000;
		break;
	case M_ESPRADE:
	case M_GAIA:
		ROM(0, 0xfffff); RAM(0x100000, 0x10ffff, b.ram); RAM(0x400000, 0x40ffff, b.spr);
		map_tmap(0, 0x500000, 0, 0); map_tmap(1, 0x600000, 0, 0); map_tmap(2, 0x700000, 0, 0);
		if (s_m == M_GAIA) {   // (more layer RAM, tested)
			RAM(0x508000, 0x50ffff, b.ram2); RAM(0x608000, 0x60ffff, b.ram2 + 0x4000);
			RAM(0x708000, 0x70ffff, b.misc);
		}
		RAM(0xc00000, 0xc0ffff, b.pal);
		s_io = s_m == M_GAIA ? s_io_gaia : s_io_ddonpach; s_pal_bytes = 0x10000;
		break;
	case M_GUWANGE:
		ROM(0, 0xfffff); RAM(0x200000, 0x20ffff, b.ram); RAM(0x400000, 0x40ffff, b.spr);
		map_tmap(0, 0x500000, 0, 0); map_tmap(1, 0x600000, 0, 0); map_tmap(2, 0x700000, 0, 0);
		RAM(0xc00000, 0xc0ffff, b.pal);
		s_io = s_io_guwange; s_pal_bytes = 0x10000;
		break;
	case M_HOTDOGST:
		ROM(0, 0xfffff); RAM(0x300000, 0x30ffff, b.ram); RAM(0x408000, 0x408fff, b.pal);
		map_tmap(0, 0x880000, 0, 0); map_tmap(1, 0x900000, 0, 0); map_tmap(2, 0x980000, 0, 0);
		RAM(0xf00000, 0xf0ffff, b.spr);
		s_io = s_io_hotdogst; s_pal_bytes = 0x1000;
		break;
	case M_KOROKORO:
	case M_CRUSHERM:
		ROM(0, 0x7ffff); map_tmap(0, 0x100000, 0, 0); RAM(0x180000, 0x187fff, b.spr);
		RAM(0x200000, 0x207fff, b.pal);
		if (s_m == M_KOROKORO) {
			RAM(0x300000, 0x30ffff, b.ram);
			s_io = s_io_korokoro;
		} else {
			RAM(0x340000, 0x34ffff, b.ram);
			s_io = s_io_crusherm;
		}
		s_pal_bytes = 0x8000;
		break;
	case M_MAZINGER:
		ROM(0, 0x7ffff); RAM(0x100000, 0x10ffff, b.ram); RAM(0x200000, 0x20ffff, b.spr);
		map_tmap(1, 0x400000, 1, 0x407fff); map_tmap(0, 0x500000, 1, 0x507fff);
		RAM(0xc08000, 0xc0ffff, b.pal);
		if (s_user1)
			m68k_bus_map(0xd00000, 0xd7ffff, s_user1, s_user1_size, 1, 0);
		s_io = s_io_mazinger; s_pal_bytes = 0x8000;
		break;
	case M_METMQSTR:
		ROM(0, 0x7ffff); ROM(0x100000, 0x17ffff); ROM(0x200000, 0x27ffff);
		RAM(0x408000, 0x408fff, b.pal);
		map_tmap(2, 0x880000, 0, 0); RAM(0x888000, 0x88ffff, b.ram2);
		map_tmap(1, 0x900000, 0, 0); RAM(0x908000, 0x90ffff, b.ram2 + 0x4000);
		map_tmap(0, 0x980000, 0, 0); RAM(0x988000, 0x98ffff, b.misc);
		RAM(0xf00000, 0xf0ffff, b.spr);
		s_io = s_io_metmqstr; s_pal_bytes = 0x1000;
		break;
	case M_PWRINST2:
		ROM(0, 0x1fffff); RAM(0x400000, 0x40ffff, b.ram);
		if (s_user1)
			m68k_bus_map(0x600000, 0x6fffff, s_user1, s_user1_size, 1, 0);
		map_tmap(2, 0x800000, 0, 0); map_tmap(0, 0x880000, 0, 0); map_tmap(1, 0x900000, 0, 0);
		map_tmap(3, 0x980000, 1, 0x987fff);
		RAM(0xa00000, 0xa0ffff, b.spr); RAM(0xa10000, 0xa1ffff, b.ram2);
		RAM(0xf00000, 0xf04fff, b.pal);
		s_io = s_io_pwrinst2; s_pal_bytes = 0x5000;
		break;
	case M_AGALLET:
	case M_SAILORMN:
		ROM(0, 0x7ffff); RAM(0x100000, 0x10ffff, b.ram); RAM(0x110000, 0x110001, b.tiny);
		ROM(0x200000, 0x3fffff); RAM(0x400000, 0x407fff, b.misc); RAM(0x408000, 0x40bfff, b.pal);
		RAM(0x40c000, 0x40ffff, b.ram2); RAM(0x410000, 0x410001, b.tiny + 1);
		RAM(0x500000, 0x50ffff, b.spr); RAM(0x510000, 0x510001, b.tiny + 2);
		map_tmap(0, 0x800000, 0, 0); map_tmap(1, 0x880000, 0, 0); map_tmap(2, 0x900000, 0, 0);
		RAM(0x908000, 0x908001, b.tiny + 3);
		s_io = s_io_sailormn; s_pal_bytes = 0x4000;
		break;
	case M_UOPOKO:
		ROM(0, 0xfffff); RAM(0x100000, 0x10ffff, b.ram); RAM(0x400000, 0x40ffff, b.spr);
		map_tmap(0, 0x500000, 0, 0); RAM(0x800000, 0x80ffff, b.pal);
		s_io = s_io_uopoko; s_pal_bytes = 0x10000;
		break;
	}
#undef ROM
#undef RAM
	s_port_in0 = rbx_port_find("IN0");
	s_port_in1 = rbx_port_find("IN1");
	s_port_dsw = rbx_port_find("DSW");
	s_in1_eep = 0;
	if (s_port_in1 >= 0)
		for (int i = g->ports[s_port_in1].first; i < g->ports[s_port_in1].first + g->ports[s_port_in1].count; i++)
			if (g->fields[i].type == RBX_IPT_CUSTOM)
				s_in1_eep |= g->fields[i].mask;
	// the EEPROM: its region's contents first, then the saved file
	s_nv_path[0] = 0;
	eeprom_init(&s_eeprom);
	{
		uint32_t es;
		const uint8_t *e = rbx_region("eeprom", &es);
		for (uint32_t i = 0; e && i < 64 && 2 * i + 1 < es; i++)
			s_eeprom.data[i] = (uint16_t)(e[2 * i] << 8 | e[2 * i + 1]);
		extern const char *rbx_nvram_dir(void);
		const char *dir = rbx_nvram_dir();
		if (dir && s_m != M_GAIA) {
			snprintf(s_nv_path, sizeof(s_nv_path), "%s%s.nv", dir, g->set);
			eeprom_load(&s_eeprom, s_nv_path);
		}
	}
	// sound
	s_noki = 0;
	s_snap = (__typeof__(s_snap))calloc(1, sizeof(*s_snap));
	s_sbm = (uint16_t *)calloc((size_t)s_mc->vw * (size_t)s_mc->vh, sizeof(uint16_t));
	if (!s_snap || !s_sbm) {
		rbx_fail("%s: out of memory", g->set);
		cave_exit();
		return -1;
	}
	if (s_mc->snd == SND_YMZ) {
		uint32_t ys = 0;
		const uint8_t *ymz = rbx_region("ymz", &ys);
		ymz280b_init(&s_ymz, s_m == M_GAIA ? 16000000 : 16934400, ymz, ys, ymz_irq, NULL);
		s_out_rate = ymz280b_rate(&s_ymz);
		s_ymz_cap = (int)(s_out_rate / 50) + 64;
		s_ymz_l = (int32_t *)calloc((size_t)s_ymz_cap, sizeof(int32_t));
		s_ymz_r = (int32_t *)calloc((size_t)s_ymz_cap, sizeof(int32_t));
		if (!s_ymz_l || !s_ymz_r) {
			rbx_fail("%s: out of memory", g->set);
			cave_exit();
			return -1;
		}
	} else {
		static const char *const okiname[2] = { "oki1", "oki2" };
		// clocks, pin 7, gains (MAME add_route) per board
		uint32_t oclk[2] = { 0, 0 };
		int pin7 = 1, gain[2] = { 256, 256 };
		s_ymclock = 4000000;
		switch (s_m) {
		case M_DONPACHI: s_noki = 2; oclk[0] = 4220000 / 4; oclk[1] = 4220000 / 2; gain[0] = 410; break;
		case M_HOTDOGST: s_noki = 1; oclk[0] = 2000000; s_zclock = 4000000; break;
		case M_MAZINGER: s_noki = 1; oclk[0] = 1056000; gain[0] = 512; s_zclock = 4000000; break;
		case M_METMQSTR: s_noki = 2; oclk[0] = oclk[1] = 2000000; gain[0] = gain[1] = 128; s_zclock = 8000000; break;
		case M_PWRINST2: s_noki = 2; oclk[0] = oclk[1] = 3000000; pin7 = 0; gain[0] = 205; s_zclock = 8000000; break;
		default: s_noki = 2; oclk[0] = oclk[1] = 2112000; s_zclock = 8000000; break;   // sailormn / agallet
		}
		for (int c = 0; c < s_noki; c++) {
			s_oki_rom[c] = rbx_region(okiname[c], &s_oki_size[c]);
			if (!s_oki_rom[c]) {
				rbx_fail("%s: %s missing", g->set, okiname[c]);
				cave_exit();
				return -1;
			}
			oki_init(&s_oki[c], oclk[c], pin7, s_oki_rom[c], s_oki_size[c], gain[c]);
			s_oki[c].user = (void *)(intptr_t)c;
			s_oki[c].read = (s_m == M_DONPACHI || s_m == M_PWRINST2) ? oki_nmk_read : oki_latch_read;
		}
		s_nmk_paged[0] = s_m != M_DONPACHI;   // (donpachi: chip 0, the music, not paged)
		s_nmk_paged[1] = 1;
		if (s_mc->snd == SND_Z80) {
			s_zrom = rbx_region("audiocpu", &s_zrom_size);
			if (!s_zrom) {
				rbx_fail("%s: audiocpu missing", g->set);
				cave_exit();
				return -1;
			}
			rbx_ym_config yc;
			memset(&yc, 0, sizeof(yc));
			yc.clock = s_ymclock;
			yc.irq = ym_irq;
			switch (s_m) {
			case M_HOTDOGST: yc.type = RBX_YM2203; yc.gain[0] = yc.gain[1] = yc.gain[2] = 51; yc.gain[3] = 205; break;
			case M_MAZINGER: yc.type = RBX_YM2203; yc.gain[0] = yc.gain[1] = yc.gain[2] = 51; yc.gain[3] = 154; break;
			case M_PWRINST2: yc.type = RBX_YM2203; yc.gain[0] = yc.gain[1] = yc.gain[2] = 102; yc.gain[3] = 205; break;
			case M_METMQSTR: yc.type = RBX_YM2151; yc.gain[0] = yc.gain[1] = 154; break;
			default: yc.type = RBX_YM2151; yc.gain[0] = yc.gain[1] = 77; break;
			}
			if (rbx_ym_init(0, &yc) || rbx_ym_set_async(0, 1)) {
				rbx_fail("%s: out of memory", g->set);
				cave_exit();
				return -1;
			}
			s_out_rate = rbx_ym_rate(0);
		} else
			s_out_rate = s_oki[1].rate;
		s_mix_cap = (int)(s_out_rate / 50) + 256;
		s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
		if (!s_mix) {
			rbx_fail("%s: out of memory", g->set);
			cave_exit();
			return -1;
		}
	}
	if (rbx_audio_setup(s_out_rate) || rbx_video_setup(s_mc->vw, s_mc->vh, 0x8000)) {
		rbx_fail("%s: out of memory", g->set);
		cave_exit();
		return -1;
	}
	cave_reset();
	return 0;
}

static void cave_exit(void)
{
	if (s_nv_path[0] && s_eeprom.dirty)
		eeprom_save(&s_eeprom, s_nv_path);
	s_nv_path[0] = 0;
	if (s_mc && s_mc->snd == SND_Z80)
		rbx_ym_exit(0);
	for (int c = 0; c < 4; c++) {
		free(s_tbuf[c]);
		s_tbuf[c] = NULL;
		free(s_tgfx[c].empty);
		s_tgfx[c].empty = NULL;
	}
	free(s_ymz_l);
	free(s_ymz_r);
	s_ymz_l = s_ymz_r = NULL;
	free(s_mix);
	s_mix = NULL;
	free(s_snap);
	s_snap = NULL;
	free(s_sbm);
	s_sbm = NULL;
	free(s_bp);   // (last: the render thread and the saves are done with it)
	s_bp = NULL;
	free(s_pal_conv_hp);
	s_pal_conv_hp = NULL;
}

static int cave_state(void *ctx, int load)
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
	rc |= rbx_state_io(ctx, "cave", &b, (uint32_t)offsetof(__typeof__(b), state_end));
	rc |= rbx_state_io(ctx, "eeprom", &s_eeprom, sizeof(s_eeprom));
	if (s_mc->snd == SND_YMZ)
		rc |= rbx_state_io(ctx, "ymz", &s_ymz, (uint32_t)YMZ280B_STATE_SIZE);
	for (int c = 0; c < s_noki; c++)
		rc |= rbx_state_io(ctx, c ? "oki2" : "oki1", s_oki[c].v, sizeof(s_oki[c].v) + sizeof(int));
	z80 z = s_z80;
	int32_t ymirq = s_ym_irq;
	if (s_mc->snd == SND_Z80) {
		rc |= rbx_state_io(ctx, "z80", &z, sizeof(z));
		rc |= rbx_state_io(ctx, "zram", s_zram, sizeof(s_zram));
		rc |= rbx_state_io(ctx, "ymirq", &ymirq, sizeof(ymirq));
		rc |= rbx_ym_state(0, ctx, load);
	}
	if (!rc && load) {
		m68k_set_context(cpu);
		m68k_bus_set_irq_pending(irq[0]);
		m68k_bus_cur->irq_asserted = irq[1];
		m68k_bus_update_irq();
		if (s_mc->snd == SND_Z80) {
			z.read_byte = z80_read;
			z.write_byte = z80_write;
			z.port_in = z80_in;
			z.port_out = z80_out;
			z.userdata = NULL;
			z.read_map = s_zmap;
			s_z80 = z;
			s_ym_irq = ymirq;
			z80_set_bank(b.z80_bank, 0xff);
		}
		s_pal_conv_valid = 0;
	}
	free(cpu);
	return rc ? -1 : 0;
}

static int cave_save(void *ctx) { return cave_state(ctx, 0); }
static int cave_load(void *ctx) { return cave_state(ctx, 1); }
static int cave_supported(const char *machine)
{
	for (int i = 0; i < M_COUNT; i++)
		if (!strcmp(machine, s_mnames[i]))
			return 1;
	return 0;
}

const rbx_driver rbx_drv_cave = { "Cave 68000", cave_init, cave_exit, cave_reset, cave_frame, cave_save, cave_load,
                                  cave_supported, cave_render, cave_render_help };
