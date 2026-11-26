/*  Sega System 1 / System 2. Ported from MAME src/mame/sega/system1.cpp, system1_v.cpp and system1.h (BSD-3-Clause,
    copyright-holders: Jarek Parchanski, Nicola Salmoria, Mirko Buffoni).
    Z80 at 4 MHz (an opcode fetch takes 2/5 of a cycle more, a video RAM access waits for the next FIXST) and a sound
    Z80 at 4 MHz with two SN76489A (2 and 4 MHz; the sound CPU waits for a write's READY). The outputs go through an
    8255 PPI (sound latch, video mode, sound control) or a Z80 PIO (sound latch with its strobe, video mode). Encrypted
    Z80s (315-50xx / 315-51xx / 317-000x, machine/segacrpt.c), the MC-8123 (machine/mc8123.c), the 8751 MCU of
    Choplifter / Shooting Master (it halts the Z80 and reaches its memory) and of Noboranka.
    Video: a fixed and a scrolling tilemap of 32x32 pages (System 1: two pages; System 2: eight, the scrolling one
    made of four chosen pages, optionally with row scroll), 32 line-buffered sprites with sprite / sprite and
    sprite / tile collision flags, mixed through a lookup PROM; 512 pixels a line. The lines are drawn on the
    emulation thread when MAME draws them (a collision access or a page change draws up to the beam; the rest at
    vblank), so the collision flags are the ones the game reads.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/z80/z80.h"
#include "../../cpu/mcs51/mcs51.h"
#include "../../sound/sn76496.h"
#include "../../machine/i8255.h"
#include "../../machine/z80pio.h"
#include "../../machine/segacrpt.h"
#include "../../video/gfxlayout.h"
#include "../machine/mc8123.h"
#include "../../rbx_prof.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define MASTER_CLOCK 20000000
#define SOUND_CLOCK 8000000
#define CPU_CLOCK (MASTER_CLOCK / 5)
#define FRAME_MHZ 60096           // set_raw(MASTER_CLOCK / 2, 640, 0, 512, 260, 0, 224)
#define LINES 260
#define VIS_H 224
#define SCR_W 512
#define NPENS 2048
#define AUDIO_RATE 62500
#define MCU_SLICES 16             // (config.set_maximum_quantum(maincpu clock / 16): sub-slices of a line with an 8751)

enum { IO_PPI, IO_PIO };
enum { V_SYS1, V_SYS2, V_SYS2ROW };
enum { OP_NONE, OP_DEC, OP_BANKED };   // the opcodes map: none, decrypted_opcodes_map, banked_decrypted_opcodes_map
enum { MC_NONE, MC_8123, MC_8123_BANKED };
enum { MCU_NONE, MCU_8751, MCU_NOB };
enum { IOM_STD, IOM_BLOCKGAL, IOM_SHTNGMST };

typedef struct {
	const char *name;
	uint8_t io, video, narrow, op, mc, mcu, nobo, iomap;
	const char *crypt;
	uint32_t snd_clk, sn0_clk, sn1_clk;
} s1_machine;

#define SC SOUND_CLOCK
static const s1_machine s_machines[] = {
	// name        io      video      narrow op       mc               mcu       nobo iomap         crypt       sound CPU / SN 1 / SN 2
	{ "sys1ppi",   IO_PPI, V_SYS1,    0, OP_NONE,   MC_NONE,        MCU_NONE, 0, IOM_STD,      NULL,       SC / 2, SC / 4, SC / 2 },
	{ "sys1pio",   IO_PIO, V_SYS1,    0, OP_NONE,   MC_NONE,        MCU_NONE, 0, IOM_STD,      NULL,       SC / 2, SC / 4, SC / 2 },
	{ "starjack",  IO_PPI, V_SYS1,    1, OP_NONE,   MC_NONE,        MCU_NONE, 0, IOM_STD,      NULL,       MASTER_CLOCK / 8, MASTER_CLOCK / 8, MASTER_CLOCK / 16 },
	{ "upndown",   IO_PPI, V_SYS1,    0, OP_DEC,    MC_NONE,        MCU_NONE, 0, IOM_STD,      "315-5098", MASTER_CLOCK / 8, MASTER_CLOCK / 8, MASTER_CLOCK / 8 },
	{ "regulus",   IO_PPI, V_SYS1,    0, OP_DEC,    MC_NONE,        MCU_NONE, 0, IOM_STD,      "315-5033", SC / 2, SC / 4, SC / 2 },
	{ "mrviking",  IO_PPI, V_SYS1,    1, OP_DEC,    MC_NONE,        MCU_NONE, 0, IOM_STD,      "315-5041", SC / 2, SC / 4, SC / 2 },
	{ "swat",      IO_PPI, V_SYS1,    0, OP_DEC,    MC_NONE,        MCU_NONE, 0, IOM_STD,      "315-5048", SC / 2, SC / 4, SC / 2 },
	{ "flickyo",   IO_PPI, V_SYS1,    0, OP_DEC,    MC_NONE,        MCU_NONE, 0, IOM_STD,      "315-5051", SC / 2, SC / 4, SC / 2 },
	{ "wmatch",    IO_PPI, V_SYS1,    1, OP_DEC,    MC_NONE,        MCU_NONE, 0, IOM_STD,      "315-5064", SC / 2, SC / 4, SC / 2 },
	{ "bullfgt",   IO_PPI, V_SYS1,    0, OP_DEC,    MC_NONE,        MCU_NONE, 0, IOM_STD,      "315-5065", SC / 2, SC / 4, SC / 2 },
	{ "wboy2",     IO_PPI, V_SYS1,    0, OP_DEC,    MC_NONE,        MCU_NONE, 0, IOM_STD,      "315-5178", SC / 2, SC / 4, SC / 2 },
	{ "wboy6",     IO_PPI, V_SYS1,    0, OP_DEC,    MC_NONE,        MCU_NONE, 0, IOM_STD,      "315-5179", SC / 2, SC / 4, SC / 2 },
	{ "nob",       IO_PPI, V_SYS1,    0, OP_NONE,   MC_NONE,        MCU_NOB,  1, IOM_STD,      NULL,       SC / 2, SC / 4, SC / 4 },
	{ "nobb",      IO_PPI, V_SYS1,    0, OP_NONE,   MC_NONE,        MCU_NONE, 1, IOM_STD,      NULL,       SC / 2, SC / 4, SC / 4 },
	{ "flicky",    IO_PIO, V_SYS1,    0, OP_DEC,    MC_NONE,        MCU_NONE, 0, IOM_STD,      "315-5051", SC / 2, SC / 4, SC / 2 },
	{ "thetogyu",  IO_PIO, V_SYS1,    0, OP_DEC,    MC_NONE,        MCU_NONE, 0, IOM_STD,      "315-5065", SC / 2, SC / 4, SC / 2 },
	{ "spatter",   IO_PIO, V_SYS1,    1, OP_DEC,    MC_NONE,        MCU_NONE, 0, IOM_STD,      "315-5096", SC / 2, SC / 4, SC / 2 },
	{ "spattera",  IO_PIO, V_SYS1,    1, OP_DEC,    MC_NONE,        MCU_NONE, 0, IOM_STD,      "315-5099", SC / 2, SC / 4, SC / 2 },
	{ "pitfall2",  IO_PIO, V_SYS1,    0, OP_DEC,    MC_NONE,        MCU_NONE, 0, IOM_STD,      "315-5093", SC / 2, SC / 4, SC / 2 },
	{ "seganinj",  IO_PIO, V_SYS1,    0, OP_DEC,    MC_NONE,        MCU_NONE, 0, IOM_STD,      "315-5102", SC / 2, SC / 4, SC / 2 },
	{ "seganinja", IO_PIO, V_SYS1,    0, OP_DEC,    MC_NONE,        MCU_NONE, 0, IOM_STD,      "315-5133", SC / 2, SC / 4, SC / 2 },
	{ "nprinceso", IO_PIO, V_SYS1,    0, OP_DEC,    MC_NONE,        MCU_NONE, 0, IOM_STD,      "315-5098", SC / 2, SC / 4, SC / 2 },
	{ "imsorry",   IO_PIO, V_SYS1,    0, OP_DEC,    MC_NONE,        MCU_NONE, 0, IOM_STD,      "315-5110", SC / 2, SC / 4, SC / 2 },
	{ "teddybb",   IO_PIO, V_SYS1,    0, OP_DEC,    MC_NONE,        MCU_NONE, 0, IOM_STD,      "315-5155", SC / 2, SC / 4, SC / 2 },
	{ "teddybboa", IO_PIO, V_SYS1,    0, OP_DEC,    MC_NONE,        MCU_NONE, 0, IOM_STD,      "315-5111", SC / 2, SC / 4, SC / 2 },
	{ "myheroj",   IO_PIO, V_SYS1,    0, OP_DEC,    MC_NONE,        MCU_NONE, 0, IOM_STD,      "315-5132", SC / 2, SC / 4, SC / 2 },
	{ "_4dwarrio", IO_PIO, V_SYS1,    0, OP_DEC,    MC_NONE,        MCU_NONE, 0, IOM_STD,      "315-5162", SC / 2, SC / 4, SC / 2 },
	{ "wboy",      IO_PIO, V_SYS1,    0, OP_DEC,    MC_NONE,        MCU_NONE, 0, IOM_STD,      "315-5177", SC / 2, SC / 4, SC / 2 },
	{ "wboyo",     IO_PIO, V_SYS1,    0, OP_DEC,    MC_NONE,        MCU_NONE, 0, IOM_STD,      "315-5135", SC / 2, SC / 4, SC / 2 },
	{ "blockgal",  IO_PIO, V_SYS1,    0, OP_DEC,    MC_8123,        MCU_NONE, 0, IOM_BLOCKGAL, NULL,       SC / 2, SC / 4, SC / 2 },
	{ "gardia",    IO_PIO, V_SYS1,    0, OP_DEC,    MC_NONE,        MCU_NONE, 0, IOM_STD,      "317-0006", SC / 2, SC / 4, SC / 4 },
	{ "sys2",      IO_PPI, V_SYS2,    0, OP_NONE,   MC_NONE,        MCU_NONE, 0, IOM_STD,      NULL,       SC / 2, SC / 4, SC / 2 },
	{ "sys2x",     IO_PPI, V_SYS2,    0, OP_BANKED, MC_8123_BANKED, MCU_NONE, 0, IOM_STD,      NULL,       SC / 2, SC / 4, SC / 2 },
	{ "sys2row",   IO_PPI, V_SYS2ROW, 0, OP_NONE,   MC_NONE,        MCU_NONE, 0, IOM_STD,      NULL,       SC / 2, SC / 4, SC / 2 },
	{ "choplift",  IO_PPI, V_SYS2ROW, 0, OP_NONE,   MC_NONE,        MCU_8751, 0, IOM_STD,      NULL,       SC / 2, SC / 4, SC / 2 },
	{ "shtngmst",  IO_PPI, V_SYS2,    0, OP_NONE,   MC_NONE,        MCU_8751, 0, IOM_SHTNGMST, NULL,       SC / 2, SC / 4, SC / 2 },
	{ "gardiab",   IO_PPI, V_SYS2,    0, OP_DEC,    MC_NONE,        MCU_NONE, 0, IOM_STD,      "317-0007", SC / 2, SC / 4, SC / 4 },
	{ "gardiaj",   IO_PPI, V_SYS2,    0, OP_DEC,    MC_NONE,        MCU_NONE, 0, IOM_STD,      "317-0006", SC / 2, SC / 4, SC / 4 },
	{ "wboysys2",  IO_PPI, V_SYS2,    0, OP_DEC,    MC_NONE,        MCU_NONE, 0, IOM_STD,      "315-5177", SC / 2, SC / 4, SC / 2 },
	{ "wboysys2a", IO_PPI, V_SYS2,    0, OP_DEC,    MC_NONE,        MCU_NONE, 0, IOM_STD,      "315-5176", SC / 2, SC / 4, SC / 2 },
	{ "wbmlb",     IO_PPI, V_SYS2,    0, OP_BANKED, MC_NONE,        MCU_NONE, 0, IOM_STD,      NULL,       SC / 2, SC / 4, SC / 2 },
	{ "blockgalb", IO_PPI, V_SYS2,    0, OP_DEC,    MC_NONE,        MCU_NONE, 0, IOM_STD,      NULL,       SC / 2, SC / 4, SC / 2 },
	{ "ufosensi",  IO_PPI, V_SYS2ROW, 0, OP_BANKED, MC_8123_BANKED, MCU_NONE, 0, IOM_STD,      NULL,       SC / 2, SC / 4, SC / 2 },
	{ "ufosensib", IO_PPI, V_SYS2ROW, 0, OP_BANKED, MC_NONE,        MCU_NONE, 0, IOM_STD,      NULL,       SC / 2, SC / 4, SC / 2 },
};
#undef SC

enum { BANK_NONE, BANK_44, BANK_0C };
enum { INIT_NONE, INIT_MYHEROK, INIT_BLOCKGAL, INIT_WBML, INIT_NOB, INIT_NOBB, INIT_BOOTLEG, INIT_BOOTSYS2, INIT_BOOTSYS2D };

static const s1_machine *s_m;
static int s_init, s_bankmode, s_dakko;
static int s_vis_x0, s_vis_w, s_pages;

// ROM
static uint8_t *s_rom, *s_zrom, *s_sprrom, *s_lookup, *s_mcurom;
static uint32_t s_rom_size, s_zrom_size, s_spr_size, s_mcu_size;
static uint8_t *s_opmem;          // 64 KB opcodes view (calloc in init: ADR-040)
static uint8_t *s_banked_op;      // MC-8123 banked: the whole region's opcodes
static const uint8_t *s_op0, *s_op1base;   // banked opcodes: 0x0000-0x7fff, the banks of 0x8000-0xbfff
static int s_op_banks;
static const uint8_t *s_bankbase;
static int s_numbanks;
static rbx_gfx s_gtile;
static uint32_t s_indirect[256];
static uint32_t s_sn_wait[2];      // sound CPU cycles of a write's READY wait
static int s_port[16];
enum { P_P1, P_P2, P_SYSTEM, P_SWA, P_SWB, P_TRIGGER, P_GUNX, P_GUNY, P_SW12, P_KEY0 };

static z80 s_main, s_snd;
static mcs51_t s_mcu;
static i8255_t s_ppi;
static z80pio_t s_pio;
static sn76496_t s_sn[2];
static const uint8_t *s_mmap[256], *s_zmap[256];
static int s_sn_div[2];

static uint16_t *s_fb[2];        // VIS_H x s_vis_w, drawn by the emulation thread / shown (calloc in init)
static uint32_t *s_pal[2];       // the colours of the two frames
static int s_show;               // the frame render() shows

static struct {
	uint8_t ram[0x1000], spr[0x800], pal[0x800];
	uint8_t vram[0x800 * 8];
	uint8_t zram[0x800];
	uint8_t mix_collide[64], sprite_collide[1024];
	uint8_t mix_collide_summary, sprite_collide_summary;
	uint8_t video_mode, videoram_bank, flip;
	uint8_t bank, bank_d;
	uint8_t latch, sound_nmi, sound_ctl, muted;
	uint8_t adjust_cycles, mcu_control;
	uint8_t nob_maincpu_latch, nob_mcu_latch, nob_mcu_status, nobb_inport23_step;
	uint8_t gun_output, gun_trigger, trig_prev;
	uint8_t mux_count, mux_clock;
	uint8_t dial[2];
	int gun[2];
	int main_irq, snd_irq, main_busreq;
	int drawn;                   // lines drawn this frame
	int line;
	uint32_t m1_frac;
	uint64_t main_frac, snd_frac, mcu_frac, smp_frac;
	uint8_t state_end;
} *t;   // (calloc in init: ADR-040)

// the sampled profile's marks (ADR-155)
enum { S1_PA_MAIN = RBX_PA_DRV, S1_PA_VIDEO, S1_PA_MCU, S1_PA_SNDCPU, S1_PA_PSG, S1_PA_END };
static const char *const s_s1_pa_names[] = { "main z80", "lines", "8751", "sound z80", "sn76496" };

// ---- palette / video ----
static void render_line(int y);
static void pc_prepare(void);
static volatile int s_part[2][2] = { { 1, 0 }, { 1, 0 } };   // (the workers' lines: first, last)
static void part_lines(int k)
{
	const int a = s_part[k][0], b = s_part[k][1];
	for (int i = a; i <= b; i++)
		render_line(i);
	s_part[k][0] = 1;
	s_part[k][1] = 0;
}
static void part_job0(void) { part_lines(0); }
static void part_job1(void) { part_lines(1); }

static void draw_upto(int y)
{
	if (y >= VIS_H)
		y = VIS_H - 1;
	if (t->drawn > y)
		return;
	const unsigned pa = rbx_prof_enter(S1_PA_VIDEO);
	const int n = y - t->drawn + 1;
	if (n < 24) {
		while (t->drawn <= y)
			render_line(t->drawn++);
	} else {
		// a run of lines (the frame at vblank) in three parts at once: the emulation thread and the two workers. A line
		// reads only the state, which no one changes meanwhile, and the tilemap pages, drawn up to date first; the
		// collision flags are only ever set (the same flags in any order)
		pc_prepare();
		const int y0 = t->drawn, a = y0 + n / 3, b = y0 + 2 * n / 3;
		s_part[0][0] = a; s_part[0][1] = b - 1;
		s_part[1][0] = b; s_part[1][1] = y;
		rbx_aux_kick();
		rbx_aux2_kick();
		for (int i = y0; i < a; i++)
			render_line(i);
		rbx_aux_sync();
		rbx_aux2_sync();
		t->drawn = y + 1;
	}
	rbx_prof_leave(pa);
}
static void update_now(void)
{
	if (t->line < VIS_H)
		draw_upto(t->line);
}

// the pages as shown (the flip of all tilemaps applied), kept as MAME keeps its tilemaps: a tile drawn again after a
// write to it (or a flip change, power on, a state loaded). Per pixel: its value (colour * 8 + pen) & 0x1ff, and from
// bit 16 its lookup bits ((pen & 7) == 0, priority << 1)
static uint32_t *s_pc_pix;           // 8 pages x 256 rows x 256 (calloc in init)
static uint32_t s_pc_dirty[8][32];   // per page and tile row: the columns to draw again
static int s_pc_flip;
static void pc_all_dirty(void) { memset(s_pc_dirty, 0xff, sizeof(s_pc_dirty)); }

static void pc_tile(int page, int tr, int c)
{
	const uint8_t *ram = t->vram + 0x800 * page;
	const uint32_t td = (uint32_t)(ram[tr * 64 + c * 2] | ram[tr * 64 + c * 2 + 1] << 8);
	const uint32_t code = rbx_wrap(((td >> 4) & 0x800) | (td & 0x7ff), s_gtile.count);   // (no divide: the Vita has none)
	const uint16_t col = (uint16_t)(((td >> 5) & 0xff) * 8);
	const uint8_t *px = s_gtile.pix + code * 64;
	const int flip = t->flip;
	for (int r = 0; r < 8; r++) {
		const int sy = tr * 8 + r, row = flip ? 255 - sy : sy;
		uint32_t *o = s_pc_pix + ((size_t)page << 16) + (size_t)row * 256;
		for (int k = 0; k < 8; k++) {
			const int x = flip ? 255 - (c * 8 + k) : c * 8 + k;
			const uint32_t v = (uint32_t)(uint16_t)(col + px[r * 8 + k]);
			o[x] = (v & 0x1ff) | (uint32_t)(((v & 7) == 0) | (((v >> 9) & 3) << 1)) << 16;
		}
	}
}

// a page's row as shown (page_line before: the same values), its tiles drawn again first if written
static const uint32_t *page_row(int page, int row)
{
	const int tr = (t->flip ? 255 - row : row) >> 3;
	uint32_t d = s_pc_dirty[page][tr];
	if (d) {
		s_pc_dirty[page][tr] = 0;
		for (int c = 0; c < 32; c++)
			if (d >> c & 1)
				pc_tile(page, tr, c);
	}
	return s_pc_pix + ((size_t)page << 16) + (size_t)row * 256;
}

// every page's written tiles drawn again (before lines are drawn on several threads: then they only read the pages)
static void pc_prepare(void)
{
	if (s_pc_flip != t->flip) {
		s_pc_flip = t->flip;
		pc_all_dirty();
	}
	for (int p = 0; p < 8; p++)
		for (int tr = 0; tr < 32; tr++) {
			const uint32_t d = s_pc_dirty[p][tr];
			if (!d)
				continue;
			s_pc_dirty[p][tr] = 0;
			for (int c = 0; c < 32; c++)
				if (d >> c & 1)
					pc_tile(p, tr, c);
		}
}

static void sprites_line(int y, uint16_t *line, int xoffset, int minx, int maxx)
{
	const uint32_t gfxbanks = s_spr_size / 0x8000;
	const int flip = t->flip;
	for (int spritenum = 0; spritenum < 32; spritenum++) {
		const uint8_t *sd = &t->spr[spritenum * 0x10];
		if (sd[0] == 0xff)   // (disables all sprites)
			return;
		int top = sd[0] + 1, bottom = sd[1] + 1;
		if (flip) {
			const int tmp = top;
			top = 256 - bottom;
			bottom = 256 - tmp;
		}
		if (y < top || y >= bottom)
			continue;
		const uint16_t stride = (uint16_t)(sd[4] + (sd[5] << 8));
		const uint16_t srcaddr = (uint16_t)(sd[6] + (sd[7] << 8) + stride * (y - top + 1));
		uint32_t bank = ((sd[3] & 0x80) >> 7) | ((sd[3] & 0x40) >> 5) | ((sd[3] & 0x20) >> 3);
		const int xstart = ((sd[2] | (sd[3] << 8)) & 0x1ff) + xoffset;
		bank = gfxbanks ? bank % gfxbanks : 0;
		const uint8_t *gb = s_sprrom + bank * 0x8000;
		const uint16_t pbase = (uint16_t)(spritenum * 0x10);
		const int delta = (srcaddr & 0x8000) ? -1 : 1;
		uint16_t cur = srcaddr;
		for (int x = xstart, n = 0; n < 192; x += 4, n++, cur = (uint16_t)(cur + delta)) {
			const uint8_t data = gb[cur & 0x7fff];
			uint8_t c[2];
			if (!(cur & 0x8000)) {
				c[0] = data >> 4;
				c[1] = data & 0x0f;
			} else {
				c[0] = data & 0x0f;
				c[1] = data >> 4;
			}
			int stop = 0;
			for (int h = 0; h < 2; h++) {
				if (c[h] == 0x0f) {   // (stop at colour 0x0f)
					stop = 1;
					break;
				}
				if (!c[h])
					continue;
				for (int i = 0; i < 2; i++) {
					const int effx = flip ? 0x1fe - (x + h * 2 + i) : (x + h * 2 + i);
					if (effx >= minx && effx <= maxx) {
						const uint16_t prev = line[effx];
						if (prev & 0x0f)
							t->sprite_collide[((prev >> 4) & 0x1f) + 32 * spritenum] = t->sprite_collide_summary = 1;
						line[effx] = (uint16_t)(c[h] | pbase);
					}
				}
			}
			if (stop)
				break;
		}
	}
}

static void render_line(int y)
{
	uint16_t *dst = s_fb[s_show ^ 1] + y * s_vis_w;
	if (t->video_mode & 0x10) {   // (screen blank)
		memset(dst, 0, sizeof(uint16_t) * (size_t)s_vis_w);
		return;
	}
	const int minx = s_vis_x0, maxx = s_vis_x0 + s_vis_w - 1;
	uint16_t spr[SCR_W];
	memset(spr + minx, 0, sizeof(uint16_t) * (size_t)s_vis_w);
	int pages[4], fgpage, xscroll, yscroll, sprx = 0;
	int rowscroll = 0;
	if (s_m->video == V_SYS1) {
		pages[0] = pages[1] = pages[2] = pages[3] = 0;
		fgpage = 1;
		xscroll = (int16_t)((t->vram[0xffc] | (t->vram[0xffd] << 8)) + 28);
		yscroll = t->vram[0xfbd];
		if (t->flip) {
			xscroll = 640 - (xscroll & 0x1ff);
			yscroll = 764 - (yscroll & 0x1ff);
		}
		rowscroll = xscroll;
	} else {
		for (int i = 0; i < 4; i++)
			pages[i] = t->vram[0x740 + i * 2] & 7;
		fgpage = 0;
		const int r = (y >> 3) & 0x1f;
		if (!t->flip) {
			const int o = s_m->video == V_SYS2ROW ? 0x7c0 + r * 2 : 0x7c0;
			xscroll = ((t->vram[o] | (t->vram[o + 1] << 8)) & 0x1ff) - 512 + 10;
			yscroll = t->vram[0x7ba];
			sprx = 14;
		} else {
			const int o = s_m->video == V_SYS2ROW ? 0x7fe - r * 2 : 0x7f6;
			xscroll = 512 + 512 + 10 - (((t->vram[o] | (t->vram[o + 1] << 8)) & 0x1ff) - 512 + 10);
			yscroll = 512 + 512 - t->vram[0x784];
			sprx = -14;
		}
		rowscroll = xscroll;
	}
	sprites_line(y, spr, sprx, minx, maxx);
	if (s_pc_flip != t->flip) {
		s_pc_flip = t->flip;
		pc_all_dirty();
	}
	// each screen pixel's tilemap words (two screen pixels a tilemap pixel): the output value and, from bit 16, the
	// lookup index's bits (the foreground's << 1, the background's << 4). Then a screen pixel is three loads, the
	// lookup and the choice (the same index and pixels as MAME's mix)
	const uint32_t *fgr = page_row(fgpage, y & 0xff);
	const int bgy = (y + yscroll) & 0x1ff;
	const uint32_t *bgr[2] = { page_row(pages[(bgy >> 8) * 2 + 0], bgy & 0xff), page_row(pages[(bgy >> 8) * 2 + 1], bgy & 0xff) };
	uint32_t fw[512], bc[512];
	for (int i = 0; i < 256; i++) {
		const uint32_t c = fgr[i], w = (0x200 | (c & 0x1ff)) | (c >> 16) << 17;
		fw[2 * i] = fw[2 * i + 1] = w;
	}
	for (int h = 0; h < 2; h++)
		for (int i = 0; i < 256; i++) {
			const uint32_t c = bgr[h][i];
			bc[h * 256 + i] = (0x400 | (c & 0x1ff)) | (c >> 16) << 20;
		}
	const uint8_t *lk = s_lookup;
	uint8_t hit = 0;
#define S1_MIX(x, sprpix, f, b) do { 		const uint8_t lv_ = lk[(((sprpix) & 0xf) == 0) | (((f) | (b)) >> 16)]; 		if (!(lv_ & 4)) { 			t->mix_collide[((lv_ & 8) << 2) | (((sprpix) >> 4) & 0x1f)] = 1; 			hit = 1; 		} 		const int sel_ = lv_ & 3; 		dst[(x) - minx] = (uint16_t)(sel_ == 0 ? ((sprpix) & 0x1ff) : sel_ == 1 ? (f) : (b)); 	} while (0)
	// two screen pixels at a time (minx is even): one foreground pixel; with no sprite pixel on either and the same
	// background word, one lookup for both (the same index: the same value, the same collision slot)
	int x = minx;
	for (; x + 1 <= maxx; x += 2) {
		const uint32_t s0 = spr[x], s1 = spr[x + 1], f = fw[x & 0x1ff];
		const uint32_t b0 = bc[((x - rowscroll) / 2) & 0x1ff], b1 = bc[((x + 1 - rowscroll) / 2) & 0x1ff];
		if (!(s0 | s1) && b0 == b1) {
			const uint8_t lv = lk[1 | ((f | b0) >> 16)];
			if (!(lv & 4)) {
				t->mix_collide[(lv & 8) << 2] = 1;
				hit = 1;
			}
			const int sel = lv & 3;
			const uint16_t v = (uint16_t)(sel == 0 ? 0 : sel == 1 ? f : b0);
			dst[x - minx] = dst[x + 1 - minx] = v;
		} else {
			S1_MIX(x, s0, f, b0);
			S1_MIX(x + 1, s1, f, b1);
		}
	}
	for (; x <= maxx; x++) {
		const uint32_t sp = spr[x], f = fw[x & 0x1ff], b = bc[((x - rowscroll) / 2) & 0x1ff];
		S1_MIX(x, sp, f, b);
	}
#undef S1_MIX
	if (hit)
		t->mix_collide_summary = 1;
}

static void s1_render(void)
{
	memcpy(rbx_video.palette, s_pal[s_show], sizeof(uint32_t) * NPENS);
	for (int y = 0; y < VIS_H; y++)
		memcpy(rbx_video.bitmap + y * rbx_video.pitch, s_fb[s_show] + y * s_vis_w, sizeof(uint16_t) * (size_t)s_vis_w);
}

static void vblank_capture(void)
{
	draw_upto(VIS_H - 1);
	rbx_video_wait_idle();
	s_show ^= 1;
	uint32_t *p = s_pal[s_show];
	for (int i = 0; i < NPENS; i++)
		p[i] = s_indirect[t->pal[i]];
	rbx_video_frame_done();
}

// ---- banking / video mode ----
static void set_bank(int b)
{
	if (s_numbanks > 0)
		b %= s_numbanks;
	else
		b = 0;
	t->bank = (uint8_t)b;
	const uint8_t *base = s_numbanks > 0 ? s_bankbase + b * 0x4000 : s_rom + 0x8000;
	for (int p = 0; p < 0x40; p++)
		s_mmap[0x80 + p] = base + p * 256;
	if (s_opmem && s_m->op == OP_DEC)   // (decrypted_opcodes_map: bank1 is the data)
		memcpy(s_opmem + 0x8000, base, 0x4000);
}
static void set_bank_d(int b)
{
	if (!s_opmem || s_m->op != OP_BANKED || !s_op1base)
		return;
	if (s_op_banks > 0)
		b %= s_op_banks;
	t->bank_d = (uint8_t)b;
	memcpy(s_opmem + 0x8000, s_op1base + b * 0x4000, 0x4000);
}

static void videomode_w(uint8_t data)
{
	if (s_dakko) {   // (bit 1 toggling on clocks the key mux)
		if ((data & 2) && !t->mux_clock)
			t->mux_count = (uint8_t)((t->mux_count + 1) % 7);
		t->mux_clock = (data >> 1) & 1;
		data &= (uint8_t)~0x02;
	}
	if (s_m->mcu != MCU_NONE)   // (bit 6: the 8751's INT1)
		mcs51_set_input(&s_mcu, MCS51_INT1_LINE, (data & 0x40) ? 0 : 1);
	if (s_bankmode == BANK_44)
		set_bank(((data & 0x40) >> 5) | ((data & 0x04) >> 2));
	else if (s_bankmode == BANK_0C) {
		set_bank((data & 0x0c) >> 2);
		set_bank_d((data & 0x0c) >> 2);
	}
	t->video_mode = data;
	t->flip = (data & 0x80) ? 1 : 0;
}

// ---- sound ----
static void sound_nmi_line(int state)
{
	if (state && !t->sound_nmi)
		z80_gen_nmi(&s_snd);
	t->sound_nmi = (uint8_t)(state ? 1 : 0);
}
static void sound_control_w(uint8_t data)
{
	if (s_m->video != V_SYS1)   // (System 2: the mute bit inverted)
		data ^= 0x01;
	t->sound_ctl = data;
	t->muted = data & 1;
	sound_nmi_line((data & 0x80) ? 0 : 1);
	t->videoram_bank = data;
}
static void ppi_out_pa(void *u, uint8_t v) { (void)u; t->latch = v; }
static void ppi_out_pb(void *u, uint8_t v) { (void)u; videomode_w(v); }
static void ppi_out_pc(void *u, uint8_t v) { (void)u; sound_control_w(v); }
static void pio_out_pa(void *u, uint8_t v) { (void)u; t->latch = v; }
static void pio_out_pb(void *u, uint8_t v) { (void)u; videomode_w(v); }
static void pio_ardy(void *u, int state) { (void)u; sound_nmi_line(state); }

static uint8_t sound_data_r(void)
{
	if (s_m->io == IO_PPI) {
		i8255_pc6_w(&s_ppi, 0);
		i8255_pc6_w(&s_ppi, 1);
		return t->latch;
	}
	const uint8_t data = z80pio_port_read(&s_pio, Z80PIO_PORT_A);
	z80pio_strobe(&s_pio, Z80PIO_PORT_A, 0);
	z80pio_strobe(&s_pio, Z80PIO_PORT_A, 1);
	return data;
}

static uint8_t snd_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x8000) return a < s_zrom_size ? s_zrom[a] : 0;
	if (a < 0xa000) return t->zram[a & 0x7ff];
	if (a >= 0xe000) return sound_data_r();
	return 0;
}
static void snd_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0x8000 && a < 0xa000)
		t->zram[a & 0x7ff] = v;
	else if (a >= 0xa000 && a < 0xe000) {
		const int i = a >= 0xc000;
		sn76496_write(&s_sn[i], v);
		s_snd.cyc += s_sn_wait[i];   // (READY low for 4 x clock_divider chip clocks: the Z80's WAIT)
	}
}
static uint8_t z_in_none(z80 *z, uint8_t port) { (void)z; (void)port; return 0xff; }
static void z_out_none(z80 *z, uint8_t port, uint8_t v) { (void)z; (void)port; (void)v; }

// ---- main CPU ----
static void vram_wait(void)
{
	// the CPU halts until the next FIXST: 3.2 CPU clocks apart (based on a dump of 315-5137)
	const uint64_t total = (uint64_t)s_main.cyc * 10u;
	const uint32_t cyc_until = 32 - (uint32_t)((total - 16) % 32);
	s_main.cyc += (cyc_until + 5) / 10;
}
static uint8_t vram_r(uint16_t off)
{
	vram_wait();
	off = (uint16_t)(off | 0x1000 * ((t->videoram_bank >> 1) % (s_pages / 2)));
	return t->vram[off];
}
static void vram_w(uint16_t off, uint8_t v)
{
	vram_wait();
	off = (uint16_t)(off | 0x1000 * ((t->videoram_bank >> 1) % (s_pages / 2)));
	if (s_pages > 2 && off >= 0x740 && off < 0x748 && !(off & 1))   // (a page change: draw up to the beam first)
		update_now();
	t->vram[off] = v;
	s_pc_dirty[(off >> 11) & 7][(off >> 6) & 31] |= 1u << ((off >> 1) & 31);   // (its tile drawn again)
}
static uint8_t mixcol_r(uint16_t off) { update_now(); return (uint8_t)(t->mix_collide[off & 0x3f] | 0x7e | (t->mix_collide_summary << 7)); }
static void mixcol_w(uint16_t off) { update_now(); t->mix_collide[off & 0x3f] = 0; }
static void mixcol_reset(void) { update_now(); t->mix_collide_summary = 0; }
static uint8_t sprcol_r(uint16_t off) { update_now(); return (uint8_t)(t->sprite_collide[off & 0x3ff] | 0x7e | (t->sprite_collide_summary << 7)); }
static void sprcol_w(uint16_t off) { update_now(); t->sprite_collide[off & 0x3ff] = 0; }
static void sprcol_reset(void) { update_now(); t->sprite_collide_summary = 0; }

static uint8_t main_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x8000) {
		if (s_init == INIT_NOB && a == 0x0001)   // (nob_start_r: the JMP at the start)
			return s_main.pc <= 0x0003 ? 0x80 : s_rom[1];
		return s_rom[a];
	}
	if (a < 0xc000) return s_mmap[a >> 8][a & 0xff];
	if (!s_m->nobo) {
		if (a < 0xd000) return t->ram[a - 0xc000];
		if (a < 0xd800) return t->spr[a - 0xd000];
		if (a < 0xe000) return t->pal[a - 0xd800];
		if (a < 0xf000) return vram_r(a - 0xe000);
		if (a < 0xf400) return mixcol_r(a);
		if (a < 0xf800) return 0;
		if (a < 0xfc00) return sprcol_r(a);
		return 0;
	}
	if (a < 0xc400) return mixcol_r(a);
	if (a < 0xc800) return 0;
	if (a < 0xcc00) return sprcol_r(a);
	if (a < 0xd000) return 0;
	if (a < 0xd800) return t->spr[a - 0xd000];
	if (a < 0xe000) return t->pal[a - 0xd800];
	if (a < 0xf000) return vram_r(a - 0xe000);
	return t->ram[a - 0xf000];
}

static void main_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a < 0xc000)
		return;
	if (!s_m->nobo) {
		if (a < 0xd000) {
			t->ram[a - 0xc000] = v;
			if (s_opmem) s_opmem[a] = v;
		} else if (a < 0xd800) {
			t->spr[a - 0xd000] = v;
			if (s_opmem) s_opmem[a] = v;
		} else if (a < 0xe000) {
			t->pal[a - 0xd800] = v;
			if (s_opmem) s_opmem[a] = v;
		} else if (a < 0xf000) vram_w(a - 0xe000, v);
		else if (a < 0xf400) mixcol_w(a);
		else if (a < 0xf800) mixcol_reset();
		else if (a < 0xfc00) sprcol_w(a);
		else sprcol_reset();
		return;
	}
	if (a < 0xc400) mixcol_w(a);
	else if (a < 0xc800) mixcol_reset();
	else if (a < 0xcc00) sprcol_w(a);
	else if (a < 0xd000) sprcol_reset();
	else if (a < 0xd800) t->spr[a - 0xd000] = v;
	else if (a < 0xe000) t->pal[a - 0xd800] = v;
	else if (a < 0xf000) vram_w(a - 0xe000, v);
	else t->ram[a - 0xf000] = v;
}

static uint8_t port8(int p) { return s_port[p] >= 0 ? (uint8_t)rbx_port_read(s_port[p]) : 0xff; }

static uint8_t io_read(uint8_t port)
{
	port &= 0x1f;
	if (s_init == INIT_NOB) {
		if (port == 0x18) return t->nob_maincpu_latch;
		if (port == 0x1c) return t->nob_mcu_status;
	} else if (s_init == INIT_NOBB) {
		if (port == 0x1c) return 0x80;
		if (port == 0x02) return 0;
		if (port == 0x03) return t->nobb_inport23_step;
	}
	if (s_m->iomap == IOM_SHTNGMST) {
		if (port == 0x10 || port == 0x11) return t->gun_output;
		if (port == 0x12 || port == 0x13) return (uint8_t)~(t->gun_trigger << 6);
		if (port >= 0x18 && port <= 0x1b) return port8(P_SW12);
		if (port == 0x1c || port == 0x1e) return (uint8_t)t->gun[0];
		if (port == 0x1d || port == 0x1f) return (uint8_t)t->gun[1];
	}
	if (s_m->iomap == IOM_BLOCKGAL && (port & 0x1d) == 0x0c)
		return (port & 1) ? port8(P_SWA) : 0;
	switch (port >> 2) {
	case 0:
		if (s_dakko) return s_port[P_KEY0 + t->mux_count] >= 0 ? (uint8_t)rbx_port_read(s_port[P_KEY0 + t->mux_count]) : 0;
		if (s_m->iomap == IOM_BLOCKGAL) return t->dial[0];
		return port8(P_P1);
	case 1:
		if (s_dakko) return (uint8_t)(1 << t->mux_count);
		if (s_m->iomap == IOM_BLOCKGAL) return t->dial[1];
		return port8(P_P2);
	case 2: return port8(P_SYSTEM);
	case 3: return (port & 1) ? port8(P_SWB) : port8(P_SWA);
	case 4: return port8(P_SWB);
	case 5: return s_m->io == IO_PPI ? i8255_read(&s_ppi, port & 3) : 0;
	case 6: return s_m->io == IO_PIO ? z80pio_read(&s_pio, port & 3) : 0;
	}
	return 0;
}

static void io_write(uint8_t port, uint8_t v)
{
	port &= 0x1f;
	if (s_init == INIT_NOB && port == 0x18) {
		t->nob_maincpu_latch = v;
		mcs51_set_input(&s_mcu, MCS51_INT0_LINE, 1);
		z80_break = 1;   // (perfect_quantum: let the MCU see it)
		return;
	}
	if (s_init == INIT_NOBB && port == 0x04) {
		t->nobb_inport23_step = v;
		return;
	}
	if (s_m->iomap == IOM_SHTNGMST && (port == 0x10 || port == 0x11)) {
		if (!(v & 1))   // (bit 0 readies the gun)
			t->gun_trigger = 0;
		t->gun_output = v;
		return;
	}
	if (s_m->io == IO_PPI && (port >> 2) == 5)
		i8255_write(&s_ppi, port & 3, v);
	else if (s_m->io == IO_PIO && (port >> 2) == 6)
		z80pio_write(&s_pio, port & 3, v);
}
static uint8_t main_in(z80 *z, uint8_t port) { (void)z; return io_read(port); }
static void main_out(z80 *z, uint8_t port, uint8_t v) { (void)z; io_write(port, v); }

// ---- 8751 MCU ----
static void mcu_control_w(uint8_t data)
{
	t->mcu_control = data;
	t->main_busreq = (data & 0x40) ? 1 : 0;
	t->main_irq = (data & 0x01) ? 0 : 1;
}
static uint8_t mcu_xdata_r(void *u, uint16_t a)
{
	(void)u;
	switch ((t->mcu_control >> 3) & 3) {
	case 0: return main_read(NULL, a);
	case 1: return (uint32_t)a + 0x10000 < s_rom_size ? s_rom[a + 0x10000] : 0;
	case 2: return io_read((uint8_t)a);
	}
	return 0xff;
}
static void mcu_xdata_w(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	switch ((t->mcu_control >> 3) & 3) {
	case 0: main_write(NULL, a, v); break;
	case 2: io_write((uint8_t)a, v); break;
	}
}
static void mcu_port_out(void *u, int port, uint8_t v)
{
	(void)u;
	if (s_m->mcu == MCU_8751) {
		if (port == 1)
			mcu_control_w(v);
		return;
	}
	switch (port) {   // Noboranka
	case 0: t->nob_mcu_latch = v; break;
	case 1: t->nob_mcu_status = v; break;
	case 2:
		if (((t->mcu_control ^ v) & 0x01) && !(v & 0x01))   // (bit 0: a read from MCU port 0)
			t->nob_mcu_latch = t->nob_maincpu_latch;
		if (((t->mcu_control ^ v) & 0x02) && !(v & 0x02))   // (bit 1: a write from MCU port 0)
			t->nob_maincpu_latch = t->nob_mcu_latch;
		if (((t->mcu_control ^ v) & 0x04) && !(v & 0x04))   // (bit 2: near the end of an IRQ)
			mcs51_set_input(&s_mcu, MCS51_INT0_LINE, 0);
		t->mcu_control = v;
		break;
	}
}
static uint8_t mcu_port_in(void *u, int port)
{
	(void)u;
	if (s_m->mcu == MCU_NOB && port == 0)
		return t->nob_mcu_latch;
	return 0xff;
}

// ---- setup ----
static void z_setup(z80 *z, uint8_t (*rd)(void *, uint16_t), void (*wr)(void *, uint16_t, uint8_t),
                    uint8_t (*in)(z80 *, uint8_t), void (*out)(z80 *, uint8_t, uint8_t), const uint8_t **map)
{
	z80_init(z);
	z->read_byte = rd;
	z->write_byte = wr;
	z->port_in = in;
	z->port_out = out;
	z->read_map = map;
	z->int_data = 0xff;
}

static void build_maps(void)
{
	memset(s_mmap, 0, sizeof(s_mmap));
	if (s_init != INIT_NOB)
		s_mmap[0] = s_rom;
	for (int p = 1; p < 0x80; p++)
		s_mmap[p] = s_rom + p * 256;
	if (!s_m->nobo) {
		for (int p = 0; p < 0x10; p++)
			s_mmap[0xc0 + p] = t->ram + p * 256;
		for (int p = 0; p < 8; p++) {
			s_mmap[0xd0 + p] = t->spr + p * 256;
			s_mmap[0xd8 + p] = t->pal + p * 256;
		}
	} else {
		for (int p = 0; p < 8; p++) {
			s_mmap[0xd0 + p] = t->spr + p * 256;
			s_mmap[0xd8 + p] = t->pal + p * 256;
		}
		for (int p = 0; p < 0x10; p++)
			s_mmap[0xf0 + p] = t->ram + p * 256;
	}
	set_bank(t->bank);
	memset(s_zmap, 0, sizeof(s_zmap));
	for (int p = 0; p < 0x80 && (uint32_t)p * 256 + 256 <= s_zrom_size; p++)
		s_zmap[p] = s_zrom + p * 256;
	for (int p = 0x80; p < 0xa0; p++)
		s_zmap[p] = t->zram + (p & 7) * 256;
	if (s_opmem) {   // (the opcodes view of RAM: the same bytes)
		if (s_m->op == OP_BANKED && s_op0)
			memcpy(s_opmem, s_op0, 0x8000);
		set_bank_d(t->bank_d);
		memcpy(s_opmem + 0xc000, t->ram, 0x1000);
		memcpy(s_opmem + 0xd000, t->spr, 0x800);
		memcpy(s_opmem + 0xd800, t->pal, 0x800);
	}
}

static void s1_reset(void)
{
	memset(t, 0, offsetof(__typeof__(*t), state_end));
	pc_all_dirty();
	t->gun[0] = 0x7f;
	t->gun[1] = 0x80;
	z_setup(&s_main, main_read, main_write, main_in, main_out, s_mmap);
	z_setup(&s_snd, snd_read, snd_write, z_in_none, z_out_none, s_zmap);
	s_main.op_rom = s_opmem;
	build_maps();
	if (s_m->io == IO_PPI) {
		i8255_reset(&s_ppi);
	} else
		z80pio_reset(&s_pio);
	sn76496_reset(&s_sn[0]);
	sn76496_reset(&s_sn[1]);
	if (s_m->mcu != MCU_NONE)
		mcs51_reset(&s_mcu);
	t->drawn = 0;
}

static void s1_exit(void);

static int find_machine(const char *machine, char *init, size_t n)
{
	const char *c = strchr(machine, ':');
	const size_t len = c ? (size_t)(c - machine) : strlen(machine);
	if (init) {
		init[0] = 0;
		if (c) {
			strncpy(init, c + 1, n - 1);
			init[n - 1] = 0;
		}
	}
	for (int i = 0; i < (int)(sizeof(s_machines) / sizeof(s_machines[0])); i++)
		if (strlen(s_machines[i].name) == len && !strncmp(s_machines[i].name, machine, len))
			return i;
	return -1;
}

static int s1_init(const rbx_game *g)
{
	char init[32];
	const int mi = find_machine(g->machine, init, sizeof(init));
	if (mi < 0) {
		rbx_fail("%s: machine %s not emulated", g->set, g->machine);
		return -1;
	}
	s_m = &s_machines[mi];
	s_init = !strcmp(init, "myherok") ? INIT_MYHEROK : !strcmp(init, "blockgal") ? INIT_BLOCKGAL
	       : !strcmp(init, "wbml") ? INIT_WBML : !strcmp(init, "nob") ? INIT_NOB : !strcmp(init, "nobb") ? INIT_NOBB
	       : !strcmp(init, "bootleg") ? INIT_BOOTLEG : !strcmp(init, "bootsys2") ? INIT_BOOTSYS2
	       : !strcmp(init, "bootsys2d") ? INIT_BOOTSYS2D : INIT_NONE;
	s_bankmode = (!strcmp(init, "bank44") || s_init == INIT_NOB || s_init == INIT_NOBB) ? BANK_44
	           : (!strcmp(init, "bank0c") || s_init == INIT_WBML || s_init == INIT_BOOTSYS2 || s_init == INIT_BOOTSYS2D)
	                 ? BANK_0C : BANK_NONE;
	s_dakko = !strcmp(g->set, "dakkochn");
	s_pages = s_m->video == V_SYS1 ? 2 : 8;
	s_vis_x0 = s_m->narrow ? 16 : 0;
	s_vis_w = s_m->narrow ? 494 - 16 + 1 : SCR_W;

	free(t);
	t = calloc(1, sizeof(*t));
	for (int i = 0; i < 2; i++) {
		free(s_fb[i]);
		free(s_pal[i]);
		s_fb[i] = calloc((size_t)VIS_H * SCR_W, sizeof(uint16_t));
		s_pal[i] = calloc(NPENS, sizeof(uint32_t));
	}
	free(s_pc_pix);
	s_pc_pix = calloc((size_t)8 << 16, sizeof(uint32_t));
	free(s_opmem);
	s_opmem = NULL;
	free(s_banked_op);
	s_banked_op = NULL;
	s_op0 = s_op1base = NULL;
	s_op_banks = 0;
	if (!t || !s_fb[0] || !s_fb[1] || !s_pal[0] || !s_pal[1] || !s_pc_pix) {
		rbx_fail("%s: out of memory", g->set);
		s1_exit();
		return -1;
	}
	uint32_t ts, ls, cps = 0;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_zrom = rbx_region("soundcpu", &s_zrom_size);
	uint8_t *tiles = rbx_region("tiles", &ts);
	s_sprrom = rbx_region("sprites", &s_spr_size);
	s_lookup = rbx_region("lookup_proms", &ls);
	const uint8_t *cprom = rbx_region_opt("color_proms", &cps);
	s_mcurom = s_m->mcu != MCU_NONE ? rbx_region("mcu", &s_mcu_size) : NULL;
	if (!s_rom || s_rom_size < 0x10000 || !s_zrom || !tiles || !s_sprrom || !s_lookup || ls < 0x100 ||
	    (s_m->mcu != MCU_NONE && !s_mcurom)) {
		rbx_fail("%s: ROM regions missing", g->set);
		s1_exit();
		return -1;
	}
	if (cprom && cps < 0x300)
		cprom = NULL;

	// driver init
	if (s_init == INIT_MYHEROK) {   // (D0 / D1 swapped program, mangled tiles; then the usual decryption)
		for (int A = 0; A < 0xc000; A++)
			s_rom[A] = (uint8_t)((s_rom[A] & 0xfc) | ((s_rom[A] & 1) << 1) | ((s_rom[A] & 2) >> 1));
		uint8_t *r = tiles;
		for (int A = 0x0000; A < 0x4000 && (uint32_t)A < ts; A++)
			r[A] = (uint8_t)((r[A] & 0xbe) | ((r[A] & 0x01) << 6) | ((r[A] & 0x40) >> 6));
		for (int A = 0x4000; A < 0x8000 && (uint32_t)A < ts; A++)
			r[A] = (uint8_t)((r[A] & 0xdd) | ((r[A] & 0x02) << 4) | ((r[A] & 0x20) >> 4));
		for (int A = 0x8000; A < 0xc000 && (uint32_t)A < ts; A++)
			r[A] = (uint8_t)((r[A] & 0xbe) | ((r[A] & 0x01) << 6) | ((r[A] & 0x40) >> 6));
		for (int A = 0; A < 0xc000 && (uint32_t)A < ts; A++) {
			const int A1 = (A & 0xffcf) | ((A & 0x0010) << 1) | ((A & 0x0020) >> 1);
			if (A < A1) {
				const uint8_t tmp = r[A];
				r[A] = r[A1];
				r[A1] = tmp;
			}
		}
	}
	if (s_m->op != OP_NONE) {
		s_opmem = calloc(1, 0x10000);
		if (!s_opmem) {
			rbx_fail("%s: out of memory", g->set);
			s1_exit();
			return -1;
		}
	}
	if (s_m->crypt && segacrpt_decode(s_m->crypt, s_rom, s_opmem)) {
		rbx_fail("%s: unknown encryption %s", g->set, s_m->crypt);
		s1_exit();
		return -1;
	}
	if (s_m->mc != MC_NONE) {
		uint32_t ks;
		const uint8_t *key = rbx_region("maincpu:key", &ks);
		if (!key || ks < 0x2000) {
			rbx_fail("%s: MC-8123 key missing", g->set);
			s1_exit();
			return -1;
		}
		if (s_m->mc == MC_8123)   // (init_blockgal: 0x0000-0x7fff)
			mc8123_decode(s_rom, s_opmem, 0x8000, key);
		else {                    // (init_wbml: the whole region, banked)
			s_banked_op = malloc(s_rom_size);
			if (!s_banked_op) {
				rbx_fail("%s: out of memory", g->set);
				s1_exit();
				return -1;
			}
			mc8123_decode(s_rom, s_banked_op, s_rom_size, key);
			s_op0 = s_banked_op;
			s_op1base = s_rom_size > 0x10000 ? s_banked_op + 0x10000 : NULL;
			s_op_banks = (int)((s_rom_size - 0x10000) / 0x4000);
		}
	}
	if (s_init == INIT_BOOTLEG && s_rom_size >= 0x18000)
		memcpy(s_opmem, s_rom + 0x10000, 0x8000);
	if (s_init == INIT_BOOTSYS2 && s_rom_size >= 0x40000) {
		s_op0 = s_rom + 0x20000;
		s_op1base = s_rom + 0x30000;
		s_op_banks = 4;
	} else if (s_init == INIT_BOOTSYS2D || (s_m->op == OP_BANKED && !s_op0)) {
		s_op0 = s_rom;
		s_op1base = s_rom_size > 0x10000 ? s_rom + 0x10000 : NULL;
		s_op_banks = 4;
	}
	s_numbanks = (int)((s_rom_size - 0x10000) / 0x4000);
	s_bankbase = s_rom + 0x10000;

	{	// palette: the PROMs (System 2) or the resistor weights (System 1)
		if (cprom)
			for (int i = 0; i < 256; i++) {
				int c[3];
				for (int k = 0; k < 3; k++) {
					const uint8_t v = cprom[i + k * 256];
					c[k] = 0x0e * (v & 1) + 0x1f * ((v >> 1) & 1) + 0x43 * ((v >> 2) & 1) + 0x8f * ((v >> 3) & 1);
				}
				s_indirect[i] = (uint32_t)c[0] << 16 | (uint32_t)c[1] << 8 | (uint32_t)c[2];
			}
		else {
			// compute_resistor_weights(0, 255, -1.0, 3, {995, 495, 250}, 3, {995, 495, 250}, 2, {495, 250}): no pull
			// resistors, so a bit weighs its conductance over the network's (each network's maximum is 255)
			const double rg[3] = { 1.0 / 995, 1.0 / 495, 1.0 / 250 }, bb[2] = { 1.0 / 495, 1.0 / 250 };
			const double srg = rg[0] + rg[1] + rg[2], sb = bb[0] + bb[1];
			double wrg[3], wb[2];
			for (int k = 0; k < 3; k++)
				wrg[k] = 255.0 * rg[k] / srg;
			for (int k = 0; k < 2; k++)
				wb[k] = 255.0 * bb[k] / sb;
			for (int i = 0; i < 256; i++) {
				const int r = (int)(wrg[0] * (i & 1) + wrg[1] * ((i >> 1) & 1) + wrg[2] * ((i >> 2) & 1) + 0.5);
				const int gg = (int)(wrg[0] * ((i >> 3) & 1) + wrg[1] * ((i >> 4) & 1) + wrg[2] * ((i >> 5) & 1) + 0.5);
				const int b = (int)(wb[0] * ((i >> 6) & 1) + wb[1] * ((i >> 7) & 1) + 0.5);
				s_indirect[i] = (uint32_t)r << 16 | (uint32_t)gg << 8 | (uint32_t)b;
			}
		}
	}
	{	// charlayout: 8x8, 3 planes RGN_FRAC(0/3, 1/3, 2/3)
		const uint32_t third = ts / 3 * 8;
		const uint32_t pl[3] = { 0, third, third * 2 };
		static const uint32_t xo[8] = { 0, 1, 2, 3, 4, 5, 6, 7 }, yo[8] = { 0, 8, 16, 24, 32, 40, 48, 56 };
		const rbx_gfx_layout l = { 8, 8, 3, pl, xo, yo, 64 };
		if (rbx_gfx_decode(&s_gtile, &l, tiles, ts / 3) || !s_gtile.count) {
			rbx_fail("%s: out of memory (graphics)", g->set);
			s1_exit();
			return -1;
		}
	}
	// SN76489A at 4 and 2 MHz: both made into AUDIO_RATE (native rate clock / 2)
	const uint32_t sclk[2] = { s_m->sn0_clk, s_m->sn1_clk };
	const int gain[2] = { 102, 154 };   // (0.40, 0.60)
	for (int i = 0; i < 2; i++) {
		s_sn_div[i] = (int)(sclk[i] / 2 / AUDIO_RATE);
		sn76496_init(&s_sn[i], sclk[i], SN76489A_CONFIG, gain[i], s_sn_div[i], 0);
		s_sn_wait[i] = (uint32_t)((uint64_t)4 * 8 * s_m->snd_clk / sclk[i]);
	}
	if (rbx_audio_setup(AUDIO_RATE) || rbx_video_setup(s_vis_w, VIS_H, NPENS)) {
		rbx_fail("%s: out of memory", g->set);
		s1_exit();
		return -1;
	}
	memset(&s_ppi, 0, sizeof(s_ppi));
	s_ppi.out_pa = ppi_out_pa;
	s_ppi.out_pb = ppi_out_pb;
	s_ppi.out_pc = ppi_out_pc;
	s_ppi.tri_pa = 0x00;
	s_ppi.tri_pb = 0x40;
	s_ppi.tri_pc = 0x80;
	memset(&s_pio, 0, sizeof(s_pio));
	s_pio.out_p[0] = pio_out_pa;
	s_pio.out_p[1] = pio_out_pb;
	s_pio.out_rdy[0] = pio_ardy;
	if (s_m->mcu != MCU_NONE) {
		mcs51_init(&s_mcu, s_mcurom, s_mcu_size);
		s_mcu.port_out = mcu_port_out;
		s_mcu.port_in = mcu_port_in;
		if (s_m->mcu == MCU_8751) {
			s_mcu.xdata_r = mcu_xdata_r;
			s_mcu.xdata_w = mcu_xdata_w;
		}
	}
	static const char *const names[P_KEY0] = { "P1", "P2", "SYSTEM", "SWA", "SWB", "TRIGGER", "GUNX", "GUNY", "SW12" };
	for (int i = 0; i < P_KEY0; i++)
		s_port[i] = rbx_port_find(names[i]);
	for (int i = 0; i < 7; i++) {
		char tag[8] = "KEY0";
		tag[3] = (char)('0' + i);
		s_port[P_KEY0 + i] = rbx_port_find(tag);
	}
	s_show = 0;
	rbx_aux_set_job(part_job0);   // (the lines' helpers: draw_upto)
	rbx_aux2_set_alone(RBX_AUX2_BY_RENDER);
	rbx_aux2_set_job(part_job1);
	rbx_prof_name(RBX_PA_DRV, s_s1_pa_names, S1_PA_END - RBX_PA_DRV);
	s1_reset();
	return 0;
}

static void s1_exit(void)
{
	rbx_aux_set_job(NULL);   // (syncs first)
	rbx_aux2_set_job(NULL);
	rbx_video_wait_idle();
	rbx_gfx_free(&s_gtile);
	for (int i = 0; i < 2; i++) {
		free(s_fb[i]);
		s_fb[i] = NULL;
		free(s_pal[i]);
		s_pal[i] = NULL;
	}
	free(s_opmem);
	s_opmem = NULL;
	free(s_banked_op);
	s_banked_op = NULL;
	free(s_pc_pix);
	s_pc_pix = NULL;
	free(t);
	t = NULL;
}

// ---- frame ----
static void read_controls(void)
{
	if (s_m->iomap == IOM_BLOCKGAL)   // (IPT_DIAL, PORT_KEYDELTA(15) PORT_REVERSE)
		for (int p = 0; p < 2; p++)
			t->dial[p] = (uint8_t)(t->dial[p] - rbx_analog_dpad(p + 1, RBX_IPT_DIAL, 15));
	if (s_m->iomap == IOM_SHTNGMST) {
		const int x = t->gun[0] + rbx_analog(1, RBX_IPT_LIGHTGUN_X);   // (PORT_MINMAX(0x00, 0xfe))
		const int y = t->gun[1] - rbx_analog(1, RBX_IPT_LIGHTGUN_Y);   // (PORT_MINMAX(0x10, 0xf0) PORT_REVERSE)
		t->gun[0] = x < 0 ? 0 : x > 0xfe ? 0xfe : x;
		t->gun[1] = y < 0x10 ? 0x10 : y > 0xf0 ? 0xf0 : y;
		const uint8_t trig = (uint8_t)(port8(P_TRIGGER) & 1);   // (gun_trigger: PORT_CHANGED on a press)
		if (trig && !t->trig_prev && (t->gun_output & 1))
			t->gun_trigger = 1;
		t->trig_prev = trig;
	}
}

static void run_main(unsigned long cycles)
{
	const unsigned long end = s_main.cyc + cycles;
	if (t->main_busreq) {   // (BUSREQ: the Z80 is halted)
		s_main.cyc = end;
		return;
	}
	while ((long)(s_main.cyc - end) < 0) {
		const uint8_t r0 = s_main.r;
		int line = t->main_irq;
		z80_break = 0;
		z80_run(&s_main, end, &line, 1);
		z80_break = 0;
		if (t->main_irq && !line) {   // (acknowledged)
			if (s_m->mcu == MCU_8751) {   // (the bus controller's INTACK clocks the MCU's T0; the line stays)
				mcs51_set_input(&s_mcu, MCS51_T0_LINE, 1);
				mcs51_set_input(&s_mcu, MCS51_T0_LINE, 0);
			} else
				t->main_irq = 0;   // (HOLD_LINE)
		}
		// an opcode fetch takes 2 more 20 MHz clocks: 2 cycles every 5 M1 (the R register counts them)
		t->m1_frac += 2u * (uint32_t)((s_main.r - r0) & 0x7f);
		s_main.cyc += t->m1_frac / 5;
		t->m1_frac %= 5;
		if (t->main_busreq) {
			if ((long)(s_main.cyc - end) < 0)
				s_main.cyc = end;
			break;
		}
	}
}

static void s1_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	read_controls();
	int32_t mix[8], m2[8];
	for (int line = 0; line < LINES; line++) {
		t->line = line;
		if (line == VIS_H) {   // vblank: the main CPU's IRQ (or the MCU's INT0), the frame
			vblank_capture();
			if (s_m->mcu == MCU_8751)
				mcs51_set_input(&s_mcu, MCS51_INT0_LINE, 1);
			else
				t->main_irq = 1;
		}
		if (line == 0) {
			t->drawn = 0;
			if (s_m->mcu == MCU_8751)
				mcs51_set_input(&s_mcu, MCS51_INT0_LINE, 0);
		}
		if ((line & 63) == 32)   // (soundirq_gen: 32V, 96V, ... HOLD_LINE)
			t->snd_irq = 1;
		const unsigned long mc = (unsigned long)rbx_line_cycles(&t->main_frac, CPU_CLOCK, FRAME_MHZ, LINES);
		rbx_prof_set(S1_PA_MAIN);
		if (s_m->mcu == MCU_NONE)
			run_main(mc);
		else {
			unsigned long done = 0;
			for (int s = 0; s < MCU_SLICES; s++) {
				const unsigned long part = mc * (unsigned long)(s + 1) / MCU_SLICES - done;
				done += part;
				run_main(part);
				const int mcyc = rbx_line_cycles(&t->mcu_frac, SOUND_CLOCK / 12, FRAME_MHZ, LINES * MCU_SLICES);
				if (mcyc > 0) {
					rbx_prof_set(S1_PA_MCU);
					mcs51_execute(&s_mcu, mcyc);
					rbx_prof_set(S1_PA_MAIN);
				}
			}
		}
		rbx_prof_set(S1_PA_SNDCPU);
		z80_run(&s_snd, s_snd.cyc + (unsigned long)rbx_line_cycles(&t->snd_frac, s_m->snd_clk, FRAME_MHZ, LINES), &t->snd_irq, 1);
		rbx_prof_set(S1_PA_PSG);
		int n = rbx_line_cycles(&t->smp_frac, AUDIO_RATE, FRAME_MHZ, LINES);
		while (n > 0) {
			const int k = n > 8 ? 8 : n;
			memset(mix, 0, sizeof(mix));
			memset(m2, 0, sizeof(m2));
			sn76496_render(&s_sn[0], (uint32_t)(k * 2 * s_sn_div[0]), mix, k);
			sn76496_render(&s_sn[1], (uint32_t)(k * 2 * s_sn_div[1]), m2, k);
			for (int i = 0; i < k; i++)
				rbx_audio_put(t->muted ? 0 : mix[i] + m2[i]);
			n -= k;
		}
	}
	rbx_prof_set(RBX_PA_DRIVER);
}

// ---- save states ----
static int z_state(void *ctx, const char *tag, z80 *z, int load, uint8_t (*rd)(void *, uint16_t),
                   void (*wr)(void *, uint16_t, uint8_t), uint8_t (*in)(z80 *, uint8_t),
                   void (*out)(z80 *, uint8_t, uint8_t), const uint8_t **map, const uint8_t *op)
{
	z80 c = *z;
	const int rc = rbx_state_io(ctx, tag, &c, sizeof(c));
	if (!rc && load) {
		c.read_byte = rd;
		c.write_byte = wr;
		c.port_in = in;
		c.port_out = out;
		c.userdata = NULL;
		c.read_map = map;
		c.op_rom = op;
		*z = c;
	}
	return rc;
}

static int s1_state(void *ctx, int load)
{
	int rc = 0;
	rc |= rbx_state_io(ctx, "s1", t, (uint32_t)offsetof(__typeof__(*t), state_end));
	rc |= z_state(ctx, "s1_main", &s_main, load, main_read, main_write, main_in, main_out, s_mmap, s_opmem);
	rc |= z_state(ctx, "s1_snd", &s_snd, load, snd_read, snd_write, z_in_none, z_out_none, s_zmap, NULL);
	rc |= rbx_state_io(ctx, "s1_sn0", &s_sn[0], sn76496_state_size());
	rc |= rbx_state_io(ctx, "s1_sn1", &s_sn[1], sn76496_state_size());
	if (s_m->io == IO_PPI)
		rc |= rbx_state_io(ctx, "s1_ppi", (uint8_t *)&s_ppi + I8255_STATE_OFFSET, (uint32_t)I8255_STATE_SIZE);
	else
		rc |= rbx_state_io(ctx, "s1_pio", (uint8_t *)&s_pio + Z80PIO_STATE_OFFSET, (uint32_t)Z80PIO_STATE_SIZE);
	if (s_m->mcu != MCU_NONE)
		rc |= rbx_state_io(ctx, "s1_mcu", &s_mcu.st, sizeof(s_mcu.st));
	if (!rc && load) {
		build_maps();
		pc_all_dirty();
	}
	return rc ? -1 : 0;
}

static int s1_save(void *ctx) { return s1_state(ctx, 0); }
static int s1_load(void *ctx) { return s1_state(ctx, 1); }
static int s1_supported(const char *machine) { return find_machine(machine, NULL, 0) >= 0; }

const rbx_driver rbx_drv_system1 = { "Sega System 1", s1_init, s1_exit, s1_reset, s1_frame, s1_save, s1_load,
                                     s1_supported, s1_render, NULL };
