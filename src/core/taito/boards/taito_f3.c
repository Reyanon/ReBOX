/*  Taito F3 Package System, ported from MAME src/mame/taito/taito_f3.cpp (BSD-3-Clause, copyright-holders:
    Bryan McPhail; thanks-to: Shiriru) and taito_f3_v.cpp (copyright-holders: Bryan McPhail, ywy, 12Me21).
    Main CPU 68EC020 (Musashi), video TC0630FDP (video/tc0630fdp.c), sound: Taito EN board (machine/taito_en.c,
    68000 + ES5505 + ES5510 ESP).
    Memory map, interrupts (vblank IRQ2, IRQ3 10000 cycles later), EEPROM and the per-game configuration
    (extend, sprite lag, 12-bit palette, ROM patches) follow MAME.
    Not emulated: the bootleg bubsympb configuration (OKI sound, different sprite format).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m68k_bus.h"
#include "../../cpu/m68k/m68k.h"
#include "../../machine/eeprom93c46.h"
#include "../machine/taito_en.h"
#include "../video/tc0630fdp.h"


#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// MAME taito_f3_state game ids
enum {
	RINGRAGE, ARABIANM, RIDINGF, GSEEKER, TRSTAR, GUNLOCK, TWINQIX, UNDRFIRE, SCFINALS, LIGHTBR,
	KAISERKN, DARIUSG, BUBSYMPH, SPCINVDX, HTHERO95, QTHEATER, EACTION2, SPCINV95, QUIZHUHU, PBOBBLE2,
	GEKIRIDO, KTIGER2, BUBBLEM, CLEOPATR, PBOBBLE3, ARKRETRN, KIRAMEKI, PUCHICAR, PBOBBLE4, POPNPOP,
	LANDMAKR, RECALH, COMMANDW, TMDRILL
};

// init_* of each parent set (clones use their parent's) and ROM patches
enum { PATCH_NONE, PATCH_PBOBBL2P, PATCH_LANDMKRP };
typedef struct { const char *set; int game; int patch; } f3_set;
static const f3_set s_sets[] = {
	{ "ringrage", RINGRAGE, 0 }, { "arabianm", ARABIANM, 0 }, { "ridingf", RIDINGF, 0 }, { "gseeker", GSEEKER, 0 },
	{ "commandw", COMMANDW, 0 }, { "cupfinal", SCFINALS, 0 }, { "trstar", TRSTAR, 0 }, { "gunlock", GUNLOCK, 0 },
	{ "scfinals", SCFINALS, 0 }, { "lightbr", LIGHTBR, 0 }, { "intcup94", SCFINALS, 0 }, { "kaiserkn", KAISERKN, 0 },
	{ "dankuga", KAISERKN, 0 }, { "dariusg", DARIUSG, 0 }, { "dariusgx", DARIUSG, 0 }, { "bublbob2", BUBSYMPH, 0 },
	{ "spcinvdj", SPCINVDX, 0 }, { "pwrgoal", HTHERO95, 0 }, { "qtheater", QTHEATER, 0 }, { "elvactr", EACTION2, 0 },
	{ "recalh", RECALH, 0 }, { "spcinv95", SPCINV95, 0 }, { "twinqix", TWINQIX, 0 }, { "quizhuhu", QUIZHUHU, 0 },
	{ "pbobble2", PBOBBLE2, PATCH_PBOBBL2P }, { "gekiridn", GEKIRIDO, 0 }, { "tcobra2", KTIGER2, 0 },
	{ "bubblem", BUBBLEM, 0 }, { "cleopatr", CLEOPATR, 0 }, { "pbobble3", PBOBBLE3, 0 }, { "arkretrn", ARKRETRN, 0 },
	{ "kirameki", KIRAMEKI, 0 }, { "puchicar", PUCHICAR, 0 }, { "pbobble4", PBOBBLE4, 0 }, { "popnpop", POPNPOP, 0 },
	{ "landmakr", LANDMAKR, 0 }, { "landmakrp", LANDMAKR, PATCH_LANDMKRP },
};

// MAME f3_config_table: extend, sprite lag
typedef struct { int game, extend, lag; } f3_cfg;
static const f3_cfg s_cfgs[] = {
	{ RINGRAGE, 0, 2 }, { ARABIANM, 0, 2 }, { RIDINGF, 1, 1 }, { GSEEKER, 0, 1 }, { COMMANDW, 1, 1 },
	{ SCFINALS, 0, 1 }, { TRSTAR, 1, 0 }, { GUNLOCK, 1, 2 }, { LIGHTBR, 1, 2 }, { KAISERKN, 0, 2 },
	{ DARIUSG, 0, 2 }, { BUBSYMPH, 1, 1 }, { SPCINVDX, 1, 1 }, { HTHERO95, 0, 1 }, { QTHEATER, 1, 1 },
	{ EACTION2, 1, 2 }, { RECALH, 1, 1 }, { SPCINV95, 0, 1 }, { TWINQIX, 1, 1 }, { QUIZHUHU, 1, 1 },
	{ PBOBBLE2, 0, 1 }, { GEKIRIDO, 0, 1 }, { KTIGER2, 0, 0 }, { BUBBLEM, 1, 1 }, { CLEOPATR, 0, 1 },
	{ PBOBBLE3, 0, 1 }, { ARKRETRN, 1, 1 }, { KIRAMEKI, 0, 1 }, { PUCHICAR, 1, 1 }, { PBOBBLE4, 0, 1 },
	{ POPNPOP, 1, 1 }, { LANDMAKR, 1, 1 }, { TMDRILL, 1, 0 },
};

// machine configurations: visible lines (f3 / f3_224a / f3_224b / f3_224c)
typedef struct { const char *name; int vis_y0, vis_y1; } f3_machine;
static const f3_machine s_machines[] = {
	{ "f3", 24, 255 }, { "f3_224a", 31, 254 }, { "f3_224b", 32, 255 }, { "f3_224c", 24, 247 },
};

#define LINES 262
#define FRAME_MHZ 58942      // 26.686 MHz / 4 / 432 / 262
#define MAIN_CLOCK 16000000

static const f3_machine *s_m;
static int s_game, s_patch;
static uint8_t *s_rom;
static uint32_t s_rom_size;
static uint8_t (*s_ram_hp)[0x20000];
#define s_ram (*s_ram_hp)
static int s_idle;   // the idle loops found (below): -1 not checked yet, else IDLE_* bits
static uint8_t (*s_pal_hp)[0x8000];
#define s_pal (*s_pal_hp)
static uint8_t (*s_spr_hp)[0x10000];
#define s_spr (*s_spr_hp)
static uint8_t (*s_vram_hp)[0x10000];   // 0x610000-0x61ffff
#define s_vram (*s_vram_hp)
static uint8_t (*s_line_hp)[0x10000];
#define s_line (*s_line_hp)
static uint8_t (*s_pivot_hp)[0x10000];
#define s_pivot (*s_pivot_hp)
static uint16_t s_ctrl0[8], s_ctrl1[8];
static uint32_t s_coin_word[2];
static int s_dial[2];
static f3v_t *s_v;
static eeprom93c46_t s_eeprom;
static char s_nv_path[512];
static int s_port_in[6], s_port_eep;
static uint64_t s_cpu_frac;
static int s_cpu_debt, s_irq3;
static int s_watchdog;            // frames left before the watchdog resets the machine (0 = not armed)

// ---- main CPU devices ----
static uint32_t control_r(int offset)
{
	uint32_t v;
	switch (offset) {
	case 0: {
		rbx_port_custom(s_port_eep, 0x01, eeprom_do(&s_eeprom) ? 1 : 0);
		uint32_t e = rbx_port_read(s_port_eep) & 0xff;
		v = (rbx_port_read(s_port_in[0]) & 0xffff) | e << 16 | e << 24;
		break;
	}
	case 1: v = (rbx_port_read(s_port_in[1]) & 0xffff) | s_coin_word[0] << 16; break;
	case 2: case 3: {
		int d = s_dial[offset - 2] & 0xfff;
		v = (rbx_port_read(s_port_in[offset]) & 0xffff0000u) | (uint32_t)(((d & 0xf) << 12) | ((d & 0xff0) >> 4));
		break;
	}
	case 4: v = rbx_port_read(s_port_in[4]); break;
	case 5: v = (rbx_port_read(s_port_in[5]) & 0xffff) | s_coin_word[1] << 16; break;
	default: v = 0xffffffff; break;
	}
	return v;
}

static void control_w(int offset, uint32_t data, uint32_t mask)
{
	switch (offset) {
	case 0:
		// MAME watchdog_timer_device without a configured time: the first kick arms a 3 second timeout
		s_watchdog = 3 * FRAME_MHZ / 1000 + 1;
		break;
	case 1:
		if (mask & 0xff000000u)
			s_coin_word[0] = (data >> 16) & 0xffff;
		break;
	case 4:
		if (mask & 0xff)
			eeprom_write_lines(&s_eeprom, data & 0x10, data & 0x08, data & 0x04);
		break;
	case 5:
		if (mask & 0xff000000u)
			s_coin_word[1] = (data >> 16) & 0xffff;
		break;
	}
}

static void sound_bankswitch_w(int offset, int low_half)
{
	if (s_game != KIRAMEKI)
		return;
	int idx = (offset << 1) & 0x1e;
	if (low_half)
		idx += 1;
	if (idx >= 8)
		idx -= 8;
	ten_set_bank(1, idx);
}

static uint16_t main_read16(uint32_t a)
{
	if (a >= 0x4a0000 && a <= 0x4a001f) {
		uint32_t v = control_r((int)((a - 0x4a0000) >> 2));
		return (uint16_t)((a & 2) ? v : v >> 16);
	}
	return 0;
}

static uint8_t main_read8(uint32_t a)
{
	return (uint8_t)(main_read16(a & ~1u) >> ((a & 1) ? 0 : 8));
}

static void main_write16m(uint32_t a, uint16_t v, uint16_t mask)
{
	if (a >= 0x660000 && a <= 0x66001f) {
		uint16_t *r = a < 0x660010 ? &s_ctrl0[(a >> 1) & 7] : &s_ctrl1[(a >> 1) & 7];
		*r = (uint16_t)((*r & ~mask) | (v & mask));
		return;
	}
	if (a >= 0x4a0000 && a <= 0x4a001f) {
		int hi = !(a & 2);
		control_w((int)((a - 0x4a0000) >> 2), hi ? (uint32_t)v << 16 : v, hi ? (uint32_t)mask << 16 : mask);
		return;
	}
	if (a >= 0x300000 && a <= 0x30007f) {
		sound_bankswitch_w((int)((a - 0x300000) >> 2), (a & 2) != 0);
		return;
	}
	if (a >= 0xc00000 && a <= 0xc00fff) { ten_dpram_w(a - 0xc00000, v, mask); return; }
	if (a >= 0xc80000 && a <= 0xc80003) { ten_set_reset_line(0); return; }
	if (a >= 0xc80100 && a <= 0xc80103) { ten_set_reset_line(1); return; }
	// 0x4c0000: timer control (unemulated in MAME); 0x4a0000 offset 0: watchdog
}

static void main_write16(uint32_t a, uint16_t v) { main_write16m(a, v, 0xffff); }
static void main_write8(uint32_t a, uint8_t v)
{
	if (a & 1)
		main_write16m(a & ~1u, v, 0x00ff);
	else
		main_write16m(a, (uint16_t)(v << 8), 0xff00);
}

// ---- video ----
static void f3_render(void)
{
	rbx_video.palette_used = f3v_render(s_v, rbx_video.bitmap, rbx_video.pitch, rbx_video.palette);
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_v->ctrl0, s_ctrl0, sizeof(s_ctrl0));
	memcpy(s_v->ctrl1, s_ctrl1, sizeof(s_ctrl1));
	memcpy(s_v->spriteram, s_spr, sizeof(s_spr));
	memcpy(s_v->vram, s_vram, sizeof(s_vram));
	memcpy(s_v->lineram, s_line, sizeof(s_line));
	memcpy(s_v->pivotram, s_pivot, sizeof(s_pivot));
	memcpy(s_v->palram, s_pal, sizeof(s_pal));
	rbx_video_frame_done();
}

// ---- init / frame ----
static const f3_machine *find_machine(const char *name)
{
	for (size_t i = 0; i < sizeof(s_machines) / sizeof(s_machines[0]); i++)
		if (!strcmp(s_machines[i].name, name))
			return &s_machines[i];
	return NULL;
}

static const f3_set *find_set(const rbx_game *g)
{
	for (size_t i = 0; i < sizeof(s_sets) / sizeof(s_sets[0]); i++)
		if (!strcmp(s_sets[i].set, g->set))
			return &s_sets[i];
	if (g->parent)
		for (size_t i = 0; i < sizeof(s_sets) / sizeof(s_sets[0]); i++)
			if (!strcmp(s_sets[i].set, g->parent)) {
				static f3_set clone;
				clone = s_sets[i];
				clone.patch = PATCH_NONE;   // patches are per set (pbobble2 / landmakrp only)
				return &clone;
			}
	return NULL;
}

static void machine_reset(int hard)
{
	if (hard) {   // power on; the watchdog's soft reset keeps the RAM (MAME schedule_soft_reset)
		memset(s_ram, 0, sizeof(s_ram));
		memset(s_spr, 0, sizeof(s_spr));
		memset(s_pal, 0, sizeof(s_pal));   // (palette / video RAM too: 0 as at start, not the last game's)
		memset(s_vram, 0, sizeof(s_vram));
		memset(s_line, 0, sizeof(s_line));
		memset(s_pivot, 0, sizeof(s_pivot));
		memset(s_ctrl0, 0, sizeof(s_ctrl0));
		memset(s_ctrl1, 0, sizeof(s_ctrl1));
		s_coin_word[0] = s_coin_word[1] = 0;
		rbx_video_wait_idle();
		f3v_reset(s_v);
	}
	ten_reset(hard);
	m68k_bus_irq_reset();
	m68k_pulse_reset();
	s_cpu_frac = 0;
	s_cpu_debt = 0;
	s_irq3 = -1;
	s_watchdog = 0;
}

static void f3_reset(void) { machine_reset(1); }

static void f3_exit(void);

static int f3_init(const rbx_game *g)
{
	if (!s_line_hp && !(s_line_hp = calloc(1, sizeof(*s_line_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	if (!s_pal_hp && !(s_pal_hp = calloc(1, sizeof(*s_pal_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	if (!s_pivot_hp && !(s_pivot_hp = calloc(1, sizeof(*s_pivot_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	if (!s_ram_hp && !(s_ram_hp = calloc(1, sizeof(*s_ram_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	if (!s_spr_hp && !(s_spr_hp = calloc(1, sizeof(*s_spr_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	if (!s_vram_hp && !(s_vram_hp = calloc(1, sizeof(*s_vram_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	s_m = find_machine(g->machine);
	s_idle = -1;
	const f3_set *fs = find_set(g);
	if (!s_m || !fs) {
		rbx_fail("%s: machine %s is not emulated", g->set, g->machine);
		return -1;
	}
	s_game = fs->game;
	s_patch = fs->patch;
	s_rom = rbx_region("maincpu", &s_rom_size);
	uint32_t ssz = 0, shsz = 0, tsz = 0, thsz = 0;
	uint8_t *spr = rbx_region("sprites", &ssz), *sprh = rbx_region("sprites_hi", &shsz);
	uint8_t *til = rbx_region("tilemap", &tsz), *tilh = rbx_region("tilemap_hi", &thsz);
	if (!s_rom || s_rom_size < 0x200000 || !spr || !til) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	m68k_bus_swap16(s_rom, s_rom_size);
	if (s_patch == PATCH_PBOBBL2P) {
		// MAME init_pbobbl2p: protection check? NOPs at 0x40092-0x40097
		uint16_t *w = (uint16_t *)(s_rom + 0x40092);
		w[0] = w[1] = w[2] = 0x4e71;
	} else if (s_patch == PATCH_LANDMKRP) {
		uint16_t *w = (uint16_t *)(s_rom + 0x1ffff8);
		w[0] = w[1] = w[2] = 0xffff;
		w[3] = 0x0003;
	}
	s_v = (f3v_t *)calloc(1, sizeof(f3v_t));
	if (!s_v) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	s_v->extend = 0;
	s_v->sprite_lag = 1;
	for (size_t i = 0; i < sizeof(s_cfgs) / sizeof(s_cfgs[0]); i++)
		if (s_cfgs[i].game == s_game) {
			s_v->extend = s_cfgs[i].extend;
			s_v->sprite_lag = s_cfgs[i].lag;
		}
	s_v->pal12 = s_game == SPCINVDX || s_game == RIDINGF || s_game == ARABIANM || s_game == RINGRAGE;
	s_v->vis_y0 = s_m->vis_y0;
	s_v->vis_y1 = s_m->vis_y1;
	if (f3v_init(s_v, spr, ssz, sprh, shsz, til, tsz, tilh, thsz)) {
		rbx_fail("%s: out of memory (graphics)", g->set);
		f3_exit();
		return -1;
	}
	for (int i = 0; i < 6; i++) {
		char tag[8];
		snprintf(tag, sizeof(tag), "IN.%d", i);
		s_port_in[i] = rbx_port_find(tag);
	}
	s_port_eep = rbx_port_find("EEPROMIN");
	s_nv_path[0] = 0;
	eeprom_init(&s_eeprom);
	{
		extern const char *rbx_nvram_dir(void);
		const char *dir = rbx_nvram_dir();
		int loaded = 0;
		if (dir) {
			snprintf(s_nv_path, sizeof(s_nv_path), "%s%s.nv", dir, g->set);
			loaded = eeprom_load(&s_eeprom, s_nv_path) == 0;
		}
		// MAME: default contents from the "eeprom" region (ROM_REGION16_BE) when there is no saved NVRAM
		uint32_t esz = 0;
		const uint8_t *edef = rbx_region("eeprom", &esz);
		if (!loaded && edef && esz >= 128)
			for (int i = 0; i < 64; i++)
				s_eeprom.data[i] = (uint16_t)(edef[2 * i] << 8 | edef[2 * i + 1]);
	}
	if (rbx_video_setup(320, s_m->vis_y1 - s_m->vis_y0 + 1, F3V_PALETTE_SIZE)) {
		f3_exit();
		return -1;
	}
	m68k_init();
	m68k_set_cpu_type(M68K_CPU_TYPE_68EC020);
	m68k_bus_reset_map();
	m68k_bus_map(0x000000, 0x1fffff, s_rom, s_rom_size, 1, 0);
	m68k_bus_map(0x400000, 0x43ffff, s_ram, sizeof(s_ram), 1, 1);
	m68k_bus_map(0x440000, 0x447fff, s_pal, sizeof(s_pal), 1, 1);
	m68k_bus_map(0x600000, 0x60ffff, s_spr, sizeof(s_spr), 1, 1);
	m68k_bus_map(0x610000, 0x61ffff, s_vram, sizeof(s_vram), 1, 1);
	m68k_bus_map(0x620000, 0x62ffff, s_line, sizeof(s_line), 1, 1);
	m68k_bus_map(0x630000, 0x63ffff, s_pivot, sizeof(s_pivot), 1, 1);
	m68k_bus_map(0xc00000, 0xc00fff, ten_dpram(), 0x1000, 1, 0);   // (written through ten_dpram_w)
	m68k_bus.read8 = main_read8;
	m68k_bus.read16 = main_read16;
	m68k_bus.write8 = main_write8;
	m68k_bus.write16 = main_write16;
	ten_config tc;
	tc.frame_rate_mhz = FRAME_MHZ;
	tc.lines = LINES;
	if (ten_init(&tc)) {
		f3_exit();
		return -1;
	}
	f3_reset();
	return 0;
}

static void f3_exit(void)
{
	if (s_nv_path[0] && s_eeprom.dirty)
		eeprom_save(&s_eeprom, s_nv_path);
	s_nv_path[0] = 0;
	ten_exit();
	if (s_v) {
		f3v_exit(s_v);
		free(s_v);
		s_v = NULL;
	}
	free(s_line_hp);
	s_line_hp = NULL;
	free(s_pal_hp);
	s_pal_hp = NULL;
	free(s_pivot_hp);
	s_pivot_hp = NULL;
	free(s_ram_hp);
	s_ram_hp = NULL;
	free(s_spr_hp);
	s_spr_hp = NULL;
	free(s_vram_hp);
	s_vram_hp = NULL;
}

// rbx_line_cycles without its 64-bit division per line (the Vita's CPU has no divider): the same sequence from the
// quotient and remainder of one line
static int line_cycles(uint64_t *frac, uint32_t clock)
{
	const uint64_t den = (uint64_t)FRAME_MHZ * LINES;
	static uint32_t q, r;
	if (!q) {
		q = (uint32_t)(((uint64_t)clock * 1000) / den);
		r = (uint32_t)(((uint64_t)clock * 1000) % den);
	}
	*frac += r;
	if (*frac >= den) {
		*frac -= den;
		return (int)q + 1;
	}
	return (int)q;
}

// ---- idle loop ----
// Bubble Memories waits for its vblank flag in a loop that keeps stepping its random numbers (a third or more of the
// main CPU's time). The main CPU takes its interrupts only between run_main slices (m68k_set_irq), so within a slice
// the loop runs here instead of in Musashi: instruction by instruction with Musashi's cycle counts, the same
// registers, flags and memory at the end, only without the interpreter. Used when the program ROM holds that code.
// The loop (found in the program ROM: Bubble Memories, Bubble Bobble 2, Light Bringer, Dungeon Magic ...):
// L: tst.b (f,a5) / bne out / bsr R / addq.w #1,(k,a5) / bra L, R: a random number step of one of two kinds:
// A: move.w (o1,a5),d0 / bne / moveq / muls.w #m1,d0 / divs.w #q1,d0 / swap / addq.w #a1,d0 / move.w d0,(o1,a5), the same
// with o2 m2 q2 a2, rts; B (xorshift): move.l (o1,a5),d0 / bne / moveq / move.l d0,d1 / moveq #17,d2 / lsl.l d2,d1 /
// eor.l d1,d0 / move.l d0,d1 / moveq #15,d2 / lsr.l d2,d1 / eor.l d1,d0 / move.l d0,(o1,a5) / rts
static struct { uint32_t l, r; uint16_t f, k, o1, o2, m1, q1, m2, q2; uint8_t a1, a2, kind; } s_bm;
typedef struct { uint8_t in_r; uint8_t off; uint8_t cyc; } bm_step;
static const bm_step BM_A[20] = {
	{ 0, 0, 7 }, { 0, 4, 4 }, { 0, 6, 7 },                       // tst.b flag / bne out / bsr R
	{ 1, 0, 7 }, { 1, 4, 6 }, { 1, 8, 29 }, { 1, 12, 58 },       // move.w / bne / muls.w / divs.w
	{ 1, 16, 4 }, { 1, 18, 2 }, { 1, 20, 5 },                    // swap / addq.w / move.w
	{ 1, 24, 7 }, { 1, 28, 6 }, { 1, 32, 29 }, { 1, 36, 58 },
	{ 1, 40, 4 }, { 1, 42, 2 }, { 1, 44, 5 }, { 1, 48, 10 },     // ... / rts
	{ 0, 10, 9 }, { 0, 14, 10 } };                               // addq.w #1 count / bra
static const bm_step BM_B[17] = {
	{ 0, 0, 7 }, { 0, 4, 4 }, { 0, 6, 7 },
	{ 1, 0, 7 }, { 1, 4, 6 }, { 1, 8, 2 }, { 1, 10, 2 }, { 1, 12, 23 },   // move.l / bne / move.l / moveq / lsl.l (6 + 17)
	{ 1, 14, 2 }, { 1, 16, 2 }, { 1, 18, 2 }, { 1, 20, 21 },              // eor.l / move.l / moveq / lsr.l (6 + 15)
	{ 1, 22, 2 }, { 1, 24, 5 }, { 1, 28, 10 },                            // eor.l / move.l / rts
	{ 0, 10, 9 }, { 0, 14, 10 } };
static uint32_t bm_pc(const bm_step *st) { return (st->in_r ? s_bm.r : s_bm.l) + st->off; }

static int bm_find(void)
{
	for (uint32_t l = 0x200; l < 0x180000; l += 2) {
		if (m68k_read_memory_16(l) != 0x4a2d || m68k_read_memory_16(l + 4) != 0x660a || m68k_read_memory_16(l + 6) != 0x6100 ||
		    m68k_read_memory_16(l + 10) != 0x526d || m68k_read_memory_16(l + 14) != 0x60f0)
			continue;
		const uint32_t r = l + 8 + (uint32_t)(int32_t)(int16_t)m68k_read_memory_16(l + 8);
		uint16_t w[25];
		for (int i = 0; i < 25; i++)
			w[i] = (uint16_t)m68k_read_memory_16(r + (uint32_t)i * 2);
		s_bm.l = l;
		s_bm.r = r;
		s_bm.f = (uint16_t)m68k_read_memory_16(l + 2);
		s_bm.k = (uint16_t)m68k_read_memory_16(l + 12);
		s_bm.o1 = w[1];
		if (w[0] == 0x302d && w[2] == 0x6602 && (w[3] & 0xff00) == 0x7000 && w[4] == 0xc1fc && w[6] == 0x81fc && w[8] == 0x4840 &&
		    (w[9] & 0xf1ff) == 0x5040 && w[10] == 0x3b40 && w[11] == w[1] && w[12] == 0x302d && w[14] == 0x6602 &&
		    (w[15] & 0xff00) == 0x7000 && w[16] == 0xc1fc && w[18] == 0x81fc && w[20] == 0x4840 && (w[21] & 0xf1ff) == 0x5040 &&
		    w[22] == 0x3b40 && w[23] == w[13] && w[24] == 0x4e75 && w[7] && w[19]) {
			s_bm.kind = 0;
			s_bm.o2 = w[13];
			s_bm.m1 = w[5];
			s_bm.q1 = w[7];
			s_bm.m2 = w[17];
			s_bm.q2 = w[19];
			s_bm.a1 = (uint8_t)(((w[9] >> 9) & 7) ? (w[9] >> 9) & 7 : 8);
			s_bm.a2 = (uint8_t)(((w[21] >> 9) & 7) ? (w[21] >> 9) & 7 : 8);
			return 1;
		}
		static const uint16_t XS[15] = { 0x202d, 0, 0x6602, 0x7002, 0x2200, 0x7411, 0xe5a9, 0xb380, 0x2200, 0x740f, 0xe4a9,
			0xb380, 0x2b40, 0, 0x4e75 };
		int k = 0;
		while (k < 15 && (k == 1 || k == 13 || w[k] == XS[k]))
			k++;
		if (k == 15 && w[13] == w[1]) {
			s_bm.kind = 1;
			return 1;
		}
	}
	return 0;
}

enum { CC_C = 1, CC_V = 2, CC_Z = 4, CC_N = 8, CC_X = 16 };
static uint32_t cc_nz16(uint32_t v) { return ((v & 0xffff) ? 0 : CC_Z) | ((v & 0x8000) ? CC_N : 0); }
static uint32_t cc_nz32(uint32_t v) { return (v ? 0 : CC_Z) | ((v & 0x80000000u) ? CC_N : 0); }

// addq.w (Musashi m68k_op_addq_16): the new word, its flags
static uint32_t addq16(uint32_t dst, uint32_t src, uint32_t *cc)
{
	uint32_t res = (dst & 0xffff) + src;
	*cc = cc_nz16(res) | (((src ^ res) & (dst ^ res) & 0x8000) ? CC_V : 0) | (res > 0xffff ? CC_C | CC_X : 0);
	return res & 0xffff;
}

// The Taito library's idle task (Darius Gaiden, Puzzle Bobble 2 ...): L: jsr.l C / bra L, C: jsr.w R / rts, R: the
// random number step move.l (o,a5),d0 / muls.l #7,d0 / addq.l #5,d0 / move.l d0,(o,a5) / swap d0 / rts, at addresses
// and with an offset o of each game (found in the program ROM). The calls absolute (jsr.l / jsr.w) or PC-relative
// (jsr (d16,pc): Quiz Theater): L's call lj bytes long, the calls' cycles in cyc
static struct { uint32_t l, c, r, lj; int cyc[2]; uint16_t o; } s_tl;
static const int TL_OFF[10] = { 0, 0, 0, 4, 12, 14, 18, 20, 4, 6 };     // the steps' pcs from L, C, R
static const uint8_t TL_BASE[10] = { 0, 1, 2, 2, 2, 2, 2, 2, 1, 0 };    // (0 = L, 1 = C, 2 = R; = the call depth)
static const int TL_CYC[10] = { 4, 4, 7, 47, 2, 5, 4, 10, 10, 10 };      // Musashi's (measured): jsr.l / jsr.w / move.l /
	// muls.l / addq.l / move.l / swap / rts / rts / bra
static uint32_t tl_pc(int i)
{
	return i == 9 ? s_tl.l + s_tl.lj : (TL_BASE[i] == 0 ? s_tl.l : TL_BASE[i] == 1 ? s_tl.c : s_tl.r) + (uint32_t)TL_OFF[i];
}

static int tl_find(void)
{
	for (uint32_t l = 0x400; l < 0x180000; l += 2) {
		uint32_t c, lj;
		int cyc0;
		const uint32_t op = m68k_read_memory_16(l);
		if (op == 0x4eb9 && m68k_read_memory_16(l + 2) == 0 && m68k_read_memory_16(l + 6) == 0x60f8) {
			c = m68k_read_memory_16(l + 4);
			lj = 6;
			cyc0 = 4;
		} else if (op == 0x4eba && m68k_read_memory_16(l + 4) == 0x60fa) {
			c = l + 2 + (uint32_t)(int32_t)(int16_t)m68k_read_memory_16(l + 2);
			lj = 4;
			cyc0 = 5;
		} else
			continue;
		uint32_t r;
		int cyc1;
		if (m68k_read_memory_16(c + 4) != 0x4e75)
			continue;
		if (m68k_read_memory_16(c) == 0x4eb8) {
			r = m68k_read_memory_16(c + 2);
			if (r >= 0x8000)
				continue;
			cyc1 = 4;
		} else if (m68k_read_memory_16(c) == 0x4eba) {
			r = c + 2 + (uint32_t)(int32_t)(int16_t)m68k_read_memory_16(c + 2);
			cyc1 = 5;
		} else
			continue;
		const uint16_t o = (uint16_t)m68k_read_memory_16(r + 2);
		const uint16_t rand[11] = { 0x202d, o, 0x4c3c, 0x0800, 0x0000, 0x0007, 0x5a80, 0x2b40, o, 0x4840, 0x4e75 };
		unsigned k = 0;
		while (k < 11 && m68k_read_memory_16(r + k * 2) == rand[k])
			k++;
		if (k < 11)
			continue;
		s_tl.l = l;
		s_tl.c = c;
		s_tl.r = r;
		s_tl.lj = lj;
		s_tl.cyc[0] = cyc0;
		s_tl.cyc[1] = cyc1;
		s_tl.o = o;
		return 1;
	}
	return 0;
}

static int match(uint32_t a, const uint16_t *w, unsigned n)
{
	for (unsigned i = 0; i < n; i++)
		if (m68k_read_memory_16(a + i * 2) != w[i])
			return 0;
	return 1;
}

// Waits that count (Arabian Magic, Cup Final, Hat Trick Hero '95 ...): L: tst.b (f,a5) / bne out / addq.l #1,(c,a5) /
// cmpi.l #n,(c,a5) / bcs L; the same with 32-bit displacements (bd,a5) (Kirameki Star Road)
static struct { uint32_t pc[5], n; int32_t f, c; int cyc[5]; } s_fc;
static int fc_find(void)
{
	static const int OFF16[5] = { 0, 4, 6, 10, 18 }, CYC16[5] = { 7, 4, 9, 7, 6 };
	static const int OFF32[5] = { 0, 8, 10, 18, 30 }, CYC32[5] = { 15, 4, 17, 15, 6 };
	for (uint32_t l = 0x200; l < 0x180000; l += 2) {
		const uint32_t op = m68k_read_memory_16(l);
		if (op == 0x4a2d) {
			if ((m68k_read_memory_16(l + 4) & 0xff80) != 0x6600 || m68k_read_memory_16(l + 6) != 0x52ad ||
			    m68k_read_memory_16(l + 10) != 0x0cad || m68k_read_memory_16(l + 18) != 0x65ec ||
			    m68k_read_memory_16(l + 8) != m68k_read_memory_16(l + 16))
				continue;
			s_fc.f = (int16_t)m68k_read_memory_16(l + 2);
			s_fc.c = (int16_t)m68k_read_memory_16(l + 8);
			s_fc.n = m68k_read_memory_32(l + 12);
			for (int i = 0; i < 5; i++) {
				s_fc.pc[i] = l + (uint32_t)OFF16[i];
				s_fc.cyc[i] = CYC16[i];
			}
			return 1;
		}
		if (op == 0x4a35 && m68k_read_memory_16(l + 2) == 0x8170 && (m68k_read_memory_16(l + 8) & 0xff80) == 0x6600 &&
		    m68k_read_memory_16(l + 10) == 0x52b5 && m68k_read_memory_16(l + 12) == 0x8170 && m68k_read_memory_16(l + 18) == 0x0cb5 &&
		    m68k_read_memory_16(l + 24) == 0x8170 && m68k_read_memory_16(l + 30) == 0x65e0 &&
		    m68k_read_memory_32(l + 14) == m68k_read_memory_32(l + 26)) {
			s_fc.f = (int32_t)m68k_read_memory_32(l + 4);
			s_fc.c = (int32_t)m68k_read_memory_32(l + 14);
			s_fc.n = m68k_read_memory_32(l + 20);
			for (int i = 0; i < 5; i++) {
				s_fc.pc[i] = l + (uint32_t)OFF32[i];
				s_fc.cyc[i] = CYC32[i];
			}
			return 1;
		}
	}
	return 0;
}

// Thunder Strike's: L: lea (x,a5),a0 / move.w (y,a5),d0 / addq.w #1,(0,a0,d0.w*2) / tst.b (z,a5) / beq L
static struct { uint32_t l; uint16_t x, y, z; } s_ix;
static const int IX_OFF[5] = { 0, 4, 8, 12, 16 }, IX_CYC[5] = { 7, 7, 11, 7, 6 };
static int ix_find(void)
{
	for (uint32_t l = 0x200; l < 0x40000; l += 2) {
		if (m68k_read_memory_16(l) != 0x41ed || m68k_read_memory_16(l + 4) != 0x302d || m68k_read_memory_16(l + 8) != 0x5270 ||
		    m68k_read_memory_16(l + 10) != 0x0200 || m68k_read_memory_16(l + 12) != 0x4a2d || m68k_read_memory_16(l + 16) != 0x67ee)
			continue;
		s_ix.l = l;
		s_ix.x = (uint16_t)m68k_read_memory_16(l + 2);
		s_ix.y = (uint16_t)m68k_read_memory_16(l + 6);
		s_ix.z = (uint16_t)m68k_read_memory_16(l + 14);
		return 1;
	}
	return 0;
}

// A delay that counts in d0 (RingRage): L: addq.w #1,d0 / cmpi.w #n,d0 / bls L
static struct { uint32_t l; uint16_t n; } s_dc;
static const int DC_OFF[3] = { 0, 2, 6 }, DC_CYC[3] = { 2, 2, 6 };
static int dc_find(void)
{
	for (uint32_t l = 0x200; l < 0x40000; l += 2)
		if (m68k_read_memory_16(l) == 0x5240 && m68k_read_memory_16(l + 2) == 0x0c40 && m68k_read_memory_16(l + 6) == 0x63f8) {
			s_dc.l = l;
			s_dc.n = (uint16_t)m68k_read_memory_16(l + 4);
			return 1;
		}
	return 0;
}

// Grid Seeker's idle task: L: bsr R / addq.w #1,(k,a5) / bra L, R: move.w (o,a5),d0 / bne +2 / moveq #1,d0 /
// muls.w #m,d0 / divs.w #q,d0 / swap d0 / move.w d0,(o,a5) / rts
static struct { uint32_t l, r; uint16_t k, o, m, q; } s_gs;
static const int GS_OFF[10] = { 0, 0, 4, 8, 12, 16, 18, 22, 4, 8 };
static const uint8_t GS_IN[10] = { 0, 1, 1, 1, 1, 1, 1, 1, 0, 0 };   // (1: in R, its return address on the stack)
static const int GS_CYC[10] = { 7, 7, 6, 29, 58, 4, 5, 10, 9, 10 };
static uint32_t gs_pc(int i) { return (GS_IN[i] ? s_gs.r : s_gs.l) + (uint32_t)GS_OFF[i]; }
static int gs_find(void)
{
	for (uint32_t l = 0x200; l < 0x40000; l += 2) {
		if (m68k_read_memory_16(l) != 0x6100 || m68k_read_memory_16(l + 4) != 0x526d || m68k_read_memory_16(l + 8) != 0x60f6)
			continue;
		const uint32_t r = l + 2 + (uint32_t)(int32_t)(int16_t)m68k_read_memory_16(l + 2);
		const uint16_t o = (uint16_t)m68k_read_memory_16(r + 2);
		if (m68k_read_memory_16(r) != 0x302d || m68k_read_memory_16(r + 4) != 0x6602 || m68k_read_memory_16(r + 6) != 0x7001 ||
		    m68k_read_memory_16(r + 8) != 0xc1fc || m68k_read_memory_16(r + 12) != 0x81fc || m68k_read_memory_16(r + 16) != 0x4840 ||
		    m68k_read_memory_16(r + 18) != 0x3b40 || m68k_read_memory_16(r + 20) != o || m68k_read_memory_16(r + 22) != 0x4e75)
			continue;
		const uint16_t q = (uint16_t)m68k_read_memory_16(r + 14);
		if (!q)
			continue;
		s_gs.l = l;
		s_gs.r = r;
		s_gs.k = (uint16_t)m68k_read_memory_16(l + 6);
		s_gs.o = o;
		s_gs.m = (uint16_t)m68k_read_memory_16(r + 10);
		s_gs.q = q;
		return 1;
	}
	return 0;
}

// The same with the watchdog kicked each time (RayForce, Gunlock): L: clr.b 0x4a0000 / addq.l #1,(c,a5) /
// cmpi.l #n,(c,a5) / bcs L (the kick sets the watchdog to the same count each time: done once at the end)
static struct { uint32_t l, n; uint16_t c; } s_wd;
static const int WD_OFF[4] = { 0, 6, 10, 18 }, WD_CYC[4] = { 8, 9, 7, 6 };
static int wd_find(void)
{
	for (uint32_t l = 0x200; l < 0x40000; l += 2) {
		if (m68k_read_memory_16(l) != 0x4239 || m68k_read_memory_16(l + 2) != 0x004a || m68k_read_memory_16(l + 4) != 0 ||
		    m68k_read_memory_16(l + 6) != 0x52ad || m68k_read_memory_16(l + 10) != 0x0cad || m68k_read_memory_16(l + 18) != 0x65ec ||
		    m68k_read_memory_16(l + 8) != m68k_read_memory_16(l + 16))
			continue;
		s_wd.l = l;
		s_wd.c = (uint16_t)m68k_read_memory_16(l + 8);
		s_wd.n = m68k_read_memory_32(l + 12);
		return 1;
	}
	return 0;
}

// A wait with the watchdog kicked (Puchi Carat, Pop'n Pop, Arkanoid Returns, Quiz Huhu Hu ...): L: move.w #0,0x4a0000 /
// tst.w (a,a5) / bne T / move.b 0x4a0000,d0 / andi.b #8,d0 / beq S / move.b 0x4a0001,d0 / andi.b #2,d0 / bne T /
// S: (to Musashi) ... T: tst.w (b,a5) / bpl L (or beq L). Both paths to T in C (the inputs do not change within a slice).
// S as Pop'n Pop has it (move.l #imm,(c,a5) then T; the input bit held low while playing) also in C: before, the whole
// slice ran in Musashi each time the loop took S
static int s_tl_kw_nt;   // (the first bne's cycles not taken)
enum { KW_KICK, KW_TSTA, KW_BNE_T, KW_IN0, KW_AND8, KW_BEQ_S, KW_IN1, KW_AND2, KW_BNE_T2, KW_TSTB, KW_LOOP, KW_S };
static struct { uint32_t pc[12]; int cyc[12]; int beq_loop, has_s; uint16_t a, b, c; uint32_t imm; } s_kw;
// a short or word branch at p: its target, its length, the cycles not taken (Musashi 68020: 6 taken, 4 / 6 not taken)
static uint32_t kw_branch(uint32_t p, uint32_t *len, int *notake)
{
	const uint32_t op = m68k_read_memory_16(p);
	if (op & 0xff) {
		*len = 2;
		*notake = 4;
		return p + 2 + (uint32_t)(int32_t)(int8_t)op;
	}
	*len = 4;
	*notake = 6;
	return p + 2 + (uint32_t)(int32_t)(int16_t)m68k_read_memory_16(p + 2);
}
static int kw_find(void)
{
	for (uint32_t l = 0x200; l < 0x40000; l += 2) {
		if (m68k_read_memory_16(l) != 0x33fc || m68k_read_memory_16(l + 2) != 0 || m68k_read_memory_16(l + 4) != 0x004a ||
		    m68k_read_memory_16(l + 6) != 0 || m68k_read_memory_16(l + 8) != 0x4a6d || (m68k_read_memory_16(l + 12) & 0xff00) != 0x6600)
			continue;
		uint32_t len, len2, len3;
		int nt, nt2, nt3;
		const uint32_t t = kw_branch(l + 12, &len, &nt);
		const uint32_t m = l + 12 + len;   // the inputs path
		if (m68k_read_memory_16(m) != 0x1039 || m68k_read_memory_16(m + 2) != 0x004a || m68k_read_memory_16(m + 4) != 0 ||
		    m68k_read_memory_16(m + 6) != 0x0200 || m68k_read_memory_16(m + 8) != 0x0008 || (m68k_read_memory_16(m + 10) & 0xff00) != 0x6700)
			continue;
		(void)kw_branch(m + 10, &len2, &nt2);
		const uint32_t n = m + 10 + len2;
		if (m68k_read_memory_16(n) != 0x1039 || m68k_read_memory_16(n + 2) != 0x004a || m68k_read_memory_16(n + 4) != 1 ||
		    m68k_read_memory_16(n + 6) != 0x0200 || m68k_read_memory_16(n + 8) != 0x0002 || (m68k_read_memory_16(n + 10) & 0xff00) != 0x6600 ||
		    kw_branch(n + 10, &len3, &nt3) != t)
			continue;
		const uint32_t loop = m68k_read_memory_16(t + 4);
		if (m68k_read_memory_16(t) != 0x4a6d || ((loop & 0xff00) != 0x6a00 && (loop & 0xff00) != 0x6700) || !(loop & 0xff) ||
		    t + 6 + (uint32_t)(int32_t)(int8_t)loop != l)
			continue;
		const uint32_t pc[11] = { l, l + 8, l + 12, m, m + 6, m + 10, n, n + 6, n + 10, t, t + 4 };
		const int cyc[11] = { 8, 7, 6, 6, 2, nt2, 6, 2, 6, 7, 6 };   // (the bne to T: taken, or not on the inputs path)
		memcpy(s_kw.pc, pc, sizeof(pc));
		memcpy(s_kw.cyc, cyc, sizeof(cyc));
		s_kw.a = (uint16_t)m68k_read_memory_16(l + 10);
		s_kw.b = (uint16_t)m68k_read_memory_16(t + 2);
		s_kw.beq_loop = (loop & 0xff00) == 0x6700;
		s_kw.cyc[KW_BNE_T] = 6;   // (taken; not taken: nt, below)
		s_kw.pc[KW_LOOP] = t + 4;
		s_tl_kw_nt = nt;
		{	// S: move.l #imm,(c,a5) falling into T, its long not over (a,a5) or (b,a5)
			uint32_t ls;
			int nts;
			const uint32_t sp = kw_branch(m + 10, &ls, &nts);
			const uint16_t c = (uint16_t)m68k_read_memory_16(sp + 6);
			const int ci = (int16_t)c, ai = (int16_t)s_kw.a, bi = (int16_t)s_kw.b;
			s_kw.has_s = m68k_read_memory_16(sp) == 0x2b7c && sp + 8 == t && (ai + 2 <= ci || ai >= ci + 4) &&
			             (bi + 2 <= ci || bi >= ci + 4);
			s_kw.pc[KW_S] = s_kw.has_s ? sp : 0xffffffffu;
			s_kw.cyc[KW_S] = 9;   // (Musashi 68020 move.l #imm,(d16,An))
			s_kw.c = c;
			s_kw.imm = m68k_read_memory_32(sp + 2);
		}
		return 1;
	}
	return 0;
}

enum { IDLE_BM = 1, IDLE_TL = 2, IDLE_FC = 4, IDLE_IX = 8, IDLE_DC = 16, IDLE_GS = 32, IDLE_WD = 64, IDLE_KW = 128, IDLE_RF = 256, IDLE_AW = 512,
       IDLE_PB = 1024 };
static int rf_find(void);
static int aw_find(void);
static int pb_find(void);
static void idle_check(void)
{
	s_idle = 0;
	if (bm_find())
		s_idle |= IDLE_BM;
	if (tl_find())
		s_idle |= IDLE_TL;
	if (fc_find())
		s_idle |= IDLE_FC;
	if (ix_find())
		s_idle |= IDLE_IX;
	if (dc_find())
		s_idle |= IDLE_DC;
	if (gs_find())
		s_idle |= IDLE_GS;
	if (wd_find())
		s_idle |= IDLE_WD;
	if (kw_find())
		s_idle |= IDLE_KW;
	if (rf_find())
		s_idle |= IDLE_RF;
	if (aw_find())
		s_idle |= IDLE_AW;
	if (pb_find())
		s_idle |= IDLE_PB;
	if (s_idle)
		rbx_log("f3: idle loops %x", s_idle);
}

// runs the loop from its current instruction: the cycles used (Musashi's: until none are left), fewer when it comes
// to what it does not handle (leaving the loop, a zero random number), 0 when the CPU is not in it
static int idle_run(int want)
{
	const bm_step *st = s_bm.kind ? BM_B : BM_A;
	const int n = s_bm.kind ? 17 : 20, last = n - 1;
	const uint32_t pc = m68k_get_reg(NULL, M68K_REG_PC);
	int i = 0;
	while (i < n && bm_pc(&st[i]) != pc)
		i++;
	if (i == n)
		return 0;
	const uint32_t a5 = m68k_get_reg(NULL, M68K_REG_A5), sr = m68k_get_reg(NULL, M68K_REG_SR), ret = s_bm.l + 10;
	uint32_t sp = m68k_get_reg(NULL, M68K_REG_A7), d0 = m68k_get_reg(NULL, M68K_REG_D0), cc = sr & 0x1f;
	uint32_t d1 = m68k_get_reg(NULL, M68K_REG_D1), d2 = m68k_get_reg(NULL, M68K_REG_D2);
#define A5OFF(o) (a5 + (uint32_t)(int32_t)(int16_t)(o))
	if (m68k_read_memory_8(A5OFF(s_bm.f)))
		return 0;   // (the flag is set: leaving)
	if (st[i].in_r && m68k_read_memory_32(sp) != ret)
		return 0;
	const uint32_t ka = A5OFF(s_bm.k), oa1 = A5OFF(s_bm.o1), oa2 = A5OFF(s_bm.o2);
	const uint32_t fa = A5OFF(s_bm.f);
	// (the values must not overlap: each kept in a local)
	if (ka - oa1 + 3 < 7 || (!s_bm.kind && (ka - oa2 + 1 < 3 || oa1 - oa2 + 1 < 3)) || fa - ka < 2 || fa - oa1 < 4 ||
	    (!s_bm.kind && fa - oa2 < 2) || sp - ka + 3 < 7 || sp - oa1 + 3 < 7)
		return 0;
	uint32_t r1 = s_bm.kind ? m68k_read_memory_32(oa1) : m68k_read_memory_16(oa1);
	uint32_t r2 = s_bm.kind ? 0 : m68k_read_memory_16(oa2);
	uint32_t cnt = m68k_read_memory_16(ka);
	int rem = want, pushed = 0;
	while (rem > 0) {
		if (i == 0)
			cc = (cc & CC_X) | CC_Z;   // tst.b (flag 0)
		else if (i == 1) {
			if (!(cc & CC_Z))
				goto stop;
		} else if (i == 2) {
			sp -= 4;
			pushed = 1;
		} else if (i == last - 2)
			sp += 4;   // rts
		else if (i == last - 1)
			cnt = addq16(cnt, 1, &cc);
		else if (i == last) {
		} else if (!s_bm.kind) {
			switch (i) {
			case 3: d0 = (d0 & 0xffff0000u) | r1; cc = (cc & CC_X) | cc_nz16(r1); break;
			case 10: d0 = (d0 & 0xffff0000u) | r2; cc = (cc & CC_X) | cc_nz16(r2); break;
			case 4: case 11: if (!(d0 & 0xffff)) goto stop; break;   // (bne taken)
			case 5: case 12:
				d0 = (uint32_t)((int32_t)(int16_t)d0 * (int32_t)(int16_t)(i == 5 ? s_bm.m1 : s_bm.m2));
				cc = (cc & CC_X) | cc_nz32(d0);
				break;
			case 6: case 13: {
				const int32_t div = (int16_t)(i == 6 ? s_bm.q1 : s_bm.q2), q = (int32_t)d0 / div, r = (int32_t)d0 % div;
				if (q != (int16_t)q)
					goto stop;   // (overflow: to Musashi)
				d0 = ((uint32_t)q & 0xffff) | ((uint32_t)r << 16);
				cc = (cc & CC_X) | cc_nz16((uint32_t)q);
				break;
			}
			case 7: case 14: d0 = (d0 << 16) | (d0 >> 16); cc = (cc & CC_X) | cc_nz32(d0); break;
			case 8: d0 = (d0 & 0xffff0000u) | addq16(d0, s_bm.a1, &cc); break;
			case 15: d0 = (d0 & 0xffff0000u) | addq16(d0, s_bm.a2, &cc); break;
			case 9: r1 = d0 & 0xffff; cc = (cc & CC_X) | cc_nz16(r1); break;
			case 16: r2 = d0 & 0xffff; cc = (cc & CC_X) | cc_nz16(r2); break;
			}
		} else {
			switch (i) {
			case 3: d0 = r1; cc = (cc & CC_X) | cc_nz32(d0); break;   // move.l (o,a5),d0
			case 4: if (!d0) goto stop; break;                          // (bne taken)
			case 5: case 9: d1 = d0; cc = (cc & CC_X) | cc_nz32(d1); break;   // move.l d0,d1
			case 6: d2 = 17; cc = cc & CC_X; break;                    // moveq #17,d2
			case 10: d2 = 15; cc = cc & CC_X; break;                   // moveq #15,d2
			case 7: {   // lsl.l d2,d1 (Musashi m68k_op_lsl_32_r)
				const uint32_t c = (d1 >> 15) & 1;
				d1 <<= 17;
				cc = cc_nz32(d1) | (c ? CC_C | CC_X : 0);
				break;
			}
			case 11: {   // lsr.l d2,d1
				const uint32_t c = (d1 >> 14) & 1;
				d1 >>= 15;
				cc = cc_nz32(d1) | (c ? CC_C | CC_X : 0);
				break;
			}
			case 8: case 12: d0 ^= d1; cc = (cc & CC_X) | cc_nz32(d0); break;   // eor.l d1,d0
			case 13: r1 = d0; cc = (cc & CC_X) | cc_nz32(d0); break;   // move.l d0,(o,a5)
			}
		}
		rem -= st[i].cyc;
		i = i == last ? 0 : i + 1;
	}
stop:
	if (rem == want)
		return 0;
	if (pushed)
		m68k_write_memory_32(st[i].in_r ? sp : sp - 4, ret);
	if (s_bm.kind)
		m68k_write_memory_32(oa1, r1);
	else {
		m68k_write_memory_16(oa1, r1);
		m68k_write_memory_16(oa2, r2);
	}
	m68k_write_memory_16(ka, cnt);
	m68k_set_reg(M68K_REG_D0, d0);
	m68k_set_reg(M68K_REG_D1, d1);
	m68k_set_reg(M68K_REG_D2, d2);
	m68k_set_reg(M68K_REG_A7, sp);
	m68k_set_reg(M68K_REG_SR, (sr & ~0x1fu) | cc);
	m68k_set_reg(M68K_REG_PC, bm_pc(&st[i]));
#undef A5OFF
	return want - rem;
}

// the library idle task as idle_run (the stack: the two return addresses its calls push)
static int idle_run_dg(int want)
{
	const uint32_t pc = m68k_get_reg(NULL, M68K_REG_PC);
	int i = 0;
	while (i < 10 && tl_pc(i) != pc)
		i++;
	if (i == 10)
		return 0;
	const uint32_t a5 = m68k_get_reg(NULL, M68K_REG_A5), sr = m68k_get_reg(NULL, M68K_REG_SR);
	uint32_t sp = m68k_get_reg(NULL, M68K_REG_A7), d0 = m68k_get_reg(NULL, M68K_REG_D0), cc = sr & 0x1f;
	const uint32_t outer = sp + 4u * TL_BASE[i];   // the stack pointer in the loop itself
	const uint32_t ret_l = s_tl.l + s_tl.lj, ret_c = s_tl.c + 4;
	if (TL_BASE[i] >= 1 && m68k_read_memory_32(outer - 4) != ret_l)
		return 0;
	if (TL_BASE[i] == 2 && m68k_read_memory_32(outer - 8) != ret_c)
		return 0;
	const uint32_t ra = a5 + (uint32_t)(int32_t)(int16_t)s_tl.o;   // ((o,a5): sign-extended)
	uint32_t r = m68k_read_memory_32(ra);
	int rem = want, pushed = 0;
	while (rem > 0) {
		switch (i) {
		case 0: pushed |= 1; break;   // jsr (the return addresses: written at the end)
		case 1: pushed |= 2; break;
		case 2: d0 = r; cc = (cc & CC_X) | cc_nz32(d0); break;   // move.l (ebaa,a5),d0
		case 3: {   // muls.l #7,d0 (Musashi m68k_op_mull_32_i, 32-bit result)
			const int64_t res = (int64_t)7 * (int64_t)(int32_t)d0;
			d0 = (uint32_t)res;
			cc = (cc & CC_X) | cc_nz32(d0) | (res != (int64_t)(int32_t)d0 ? CC_V : 0);
			break;
		}
		case 4: {   // addq.l #5,d0 (m68k_op_addq_32_d)
			const uint32_t res = d0 + 5;
			cc = cc_nz32(res) | (((5 ^ res) & (d0 ^ res) & 0x80000000u) ? CC_V : 0) | (res < d0 ? CC_C | CC_X : 0);
			d0 = res;
			break;
		}
		case 5: r = d0; cc = (cc & CC_X) | cc_nz32(d0); break;   // move.l d0,(ebaa,a5)
		case 6: d0 = (d0 << 16) | (d0 >> 16); cc = (cc & CC_X) | cc_nz32(d0); break;   // swap
		default: break;   // rts / rts / bra
		}
		rem -= i < 2 ? s_tl.cyc[i] : TL_CYC[i];
		i = i == 9 ? 0 : i + 1;
	}
	if (rem == want)
		return 0;
	if (pushed & 1)
		m68k_write_memory_32(outer - 4, ret_l);
	if (pushed & 2)
		m68k_write_memory_32(outer - 8, ret_c);
	m68k_write_memory_32(ra, r);
	m68k_set_reg(M68K_REG_D0, d0);
	m68k_set_reg(M68K_REG_A7, outer - 4u * TL_BASE[i]);
	m68k_set_reg(M68K_REG_SR, (sr & ~0x1fu) | cc);
	m68k_set_reg(M68K_REG_PC, tl_pc(i));
	return want - rem;
}

static uint32_t cc_sub32(uint32_t src, uint32_t dst, uint32_t res)   // (cmp: N Z V C, Musashi's)
{
	return cc_nz32(res) | ((((src ^ dst) & (res ^ dst)) & 0x80000000u) ? CC_V : 0) |
		((((src & res) | (~dst & (src | res))) & 0x80000000u) ? CC_C : 0);
}

// the counting wait (s_fc) as idle_run
static int idle_run_fc(int want)
{
	const uint32_t pc = m68k_get_reg(NULL, M68K_REG_PC);
	int i = 0;
	while (i < 5 && s_fc.pc[i] != pc)
		i++;
	if (i == 5)
		return 0;
	const uint32_t a5 = m68k_get_reg(NULL, M68K_REG_A5), sr = m68k_get_reg(NULL, M68K_REG_SR);
	const uint32_t fa = a5 + (uint32_t)s_fc.f, ca = a5 + (uint32_t)s_fc.c;
	if (fa - ca < 4)
		return 0;   // (the flag inside the counter)
	const uint32_t flag = m68k_read_memory_8(fa);
	uint32_t cnt = m68k_read_memory_32(ca), cc = sr & 0x1f;
	int rem = want;
	while (rem > 0) {
		switch (i) {
		case 0: cc = (cc & CC_X) | ((flag & 0xff) ? 0 : CC_Z) | ((flag & 0x80) ? CC_N : 0); break;   // tst.b
		case 1: if (!(cc & CC_Z)) goto stop; break;   // (bne taken: out)
		case 2: {   // addq.l #1,(c,a5)
			const uint32_t res = cnt + 1;
			cc = cc_nz32(res) | (((1 ^ res) & (cnt ^ res) & 0x80000000u) ? CC_V : 0) | (res < cnt ? CC_C | CC_X : 0);
			cnt = res;
			break;
		}
		case 3: cc = (cc & CC_X) | cc_sub32(s_fc.n, cnt, cnt - s_fc.n); break;   // cmpi.l #n,(c,a5)
		case 4: if (!(cc & CC_C)) goto stop; break;   // (bcs not taken: out)
		}
		rem -= s_fc.cyc[i];
		i = i == 4 ? 0 : i + 1;
	}
stop:
	if (rem == want)
		return 0;
	m68k_write_memory_32(ca, cnt);
	m68k_set_reg(M68K_REG_SR, (sr & ~0x1fu) | cc);
	m68k_set_reg(M68K_REG_PC, s_fc.pc[i]);
	return want - rem;
}

// the counting wait (s_ix) as idle_run
static int idle_run_ix(int want)
{
	const uint32_t pc = m68k_get_reg(NULL, M68K_REG_PC);
	int i = 0;
	while (i < 5 && s_ix.l + (uint32_t)IX_OFF[i] != pc)
		i++;
	if (i == 5)
		return 0;
	const uint32_t a5 = m68k_get_reg(NULL, M68K_REG_A5), sr = m68k_get_reg(NULL, M68K_REG_SR);
	uint32_t a0 = m68k_get_reg(NULL, M68K_REG_A0), d0 = m68k_get_reg(NULL, M68K_REG_D0), cc = sr & 0x1f;
	const uint32_t ya = a5 + (uint32_t)(int32_t)(int16_t)s_ix.y, za = a5 + (uint32_t)(int32_t)(int16_t)s_ix.z;
	const uint32_t flag = m68k_read_memory_8(za), y = m68k_read_memory_16(ya);
	// the counter: (0,a0,d0.w*2) of the values this run gives a0 and d0 (a0 = a5 + x, d0.w = (y,a5) from step 1 on)
	const uint32_t ca = (i <= 1 ? a5 + (uint32_t)(int32_t)(int16_t)s_ix.x : a0) +
		(uint32_t)(int32_t)(int16_t)(i <= 1 ? y : d0 & 0xffff) * 2;
	if (ca - ya + 1 < 3 || za - ca < 2)
		return 0;   // (the counter over the index word or the flag)
	if (i >= 2 && (a0 != a5 + (uint32_t)(int32_t)(int16_t)s_ix.x || (d0 & 0xffff) != y))
		return 0;   // (entered mid-loop with other registers)
	uint32_t cnt = m68k_read_memory_16(ca);
	int rem = want, touched = 0;
	while (rem > 0) {
		switch (i) {
		case 0: a0 = a5 + (uint32_t)(int32_t)(int16_t)s_ix.x; break;   // lea
		case 1: d0 = (d0 & 0xffff0000u) | y; cc = (cc & CC_X) | cc_nz16(y); break;   // move.w (y,a5),d0
		case 2: cnt = addq16(cnt, 1, &cc); touched = 1; break;   // addq.w #1,(0,a0,d0.w*2)
		case 3: cc = (cc & CC_X) | ((flag & 0xff) ? 0 : CC_Z) | ((flag & 0x80) ? CC_N : 0); break;   // tst.b
		case 4: if (!(cc & CC_Z)) goto stop; break;   // (beq not taken: out)
		}
		rem -= IX_CYC[i];
		i = i == 4 ? 0 : i + 1;
	}
stop:
	if (rem == want)
		return 0;
	if (touched)
		m68k_write_memory_16(ca, cnt);
	m68k_set_reg(M68K_REG_A0, a0);
	m68k_set_reg(M68K_REG_D0, d0);
	m68k_set_reg(M68K_REG_SR, (sr & ~0x1fu) | cc);
	m68k_set_reg(M68K_REG_PC, s_ix.l + (uint32_t)IX_OFF[i]);
	return want - rem;
}

// the watchdog count (s_wd) as idle_run
static int idle_run_wd(int want)
{
	const uint32_t pc = m68k_get_reg(NULL, M68K_REG_PC);
	int i = 0;
	while (i < 4 && s_wd.l + (uint32_t)WD_OFF[i] != pc)
		i++;
	if (i == 4)
		return 0;
	const uint32_t a5 = m68k_get_reg(NULL, M68K_REG_A5), sr = m68k_get_reg(NULL, M68K_REG_SR);
	const uint32_t ca = a5 + (uint32_t)(int32_t)(int16_t)s_wd.c;
	if (ca - 0x4a0000u + 3 < 0x24)
		return 0;   // (the counter in the control registers)
	uint32_t cnt = m68k_read_memory_32(ca), cc = sr & 0x1f;
	int rem = want, kicked = 0;
	while (rem > 0) {
		switch (i) {
		case 0: cc = (cc & CC_X) | CC_Z; kicked = 1; break;   // clr.b 0x4a0000
		case 1: {   // addq.l #1,(c,a5)
			const uint32_t res = cnt + 1;
			cc = cc_nz32(res) | (((1 ^ res) & (cnt ^ res) & 0x80000000u) ? CC_V : 0) | (res < cnt ? CC_C | CC_X : 0);
			cnt = res;
			break;
		}
		case 2: cc = (cc & CC_X) | cc_sub32(s_wd.n, cnt, cnt - s_wd.n); break;   // cmpi.l #n,(c,a5)
		case 3: if (!(cc & CC_C)) goto stop; break;   // (bcs not taken: out)
		}
		rem -= WD_CYC[i];
		i = i == 3 ? 0 : i + 1;
	}
stop:
	if (rem == want)
		return 0;
	if (kicked)
		m68k_write_memory_8(0x4a0000, 0);
	m68k_write_memory_32(ca, cnt);
	m68k_set_reg(M68K_REG_SR, (sr & ~0x1fu) | cc);
	m68k_set_reg(M68K_REG_PC, s_wd.l + (uint32_t)WD_OFF[i]);
	return want - rem;
}

// the kicked wait (s_kw) as idle_run
static int idle_run_kw(int want)
{
	const uint32_t pc = m68k_get_reg(NULL, M68K_REG_PC);
	int i = 0;
	while (i < 12 && s_kw.pc[i] != pc)
		i++;
	if (i == 12)
		return 0;
	const uint32_t a5 = m68k_get_reg(NULL, M68K_REG_A5), sr = m68k_get_reg(NULL, M68K_REG_SR);
	const uint32_t av = m68k_read_memory_16(a5 + (uint32_t)(int32_t)(int16_t)s_kw.a);
	const uint32_t bv = m68k_read_memory_16(a5 + (uint32_t)(int32_t)(int16_t)s_kw.b);
	const uint32_t in0 = m68k_read_memory_8(0x4a0000), in1 = m68k_read_memory_8(0x4a0001);
	uint32_t d0 = m68k_get_reg(NULL, M68K_REG_D0), cc = sr & 0x1f;
	int rem = want, kicked = 0, wrote = 0;
	while (rem > 0) {
		int next = i + 1, cyc = s_kw.cyc[i];
		switch (i) {
		case KW_KICK: cc = (cc & CC_X) | CC_Z; kicked = 1; break;   // move.w #0,0x4a0000
		case KW_TSTA: cc = (cc & CC_X) | cc_nz16(av); break;         // tst.w (a,a5)
		case KW_BNE_T:
			if (cc & CC_Z)
				cyc = s_tl_kw_nt;   // (not taken: the inputs)
			else
				next = KW_TSTB;
			break;
		case KW_IN0: case KW_IN1:   // move.b 0x4a0000 / 0x4a0001,d0
			d0 = (d0 & ~0xffu) | (i == KW_IN0 ? in0 : in1);
			cc = (cc & CC_X) | ((d0 & 0xff) ? 0 : CC_Z) | ((d0 & 0x80) ? CC_N : 0);
			break;
		case KW_AND8: case KW_AND2:   // andi.b #8 / #2,d0
			d0 &= ~0xffu | (i == KW_AND8 ? 8u : 2u);
			cc = (cc & CC_X) | ((d0 & 0xff) ? 0 : CC_Z) | ((d0 & 0x80) ? CC_N : 0);
			break;
		case KW_BEQ_S:
			if (cc & CC_Z) {   // (taken: S)
				if (!s_kw.has_s)
					goto stop;   // (to Musashi)
				next = KW_S;
				cyc = 6;
			}
			break;
		case KW_S: wrote = 1; next = KW_TSTB; break;   // move.l #imm,(c,a5) (written below, the same each time)
		case KW_BNE_T2:
			if (cc & CC_Z)
				goto stop;   // (not taken: to Musashi)
			next = KW_TSTB;
			break;
		case KW_TSTB: cc = (cc & CC_X) | cc_nz16(bv); break;   // tst.w (b,a5)
		case KW_LOOP:
			if (s_kw.beq_loop ? !(cc & CC_Z) : (cc & CC_N))
				goto stop;   // (bpl / beq not taken: out)
			next = KW_KICK;
			break;
		}
		rem -= cyc;
		i = next;
	}
stop:
	if (rem == want)
		return 0;
	if (kicked)
		m68k_write_memory_16(0x4a0000, 0);
	if (wrote)
		m68k_write_memory_32(m68k_get_reg(NULL, M68K_REG_A5) + (uint32_t)(int32_t)(int16_t)s_kw.c, s_kw.imm);
	m68k_set_reg(M68K_REG_D0, d0);
	m68k_set_reg(M68K_REG_SR, (sr & ~0x1fu) | cc);
	m68k_set_reg(M68K_REG_PC, s_kw.pc[i]);
	return want - rem;
}

// the d0 delay (s_dc) as idle_run
static int idle_run_dc(int want)
{
	const uint32_t pc = m68k_get_reg(NULL, M68K_REG_PC);
	int i = 0;
	while (i < 3 && s_dc.l + (uint32_t)DC_OFF[i] != pc)
		i++;
	if (i == 3)
		return 0;
	const uint32_t sr = m68k_get_reg(NULL, M68K_REG_SR);
	uint32_t d0 = m68k_get_reg(NULL, M68K_REG_D0), cc = sr & 0x1f;
	int rem = want;
	while (rem > 0) {
		switch (i) {
		case 0: d0 = (d0 & 0xffff0000u) | addq16(d0, 1, &cc); break;   // addq.w #1,d0
		case 1: {   // cmpi.w #n,d0 (Musashi m68k_op_cmpi_16_d)
			const uint32_t src = s_dc.n, dst = d0 & 0xffff, res = dst - src;
			cc = (cc & CC_X) | cc_nz16(res) | ((((src ^ dst) & (res ^ dst)) & 0x8000) ? CC_V : 0) | ((res & 0x10000) ? CC_C : 0);
			break;
		}
		case 2: if (!(cc & (CC_C | CC_Z))) goto stop; break;   // (bls not taken: out)
		}
		rem -= DC_CYC[i];
		i = i == 2 ? 0 : i + 1;
	}
stop:
	if (rem == want)
		return 0;
	m68k_set_reg(M68K_REG_D0, d0);
	m68k_set_reg(M68K_REG_SR, (sr & ~0x1fu) | cc);
	m68k_set_reg(M68K_REG_PC, s_dc.l + (uint32_t)DC_OFF[i]);
	return want - rem;
}

// the Grid Seeker task (s_gs) as idle_run
static int idle_run_gs(int want)
{
	const uint32_t pc = m68k_get_reg(NULL, M68K_REG_PC);
	int i = 0;
	while (i < 10 && gs_pc(i) != pc)
		i++;
	if (i == 10)
		return 0;
	const uint32_t a5 = m68k_get_reg(NULL, M68K_REG_A5), sr = m68k_get_reg(NULL, M68K_REG_SR), ret = s_gs.l + 4;
	uint32_t sp = m68k_get_reg(NULL, M68K_REG_A7), d0 = m68k_get_reg(NULL, M68K_REG_D0), cc = sr & 0x1f;
	const uint32_t outer = sp + (GS_IN[i] ? 4u : 0u);
	if (GS_IN[i] && m68k_read_memory_32(outer - 4) != ret)
		return 0;
	const uint32_t oa = a5 + (uint32_t)(int32_t)(int16_t)s_gs.o, ka = a5 + (uint32_t)(int32_t)(int16_t)s_gs.k;
	if (oa - ka + 1 < 3)
		return 0;   // (the two words over each other)
	uint32_t rv = m68k_read_memory_16(oa), kv = m68k_read_memory_16(ka);
	int rem = want, pushed = 0;
	while (rem > 0) {
		switch (i) {
		case 0: pushed = 1; break;   // bsr
		case 1: d0 = (d0 & 0xffff0000u) | rv; cc = (cc & CC_X) | cc_nz16(rv); break;   // move.w (o,a5),d0
		case 2: if (!(d0 & 0xffff)) goto stop; break;   // (bne taken; else the moveq: to Musashi)
		case 3: d0 = (uint32_t)((int32_t)(int16_t)d0 * (int32_t)(int16_t)s_gs.m); cc = (cc & CC_X) | cc_nz32(d0); break;
		case 4: {   // divs.w #q,d0
			const int32_t q = (int32_t)d0 / (int16_t)s_gs.q, r = (int32_t)d0 % (int16_t)s_gs.q;
			if (q != (int16_t)q)
				goto stop;
			d0 = ((uint32_t)q & 0xffff) | ((uint32_t)r << 16);
			cc = (cc & CC_X) | cc_nz16((uint32_t)q);
			break;
		}
		case 5: d0 = (d0 << 16) | (d0 >> 16); cc = (cc & CC_X) | cc_nz32(d0); break;   // swap
		case 6: rv = d0 & 0xffff; cc = (cc & CC_X) | cc_nz16(rv); break;   // move.w d0,(o,a5)
		case 7: break;   // rts
		case 8: kv = addq16(kv, 1, &cc); break;   // addq.w #1,(k,a5)
		case 9: break;   // bra
		}
		rem -= GS_CYC[i];
		i = i == 9 ? 0 : i + 1;
	}
stop:
	if (rem == want)
		return 0;
	if (pushed)
		m68k_write_memory_32(outer - 4, ret);
	m68k_write_memory_16(oa, rv);
	m68k_write_memory_16(ka, kv);
	m68k_set_reg(M68K_REG_D0, d0);
	m68k_set_reg(M68K_REG_A7, outer - (GS_IN[i] ? 4u : 0u));
	m68k_set_reg(M68K_REG_SR, (sr & ~0x1fu) | cc);
	m68k_set_reg(M68K_REG_PC, gs_pc(i));
	return want - rem;
}

// Riding Fight's wait: L: jsr.l R / bsr.w C / bra L; R: the C library random number step (s = s * k + a through a
// 32-bit multiply subroutine M of mulu.w pieces); C: addq.w #1,(o,a5) / cmpi.w #n,(o,a5) / beq out / rts. One pass is
// RF_CYC cycles (Musashi's, the same each time); a pass from L that fits in the slice is done here at once (its
// results: the registers, the stack words below a7 and the two variables), the rest of the slice by Musashi
static struct { uint32_t l, r, k, a; uint16_t s, o, n; } s_rf;
#define RF_CYC 282
static int rf_find(void)
{
	static const uint16_t MUL[34] = { 0x2e2f, 0x0004, 0x2f41, 0x0004, 0x222f, 0x0008, 0x2f40, 0x0008, 0x2f02, 0xcf40,
		0x2400, 0xc4c1, 0x2e00, 0x4847, 0xcec1, 0x4847, 0x4247, 0xde82, 0x4841, 0xc2c0, 0x4841, 0x4241, 0xde81, 0x242f,
		0x0004, 0x2f6f, 0x000c, 0x0008, 0x2f42, 0x000c, 0x241f, 0x201f, 0x221f, 0x4e75 };
	for (uint32_t l = 0x400; l < 0x180000; l += 2) {
		if (m68k_read_memory_16(l) != 0x4eb9 || m68k_read_memory_16(l + 6) != 0x6100 || m68k_read_memory_16(l + 8) != 0x0004 ||
		    m68k_read_memory_16(l + 10) != 0x60f4)
			continue;
		const uint32_t c = l + 12, r = m68k_read_memory_32(l + 2);
		const uint16_t o = (uint16_t)m68k_read_memory_16(c + 2), s = (uint16_t)m68k_read_memory_16(r + 2);
		const uint16_t cw[8] = { 0x526d, o, 0x0c6d, (uint16_t)m68k_read_memory_16(c + 6), o, 0x6702, 0x4e75 };
		const uint16_t rw[13] = { 0x2f2d, s, 0x2f3c, (uint16_t)m68k_read_memory_16(r + 6), (uint16_t)m68k_read_memory_16(r + 8),
			0x4eba, 0x0016, 0x0687, (uint16_t)m68k_read_memory_16(r + 16), (uint16_t)m68k_read_memory_16(r + 18), 0x2b47, s, 0x4e75 };
		if (r >= 0x180000 || !match(c, cw, 7) || !match(r, rw, 13) || !match(r + 0x22, MUL, 34))
			continue;
		s_rf.l = l;
		s_rf.r = r;
		s_rf.s = s;
		s_rf.o = o;
		s_rf.n = (uint16_t)m68k_read_memory_16(c + 6);
		s_rf.k = m68k_read_memory_32(r + 6);
		s_rf.a = m68k_read_memory_32(r + 16);
		return 1;
	}
	return 0;
}

// the wait (s_rf) as idle_run: Musashi up to L (an instruction at a time), then whole passes here
static int idle_run_rf(int want)
{
	uint32_t pc = m68k_get_reg(NULL, M68K_REG_PC);
	const int in_l = pc >= s_rf.l && pc < s_rf.l + 28, in_r = pc >= s_rf.r && pc < s_rf.r + 0x66;
	if (!in_l && !in_r)
		return 0;
	int used = 0;
	for (int k = 0; pc != s_rf.l && k < 40 && used < want; k++) {
		used += m68k_execute(1);
		pc = m68k_get_reg(NULL, M68K_REG_PC);
	}
	if (pc != s_rf.l || want - used < RF_CYC)
		return used;
	const uint32_t a5 = m68k_get_reg(NULL, M68K_REG_A5), sp = m68k_get_reg(NULL, M68K_REG_A7);
	const uint32_t sa = a5 + (uint32_t)(int32_t)(int16_t)s_rf.s, oa = a5 + (uint32_t)(int32_t)(int16_t)s_rf.o;
#define OVER(a, la, b, lb) ((a) < (b) + (lb) && (b) < (a) + (la))
	if (OVER(sa, 4, oa, 2) || OVER(sa, 4, sp - 20, 20) || OVER(oa, 2, sp - 20, 20))
		return used;   // (the variables and the stack words over each other)
#undef OVER
	uint32_t d0 = m68k_get_reg(NULL, M68K_REG_D0), d1 = m68k_get_reg(NULL, M68K_REG_D1);
	const uint32_t d2 = m68k_get_reg(NULL, M68K_REG_D2);
	uint32_t seed = m68k_read_memory_32(sa), w = m68k_read_memory_16(oa), cc = 0;
	int n = 0;
	while (want - used >= RF_CYC && ((w + 1) & 0xffff) != s_rf.n) {
		seed = seed * s_rf.k + s_rf.a;   // (the subroutine's product: the low 32 bits)
		d1 = d0;                         // (its returns: d1 = the d0 before, d0 = its return address)
		d0 = s_rf.r + 14;
		const uint32_t x = w + 1;        // addq.w #1 / cmpi.w #n
		w = x & 0xffff;
		const uint32_t res = (w - s_rf.n) & 0xffff;
		cc = (x > 0xffff ? CC_X : 0) | cc_nz16(res) | ((((s_rf.n ^ w) & (res ^ w)) & 0x8000) ? CC_V : 0) |
			(s_rf.n > w ? CC_C : 0);
		used += RF_CYC;
		n++;
	}
	if (!n)
		return used;
	const uint32_t sr = m68k_get_reg(NULL, M68K_REG_SR);
	m68k_write_memory_32(sp - 4, s_rf.l + 10);    // bsr's return (over jsr's)
	m68k_write_memory_32(sp - 8, s_rf.r + 14);    // (the subroutine's stack words, as it leaves them)
	m68k_write_memory_32(sp - 12, d1);
	m68k_write_memory_32(sp - 16, s_rf.r + 14);
	m68k_write_memory_32(sp - 20, d2);
	m68k_write_memory_32(sa, seed);
	m68k_write_memory_16(oa, w);
	m68k_set_reg(M68K_REG_D0, d0);
	m68k_set_reg(M68K_REG_D1, d1);
	m68k_set_reg(M68K_REG_D7, seed);
	m68k_set_reg(M68K_REG_SR, (sr & ~0x1fu) | cc);
	m68k_set_reg(M68K_REG_PPC, s_rf.l + 10);   // (the last instruction: bra L)
	m68k_set_reg(M68K_REG_IR, 0x60f4);
	return used;
}

// Puzzle Bobble 3's wait: L: jsr.l R / bra.s L; R: jsr.w C / rts; C: move.l (s,a5),d0 / mulu.l #k,d0 / addq.l #q,d0 /
// move.l d0,(s,a5) / swap d0 / rts (a random number stepped until an interrupt changes something). A pass is PB_CYC
// cycles (Musashi's 68020, the same each time); the passes from L that fit in the slice are done here (the seed, d0,
// the flags and the two return addresses left below a7), the rest by Musashi (~9000 of the main CPU's 45000
// instructions a frame in heavy play; most of the light frames)
static struct { uint32_t l, r, c, k; uint16_t s; uint8_t q; } s_pb;
#define PB_CYC 103
static int pb_find(void)
{
	for (uint32_t l = 0x400; l < 0x180000; l += 2) {
		if (m68k_read_memory_16(l) != 0x4eb9 || m68k_read_memory_16(l + 6) != 0x60f8)
			continue;
		const uint32_t r = m68k_read_memory_32(l + 2);
		if (r >= 0x180000 || (r & 1) || m68k_read_memory_16(r) != 0x4eb8 || m68k_read_memory_16(r + 4) != 0x4e75)
			continue;
		const uint32_t c = (uint32_t)(int32_t)(int16_t)m68k_read_memory_16(r + 2);
		if (c >= 0x8000 || (c & 1))
			continue;
		const uint16_t sv = (uint16_t)m68k_read_memory_16(c + 2), aq = (uint16_t)m68k_read_memory_16(c + 12);
		if (m68k_read_memory_16(c) != 0x202d || m68k_read_memory_16(c + 4) != 0x4c3c || m68k_read_memory_16(c + 6) != 0x0800 ||
		    (aq & 0xf1ff) != 0x5080 || m68k_read_memory_16(c + 14) != 0x2b40 || m68k_read_memory_16(c + 16) != sv ||
		    m68k_read_memory_16(c + 18) != 0x4840 || m68k_read_memory_16(c + 20) != 0x4e75)
			continue;
		s_pb.l = l;
		s_pb.r = r;
		s_pb.c = c;
		s_pb.s = sv;
		s_pb.k = m68k_read_memory_32(c + 8);   // (mulu.l #k: 4c3c 0800 then the long)
		s_pb.q = (uint8_t)((aq >> 9) & 7 ? (aq >> 9) & 7 : 8);
		return 1;
	}
	return 0;
}

static int idle_run_pb(int want)
{
	uint32_t pc = m68k_get_reg(NULL, M68K_REG_PC);
	const int in = (pc >= s_pb.l && pc < s_pb.l + 8) || (pc >= s_pb.r && pc < s_pb.r + 6) || (pc >= s_pb.c && pc < s_pb.c + 22);
	if (!in)
		return 0;
	int used = 0;
	for (int k = 0; pc != s_pb.l && k < 12 && used < want; k++) {
		used += m68k_execute(1);
		pc = m68k_get_reg(NULL, M68K_REG_PC);
	}
	if (pc != s_pb.l || want - used < PB_CYC)
		return used;
	const uint32_t a5 = m68k_get_reg(NULL, M68K_REG_A5), sp = m68k_get_reg(NULL, M68K_REG_A7);
	const uint32_t sa = a5 + (uint32_t)(int32_t)(int16_t)s_pb.s;
	if (sa < sp && sp - 8 < sa + 4)
		return used;   // (the seed among the stack words)
	uint32_t seed = m68k_read_memory_32(sa), x = 0;
	while (want - used >= PB_CYC) {
		const uint32_t m = seed * s_pb.k;   // mulu.l #k (the low 32 bits)
		seed = m + s_pb.q;                  // addq.l #q: X = its carry
		x = seed < m;
		used += PB_CYC;
	}
	const uint32_t d0 = seed << 16 | seed >> 16;   // swap d0: N, Z of it, V = C = 0 (X from the addq)
	const uint32_t sr = m68k_get_reg(NULL, M68K_REG_SR);
	m68k_write_memory_32(sp - 4, s_pb.l + 6);    // jsr.l's return
	m68k_write_memory_32(sp - 8, s_pb.r + 4);    // jsr.w's return
	m68k_write_memory_32(sa, seed);
	m68k_set_reg(M68K_REG_D0, d0);
	m68k_set_reg(M68K_REG_SR, (sr & ~0x1fu) | (x ? CC_X : 0) | ((d0 & 0x80000000u) ? CC_N : 0) | (d0 ? 0 : CC_Z));
	m68k_set_reg(M68K_REG_PPC, s_pb.l + 6);   // (the last instruction: bra.s L)
	m68k_set_reg(M68K_REG_IR, 0x60f8);
	return used;
}

// Twin Cobra II's wait: L: move.b #i,$4a0000 (the watchdog) / tst.w f.l / beq L, f in the main RAM. A pass is 20 cycles;
// the passes that fit in the slice are done here (the watchdog kick once: each one sets the same timeout)
static struct { uint32_t l, f; uint8_t i; } s_aw;
#define AW_CYC 20
static int aw_find(void)
{
	for (uint32_t l = 0x400; l < 0x180000; l += 2) {
		if (m68k_read_memory_16(l) != 0x13fc || (m68k_read_memory_16(l + 2) & 0xff00) || m68k_read_memory_32(l + 4) != 0x4a0000 ||
		    m68k_read_memory_16(l + 8) != 0x4a79 || m68k_read_memory_16(l + 14) != 0x67f0)
			continue;
		const uint32_t f = m68k_read_memory_32(l + 10);
		if (f < 0x400000 || f > 0x41fffe || (f & 1))
			continue;
		s_aw.l = l;
		s_aw.f = f;
		s_aw.i = (uint8_t)m68k_read_memory_16(l + 2);
		return 1;
	}
	return 0;
}

static int idle_run_aw(int want)
{
	uint32_t pc = m68k_get_reg(NULL, M68K_REG_PC);
	if (pc != s_aw.l && pc != s_aw.l + 8 && pc != s_aw.l + 14)
		return 0;
	int used = 0;
	for (int k = 0; pc != s_aw.l && k < 3 && used < want; k++) {
		used += m68k_execute(1);
		pc = m68k_get_reg(NULL, M68K_REG_PC);
	}
	if (pc != s_aw.l || want - used < AW_CYC || m68k_read_memory_16(s_aw.f))
		return used;
	used += (want - used) / AW_CYC * AW_CYC;
	m68k_write_memory_8(0x4a0000, s_aw.i);
	const uint32_t sr = m68k_get_reg(NULL, M68K_REG_SR);
	m68k_set_reg(M68K_REG_SR, (sr & ~0x0fu) | CC_Z);   // (tst.w of 0; X kept)
	m68k_set_reg(M68K_REG_PPC, s_aw.l + 14);
	m68k_set_reg(M68K_REG_IR, 0x67f0);
	return used;
}

static void run_main(int cycles)
{
	int want = cycles - s_cpu_debt;
	if (want <= 0) {
		s_cpu_debt = -want;
		return;
	}
	if (s_idle < 0) {
		idle_check();
#ifndef __vita__
		if (getenv("RBX_F3_NOIDLE"))   // (PC harness: Musashi only, to check the idle loops' results)
			s_idle = 0;
#endif
	}
	int used = 0;
	if (s_idle) {
		if ((s_idle & IDLE_BM) && !used)
			used = idle_run(want);
		if ((s_idle & IDLE_TL) && !used)
			used = idle_run_dg(want);
		if ((s_idle & IDLE_FC) && !used)
			used = idle_run_fc(want);
		if ((s_idle & IDLE_IX) && !used)
			used = idle_run_ix(want);
		if ((s_idle & IDLE_DC) && !used)
			used = idle_run_dc(want);
		if ((s_idle & IDLE_GS) && !used)
			used = idle_run_gs(want);
		if ((s_idle & IDLE_WD) && !used)
			used = idle_run_wd(want);
		if ((s_idle & IDLE_KW) && !used)
			used = idle_run_kw(want);
		if ((s_idle & IDLE_RF) && !used)
			used = idle_run_rf(want);
		if ((s_idle & IDLE_AW) && !used)
			used = idle_run_aw(want);
		if ((s_idle & IDLE_PB) && !used)
			used = idle_run_pb(want);
	}
#ifndef __vita__
	{	// (PC harness: F3STEP=pc prints the cycles Musashi gives each instruction from there, for the idle loops above)
		static int n;
		const char *st = getenv("F3STEP");
		if (st && n < 2 && m68k_get_reg(NULL, M68K_REG_PC) == strtoul(st, NULL, 16)) {
			n++;
			for (int k = 0; k < 45; k++) {
				uint32_t pc = m68k_get_reg(NULL, M68K_REG_PC), sr = m68k_get_reg(NULL, M68K_REG_SR), d0 = m68k_get_reg(NULL, M68K_REG_D0);
				int cyc = m68k_execute(1);
				fprintf(stderr, "S %06x %d sr=%04x d0=%08x\n", pc, cyc, sr, d0);
			}
		}
	}
#endif
	if (used < want)
		used += m68k_execute(want - used);
	s_cpu_debt = used - want;
#ifdef RBX_F3_PCPROF
	{	// (device profiling: slice-end PC samples of the main CPU, the top ones logged every 600 frames)
		static uint32_t pcs[1024], cnt[1024];
		static unsigned n;
		const uint32_t pc = m68k_get_reg(NULL, M68K_REG_PC) & ~0xfu;
		const unsigned h = (pc >> 4) * 2654435761u >> 22;
		unsigned k = h, probes = 0;
		while (cnt[k] && pcs[k] != pc && ++probes < 1024)
			k = (k + 1) & 1023;
		if (probes < 1024) {   // (a full table drops the sample)
			pcs[k] = pc;
			cnt[k]++;
		}
		if (++n == 600 * 262) {
			char buf[256];
			int len = 0;
			unsigned tot = n;
			for (int t = 0; t < 8; t++) {
				unsigned best = 0, bi = 0;
				for (unsigned j = 0; j < 1024; j++)
					if (cnt[j] > best) { best = cnt[j]; bi = j; }
				if (!best)
					break;
				len += snprintf(buf + len, sizeof(buf) - (size_t)len, " %06x:%u%%", (unsigned)pcs[bi], best * 100 / tot);
				cnt[bi] = 0;
			}
			rbx_log("pcprof:%s", buf);
			for (unsigned j = 0; j < 1024; j++) cnt[j] = 0;
			n = 0;
		}
	}
#endif
}

static void f3_frame(void)
{
	int vblank = s_m->vis_y1 + 1;
	for (int i = 0; i < 2; i++)
		s_dial[i] += rbx_analog_dpad(i + 1, RBX_IPT_DIAL, 6);   // (d-pad: key delta 25 x sensitivity 25%)
	rbx_audio.frame_mhz = FRAME_MHZ;   // (58.94 Hz: its sound is played in 60 Hz frames, as its picture)
	ten_frame_begin();
	for (int line = 0; line < LINES; line++) {
		if (line == vblank) {
			vblank_capture();
			m68k_bus_hold_irq(2);
			s_irq3 = 10000;   // MAME: interrupt3 timer, 10000 main CPU cycles after vblank
		}
		int c = line_cycles(&s_cpu_frac, MAIN_CLOCK);
		if (s_irq3 >= 0 && s_irq3 <= c) {
			run_main(s_irq3);
			m68k_bus_hold_irq(3);
			run_main(c - s_irq3);
			s_irq3 = -1;
		} else {
			if (s_irq3 >= 0)
				s_irq3 -= c;
			run_main(c);
		}
		ten_line();
	}
	ten_frame_end();
	if (s_watchdog && --s_watchdog == 0)
		machine_reset(0);   // MAME: watchdog_fired -> soft reset
}

static int f3_state(void *ctx, int load)
{
	uint8_t cpu[1024];
	uint32_t csize = m68k_context_size();
	if (csize > sizeof(cpu))
		return -1;
	if (!load)
		m68k_get_context(cpu);
	uint32_t irq = m68k_bus_irq_pending();
	int rc = 0;
	rc |= rbx_state_io(ctx, "m68k", cpu, csize);
	rc |= rbx_state_io(ctx, "irq", &irq, 4);
	rc |= rbx_state_io(ctx, "ram", s_ram, sizeof(s_ram));
	rc |= rbx_state_io(ctx, "pal", s_pal, sizeof(s_pal));
	rc |= rbx_state_io(ctx, "spr", s_spr, sizeof(s_spr));
	rc |= rbx_state_io(ctx, "vram", s_vram, sizeof(s_vram));
	rc |= rbx_state_io(ctx, "line", s_line, sizeof(s_line));
	rc |= rbx_state_io(ctx, "pivot", s_pivot, sizeof(s_pivot));
	rc |= rbx_state_io(ctx, "ctrl0", s_ctrl0, sizeof(s_ctrl0));
	rc |= rbx_state_io(ctx, "ctrl1", s_ctrl1, sizeof(s_ctrl1));
	rc |= rbx_state_io(ctx, "coin", s_coin_word, sizeof(s_coin_word));
	rc |= rbx_state_io(ctx, "dial", s_dial, sizeof(s_dial));
	rc |= rbx_state_io(ctx, "eeprom", &s_eeprom, sizeof(s_eeprom));
	rc |= rbx_state_io(ctx, "frac", &s_cpu_frac, sizeof(s_cpu_frac));
	rc |= rbx_state_io(ctx, "debt", &s_cpu_debt, sizeof(s_cpu_debt));
	rc |= rbx_state_io(ctx, "irq3", &s_irq3, sizeof(s_irq3));
	rc |= rbx_state_io(ctx, "watchdog", &s_watchdog, sizeof(s_watchdog));
	rc |= ten_state(ctx, load);
	if (rc)
		return -1;
	if (load) {
		m68k_set_context(cpu);
		m68k_bus_set_irq_pending(irq);
	}
	return 0;
}

static int f3_save(void *ctx) { return f3_state(ctx, 0); }
static int f3_load(void *ctx) { return f3_state(ctx, 1); }
static int f3_supported(const char *machine) { return find_machine(machine) != NULL; }

const rbx_driver rbx_drv_taito_f3 = { "Taito F3 System", f3_init, f3_exit, f3_reset, f3_frame, f3_save, f3_load, f3_supported, f3_render, f3v_help };
