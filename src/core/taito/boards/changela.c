/*  Change Lanes hardware, ported from MAME src/mame/taito/changela.cpp (GPL-2.0+, copyright-holders: Jarek
    Burczynski, Phil Stroffolino, Tomasz Slanina, Adam Bousley).
    Z80 (5 MHz), MC68705P3 MCU (cpu/m6805), 2x AY-3-8910 (1.25 MHz). The video is MAME's scanline model: every line the
    river, sprite (OBJ0) and two tree generators are drawn into line bitmaps by the slope ROM / PROM state machines and
    the collision latches are updated from them (the game reads them back); the text layer (OBJ1) is drawn at the end
    of the frame. The gas pedal (MAME IPT_PEDAL 0-2) is button 1 (full throttle), the gear shift toggles on button 2,
    the steering wheel (IPT_DIAL) follows the analog stick or left / right.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../rbx_core.h"
#include "../../cpu/z80/z80.h"
#include "../../cpu/m6805/m68705.h"
#include "../../sound/ay8910.h"

#include <stdlib.h>
#include <string.h>

// MAME set_raw(20 MHz / 4, 318, 8, 256, 262, 32, 256)
#define LINES 262
#define HTOTAL 318
#define VIS_X0 8
#define VIS_Y0 32
#define VIS_W 248
#define VIS_H 224
#define MCU_CLOCK 625000    // 2.5 MHz / 4
#define MAIN_CLOCK 5000000
#define SLICES 2

typedef struct {
	z80 z;
	int irq;
	uint8_t vector;
} zcpu;

static uint8_t *s_rom, *s_tilerom, *s_obj0rom, *s_obj1rom, *s_sloperom, *s_treerom, *s_prom;
static uint8_t s_spriteram[0x400], s_videoram[0x800], s_ram[0x800];
// obj0's sprites of each line: a bit per sprite (drawn in sprite order), kept as a sprite's y / height bytes are
// written (obj_set); all again after a reset or a load (s_obj_dirty)
static uint32_t s_obj_mask[256][8];
static int s_obj_dirty = 1;

// sprite k on (or off) its lines: (uint32_t)(sy - ypos) <= vsize, sy in ypos..ypos+vsize of 0..255
static void obj_set(int k, int on)
{
	const uint8_t *s = s_spriteram + k * 4;
	const int vsize = s[0] & 0x1f, ypos = (uint8_t)(~s[1] + 1);
	const uint32_t bit = 1u << (k & 31);
	for (int y = ypos; y <= ypos + vsize && y < 256; y++) {
		if (on)
			s_obj_mask[y][k >> 5] |= bit;
		else
			s_obj_mask[y][k >> 5] &= ~bit;
	}
}
static uint8_t s_riverram[0x800], s_treeram[0x800], s_treeram2[0x40], s_stateram[0xc0];
static uint32_t s_palette[64];
static zcpu s_main;
static m68705_t s_mcu;
static int s_line;
static uint64_t s_frac_mcu;
// video state
static uint8_t s_mem_dev_selected, s_tree_en, s_horizon, s_v_count_river, s_v_count_tree, s_tree_prio, s_tree_on[2];
static uint32_t s_sloperom_bank;
static uint8_t *s_bm_river, *s_bm_obj0, *s_bm_tree[2];   // 256 x 256 line bitmaps
// (two sets of them: the frame's lines are drawn into one while the render thread composes the last frame's set)
static uint8_t *s_bm_mem, *s_snap_bm;
static int s_bm_set, s_snap_tree_prio;

static void bm_point(void)
{
	s_bm_river = s_bm_mem + s_bm_set * 4 * 0x10000;
	s_bm_obj0 = s_bm_river + 0x10000;
	s_bm_tree[0] = s_bm_river + 0x20000;
	s_bm_tree[1] = s_bm_river + 0x30000;
}
// mcu
static uint8_t s_port_a_out, s_port_c_out, s_mcu_out, s_mcu_in;
// misc
static uint8_t s_tree0_col, s_tree1_col, s_left_bank_col, s_right_bank_col, s_boat_shore_col;
static uint8_t s_collision_reset, s_tree_collision_reset, s_prev_value_31, s_dir_31, s_outlatch;
static int s_wheel, s_gear, s_gear_prev;
// snapshot
static uint8_t s_snap_vram[0x800];
static uint32_t s_snap_pal[64];
// ports
static int s_p_in0, s_p_in1, s_p_mcu, s_p_dsw[4], s_p_gear;
// sound
static ay8910_t s_ay[2];
static int32_t *s_mix;
static int s_mix_cap, s_mix_n;
static uint32_t s_snd_done;
static unsigned long s_frame_start;

static uint8_t port8(int p) { return (uint8_t)rbx_port_read(p); }

// ---- sound ----
static void snd_sync(void)
{
	uint32_t pos = (uint32_t)((s_main.z.cyc - s_frame_start) / 4);   // AY clock = CPU clock / 4
	if (pos <= s_snd_done)
		return;
	int n = ay8910_render(&s_ay[0], pos - s_snd_done, s_mix + s_mix_n, s_mix_cap - s_mix_n);
	ay8910_render(&s_ay[1], pos - s_snd_done, s_mix + s_mix_n, s_mix_cap - s_mix_n);
	s_mix_n += n;
	s_snd_done = pos;
}
static uint8_t ay_port_a(void *u) { return port8(s_p_dsw[(intptr_t)u * 2]); }
static uint8_t ay_port_b(void *u) { return port8(s_p_dsw[(intptr_t)u * 2 + 1]); }

// ---- river / tree / sprite generators (MAME changela_scanline) ----
// the 32 PROM steps of a line's slope / tree state machine (the PROM is fixed): what each step does, worked out once
// (st_build); the math train is a ring (a shift: one copy) of which MT(j) is math_train[j]
enum { ST_RAMINC = 1, ST_ROMINC = 2, ST_CARRY = 4, ST_CARRYCLR = 8, ST_HOSC = 16, ST_SHIFTM = 32 };
static struct { uint8_t cur, f, cat; } s_st[0x20];   // cat: 1 pre=ROM, 2 RAM=ROM, 3 pre=RAM, 4 RAM=sum, 0 none

static void st_build(void)
{
	uint8_t prev = 0;
	for (int i = 0; i < 0x20; i++) {
		const uint8_t c = s_prom[i];
		uint8_t f = 0, cat = 0;
		if (prev & 0x80) f |= ST_RAMINC;
		if ((c & 0x40) && !(prev & 0x40)) f |= ST_ROMINC;
		if (prev & 0x02) f |= ST_CARRY;
		if (!(c & 0x08)) f |= ST_CARRYCLR;
		if (prev & 0x10) f |= ST_HOSC;
		if (c & 0x02) f |= ST_SHIFTM;
		switch (c) {
		case 0x01: case 0x09: case 0x19: case 0x0d: case 0x8d: cat = 1; break;
		case 0x0f: case 0x2f: cat = 2; break;
		case 0x4d: case 0x69: case 0x6d: case 0xc5: case 0xcd: cat = 3; break;
		case 0xea: case 0xee: cat = 4; break;
		default: break;
		}
		s_st[i].cur = c;
		s_st[i].f = f;
		s_st[i].cat = cat;
		prev = c;
	}
}
#define MT(j) mt[(mh - (unsigned)(j)) & 15]

static void draw_river(int sy)
{
	const uint8_t *ROM = s_sloperom, *TILE_ROM = s_tilerom, *TILE_RAM = s_riverram;
	uint8_t *RAM = s_stateram;
	int preload = sy < 32;
	uint8_t math_train[10], pre_train[3] = { 0 };
	uint8_t ram_count = 0, rom_count = 0;
	int hosc = 0, carry = 0;
	if (sy == 32)
		s_v_count_river = s_horizon;
	s_v_count_river = (uint8_t)(s_v_count_river + 1);
	uint8_t mt[16] = { 0 };
	unsigned mh = 0;
	const int mux61 = s_v_count_river & 0x01;
	const int rom_hi = (int)s_sloperom_bank | ((s_v_count_river & 0x7e) << 2);
	for (int i = 0; i < 0x20; i++) {
		const uint8_t curr_state = s_st[i].cur, f = s_st[i].f;
		if (f & ST_RAMINC)
			ram_count = (ram_count + 1) & 0x0f;
		if (f & ST_ROMINC)
			rom_count = (rom_count + 1) & 0x0f;
		if (f & ST_CARRY)
			carry = (pre_train[1] + pre_train[2] + carry) > 0x0f;
		if (f & ST_CARRYCLR)
			carry = 0;
		if (f & ST_HOSC)
			hosc = (MT(8) << 4) | MT(9);
		if (s_st[i].cat) {
			const int rom_addr = rom_hi | ((rom_count & 0x0e) >> 1);
			const int ram_a5 = ((curr_state & 0x01) & ((curr_state & 0x40) >> 6) & preload) ^ 0x01;
			const int ram_addr = (ram_a5 << 5) | (ram_count << 1) | ((curr_state & 0x20) >> 5);
			const int mux45 = rom_count & 0x01;
			switch (s_st[i].cat) {
			case 1:
				pre_train[0] = mux45 ? (ROM[rom_addr] & 0xf0) >> 4 : ROM[rom_addr] & 0x0f;
				break;
			case 2:
				MT(0) = RAM[ram_addr] = mux45 ? (ROM[rom_addr] & 0xf0) >> 4 : ROM[rom_addr] & 0x0f;
				break;
			case 3:
				pre_train[0] = RAM[ram_addr] & 0x0f;
				break;
			default:
				MT(0) = RAM[ram_addr] = mux61 ? pre_train[1] : (uint8_t)((pre_train[1] + pre_train[2] + carry) & 0x0f);
				break;
			}
		}
		if (f & ST_SHIFTM) {
			mt[(mh + 1) & 15] = mt[mh & 15];
			mh++;
		} else {
			pre_train[2] = pre_train[1];
			pre_train[1] = pre_train[0];
		}
	}
	for (int j = 0; j < 10; j++)
		math_train[j] = MT(j);
	if (!(s_v_count_river & 0x80)) {
		uint8_t *line = s_bm_river + sy * 256;
		int h_count = 0x80 | (hosc >> 1);
		int tile_v = ((math_train[3] & 0x0c) >> 2) | ((math_train[2] & 0x0f) << 2) | ((math_train[1] & 0x07) << 6);
		int tile_h = ((math_train[7] & 0x0f) | ((math_train[6] & 0x0f) << 4) | ((math_train[5] & 0x01) << 8)) + 4;
		int t_h = -1, t_col = 0;
		for (int sx = 0; sx < 256; sx++) {
			const int steps = sx < 16 ? 1 : 4;
			if (h_count + steps <= 0xff)   // (no wrap in the steps: one add)
				h_count += steps;
			else
				for (int i = 0; i < steps; i++) {
					if (h_count > 0xff) {
						h_count = ((math_train[9] & 0x0f) >> 1) | ((math_train[8] & 0x0f) << 3) | 0x80;
						tile_h = (tile_h + 1) & 0xfff;
						if ((math_train[9] & 0x01) && (tile_h & 0x01))
							h_count--;
					} else
						h_count++;
				}
			if (tile_h != t_h) {   // (the tile's pixel: tile_h changes only at an h counter wrap)
				t_h = tile_h;
				const int ram_addr = ((tile_h & 0x1f8) >> 3) | ((tile_v & 0x1f0) << 2);
				const uint8_t d = TILE_ROM[((tile_h & 0x06) >> 1) | ((tile_v & 0x0f) << 2) | ((TILE_RAM[ram_addr & 0x7ff] & 0x7f) << 6)];
				t_col = (tile_h & 0x01) ? d & 0x0f : (d & 0xf0) >> 4;
			}
			line[sx] = (uint8_t)t_col;
			if (sx >= 15) {   // (the next pixels before a wrap: the same tile, the same pixel)
				int k = (0xff - h_count) / 4;
				if (k > 255 - sx)
					k = 255 - sx;
				if (k > 0) {
					h_count += 4 * k;
					memset(line + sx + 1, t_col, (size_t)k);
					sx += k;
				}
			}
		}
	}
	if (s_v_count_river == 0x7f)
		s_tree_prio = (math_train[2] >> 2) & 1;
}

static void draw_tree(int sy, int tree_num)
{
	const uint8_t *ROM = s_sloperom;
	uint8_t *RAM = s_stateram + 0x40 + 0x40 * tree_num;
	uint8_t *RAM2 = s_treeram2 + 0x20 * tree_num;
	const uint8_t *TILE_ROM = tree_num ? s_treerom + 0x1000 : s_tilerom + 0x2000;
	const uint8_t *TILE_RAM = tree_num ? s_treerom : s_treeram;
	int preload = sy < 32;
	uint8_t math_train[10], pre_train[3] = { 0 }, tree_train[3] = { 0 };
	uint8_t ram_count = 0, rom_count = 0;
	int hosc = 0, carry = 0, tree_carry = 0;
	if (sy == 32) {
		s_tree_on[tree_num] = 0;
		if (tree_num == 0)
			s_v_count_tree = s_horizon;
	}
	if (tree_num == 0)
		s_v_count_tree = (uint8_t)(s_v_count_tree + 1);
	uint8_t mt[16] = { 0 };
	unsigned mh = 0;
	const int mux61 = s_v_count_tree & 0x01;
	const int rom_hi = (int)s_sloperom_bank | ((s_v_count_tree & 0x7e) << 2);
	const int on = s_tree_on[tree_num];   // (not changed within the steps)
	for (int i = 0; i < 0x20; i++) {
		const uint8_t curr_state = s_st[i].cur, f = s_st[i].f;
		if (f & ST_RAMINC)
			ram_count = (ram_count + 1) & 0x0f;
		if (f & ST_ROMINC)
			rom_count = (rom_count + 1) & 0x0f;
		if (f & ST_CARRY) {
			carry = (pre_train[1] + pre_train[2] + carry) > 0x0f;
			tree_carry = (tree_train[1] + tree_train[2] + tree_carry) > 0x0f;
		}
		if (f & ST_CARRYCLR)
			carry = tree_carry = 0;
		if (f & ST_HOSC)
			hosc = (MT(8) << 4) | MT(9);
		const int cat = s_st[i].cat;
		if (cat) {
			const int rom_addr = rom_hi | ((rom_count & 0x0e) >> 1);
			const int ram_a5 = ((curr_state & 0x01) & ((curr_state & 0x40) >> 6) & preload) ^ 0x01;
			const int ram_addr = (ram_a5 << 5) | (ram_count << 1) | ((curr_state & 0x20) >> 5);
			const int ram2_addr = (ram_count << 1) | ((curr_state & 0x20) >> 5);
			const int mux45 = rom_count & 0x01;
			switch (cat) {
			case 1:
				pre_train[0] = mux45 ? (ROM[rom_addr] & 0xf0) >> 4 : ROM[rom_addr] & 0x0f;
				break;
			case 2:
				RAM[ram_addr] = mux45 ? (ROM[rom_addr] & 0xf0) >> 4 : ROM[rom_addr] & 0x0f;
				break;
			case 3:
				pre_train[0] = RAM[ram_addr] & 0x0f;
				break;
			default:
				RAM[ram_addr] = mux61 ? pre_train[1] : (uint8_t)((pre_train[1] + pre_train[2] + carry) & 0x0f);
				break;
			}
			const int mux82 = !on ? (s_v_count_tree & 0x01) ^ 0x01 : ((curr_state & 0x04) ? 0 : 1);
			if (cat == 1 || cat == 3)
				tree_train[0] = !on ? (RAM2[ram2_addr] = pre_train[0]) : RAM2[ram2_addr];
			else if (cat == 2)
				MT(0) = !on ? (RAM2[ram2_addr] = RAM[ram_addr] & 0x0f) : RAM2[ram2_addr];
			else
				MT(0) = RAM2[ram2_addr] = mux82 ? (uint8_t)((tree_train[1] + tree_train[2] + tree_carry) & 0x0f) : tree_train[1];
		}
		if (f & ST_SHIFTM) {
			mt[(mh + 1) & 15] = mt[mh & 15];
			mh++;
		} else {
			pre_train[2] = pre_train[1];
			pre_train[1] = pre_train[0];
			tree_train[2] = tree_train[1];
			tree_train[1] = tree_train[0];
		}
	}
	for (int j = 0; j < 10; j++)
		math_train[j] = MT(j);
	uint8_t *line = s_bm_tree[tree_num] + sy * 256;
	int h_count = 0x80 | (hosc >> 1);
	int tile_v = ((math_train[3] & 0x0c) >> 2) | ((math_train[2] & 0x0f) << 2) | ((math_train[1] & 0x07) << 6);
	int tile_h = ((math_train[7] & 0x0f) | ((math_train[6] & 0x0f) << 4) | ((math_train[5] & 0x01) << 8)) + 4;
	int all_ff = 1;
	const int enabled = !(s_v_count_tree & 0x80) && (s_tree_en & (1 << tree_num));
	if (!enabled && !s_tree_on[tree_num])   // (it cannot come on: the line is left as it is, all_ff, off)
		return;
	int t_h = -1, t_on = 0, t_col = 0;
	const int h_reset = ((math_train[9] & 0x0f) >> 1) | ((math_train[8] & 0x0f) << 3) | 0x80, h_dec = math_train[9] & 0x01;
	for (int sx = 0; sx < 256; sx++) {
		const int steps = sx < 16 ? 1 : 4;
		if (h_count + steps <= 0xff) {   // (no wrap in the steps: one add)
			h_count += steps;
			goto step_done;
		}
		for (int st = steps; st; ) {   // (the steps up to the wrap at once: the counter only counts up to it)
			if (h_count > 0xff) {
				h_count = h_reset;
				tile_h = (tile_h + 1) & 0xfff;
				if (h_dec && (tile_h & 0x01))
					h_count--;
				st--;
			} else {
				int k = 0x100 - h_count;
				if (k > st)
					k = st;
				h_count += k;
				st -= k;
			}
		}
	step_done:
		if (tile_h != t_h) {   // (the tile's values: tile_h changes only at an h counter wrap, every few pixels)
			t_h = tile_h;
			const int ram_addr = ((tile_h & 0x1f8) >> 3) | ((tile_v & 0x1f0) << 2);
			const uint8_t d = TILE_ROM[((tile_h & 0x06) >> 1) | ((tile_v & 0x0f) << 2) | ((TILE_RAM[ram_addr & 0x7ff] & 0x7f) << 6)];
			t_on = (d & 0xf0) == 0;
			t_col = (tile_h & 0x01) ? d & 0x0f : (d & 0xf0) >> 4;
		}
		if (enabled && t_on)
			s_tree_on[tree_num] = 1;
		if (s_tree_on[tree_num]) {
			const int col = t_col;
			if (col != 0x0f)
				all_ff = 0;
			if (col != 0x0f && col != 0x00)
				line[sx] = (uint8_t)(col | 0x30);
		}
		if (sx >= 15) {   // (the next pixels before a wrap: 4 steps each, the same tile, the same pixel and state)
			int k = (0xff - h_count) / 4;
			if (k > 255 - sx)
				k = 255 - sx;
			if (k > 0) {
				h_count += 4 * k;
				if (s_tree_on[tree_num] && t_col != 0x0f && t_col != 0x00)
					memset(line + sx + 1, t_col | 0x30, (size_t)k);
				sx += k;
			}
		}
	}
	// the tree stays on only if a pixel other than 0xf is met (any such pixel sets U56)
	if (all_ff)
		s_tree_on[tree_num] = 0;
}

static void obj_lines(void)
{
	memset(s_obj_mask, 0, sizeof(s_obj_mask));
	for (int k = 0; k < 256; k++)
		obj_set(k, 1);
	s_obj_dirty = 0;
}

static void draw_obj0(int sy)
{
	const uint8_t *ROM = s_obj0rom, *RAM = s_spriteram;
	uint8_t *line = s_bm_obj0 + sy * 256;
	if (s_obj_dirty)
		obj_lines();
	for (int w = 0; w < 8; w++)
	for (uint32_t m = s_obj_mask[sy][w]; m; m &= m - 1) {
		const int sx = w * 32 + __builtin_ctz(m);
		const uint8_t *s = RAM + sx * 4;
		const uint32_t vsize = s[0] & 0x1f;
		const uint8_t ypos = (uint8_t)(~s[1] + 1);
		if ((uint32_t)(sy - ypos) > vsize)   // MAME compares int with u32: negative differences fail too
			continue;
		const int vr = (s[0] & 0x80) >> 7, hr = (s[0] & 0x40) >> 6, hs = (s[0] & 0x20) >> 5;
		const uint8_t tile = s[2], xpos = s[3];
		const uint8_t sum = (uint8_t)(sy - ypos);
		// (what does not change over the 16 pixels: the address but the counter's bits)
		const uint32_t A8 = ((tile & 0x02) >> 1) ^ ((hr & hs) ^ hs);
		const uint32_t A7 = ((((vr ^ ((sum & 0x10) >> 4)) & ((vsize & 0x10) >> 4)) ^ 0x01) & (tile & 0x01)) ^ 0x01;
		uint32_t base = ((sum & 0x0f) << 3) | (A7 << 7) | (A8 << 8) | ((uint32_t)(tile >> 2) << 9);
		if (vr)
			base ^= (0x0f << 3);
		// (pixels i = 2j, 2j + 1 share a ROM byte: counter >> 1 = j ^ (hr ? 7 : 0); the even pixel takes the high
		// nibble, the odd one the low, swapped when hr. The two halves (hs) do not overlap: the order is free)
		const int lim = 256 - xpos;
		for (int j = 0; j < 8; j++) {
			const uint32_t rom_addr = (base | (uint32_t)(j ^ (hr ? 7 : 0))) & 0x7fff;
			const int i0 = 2 * j;
			uint8_t bt = ROM[rom_addr];
			uint8_t de = hr ? bt & 0x0f : bt >> 4, dd = hr ? bt >> 4 : bt & 0x0f;
			if (de != 0x0f && de != 0 && i0 < lim)
				line[xpos + i0] = de | 0x10;
			if (dd != 0x0f && dd != 0 && i0 + 1 < lim)
				line[xpos + i0 + 1] = dd | 0x10;
			if (hs) {
				bt = ROM[rom_addr ^ 0x100];
				de = hr ? bt & 0x0f : bt >> 4;
				dd = hr ? bt >> 4 : bt & 0x0f;
				if (de != 0x0f && de != 0 && i0 + 16 < lim)
					line[xpos + i0 + 16] = de | 0x10;
				if (dd != 0x0f && dd != 0 && i0 + 17 < lim)
					line[xpos + i0 + 17] = dd | 0x10;
			}
		}
	}
}

static void scanline(int sy)
{
	if (sy == 256) {
		s_main.irq = 1;
		s_main.vector = 0xdf;
	} else if ((sy % 64) == 0) {
		s_main.irq = 1;
		s_main.vector = 0xcf;
	}
	if (sy < 256) {
		memset(s_bm_river + sy * 256, 0, 256);
		memset(s_bm_obj0 + sy * 256, 0, 256);
		memset(s_bm_tree[0] + sy * 256, 0, 256);
		memset(s_bm_tree[1] + sy * 256, 0, 256);
		draw_river(sy);
		draw_obj0(sy);
		draw_tree(sy, 0);
		draw_tree(sy, 1);
	}
	if (sy >= 32 && sy < 256) {
		const uint8_t *riv = s_bm_river + sy * 256, *obj = s_bm_obj0 + sy * 256;
		const uint8_t *t0 = s_bm_tree[0] + sy * 256, *t1 = s_bm_tree[1] + sy * 256;
		for (int sx = 1; sx < 256; sx++) {
			if (obj[sx] != 0x14)   // car outline color
				continue;
			int riv_col = riv[sx] >= 0x08 && riv[sx] <= 0x0a;
			int prev_col = riv[sx - 1] >= 0x08 && riv[sx - 1] <= 0x0a;
			if (t0[sx]) s_tree0_col = 1;
			if (t1[sx]) s_tree1_col = 1;
			if (!riv_col && prev_col) s_right_bank_col = 1;
			if (riv_col && !prev_col) s_left_bank_col = 1;
			if (riv_col) s_boat_shore_col = 1;
		}
		if (!s_tree_collision_reset)
			s_tree0_col = s_tree1_col = 0;
		if (!s_collision_reset)
			s_left_bank_col = s_right_bank_col = s_boat_shore_col = 0;
	}
}

// ---- I/O ----
static void colors_w(int offset, uint8_t data)
{
	static const uint8_t color_table[8] = { 0, 7, 18, 31, 58, 88, 146, 255 };
	uint32_t c = (data | ((offset & 1) << 8)) ^ 0x1ff;   // a0 is the 9th data bit, active low
	int idx = (offset >> 1) ^ 0x30;                       // A4 and A5 negated
	s_palette[idx & 63] = (uint32_t)color_table[c & 7] << 16 | (uint32_t)color_table[(c >> 3) & 7] << 8 | color_table[(c >> 6) & 7];
}

static void mem_device_w(int offset, uint8_t data)
{
	switch (s_mem_dev_selected) {
	case 1:
		for (int i = 0; i < 3; i++)
			s_stateram[i * 0x40 + (offset & 0x3f)] = data & 0x0f;
		break;
	case 2: s_riverram[offset] = data; break;
	case 3: s_treeram[offset] = data; break;
	}
}

static uint8_t mem_device_r(int offset)
{
	switch (s_mem_dev_selected) {
	case 1: return s_stateram[offset & 0x3f];
	case 2: return s_riverram[offset];
	case 3: return s_treeram[offset];
	case 4: return s_treerom[offset];
	case 5: return s_sloperom[offset];
	}
	return 0;
}

static void mcu_port_c_w(void *u, int n, uint8_t data, uint8_t mask)
{
	(void)u; (void)n; (void)mask;
	// PC3 clocks the LS374 (rising edge), PC2 is its /OE
	if ((data & 8) && !(s_port_c_out & 8))
		s_mcu_out = s_port_a_out;
	m68705_port_input(&s_mcu, 0, (data & 4) ? 0xff : s_mcu_in);
	s_port_c_out = data;
}
static void mcu_port_a_w(void *u, int n, uint8_t data, uint8_t mask) { (void)u; (void)n; (void)mask; s_port_a_out = data; }
static uint8_t mcu_port_b_r(void *u, int n, uint8_t mask) { (void)u; (void)n; (void)mask; return port8(s_p_mcu); }

static uint8_t wheel_r(void) { return (uint8_t)s_wheel; }

static uint8_t main_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x8000 || (a >= 0xb000 && a < 0xc000)) return s_rom[a];
	if (a < 0x8400) return s_spriteram[a - 0x8000];
	if (a >= 0x9000 && a < 0x9800) return s_videoram[a - 0x9000];
	if (a >= 0xc000 && a < 0xc800) return mem_device_r(a - 0xc000);
	if (a == 0xd000 || a == 0xd001 || a == 0xd010 || a == 0xd011) { snd_sync(); return ay8910_data_r(&s_ay[(a >> 4) & 1]); }
	switch (a) {
	case 0xd024: return (uint8_t)((((s_port_c_out >> 1) & 1) << 3) | 0x07);
	case 0xd025: return (uint8_t)((s_tree1_col << 3) | (s_tree0_col << 2) | 0x03);
	case 0xd028: return s_mcu_out & ((s_port_c_out & 4) ? 0xff : s_mcu_in);
	case 0xd02c: {
		// GEAR conditions: bit 6 (FWD) when GEAR = 0, bit 7 (REV) when GEAR = 1
		uint8_t v = port8(s_p_in0) & 0x3f;
		return (uint8_t)(v | (s_gear ? 0x80 : 0x40));
	}
	case 0xd02d: {
		static const uint8_t lut_gas[3] = { 1, 0, 2 };
		int gas = (rbx_input_bits(1) & RBX_IN_B1) ? 2 : 0;
		return (uint8_t)((port8(s_p_in1) & 0x20) | (lut_gas[gas] << 6) | (((s_line >> 8) & 1) << 4));
	}
	case 0xd030: return wheel_r() & 0x0f;
	case 0xd031: {
		uint8_t cur = wheel_r(), prev = s_prev_value_31;
		if ((cur < prev && (prev - cur) < 0x80) || (cur > prev && (cur - prev) > 0x80))
			s_dir_31 = 1;
		if ((prev < cur && (cur - prev) < 0x80) || (prev > cur && (prev - cur) > 0x80))
			s_dir_31 = 0;
		s_prev_value_31 = cur;
		return (uint8_t)((s_dir_31 << 3) | (s_left_bank_col << 2) | (s_right_bank_col << 1) | s_boat_shore_col);
	}
	}
	if (a >= 0xf000 && a < 0xf800) return s_ram[a - 0xf000];
	return 0;
}

static void outlatch_w(int bit, int state)
{
	int old = (s_outlatch >> bit) & 1;
	s_outlatch = (uint8_t)((s_outlatch & ~(1 << bit)) | (state << bit));
	if (old == state)
		return;
	switch (bit) {
	case 0: s_collision_reset = (uint8_t)state; break;
	case 4: m68705_port_input(&s_mcu, 2, (uint8_t)(0xfe | state)); break;
	case 5: s_tree_collision_reset = (uint8_t)state; break;
	}
}

static void main_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0x8000 && a < 0x8400) {
		const int o = a - 0x8000;
		if ((o & 3) < 2 && s_spriteram[o] != v && !s_obj_dirty) {   // (a sprite's height / y: its lines)
			obj_set(o >> 2, 0);
			s_spriteram[o] = v;
			obj_set(o >> 2, 1);
		} else
			s_spriteram[o] = v;
		return;
	}
	if (a >= 0x9000 && a < 0x9800) { s_videoram[a - 0x9000] = v; return; }
	if (a >= 0xa000 && a < 0xa080) { colors_w(a - 0xa000, v); return; }
	if (a >= 0xc000 && a < 0xc800) { mem_device_w(a - 0xc000, v); return; }
	switch (a) {
	case 0xc900: s_mem_dev_selected = v & 7; s_tree_en = (v & 0x30) >> 4; return;
	case 0xca00: s_sloperom_bank = (uint32_t)(v & 3) << 9; return;
	case 0xcb00: s_horizon = v; return;
	case 0xd000: case 0xd001: case 0xd010: case 0xd011:
		snd_sync();
		ay8910_write(&s_ay[(a >> 4) & 1], a & 1, v);
		return;
	case 0xd030:   // LS374 at U39
		s_mcu_in = v;
		if (!(s_port_c_out & 4))
			m68705_port_input(&s_mcu, 0, v);
		return;
	}
	if (a >= 0xd020 && a <= 0xd027) { outlatch_w(a & 7, v & 1); return; }
	if (a >= 0xf000 && a < 0xf800) { s_ram[a - 0xf000] = v; return; }
}

static uint8_t no_in(z80 *z, uint8_t p) { (void)z; (void)p; return 0xff; }
static void no_out(z80 *z, uint8_t p, uint8_t v) { (void)z; (void)p; (void)v; }

// the main CPU's read map: ROM and RAM pages read straight (the same bytes main_read returns)
static const uint8_t *s_main_map[256];

static void main_map(void)
{
	memset(s_main_map, 0, sizeof(s_main_map));
	for (int pg = 0x00; pg < 0x80; pg++) s_main_map[pg] = s_rom + pg * 0x100;
	for (int pg = 0xb0; pg < 0xc0; pg++) s_main_map[pg] = s_rom + pg * 0x100;
	for (int pg = 0x80; pg < 0x84; pg++) s_main_map[pg] = s_spriteram + (pg - 0x80) * 0x100;
	for (int pg = 0x90; pg < 0x98; pg++) s_main_map[pg] = s_videoram + (pg - 0x90) * 0x100;
	for (int pg = 0xf0; pg < 0xf8; pg++) s_main_map[pg] = s_ram + (pg - 0xf0) * 0x100;
	s_main.z.read_map = s_main_map;
}

static void main_run(int cycles)
{
	// (z80_run: the IRQ line held until taken, HOLD_LINE; the vector is the line's, set before the run)
	s_main.z.int_data = s_main.vector;
	z80_run(&s_main.z, s_main.z.cyc + (unsigned long)cycles, &s_main.irq, 1);
}

// ---- video ----
static void cl_render(void)
{
	memcpy(rbx_video.palette, s_snap_pal, sizeof(s_snap_pal));
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	// the layers of the captured set: river, obj0, then the trees in their priority
	const uint8_t *riv = s_snap_bm, *obj = s_snap_bm + 0x10000;
	const uint8_t *tf = s_snap_bm + 0x20000 + 0x10000 * (0 ^ s_snap_tree_prio);
	const uint8_t *tb = s_snap_bm + 0x20000 + 0x10000 * (1 ^ s_snap_tree_prio);
	for (int y = 0; y < VIS_H; y++) {
		const int o = (y + VIS_Y0) * 256 + VIS_X0;
		uint16_t *d = bm + y * pitch;
		for (int x = 0; x < VIS_W; x++) {
			uint8_t c = riv[o + x];
			if (obj[o + x]) c = obj[o + x];
			if (tf[o + x]) c = tf[o + x];
			if (tb[o + x]) c = tb[o + x];
			d[x] = c;
		}
	}
	// OBJ1 text layer: the attribute latch and the shift registers run over the whole frame
	const uint8_t *ROM = s_obj1rom, *RAM = s_snap_vram;
	uint8_t reg[4] = { 0 }, attrib = 0;
	for (int sy = 0; sy < 256; sy++) {
		uint16_t *d = (sy >= VIS_Y0) ? bm + (sy - VIS_Y0) * pitch : NULL;
		for (int sx = 0; sx < 256; sx++) {
			int ram_addr = ((sx & 0xf8) >> 2) | ((sy & 0xf8) << 3);
			int tile_addr = RAM[ram_addr];
			if (!(RAM[ram_addr + 1] & 0x10) && (sx & 0x04))
				attrib = RAM[ram_addr + 1];
			reg[(sx & 0x0c) >> 2] = ROM[((tile_addr << 4) | ((sx & 0x04) >> 2) | ((sy & 0x07) << 1)) & 0xfff];
			int sum = (sx & 0x0f) + (attrib & 0x0f);
			uint8_t r = reg[(sum & 0x0c) >> 2];
			int sh = 3 - (sum & 3);
			int c0 = (r >> sh) & 1, c1 = (r >> (sh + 4)) & 1;
			int col = c0 | (c1 << 1) | ((attrib & 0xc0) >> 4);
			if (d && sx >= VIS_X0 && (col & 0x07) != 0x07)
				d[sx - VIS_X0] = (uint16_t)(col | 0x20);
		}
	}
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	s_snap_bm = s_bm_river;   // (all its lines 0..255 drawn: the render composes them, the next frame draws the other set)
	s_snap_tree_prio = s_tree_prio;
	s_bm_set ^= 1;
	bm_point();
	memcpy(s_snap_vram, s_videoram, sizeof(s_videoram));
	memcpy(s_snap_pal, s_palette, sizeof(s_palette));
	rbx_video_frame_done();
}

// ---- init / frame ----
static void cl_reset(void)
{
	memset(s_spriteram, 0, sizeof(s_spriteram));
	s_obj_dirty = 1;
	memset(s_videoram, 0, sizeof(s_videoram));
	memset(s_ram, 0, sizeof(s_ram));
	memset(s_riverram, 0, sizeof(s_riverram));
	memset(s_treeram, 0, sizeof(s_treeram));
	memset(s_treeram2, 0, sizeof(s_treeram2));
	memset(s_stateram, 0, sizeof(s_stateram));
	memset(s_palette, 0, sizeof(s_palette));
	memset(&s_main, 0, sizeof(s_main));
	z80_init(&s_main.z);
	s_main.z.read_byte = main_read;
	s_main.z.write_byte = main_write;
	s_main.z.port_in = no_in;
	s_main.z.port_out = no_out;
	main_map();
	s_main.vector = 0xff;
	m68705_reset(&s_mcu);
	s_sloperom_bank = 0;
	s_tree_en = s_horizon = s_mem_dev_selected = s_v_count_river = s_v_count_tree = s_tree_prio = 0;
	s_tree_on[0] = s_tree_on[1] = 0;
	s_port_a_out = s_port_c_out = s_mcu_out = s_mcu_in = 0xff;
	s_tree0_col = s_tree1_col = s_left_bank_col = s_right_bank_col = s_boat_shore_col = 0;
	s_collision_reset = s_tree_collision_reset = s_prev_value_31 = s_dir_31 = 0;
	s_outlatch = 0;
	m68705_port_input(&s_mcu, 2, 0xfe);
	s_wheel = 0;
	s_gear = 1;   // GEAR field default (active low, not pressed)
	s_gear_prev = 0;
	s_frac_mcu = 0;
	ay8910_reset(&s_ay[0]);
	ay8910_reset(&s_ay[1]);
	s_mix_n = 0;
	s_snd_done = 0;
}

static void cl_exit(void)
{
	free(s_bm_mem);
	free(s_mix);
	s_bm_mem = s_bm_river = s_snap_bm = NULL;
	s_mix = NULL;
}

static int cl_init(const rbx_game *g)
{
	uint32_t rsize, msize, tsize, o0size, o1size, ssize, trsize, psize;
	s_rom = rbx_region("maincpu", &rsize);
	uint8_t *mcu = rbx_region("mcu", &msize);
	s_tilerom = rbx_region("tiles", &tsize);
	s_obj0rom = rbx_region("obj0", &o0size);
	s_obj1rom = rbx_region("obj1", &o1size);
	s_sloperom = rbx_region("slope", &ssize);
	s_treerom = rbx_region("tree", &trsize);
	s_prom = rbx_region("proms", &psize);
	if (s_prom && psize >= 0x20)
		st_build();
	if (!s_rom || rsize < 0x10000 || !mcu || !s_tilerom || tsize < 0x4000 || !s_obj0rom || o0size < 0x8000 ||
	    !s_obj1rom || o1size < 0x1000 || !s_sloperom || ssize < 0x1000 || !s_treerom || trsize < 0x3000 || !s_prom || psize < 0x20) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	if (m68705_init(&s_mcu, M68705_P3, mcu, msize)) {
		rbx_fail("%s: MCU ROM missing", g->set);
		return -1;
	}
	s_mcu.port_w[0] = mcu_port_a_w;
	s_mcu.port_r[1] = mcu_port_b_r;
	s_mcu.port_w[2] = mcu_port_c_w;
	s_bm_mem = (uint8_t *)calloc(2 * 4 * 0x10000, 1);
	if (!s_bm_mem) {
		cl_exit();
		return -1;
	}
	s_bm_set = 0;
	bm_point();
	s_snap_bm = s_bm_mem + 4 * 0x10000;   // (the other set, empty, until the first capture)
	s_p_in0 = rbx_port_find("IN0");
	s_p_in1 = rbx_port_find("IN1");
	s_p_mcu = rbx_port_find("MCU");
	s_p_gear = rbx_port_find("GEAR");
	static const char *dsw[4] = { "DSWA", "DSWB", "DSWC", "DSWD" };
	for (int i = 0; i < 4; i++)
		s_p_dsw[i] = rbx_port_find(dsw[i]);
	// AY-3-8910 1.25 MHz, route 0.5: native clock / 8, 4 averaged a sample (39 kHz)
	for (int i = 0; i < 2; i++) {
		ay8910_init(&s_ay[i], AY8910_TYPE_AY8910, 1250000, AY8910_LEGACY_OUTPUT, 128, 4, 1);
		s_ay[i].port_a_read = ay_port_a;
		s_ay[i].port_b_read = ay_port_b;
		s_ay[i].user = (void *)(intptr_t)i;
	}
	uint32_t rate = ay8910_rate(&s_ay[0]);
	s_mix_cap = (int)(rate / 50) + 64;
	s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	if (!s_mix || rbx_audio_setup(rate) || rbx_video_setup(VIS_W, VIS_H, 64)) {
		cl_exit();
		return -1;
	}
	cl_reset();
	return 0;
}

static void cl_frame(void)
{
	rbx_audio.frame_mhz = 60013;   // (the board's frame rate: a frame's sound is played in a 60 Hz frame, as its picture)
	// steering wheel (IPT_DIAL, sensitivity 50, key delta 8) and the toggling gear shift
	int d = rbx_analog(1, RBX_IPT_DIAL) / 2;
	uint32_t in = rbx_input_bits(1);
	if (!d)
		d = ((in & RBX_IN_RIGHT) ? 8 : 0) - ((in & RBX_IN_LEFT) ? 8 : 0);
	s_wheel = (s_wheel + d) & 0xff;
	int gear_pressed = !(port8(s_p_gear) & 1);
	if (gear_pressed && !s_gear_prev)
		s_gear ^= 1;
	s_gear_prev = gear_pressed;

	s_frame_start = s_main.z.cyc;
	s_snd_done = 0;
	for (int line = 0; line < LINES; line++) {
		s_line = line;
		scanline(line);
		if (line == 256) {
			vblank_capture();
			m68705_set_irq_line(&s_mcu, 1);   // vblank interrupt (pulse)
		} else if (line == 257)
			m68705_set_irq_line(&s_mcu, 0);
		int uc = rbx_line_cycles(&s_frac_mcu, MCU_CLOCK, 60013, LINES);   // 5 MHz / (318 x 262) = 60.013 Hz
		for (int s = 0; s < SLICES; s++) {
			main_run(HTOTAL / SLICES);
			m68705_execute(&s_mcu, s ? uc - uc / SLICES : uc / SLICES);
		}
	}
	snd_sync();
	for (int i = 0; i < s_mix_n; i++)
		rbx_audio_put(s_mix[i]);
	memset(s_mix, 0, sizeof(int32_t) * (size_t)s_mix_n);
	s_mix_n = 0;
}

static int cl_state(void *ctx, int load)
{
	int rc = 0;
	zcpu t = s_main;
	rc |= rbx_state_io(ctx, "main", &t, sizeof(t));
	rc |= rbx_state_io(ctx, "spriteram", s_spriteram, sizeof(s_spriteram));
	s_obj_dirty = 1;   // (loaded: the line lists again)
	rc |= rbx_state_io(ctx, "videoram", s_videoram, sizeof(s_videoram));
	rc |= rbx_state_io(ctx, "ram", s_ram, sizeof(s_ram));
	rc |= rbx_state_io(ctx, "riverram", s_riverram, sizeof(s_riverram));
	rc |= rbx_state_io(ctx, "treeram", s_treeram, sizeof(s_treeram));
	rc |= rbx_state_io(ctx, "treeram2", s_treeram2, sizeof(s_treeram2));
	rc |= rbx_state_io(ctx, "stateram", s_stateram, sizeof(s_stateram));
	rc |= rbx_state_io(ctx, "palette", s_palette, sizeof(s_palette));
	rc |= rbx_state_io(ctx, "m68705", &s_mcu.st, sizeof(s_mcu.st));
	rc |= rbx_state_io(ctx, "ay0", &s_ay[0], ay8910_state_size());
	rc |= rbx_state_io(ctx, "ay1", &s_ay[1], ay8910_state_size());
	uint8_t v[24] = { s_mem_dev_selected, s_tree_en, s_horizon, s_v_count_river, s_v_count_tree, s_tree_prio,
		s_tree_on[0], s_tree_on[1], s_port_a_out, s_port_c_out, s_mcu_out, s_mcu_in, s_tree0_col, s_tree1_col,
		s_left_bank_col, s_right_bank_col, s_boat_shore_col, s_collision_reset, s_tree_collision_reset,
		s_prev_value_31, s_dir_31, s_outlatch, (uint8_t)s_wheel, (uint8_t)s_gear };
	uint32_t bank = s_sloperom_bank;
	rc |= rbx_state_io(ctx, "misc", v, sizeof(v));
	rc |= rbx_state_io(ctx, "slope", &bank, sizeof(bank));
	if (rc)
		return -1;
	if (load) {
		t.z.read_byte = s_main.z.read_byte;
		t.z.write_byte = s_main.z.write_byte;
		t.z.port_in = s_main.z.port_in;
		t.z.port_out = s_main.z.port_out;
		t.z.userdata = s_main.z.userdata;
		t.z.read_map = s_main.z.read_map;
		s_main = t;
		s_mem_dev_selected = v[0]; s_tree_en = v[1]; s_horizon = v[2]; s_v_count_river = v[3]; s_v_count_tree = v[4];
		s_tree_prio = v[5]; s_tree_on[0] = v[6]; s_tree_on[1] = v[7]; s_port_a_out = v[8]; s_port_c_out = v[9];
		s_mcu_out = v[10]; s_mcu_in = v[11]; s_tree0_col = v[12]; s_tree1_col = v[13]; s_left_bank_col = v[14];
		s_right_bank_col = v[15]; s_boat_shore_col = v[16]; s_collision_reset = v[17]; s_tree_collision_reset = v[18];
		s_prev_value_31 = v[19]; s_dir_31 = v[20]; s_outlatch = v[21]; s_wheel = v[22]; s_gear = v[23];
		s_sloperom_bank = bank & 0x600;
	}
	return 0;
}

static int cl_save(void *ctx) { return cl_state(ctx, 0); }
static int cl_load(void *ctx) { return cl_state(ctx, 1); }
static int cl_supported(const char *machine) { return !strcmp(machine, "changela"); }

const rbx_driver rbx_drv_changela = { "Change Lanes", cl_init, cl_exit, cl_reset, cl_frame, cl_save, cl_load, cl_supported, cl_render };
