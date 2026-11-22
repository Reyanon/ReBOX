/*  Seibu COP: see seibucop.h.
    Ported from MAME 0.289 src/mame/seibu/seibucop.cpp, seibucop_cmd.ipp, seibucop_dma.ipp (LGPL-2.1+,
    copyright-holders: Olivier Galibert, Angelo Salese, David Haywood, Tomasz Slanina).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "seibucop.h"
#include "../../rbx.h"
#include "../../cpu/m68k/m68k.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

unsigned int m68k_read_memory_8(unsigned int a);
unsigned int m68k_read_memory_16(unsigned int a);
void m68k_write_memory_8(unsigned int a, unsigned int v);
void m68k_write_memory_16(unsigned int a, unsigned int v);

static seibucop_config s_cfg;

typedef struct {
	int16_t pos[3];
	int8_t dx[3];
	uint8_t size[3];
	uint8_t allow_swap, pad;
	uint16_t flags_swap;
	uint32_t spradr;
	int16_t min[3], max[3];
} colinfo;

static struct {
	uint16_t cop_func_trigger[0x100 / 8], cop_func_value[0x100 / 8], cop_func_mask[0x100 / 8];
	uint16_t cop_program[0x100];
	uint16_t cop_latch_addr, cop_latch_trigger, cop_latch_value, cop_latch_mask;
	uint16_t cop_dma_v1, cop_dma_v2, cop_dma_mode;
	uint16_t cop_dma_src[0x200], cop_dma_dst[0x200], cop_dma_size[0x200];
	uint16_t cop_dma_adr_rel, pal_brightness_val, pal_brightness_mode;
	uint32_t cop_itoa;
	uint16_t cop_itoa_mode;
	uint8_t cop_itoa_digits[10];
	uint32_t cop_regs[8];
	uint16_t cop_status, cop_scale, cop_angle, cop_dist, cop_angle_target, cop_angle_step;
	colinfo cop_collision_info[2];
	uint16_t cop_hit_status, cop_hit_baseadr;
	int16_t cop_hit_val[3];
	uint16_t cop_hit_val_stat;
	uint32_t cop_sort_ram_addr, cop_sort_lookup;
	uint16_t cop_sort_param, m_cop_rng_max_value;
	uint32_t m_cop_sprite_dma_param, m_cop_sprite_dma_src;
	int32_t m_cop_sprite_dma_size;
	uint16_t m_cop_unk_param_a, m_cop_unk_param_b, m_cop_rom_addr_lo, m_cop_rom_addr_hi, m_cop_precmd;
	int32_t m_cop_sprite_dma_abs_x, m_cop_sprite_dma_abs_y, m_sprite_dma_rel_x, m_sprite_dma_rel_y, m_sprite_dma_x_clip;
	int32_t m_LEGACY_r0, m_LEGACY_r1;
	uint8_t state_end;
} c;

// ---- the host (a 68000: big-endian, MAME's m_host_endian; words ^ 2, bytes ^ 3 in cop_read/write. A V30 through
// the config's callbacks: little-endian, no swaps) ----
static int s_le;   // (a little-endian host)
static uint16_t rw(uint32_t a)
{
	return s_le ? s_cfg.read16(a) : (uint16_t)m68k_read_memory_16(a & 0xfffffe);
}
static uint8_t rb(uint32_t a) { return s_le ? s_cfg.read8(a) : (uint8_t)m68k_read_memory_8(a & 0xffffff); }
static void ww(uint32_t a, uint16_t v)
{
	if (s_le)
		s_cfg.write16(a, v);
	else
		m68k_write_memory_16(a & 0xfffffe, v);
}
static void wb(uint32_t a, uint8_t v)
{
	if (s_le)
		s_cfg.write8(a, v);
	else
		m68k_write_memory_8(a & 0xffffff, v);
}
static uint32_t rd(uint32_t a) { return s_le ? rw(a) | (uint32_t)rw(a + 2) << 16 : (uint32_t)rw(a) << 16 | rw(a + 2); }
static void wd(uint32_t a, uint32_t v)
{
	if (s_le) {
		ww(a, (uint16_t)v);
		ww(a + 2, (uint16_t)(v >> 16));
	} else {
		ww(a, (uint16_t)(v >> 16));
		ww(a + 2, (uint16_t)v);
	}
}
static uint16_t cop_read_word(uint32_t a) { return rw(a ^ (s_le ? 0 : 2)); }
static uint8_t cop_read_byte(uint32_t a) { return rb(a ^ (s_le ? 0 : 3)); }
static void cop_write_word(uint32_t a, uint16_t v) { ww(a ^ (s_le ? 0 : 2), v); }
static void cop_write_byte(uint32_t a, uint8_t v) { wb(a ^ (s_le ? 0 : 3), v); }

#define COMBINE(var) (var) = (uint16_t)(((var) & ~mem_mask) | (data & mem_mask))

// ---- the macro table ----
static void cop_pgm_data_w(uint16_t data)
{
	c.cop_program[c.cop_latch_addr & 0xff] = data;
	const int idx = (c.cop_latch_addr & 0xff) >> 3;
	c.cop_func_trigger[idx] = c.cop_latch_trigger;
	c.cop_func_value[idx] = c.cop_latch_value;
	c.cop_func_mask[idx] = c.cop_latch_mask;
}

static int find_trigger_match(uint16_t triggerval, uint16_t mask)
{
	int matched = 0, command = -1;
	for (int i = 0; i < 32; i++)
		if ((triggerval & mask) == (c.cop_func_trigger[i] & mask) && c.cop_func_trigger[i] != 0) {
			command = i;
			matched++;
		}
	return matched == 1 ? command : -1;
}

static int check_command_matches(int command, uint16_t seq0, uint16_t seq1, uint16_t seq2, uint16_t seq3, uint16_t seq4,
                                 uint16_t seq5, uint16_t seq6, uint16_t seq7, uint16_t funcval, uint16_t funcmask)
{
	const uint16_t *p = c.cop_program + command * 8;
	return p[0] == seq0 && p[1] == seq1 && p[2] == seq2 && p[3] == seq3 && p[4] == seq4 && p[5] == seq5 && p[6] == seq6 &&
	       p[7] == seq7 && c.cop_func_value[command] == funcval && c.cop_func_mask[command] == funcmask;
}

// ---- the DMAs ----
static uint8_t fade_table(int v)
{
	const int low = v & 0x001f, high = v & 0x03e0;
	return (uint8_t)((low * (high | (high >> 5)) + 0x210) >> 10);
}

static void dma_tilemap_buffer(void)
{
	uint32_t src = (uint32_t)c.cop_dma_src[c.cop_dma_mode] << 6;
	if (src == 0xcfc0)
		src = 0xd000;
	for (int i = 0; i < 0x2800 / 2; i++) {
		const uint16_t v = rw(src);
		src += 2;
		if (s_cfg.videoram_w)
			s_cfg.videoram_w(i, v);
	}
}

static void dma_palette_buffer(void)
{
	uint32_t src = (uint32_t)c.cop_dma_src[c.cop_dma_mode] << 6;
	for (int i = 0; i < 0x1000 / 2; i++) {
		const uint16_t v = rw(src);
		src += 2;
		if (s_cfg.palette_w)
			s_cfg.palette_w(i, v);
	}
}

static void dma_palette_brightness(void)
{
	const int m = c.cop_dma_mode;
	uint32_t src = (uint32_t)c.cop_dma_src[m] << 6, dst = (uint32_t)c.cop_dma_dst[m] << 6;
	const uint32_t size = (((uint32_t)c.cop_dma_size[m] << 5) - ((uint32_t)c.cop_dma_dst[m] << 6) + 0x20) / 2;
	for (uint32_t i = 0; i < size; i++) {
		uint16_t pal_val;
		int r, g, b, rt, gt, bt;
		if (c.pal_brightness_mode == 5) {
			const uint16_t paldata = rw(src);
			if (paldata & 0x8000)
				pal_val = paldata;
			else {
				const uint16_t t = rw(src + c.cop_dma_adr_rel * 0x400u);
				bt = fade_table(((t & 0x7c00) >> 5) | (c.pal_brightness_val ^ 0));
				b = fade_table(((paldata & 0x7c00) >> 5) | (c.pal_brightness_val ^ 0x1f));
				pal_val = (uint16_t)(((b + bt) & 0x1f) << 10);
				gt = fade_table((t & 0x03e0) | (c.pal_brightness_val ^ 0));
				g = fade_table((paldata & 0x03e0) | (c.pal_brightness_val ^ 0x1f));
				pal_val |= (uint16_t)(((g + gt) & 0x1f) << 5);
				rt = fade_table(((t & 0x001f) << 5) | (c.pal_brightness_val ^ 0));
				r = fade_table(((paldata & 0x001f) << 5) | (c.pal_brightness_val ^ 0x1f));
				pal_val |= (uint16_t)((r + rt) & 0x1f);
			}
		} else if (c.pal_brightness_mode == 4) {
			const uint16_t t = rw(src + c.cop_dma_adr_rel * 0x400u), paldata = rw(src);
			bt = (t & 0x7c00) >> 10;
			b = (paldata & 0x7c00) >> 10;
			gt = (t & 0x03e0) >> 5;
			g = (paldata & 0x03e0) >> 5;
			rt = t & 0x001f;
			r = paldata & 0x001f;
			if (c.pal_brightness_val == 0x10 || c.pal_brightness_val == 0xffff)
				pal_val = (uint16_t)(bt << 10 | gt << 5 | rt);
			else {
				const int v = c.pal_brightness_val * 2;
				bt = fade_table(bt << 5 | (v ^ 0));
				b = fade_table(b << 5 | (v ^ 0x1f));
				pal_val = (uint16_t)(((b + bt) & 0x1f) << 10);
				gt = fade_table(gt << 5 | (v ^ 0));
				g = fade_table(g << 5 | (v ^ 0x1f));
				pal_val |= (uint16_t)(((g + gt) & 0x1f) << 5);
				rt = fade_table(rt << 5 | (v ^ 0));
				r = fade_table(r << 5 | (v ^ 0x1f));
				pal_val |= (uint16_t)((r + rt) & 0x1f);
			}
		} else
			pal_val = rw(src);
		ww(dst, pal_val);
		src += 2;
		dst += 2;
	}
}

static void dma_fill(void)
{
	const int m = c.cop_dma_mode;
	if (c.cop_dma_dst[m] != 0)
		return;
	const uint32_t address = (uint32_t)c.cop_dma_src[m] << 6, length = ((uint32_t)c.cop_dma_size[m] + 1) << 5;
	for (uint32_t i = address; i < address + length; i += 4)
		wd(i, c.cop_dma_v1 | (uint32_t)c.cop_dma_v2 << 16);
}

typedef struct { int32_t key; uint16_t val; } sort_entry;

static void dma_zsorting(uint16_t n)
{
	sort_entry *e = (sort_entry *)malloc(sizeof(sort_entry) * (n ? n : 1));
	if (!e)
		return;
	for (int i = 0; i < n; i++) {
		e[i].val = rw(c.cop_sort_lookup + 2u * i);
		e[i].key = (int32_t)rd(c.cop_sort_ram_addr + e[i].val);
	}
	if (c.cop_sort_param == 1 || c.cop_sort_param == 2)   // (std::stable_sort: an insertion sort)
		for (int i = 1; i < n; i++) {
			const sort_entry x = e[i];
			int j = i - 1;
			while (j >= 0 && (c.cop_sort_param == 2 ? x.key > e[j].key : x.key < e[j].key)) {
				e[j + 1] = e[j];
				j--;
			}
			e[j + 1] = x;
		}
	for (int i = 0; i < n; i++)
		ww(c.cop_sort_lookup + 2u * i, e[i].val);
	free(e);
}

static void cop_dma_trigger_w(void)
{
	const int m = c.cop_dma_mode;
	switch (m) {
	case 0x14: dma_tilemap_buffer(); break;
	case 0x15: dma_palette_buffer(); break;
	case 0x80: case 0x81: case 0x82: case 0x83: case 0x84: case 0x85: case 0x86: case 0x87: dma_palette_brightness(); break;
	case 0x09: case 0x0e: {
		uint32_t src = (uint32_t)c.cop_dma_src[m] << 6, dst = (uint32_t)c.cop_dma_dst[m] << 6;
		const uint32_t size = (((uint32_t)c.cop_dma_size[m] << 5) - ((uint32_t)c.cop_dma_dst[m] << 6) + 0x20) / 2;
		for (uint32_t i = 0; i < size; i++) {
			ww(dst, rw(src));
			src += 2;
			dst += 2;
		}
		break;
	}
	case 0x116: {
		const uint32_t address = (uint32_t)c.cop_dma_src[m] << 6, length = ((uint32_t)c.cop_dma_size[m] + 1) << 4;
		for (uint32_t i = address; i < address + length; i += 4)
			wd(i, c.cop_dma_v1 | (uint32_t)c.cop_dma_v2 << 16);
		break;
	}
	case 0x118: case 0x119: case 0x11a: case 0x11b: case 0x11c: case 0x11d: case 0x11e: case 0x11f: dma_fill(); break;
	default: break;
	}
}

// ---- number conversion ----
static void bcd_update(void)
{
	uint32_t val = c.cop_itoa;
	for (int i = 0; i < 9; i++) {
		if (!val && i)
			c.cop_itoa_digits[i] = c.cop_itoa_mode == 3 ? 0x30 : 0x20;
		else {
			c.cop_itoa_digits[i] = (uint8_t)(0x30 | (val % 10));
			val /= 10;
		}
	}
	c.cop_itoa_digits[9] = 0;
}

// ---- collisions ----
static void cop_collision_read_pos(int slot, uint32_t spradr, int allow_swap)
{
	colinfo *ci = &c.cop_collision_info[slot];
	ci->allow_swap = allow_swap != 0;
	ci->flags_swap = cop_read_word(spradr + 2);
	ci->spradr = spradr;
	for (int i = 0; i < 3; i++)
		ci->pos[i] = (int16_t)cop_read_word(spradr + 6 + 4 * i);
}

static void cop_collision_update_hitbox(uint16_t data, int slot, uint32_t hitadr)
{
	uint32_t hitadr2 = rw(hitadr) | (uint32_t)c.cop_hit_baseadr << 16;
	const int num_axis = (data & 0x0100) ? 3 : 2, extraxor = s_le ? 0 : 1;
	colinfo *ci = &c.cop_collision_info[slot];
	for (int i = 0; i < 3; i++) {
		ci->dx[i] = 0;
		ci->size[i] = 0;
	}
	for (int i = 0; i < num_axis; i++) {
		ci->dx[i] = (int8_t)rb(extraxor ^ hitadr2++);
		ci->size[i] = rb(extraxor ^ hitadr2++);
	}
	int16_t dx[3] = { 0, 0, 0 }, size[3] = { 0, 0, 0 };
	for (int i = 0; i < num_axis; i++) {
		size[i] = ci->size[i];
		dx[i] = ci->dx[i];
	}
	uint8_t res = num_axis == 3 ? 7 : 3;
	colinfo *c0 = &c.cop_collision_info[0], *c1 = &c.cop_collision_info[1];
	for (int i = 0; i < num_axis; i++) {
		if (ci->allow_swap && (ci->flags_swap & (1 << i))) {
			ci->max[i] = (int16_t)(ci->pos[i] - dx[i]);
			ci->min[i] = (int16_t)(ci->max[i] - size[i]);
		} else {
			ci->min[i] = (int16_t)(ci->pos[i] + dx[i]);
			ci->max[i] = (int16_t)(ci->min[i] + size[i]);
		}
		if (c0->max[i] > c1->min[i] && c0->min[i] < c1->max[i])
			res &= (uint8_t)~(1 << i);
		if (c1->max[i] > c0->min[i] && c1->min[i] < c0->max[i])
			res &= (uint8_t)~(1 << i);
		c.cop_hit_val[i] = (int16_t)(c0->pos[i] - c1->pos[i]);
	}
	c.cop_hit_val_stat = res;
	c.cop_hit_status = res;
}

// ---- the macros (seibucop_cmd.ipp) ----
static void execute_0205(int offset)
{
	const int32_t ppos = (int32_t)rd(c.cop_regs[0] + 0x04 + offset * 4);
	const int32_t npos = (int32_t)((uint32_t)ppos + rd(c.cop_regs[0] + 0x10 + offset * 4));
	const int delta = (npos >> 16) - (ppos >> 16);
	wd(c.cop_regs[0] + 4 + offset * 4, (uint32_t)npos);
	cop_write_word(c.cop_regs[0] + 0x1e + offset * 4, (uint16_t)(cop_read_word(c.cop_regs[0] + 0x1e + offset * 4) + delta));
}

static void execute_0904(int offset, uint16_t data)
{
	const uint32_t a = c.cop_regs[0] + 16 + offset * 4, v = rd(a), d = rd(c.cop_regs[0] + 0x28 + offset * 4);
	wd(a, (data & 1) ? v + d : v - d);
}

static void angle_of(int32_t num, int32_t den)   // (cop_angle from num / den: atan, MAME's doubles)
{
	c.cop_angle = (uint16_t)(int)(atan((double)num / (double)den) * 128.0 / M_PI);
	if (den < 0)
		c.cop_angle += 0x80;
	c.cop_angle &= 0xff;
}

static void LEGACY_execute_130e_cupsoc(uint16_t data)
{
	const int dy = (int)(rd(c.cop_regs[1] + 4) - rd(c.cop_regs[0] + 4));
	const int dx = (int)(rd(c.cop_regs[1] + 8) - rd(c.cop_regs[0] + 8));
	c.cop_status = 7;
	if (!dx) {
		c.cop_status |= 0x8000;
		c.cop_angle = 0;
	} else
		angle_of(dy, dx);
	c.m_LEGACY_r0 = dy;
	c.m_LEGACY_r1 = dx;
	if (data & 0x80)
		cop_write_byte(c.cop_regs[0] + 0x34, (uint8_t)c.cop_angle);
}

static void execute_338e(uint16_t data, int is_yflip)
{
	const int dx = (int)(rd(c.cop_regs[1] + 4) - rd(c.cop_regs[0] + 4));
	const int dy = (int)(rd(c.cop_regs[1] + 8) - rd(c.cop_regs[0] + 8));
	c.cop_status = 7;
	if (!dy) {
		c.cop_status |= 0x8000;
		c.cop_angle = 0;
	} else
		angle_of(dx, dy);
	if (data & 0x0080)
		cop_write_byte(c.cop_regs[0] + 0x34, (uint8_t)(is_yflip ? c.cop_angle ^ 0x80 : c.cop_angle));
}

static void execute_3b30(uint16_t data)
{
	int dx = (int)(rd(c.cop_regs[1] + 4) - rd(c.cop_regs[0] + 4));
	int dy = (int)(rd(c.cop_regs[1] + 8) - rd(c.cop_regs[0] + 8));
	dx >>= 16;
	dy >>= 16;
	c.cop_dist = (uint16_t)sqrt((double)(dx * dx + dy * dy));
	if (data & 0x0080)
		cop_write_word(c.cop_regs[0] + ((data & 0x200) ? 0x3a : 0x38), c.cop_dist);
}

static void execute_42c2(void)
{
	const int div = cop_read_word(c.cop_regs[0] + 0x36);
	if (!div) {
		c.cop_status |= 0x8000;
		cop_write_word(c.cop_regs[0] + 0x38, 0);
		return;
	}
	c.cop_status = 7;
	cop_write_word(c.cop_regs[0] + 0x38, (uint16_t)(((int)c.cop_dist << (5 - c.cop_scale)) / div));
}

static void execute_6200(int primary_reg, int primary_offset)
{
	uint8_t angle = cop_read_byte(c.cop_regs[primary_reg] + primary_offset);
	uint16_t flags = cop_read_word(c.cop_regs[primary_reg]);
	c.cop_angle_target &= 0xff;
	c.cop_angle_step &= 0xff;
	flags &= (uint16_t)~0x0004;
	int delta = angle - c.cop_angle_target;
	if (delta >= 128)
		delta -= 256;
	else if (delta < -128)
		delta += 256;
	if (delta < 0) {
		if (delta >= -c.cop_angle_step) {
			angle = (uint8_t)c.cop_angle_target;
			flags |= 0x0004;
		} else
			angle = (uint8_t)(angle + c.cop_angle_step);
	} else {
		if (delta <= c.cop_angle_step) {
			angle = (uint8_t)c.cop_angle_target;
			flags |= 0x0004;
		} else
			angle = (uint8_t)(angle - c.cop_angle_step);
	}
	cop_write_word(c.cop_regs[primary_reg], flags);
	if (s_le)
		cop_write_byte(c.cop_regs[primary_reg] + primary_offset, angle);
	else
		cop_write_word(c.cop_regs[primary_reg] + primary_offset, angle);   // (68000: a word)
}

static void LEGACY_execute_6980(int offset)
{
	const int offs = (offset & 3) * 4;
	const int rel_xy = rw(c.m_cop_sprite_dma_src + 4 + offs);
	c.m_sprite_dma_rel_x = rel_xy & 0xff;
	c.m_sprite_dma_rel_y = (rel_xy & 0xff00) >> 8;
}

static void execute_8100(void)
{
	const int raw_angle = cop_read_word(c.cop_regs[0] + 0x34) & 0xff;
	const double angle = raw_angle * M_PI / 128;
	double amp = (65536 >> 5) * (cop_read_word(c.cop_regs[0] + 0x36) & 0xff);
	if (raw_angle == 0xc0)
		amp *= 2;
	const int res = (int)(amp * sin(angle)) << c.cop_scale;
	wd(c.cop_regs[0] + 16, (uint32_t)res);
}

static void execute_8900(void)
{
	const int raw_angle = cop_read_word(c.cop_regs[0] + 0x34) & 0xff;
	const double angle = raw_angle * M_PI / 128;
	double amp = (65536 >> 5) * (cop_read_word(c.cop_regs[0] + 0x36) & 0xff);
	if (raw_angle == 0x80)
		amp *= 2;
	const int res = (int)(amp * cos(angle)) << c.cop_scale;
	wd(c.cop_regs[0] + 20, (uint32_t)res);
}

static void LEGACY_execute_c480(int offset)
{
	const int offs = (offset & 3) * 4;
	const uint16_t sprite_info = (uint16_t)(rw(c.m_cop_sprite_dma_src + offs) + (c.m_cop_sprite_dma_param & 0x3f));
	ww(c.cop_regs[4] + offs + 0, sprite_info);
	const int abs_x = rw(c.cop_regs[0] + 8) - c.m_cop_sprite_dma_abs_x;
	const int abs_y = rw(c.cop_regs[0] + 4) - c.m_cop_sprite_dma_abs_y;
	const int fx = (sprite_info & 0x4000) >> 14, fy = (sprite_info & 0x2000) >> 13;
	const int dx = ((sprite_info & 0x1c00) >> 10) + 1;
	uint16_t sprite_x, sprite_y;
	const int rx = c.m_sprite_dma_rel_x, ry = c.m_sprite_dma_rel_y;
	if (fx) {
		sprite_x = (uint16_t)(0x100 - dx * 16);
		sprite_x = (uint16_t)(sprite_x + abs_x - (rx & 0x78));
		sprite_x = (uint16_t)(sprite_x - ((rx & 0x80) ? 0x80 : 0x100));
	} else
		sprite_x = (uint16_t)((rx & 0x78) + abs_x - ((rx & 0x80) ? 0x80 : 0));
	if (fy)
		sprite_y = (uint16_t)(abs_y + ((ry & 0x80) ? 0x80 : 0) - (ry & 0x78));
	else
		sprite_y = (uint16_t)((ry & 0x78) + abs_y - ((ry & 0x80) ? 0x80 : 0));
	if (c.m_cop_sprite_dma_param & 0x00020000) {
		const uint16_t pri = rw(c.cop_regs[4] + offs + 2) | 0x8000;
		ww(c.cop_regs[4] + offs + 2, pri);
	}
	c.m_sprite_dma_x_clip = sprite_x;
	ww(c.cop_regs[4] + offs + 4, (uint16_t)c.m_sprite_dma_x_clip);
	ww(c.cop_regs[4] + offs + 6, sprite_y);
}

static void LEGACY_execute_dde5(int offset)
{
	const int offs = (offset & 3) * 4;
	int div = rw(c.cop_regs[4] + offs);
	const int16_t dir_offset = (int16_t)rw(c.cop_regs[4] + offs + 8);
	if (div == 0)
		div = 1;
	ww(c.cop_regs[6] + offs + 4, (uint16_t)((rw(c.cop_regs[5] + offs + 4) + dir_offset) / div));
}

static void LEGACY_execute_e30e(uint16_t data)
{
	const int dy = (int)(rd(c.cop_regs[2] + 4) - rd(c.cop_regs[0] + 4));
	const int dx = (int)(rd(c.cop_regs[2] + 8) - rd(c.cop_regs[0] + 8));
	c.cop_status = 7;
	if (!dx) {
		c.cop_status |= 0x8000;
		c.cop_angle = 0;
	} else
		angle_of(dy, dx);
	if (data & 0x0080)
		cop_write_byte(c.cop_regs[0] + 0x34, (uint8_t)c.cop_angle);
}

static void LEGACY_cop_cmd_w(int offset, uint16_t data)
{
	const int command = find_trigger_match(data, 0xf800);
	if (command == -1)
		return;
	const uint16_t funcval = c.cop_func_value[command], funcmask = c.cop_func_mask[command];
	if (check_command_matches(command, 0x188, 0x282, 0x082, 0xb8e, 0x98e, 0, 0, 0, 6, 0xffeb)) { execute_0205(offset); return; }
	if (check_command_matches(command, 0x194, 0x288, 0x088, 0, 0, 0, 0, 0, 6, 0xfbfb)) { execute_0904(offset, data); return; }
	if (check_command_matches(command, 0xb9a, 0xb88, 0x888, 0, 0, 0, 0, 0, 7, 0xfdfb)) { execute_8100(); return; }
	if (check_command_matches(command, 0xb9a, 0xb8a, 0x88a, 0, 0, 0, 0, 0, 7, 0xfdfb)) { execute_8900(); return; }
	if (check_command_matches(command, 0x984, 0xaa4, 0xd82, 0xaa2, 0x39b, 0xb9a, 0xb9a, 0xa9a, 5, 0xbf7f)) { LEGACY_execute_130e_cupsoc(data); return; }
	if (check_command_matches(command, 0x984, 0xaa4, 0xd82, 0xaa2, 0x39b, 0xb9a, 0xb9a, 0xb9a, 5, 0xbf7f)) { execute_338e(data, 1); return; }
	if (check_command_matches(command, 0xf9c, 0xb9c, 0xb9c, 0xb9c, 0xb9c, 0xb9c, 0xb9c, 0x99c, 4, 0x007f)) { execute_3b30(data); return; }
	if (check_command_matches(command, 0xf9a, 0xb9a, 0xb9c, 0xb9c, 0xb9c, 0x29c, 0, 0, 5, 0xfcdd)) { execute_42c2(); return; }
	if (check_command_matches(command, 0xb80, 0xb82, 0xb84, 0xb86, 0, 0, 0, 0, funcval, funcmask)) { cop_collision_read_pos(0, c.cop_regs[0], data & 0x0080); return; }
	if (check_command_matches(command, 0xb40, 0xbc0, 0xbc2, 0, 0, 0, 0, 0, funcval, funcmask)) { cop_collision_update_hitbox(data, 0, c.cop_regs[2]); return; }
	if (check_command_matches(command, 0xba0, 0xba2, 0xba4, 0xba6, 0, 0, 0, 0, funcval, funcmask)) { cop_collision_read_pos(1, c.cop_regs[1], data & 0x0080); return; }
	if (check_command_matches(command, 0xb60, 0xbe0, 0xbe2, 0, 0, 0, 0, 0, funcval, funcmask)) { cop_collision_update_hitbox(data, 1, c.cop_regs[3]); return; }
	if (check_command_matches(command, 0xb80, 0xba0, 0, 0, 0, 0, 0, 0, 10, 0xfff3)) { LEGACY_execute_6980(offset); return; }
	if (check_command_matches(command, 0x080, 0x882, 0, 0, 0, 0, 0, 0, 10, 0xff00)) { LEGACY_execute_c480(offset); return; }
	if (check_command_matches(command, 0xf80, 0xaa2, 0x984, 0x0c2, 0, 0, 0, 0, 5, 0x7ff7)) { LEGACY_execute_dde5(offset); return; }
	if (check_command_matches(command, 0x3a0, 0x3a6, 0x380, 0xaa0, 0x2a6, 0, 0, 0, 8, 0xf3e7)) { execute_6200(1, 0xc); return; }
	if (check_command_matches(command, 0x380, 0x39a, 0x380, 0xa80, 0x29a, 0, 0, 0, 8, 0xf3e7)) { execute_6200(0, 0x34); return; }
	if (check_command_matches(command, 0x984, 0xac4, 0xd82, 0xac2, 0x39b, 0xb9a, 0xb9a, 0xa9a, 5, 0xb07f)) { LEGACY_execute_e30e(data); return; }
	// (d104, 5105, 5905, f105: nothing done in MAME)
}

// ---- the V30 boards' macros (seibucop_cmd.ipp), MAME's accesses kept (raw words, bytes where it uses them) ----
static void execute_2288(uint16_t data)
{
	const int dx = rw(c.cop_regs[0] + 0x12), dy = rw(c.cop_regs[0] + 0x16);   // (read_word: unsigned)
	if (!dy) {
		c.cop_status |= 0x8000;
		c.cop_angle = 0;
	} else {
		c.cop_angle = (uint16_t)(int)(atan((double)dx / (double)dy) * 128 / M_PI);
		if (dy < 0)
			c.cop_angle += 0x80;
	}
	if (data & 0x0080)
		wb(c.cop_regs[0] + 0x34, (uint8_t)c.cop_angle);
}

static void execute_2a05(int offset)
{
	const int delta = rw(c.cop_regs[1] + 0x1e + offset * 4);
	wd(c.cop_regs[0] + 4 + 2 + offset * 4, (uint32_t)(rw(c.cop_regs[0] + 4 + 2 + offset * 4) + delta));
	wd(c.cop_regs[0] + 0x1e + offset * 4, (uint32_t)(rw(c.cop_regs[0] + 0x1e + offset * 4) + delta));
}

static void execute_4aa0(void)
{
	int div = rw(c.cop_regs[0] + 0x38);
	if (!div)
		div = 1;
	c.cop_status = 7;
	ww(c.cop_regs[0] + 0x36, (uint16_t)(((int)c.cop_dist << (5 - c.cop_scale)) / div));
}

static void execute_ede5(void)
{
	const int32_t v1 = (int32_t)rd(c.cop_regs[4] + 8), v2 = (int32_t)rd(c.cop_regs[4] + 12), v3 = (int32_t)rd(c.cop_regs[4] + 16);
	if (v3 != 0)
		cop_write_word(c.cop_regs[4] + 22, (uint16_t)((v1 + v2) / v3));
}

static void cop_cmd_w(int offset, uint16_t data)
{
	c.cop_status &= 0x7fff;
	switch (data) {
	case 0x0205: execute_0205(offset); break;
	case 0x0904: case 0x0905: execute_0904(offset, data); break;
	case 0x130e: case 0x138e: case 0x330e: execute_338e(data, 0); break;   // (execute_130e)
	case 0x338e: execute_338e(data, 0); break;
	case 0x2208: case 0x2288: execute_2288(data); break;
	case 0x2a05: execute_2a05(offset); break;
	case 0x39b0: case 0x3b30: case 0x3bb0: execute_3b30(data); break;
	case 0x42c2: execute_42c2(); break;
	case 0x4aa0: execute_4aa0(); break;
	case 0x6200: execute_6200(0, 0x34); break;
	case 0x8100: execute_8100(); break;
	case 0x8900: execute_8900(); break;
	case 0x5205: case 0x5a05: wd(c.cop_regs[1], rd(c.cop_regs[0])); break;
	case 0xf205: wd(c.cop_regs[2], rd(c.cop_regs[0] + 4)); break;
	case 0x7e05: wb(0x470, rb(c.cop_regs[4])); break;   // (raidendx)
	case 0xa100: case 0xa180: cop_collision_read_pos(0, c.cop_regs[0], data & 0x0080); break;
	case 0xa900: case 0xa980: cop_collision_read_pos(1, c.cop_regs[1], data & 0x0080); break;
	case 0xb100: cop_collision_update_hitbox(data, 0, c.cop_regs[2]); break;
	case 0xb900: cop_collision_update_hitbox(data, 1, c.cop_regs[3]); break;
	case 0xede5: execute_ede5(); break;
	case 0xf790: c.cop_dist = (uint16_t)(uint32_t)sqrt((double)rd(c.cop_regs[4] + 8)); break;
	case 0xfc84: wd(c.cop_regs[4] + 8, rd(c.cop_regs[4] + 0) - rd(c.cop_regs[4] + 4)); break;
	default: break;
	}
}

// ---- the bus ----
void seibucop_init(const seibucop_config *cfg)
{
	s_cfg = *cfg;
	s_le = s_cfg.read16 != NULL;
	seibucop_reset();
}

void seibucop_reset(void) { memset(&c, 0, offsetof(__typeof__(c), state_end)); }

int seibucop_read(uint32_t off, uint16_t *v)
{
	switch (off) {
	case 0x2c: *v = c.m_cop_rng_max_value; return 1;
	case 0x7e: *v = c.cop_dma_mode; return 1;
	case 0x180: *v = c.cop_hit_status; return 1;
	case 0x182: case 0x184: case 0x186: *v = (uint16_t)c.cop_hit_val[(off - 0x182) >> 1]; return 1;
	case 0x188: *v = c.cop_hit_val_stat; return 1;
	case 0x1b0: *v = c.cop_status; return 1;
	case 0x1b2: *v = c.cop_dist; return 1;
	case 0x1b4: *v = c.cop_angle; return 1;
	default: break;
	}
	if (off >= 0xa0 && off < 0xae) {
		*v = (uint16_t)(c.cop_regs[(off - 0xa0) >> 1] >> 16);
		return 1;
	}
	if (off >= 0xc0 && off < 0xce) {
		*v = (uint16_t)c.cop_regs[(off - 0xc0) >> 1];
		return 1;
	}
	if (off >= 0x190 && off < 0x19a) {
		const int o = (int)(off - 0x190) >> 1;
		*v = (uint16_t)(c.cop_itoa_digits[o * 2] | c.cop_itoa_digits[o * 2 + 1] << 8);
		return 1;
	}
	if (off >= 0x1a0 && off < 0x1a8) {   // cop_prng_r: the 68000's total cycles
		*v = (uint16_t)(s_cfg.host_cycles() % ((uint64_t)c.m_cop_rng_max_value + 1));
		return 1;
	}
	return 0;
}

int seibucop_write(uint32_t off, uint16_t data, uint16_t mem_mask)
{
	switch (off) {
	case 0x00: c.m_cop_sprite_dma_param = (c.m_cop_sprite_dma_param & 0xffff0000u) | data; return 1;
	case 0x02: c.m_cop_sprite_dma_param = (c.m_cop_sprite_dma_param & 0x0000ffffu) | (uint32_t)data << 16; return 1;
	case 0x04: case 0x0a: case 0x0e: return 1;
	case 0x0c: c.m_cop_sprite_dma_size = data; return 1;
	case 0x10:   // cop_sprite_dma_inc_w
		if (!data) {
			if (c.m_sprite_dma_x_clip >= -160 && c.m_sprite_dma_x_clip < 320)
				c.cop_regs[4] += 8;
			c.m_cop_sprite_dma_src += 6;
			c.m_cop_sprite_dma_size--;
			if (c.m_cop_sprite_dma_size > 0)
				c.cop_status &= (uint16_t)~2;
			else
				c.cop_status |= 2;
		}
		return 1;
	case 0x12: c.m_cop_sprite_dma_src = (c.m_cop_sprite_dma_src & 0x0000ffffu) | (uint32_t)data << 16; return 1;
	case 0x14: c.m_cop_sprite_dma_src = (c.m_cop_sprite_dma_src & 0xffff0000u) | data; return 1;
	case 0x1c: COMBINE(c.cop_angle_target); return 1;
	case 0x1e: COMBINE(c.cop_angle_step); return 1;
	case 0x20: c.cop_itoa = (c.cop_itoa & ~(uint32_t)mem_mask) | (data & mem_mask); bcd_update(); return 1;
	case 0x22: c.cop_itoa = (c.cop_itoa & ~((uint32_t)mem_mask << 16)) | ((uint32_t)(data & mem_mask) << 16); bcd_update(); return 1;
	case 0x24: COMBINE(c.cop_itoa_mode); return 1;
	case 0x28: COMBINE(c.cop_dma_v1); return 1;
	case 0x2a: COMBINE(c.cop_dma_v2); return 1;
	case 0x2c: COMBINE(c.m_cop_rng_max_value); return 1;
	case 0x32: cop_pgm_data_w(data); return 1;
	case 0x34: c.cop_latch_addr = data; return 1;
	case 0x36: COMBINE(c.cop_hit_baseadr); return 1;
	case 0x38: c.cop_latch_value = data; return 1;
	case 0x3a: c.cop_latch_mask = data; return 1;
	case 0x3c: c.cop_latch_trigger = data; return 1;
	case 0x40: COMBINE(c.m_cop_unk_param_a); return 1;
	case 0x42: COMBINE(c.m_cop_unk_param_b); return 1;
	case 0x44: COMBINE(c.cop_scale); c.cop_scale &= 3; return 1;
	case 0x46: COMBINE(c.m_cop_rom_addr_hi); return 1;
	case 0x48: COMBINE(c.m_cop_rom_addr_lo); return 1;
	case 0x4a: COMBINE(c.m_cop_precmd); return 1;
	case 0x50: c.cop_sort_ram_addr = (c.cop_sort_ram_addr & 0x0000ffffu) | (uint32_t)data << 16; return 1;
	case 0x52: c.cop_sort_ram_addr = (c.cop_sort_ram_addr & 0xffff0000u) | data; return 1;
	case 0x54: c.cop_sort_lookup = (c.cop_sort_lookup & 0x0000ffffu) | (uint32_t)data << 16; return 1;
	case 0x56: c.cop_sort_lookup = (c.cop_sort_lookup & 0xffff0000u) | data; return 1;
	case 0x58: c.cop_sort_param = data; return 1;
	case 0x5a: COMBINE(c.pal_brightness_val); return 1;
	case 0x5c: COMBINE(c.pal_brightness_mode); return 1;
	case 0x76: COMBINE(c.cop_dma_adr_rel); return 1;
	case 0x78: COMBINE(c.cop_dma_src[c.cop_dma_mode & 0x1ff]); return 1;
	case 0x7a: COMBINE(c.cop_dma_size[c.cop_dma_mode & 0x1ff]); return 1;
	case 0x7c: COMBINE(c.cop_dma_dst[c.cop_dma_mode & 0x1ff]); return 1;
	case 0x7e: COMBINE(c.cop_dma_mode); return 1;
	case 0x8c: c.m_cop_sprite_dma_abs_y = data; return 1;
	case 0x8e: c.m_cop_sprite_dma_abs_x = data; return 1;
	case 0x100: case 0x102: case 0x104:
		if (s_cfg.raiden2)
			cop_cmd_w((int)(off - 0x100) >> 1, data);
		else
			LEGACY_cop_cmd_w((int)(off - 0x100) >> 1, data);
		return 1;
	case 0x2fc: cop_dma_trigger_w(); return 1;
	case 0x2fe: dma_zsorting(data); return 1;
	default: break;
	}
	if (off >= 0xa0 && off < 0xae) {
		uint32_t *r = &c.cop_regs[(off - 0xa0) >> 1];
		*r = (*r & ~((uint32_t)mem_mask << 16)) | ((uint32_t)(data & mem_mask) << 16);
		return 1;
	}
	if (off >= 0xc0 && off < 0xce) {
		uint32_t *r = &c.cop_regs[(off - 0xc0) >> 1];
		*r = (*r & ~(uint32_t)mem_mask) | (data & mem_mask);
		return 1;
	}
	return 0;
}

int seibucop_state(void *ctx, int load)
{
	(void)load;
	return rbx_state_io(ctx, "seibucop", &c, (uint32_t)offsetof(__typeof__(c), state_end));
}
