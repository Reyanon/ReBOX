/*  Sega 315-5313 (the Mega Drive VDP) for System 18. Ported from MAME src/mame/sega/315_5313.cpp (BSD-3-Clause,
    copyright-holders: David Haywood): the data / control ports and their commands, the instant DMAs (68000 to VRAM /
    CRAM / VSRAM, VRAM fill, VRAM copy), the status word and the HV counter, the alternate timing's scanline callback
    and a line rendered (planes A and B, the window, the sprites, shadow / hilight) into MAME's raw line.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "s315_5313.h"

#include <string.h>

#define R(n) (v->regs[n])
#define REG0_IRQ4_ENABLE(v)      (((v)->regs[0x00] & 0x10) >> 4)
#define REG0_SPECIAL_PAL(v)      (((v)->regs[0x00] & 0x04) >> 2)
#define REG0_DISPLAY_DISABLE(v)  ((v)->regs[0x00] & 0x01)
#define REG01_DISP_ENABLE(v)     (((v)->regs[0x01] & 0x40) >> 6)
#define REG01_DMA_ENABLE(v)      (((v)->regs[0x01] & 0x10) >> 4)
#define REG01_240_LINE(v)        (((v)->regs[0x01] & 0x08) >> 3)
#define REG02_PATTERN_ADDR_A(v)  (((v)->regs[0x02] & 0x38) >> 3)
#define REG03_PATTERN_ADDR_W(v)  (((v)->regs[0x03] & 0x3e) >> 1)
#define REG04_PATTERN_ADDR_B(v)  ((v)->regs[0x04] & 0x07)
#define REG05_SPRITE_ADDR(v)     ((v)->regs[0x05] & 0x7f)
#define REG07_BGCOLOUR(v)        ((v)->regs[0x07] & 0x3f)
#define REG0A_HINT_VALUE(v)      ((v)->regs[0x0a] & 0xff)
#define REG0B_VSCROLL_MODE(v)    (((v)->regs[0x0b] & 0x04) >> 2)
#define REG0B_HSCROLL_MODE(v)    ((v)->regs[0x0b] & 0x03)
#define REG0C_RS0(v)             (((v)->regs[0x0c] & 0x80) >> 7)
#define REG0C_SHADOW_HIGLIGHT(v) (((v)->regs[0x0c] & 0x08) >> 3)
#define REG0C_INTERLEAVE(v)      (((v)->regs[0x0c] & 0x06) >> 1)
#define REG0C_RS1(v)             ((v)->regs[0x0c] & 0x01)
#define REG0D_HSCROLL_ADDR(v)    ((v)->regs[0x0d] & 0x3f)
#define REG0F_AUTO_INC(v)        ((v)->regs[0x0f] & 0xff)
#define REG10_VSCROLL_SIZE(v)    (((v)->regs[0x10] & 0x30) >> 4)
#define REG10_HSCROLL_SIZE(v)    ((v)->regs[0x10] & 0x03)
#define REG11_WINDOW_RIGHT(v)    (((v)->regs[0x11] & 0x80) >> 7)
#define REG11_WINDOW_HPOS(v)     ((v)->regs[0x11] & 0x1f)
#define REG12_WINDOW_DOWN(v)     (((v)->regs[0x12] & 0x80) >> 7)
#define REG12_WINDOW_VPOS(v)     ((v)->regs[0x12] & 0x1f)
#define REG17_DMATYPE(v)         (((v)->regs[0x17] & 0xc0) >> 6)
#define VRAM(a) (v->vram[(a) & 0x7fff])

#define CODE_DMA(v)          ((v)->vdp_code & 0x20)
#define CODE_VRAM_WRITE(v)   (((v)->vdp_code & 0x0f) == 0x01)
#define CODE_CRAM_WRITE(v)   (((v)->vdp_code & 0x0f) == 0x03)
#define CODE_VSRAM_WRITE(v)  (((v)->vdp_code & 0x0f) == 0x05)
#define CODE_VRAM_COPY(v)    (((v)->vdp_code & 0x30) == 0x30)

static int get_hres(const s315_5313_t *v) { return (REG0C_RS0(v) | (REG0C_RS1(v) << 1)) & 3; }

uint32_t s315_5313_rgb(int index)
{
	static const uint8_t level[15] = { 0, 29, 52, 70, 87, 101, 116, 130, 144, 158, 172, 187, 206, 228, 255 };
	const int i = index & 0x1ff, set = (index >> 9) & 3;
	const int r = i & 7, g = (i >> 3) & 7, b = (i >> 6) & 7;
	int lr, lg, lb;
	if (set == 0) { lr = level[r << 1]; lg = level[g << 1]; lb = level[b << 1]; }
	else if (set == 1) { lr = level[r]; lg = level[g]; lb = level[b]; }
	else { lr = level[7 + r]; lg = level[7 + g]; lb = level[7 + b]; }
	return (uint32_t)(lr << 16 | lg << 8 | lb);
}

void s315_5313_init(s315_5313_t *v)
{
	memset(v, 0, sizeof(*v));
}

void s315_5313_reset(s315_5313_t *v)
{
	memset(v, 0, S315_5313_STATE_SIZE);   // (device_start's zeroed memories, device_reset)
	v->irq4counter = -1;
	v->total_scanlines = 262;
	v->visible_scanlines = 224;
	v->irq6_scanline = 224;
	v->z80irq_scanline = 226;
}

static void vram_w(s315_5313_t *v, int offset, uint16_t data, uint16_t mask)
{
	uint16_t *p = &v->vram[offset & 0x7fff];
	*p = (uint16_t)((*p & ~mask) | (data & mask));
}
static void address_inc(s315_5313_t *v) { v->vdp_address = (uint16_t)(v->vdp_address + REG0F_AUTO_INC(v)); }

static void vram_write(s315_5313_t *v, uint16_t data)
{
	const uint16_t sprite_base = REG0C_RS1(v) ? (uint16_t)((REG05_SPRITE_ADDR(v) & 0x7e) << 9) : (uint16_t)((REG05_SPRITE_ADDR(v) & 0x7f) << 9);
	const int table_size = REG0C_RS1(v) ? 0x400 : 0x200;
	if (v->vdp_address & 1)
		data = (uint16_t)((data >> 8) | (data << 8));
	vram_w(v, v->vdp_address >> 1, data, 0xffff);
	if (v->vdp_address >= sprite_base && v->vdp_address < sprite_base + table_size)
		v->sat[(v->vdp_address & (table_size - 1)) >> 1] = data;
	address_inc(v);
}
static void write_cram_value(s315_5313_t *v, int offset, uint16_t data)
{
	v->cram[offset] = data;
	const uint16_t c = (uint16_t)(((data & 0xe) >> 1) | ((data & 0xe0) >> 2) | ((data & 0xe00) >> 3));
	v->palette_lookup[offset] = c;
	if (v->cram_w)
		v->cram_w(v->user, offset, c);
}

static void data_port_w(s315_5313_t *v, uint16_t data)
{
	v->command_pending = 0;
	if (v->vram_fill_pending) {
		if (v->vdp_address & 1)
			vram_w(v, v->vdp_address >> 1, data & 0x00ff, 0x00ff);
		else
			vram_w(v, v->vdp_address >> 1, (uint16_t)((data & 0x00ff) << 8), 0xff00);
		for (int count = 0; count <= v->vram_fill_length; count++) {
			if (v->vdp_address & 1)
				vram_w(v, v->vdp_address >> 1, data & 0xff00, 0xff00);
			else
				vram_w(v, v->vdp_address >> 1, (uint16_t)((data & 0xff00) >> 8), 0x00ff);
			address_inc(v);
		}
		v->regs[0x13] = 0;
		v->regs[0x14] = 0;
		return;
	}
	switch (v->vdp_code & 0x0f) {
	case 0x1: vram_write(v, data); break;
	case 0x3: write_cram_value(v, (v->vdp_address & 0x7e) >> 1, data); address_inc(v); break;
	case 0x5: v->vsram[(v->vdp_address & 0x7e) >> 1] = data; address_inc(v); break;
	}
}

static void set_register(s315_5313_t *v, int regnum, uint8_t value)
{
	if (!(v->regs[0x01] & 0x04) && regnum > 10)
		return;
	v->regs[regnum] = value;   // (the IRQ lines: not wired on System 18)
}

static uint16_t get_word_68k(s315_5313_t *v, uint32_t source)
{
	if (source <= 0x3fffff || (source >= 0xe00000 && source <= 0xffffff))
		return v->read68k ? v->read68k(v->user, source) : 0;
	return 0;   // (MAME: machine().rand())
}
static void dma_done_regs(s315_5313_t *v, uint32_t source)
{
	v->regs[0x13] = 0;
	v->regs[0x14] = 0;
	v->regs[0x15] = (source >> 1) & 0xff;
	v->regs[0x16] = (source >> 9) & 0xff;
	v->regs[0x17] = (source >> 17) & 0xff;
}
static void handle_dma_bits(s315_5313_t *v)
{
	const int type = REG17_DMATYPE(v);
	if (type == 0 || type == 1) {
		uint32_t source = ((uint32_t)R(0x15) | ((uint32_t)R(0x16) << 8) | (((uint32_t)R(0x17) & 0x7f) << 16)) << 1;
		uint32_t length = (uint16_t)(((uint32_t)R(0x13) | ((uint32_t)R(0x14) << 8)) << 1);
		if (!REG01_DMA_ENABLE(v))
			return;
		if (CODE_VRAM_WRITE(v)) {
			int len = length ? (int)length : 0xffff;
			if (v->spin)
				v->spin(v->user, len * 1000 / 3500);
			for (int count = 0; count < (len >> 1); count++) {
				vram_write(v, get_word_68k(v, source));
				source += 2;
				if (source > 0xffffff) source = 0xe00000;
			}
			dma_done_regs(v, source);
		} else if (CODE_CRAM_WRITE(v)) {
			const int len = length ? (int)length : 0xffff;
			for (int count = 0; count < (len >> 1); count++) {
				write_cram_value(v, (v->vdp_address & 0x7e) >> 1, get_word_68k(v, source));
				source += 2;
				if (source > 0xffffff) source = 0xfe0000;
				address_inc(v);
			}
			dma_done_regs(v, source);
		} else if (CODE_VSRAM_WRITE(v)) {
			const int len = length ? (int)length : 0xffff;
			for (int count = 0; count < (len >> 1); count++) {
				if (v->vdp_address >= 0x80)
					return;
				v->vsram[(v->vdp_address & 0x7e) >> 1] = get_word_68k(v, source);
				source += 2;
				if (source > 0xffffff) source = 0xfe0000;
				address_inc(v);
			}
			dma_done_regs(v, source);
		}
	} else if (type == 2) {
		if ((CODE_VRAM_WRITE(v) || CODE_CRAM_WRITE(v) || CODE_VSRAM_WRITE(v)) && REG01_DMA_ENABLE(v)) {
			v->vram_fill_pending = 1;
			v->vram_fill_length = (uint16_t)(R(0x13) | (R(0x14) << 8));
		}
	} else if (CODE_VRAM_COPY(v) || CODE_VRAM_WRITE(v)) {
		uint32_t source = (uint32_t)(R(0x15) | (R(0x16) << 8));
		const uint16_t length = (uint16_t)(R(0x13) | (R(0x14) << 8));
		if (!REG01_DMA_ENABLE(v))
			return;
		for (int x = 0; x < length; x++) {
			const uint16_t w = VRAM((source & 0xffff) >> 1);
			const uint8_t b = (source & 1) ? (uint8_t)w : (uint8_t)(w >> 8);
			if (v->vdp_address & 1)
				vram_w(v, v->vdp_address >> 1, b, 0x00ff);
			else
				vram_w(v, v->vdp_address >> 1, (uint16_t)(b << 8), 0xff00);
			source++;
			address_inc(v);
		}
	}
}

static void update_code_and_address(s315_5313_t *v)
{
	v->vdp_code = (uint8_t)(((v->command_part1 & 0xc000) >> 14) | ((v->command_part2 & 0x00f0) >> 2));
	v->vdp_address = (uint16_t)((v->command_part1 & 0x3fff) | ((v->command_part2 & 0x0003) << 14));
}
static void ctrl_port_w(s315_5313_t *v, uint16_t data)
{
	v->vram_fill_pending = 0;
	if (v->command_pending) {
		v->command_pending = 0;
		v->command_part2 = data;
		update_code_and_address(v);
		if (CODE_DMA(v))
			handle_dma_bits(v);
	} else if ((data & 0xc000) == 0x8000) {
		set_register(v, ((data & 0x3f00) >> 8) & 0x1f, (uint8_t)data);
		v->vdp_code = 0;
		v->vdp_address = 0;
	} else {
		v->command_pending = 1;
		v->command_part1 = data;
		update_code_and_address(v);
	}
}

void s315_5313_w(s315_5313_t *v, int offset, uint16_t data, uint16_t mem_mask)
{
	switch ((offset << 1) & 0x1e) {
	case 0x00: case 0x02:
		if (!(mem_mask & 0xff00))
			data = (uint16_t)((data & 0x00ff) | data << 8);
		else if (!(mem_mask & 0x00ff))
			data = (uint16_t)((data & 0xff00) | data >> 8);
		data_port_w(v, data);
		break;
	case 0x04: case 0x06:
		ctrl_port_w(v, data);
		break;
	}
}

static uint16_t data_port_r(s315_5313_t *v)
{
	uint16_t r = 0;
	v->command_pending = 0;
	switch (v->vdp_code & 0x0f) {
	case 0x0: r = VRAM((v->vdp_address & 0xfffe) >> 1); address_inc(v); break;
	case 0x4: r = v->vsram[(v->vdp_address & 0x7e) >> 1]; address_inc(v); break;
	case 0x8: r = v->cram[(v->vdp_address & 0x7e) >> 1]; address_inc(v); break;
	}
	return r;
}
static uint16_t ctrl_port_r(s315_5313_t *v, int hpos)
{
	int odd_frame = 0, hblank = 0, vblank = v->vblank_flag;
	if (v->imode & 1)
		odd_frame = v->imode_odd_frame ^ 1;
	if (hpos > 400) hblank = 1;
	if (hpos > 460) hblank = 0;
	if (!REG01_DISP_ENABLE(v))
		vblank = 1;
	return (uint16_t)((1 << 13) | (1 << 12) | (1 << 10) | (1 << 9) | (v->irq6_pending << 7) | (v->sprite_collision << 5) |
	                  (odd_frame << 4) | (vblank << 3) | (hblank << 2));
}
static uint16_t hv_counters(s315_5313_t *v, int vpos, int hpos)
{
	if (hpos > 460) vpos++;
	if (vpos < 0) vpos = v->total_scanlines;
	vpos %= v->total_scanlines;
	if (REG01_240_LINE(v))   // (vc_ntsc_240: 00-FF, 00-05)
		vpos = vpos < 256 ? vpos : vpos - 256;
	else                     // (vc_ntsc_224: 00-EA, E5-FF)
		vpos = vpos <= 0xea ? vpos : vpos - 6;
	if (hpos > 0xf7) hpos -= 0x49;
	return (uint16_t)(((vpos & 0xff) << 8) | (hpos & 0xff));
}

uint16_t s315_5313_r(s315_5313_t *v, int offset, uint16_t mem_mask, int vpos, int hpos)
{
	(void)mem_mask;
	switch ((offset << 1) & 0x1e) {
	case 0x00: case 0x02: return data_port_r(v);
	case 0x04: case 0x06: return ctrl_port_r(v, hpos);
	case 0x08: case 0x0a: case 0x0c: case 0x0e: return hv_counters(v, vpos, hpos);
	}
	return 0;
}

// vdp_handle_eof + vdp_handle_scanline_callback (the timers' IRQs go nowhere on System 18; the flags kept)
void s315_5313_scanline(s315_5313_t *v, int vpos)
{
	if (vpos == 0) {
		v->vblank_flag = 0;
		v->sprite_collision = 0;
		v->imode = (uint8_t)REG0C_INTERLEAVE(v);
		v->imode_odd_frame ^= 1;
		v->total_scanlines = 262;
		v->visible_scanlines = v->irq6_scanline = v->z80irq_scanline = REG01_240_LINE(v) ? 240 : 224;
	}
	if (vpos == v->total_scanlines - 1)
		return;
	if (vpos == v->irq6_scanline) {
		v->irq6_pending = 1;
		v->vblank_flag = 1;
	}
	if (vpos <= 224) {
		if (--v->irq4counter == -1) {
			v->irq4counter = REG0A_HINT_VALUE(v);
			v->irq4_pending = 1;
		}
	} else
		v->irq4counter = REG0A_HINT_VALUE(v);
}

// ---- rendering (the line buffers on the caller's stack: lines drawn on several threads at once) ----
typedef struct { uint8_t sprite_line[1024], highpri[320]; uint32_t video[320]; } scratch_t;
#define s_sprite_line (sc->sprite_line)
#define s_highpri (sc->highpri)
#define s_video (sc->video)

// a tile's row of 8 pixels as a 32-bit word (pixel 0 in the top nibble)
static inline uint32_t tile_row(const s315_5313_t *v, int code, int row)
{
	const uint32_t a = (uint32_t)((code & 0x7ff) * 16 + row * 2);
	return (uint32_t)v->vram[a & 0x7fff] << 16 | v->vram[(a + 1) & 0x7fff];
}

static void render_sprites(s315_5313_t *v, scratch_t *sc, int scanline)
{
	int maxsprites = 64, maxpixels = 256;
	uint16_t base = (uint16_t)((REG05_SPRITE_ADDR(v) & 0x7f) << 9);
	if (get_hres(v) >= 2) {
		maxsprites = 80;
		maxpixels = 320;
		base = (uint16_t)((REG05_SPRITE_ADDR(v) & 0x7e) << 9);
	}
	memset(s_sprite_line, 0, sizeof(s_sprite_line));
	int spritemask = 0, spritenum = 0, link;
	do {
		const uint16_t s0 = v->sat[(spritenum * 4 + 0) & 0x1ff], s1 = v->sat[(spritenum * 4 + 1) & 0x1ff];
		const int height = ((s1 & 0x0300) >> 8) + 1, width = ((s1 & 0x0c00) >> 10) + 1;
		link = s1 & 0x7f;
		const int xpos = VRAM((base >> 1) + spritenum * 4 + 3) & 0x1ff;
		const int ypos = s0 & 0x1ff, drawypos = ypos - 128, drawheight = height * 8;
		if (drawypos <= scanline && drawypos + drawheight > scanline) {
			const uint16_t a = VRAM((base >> 1) + spritenum * 4 + 2);
			const int addr = a & 0x7ff, xflip = (a >> 11) & 1, yflip = (a >> 12) & 1, colour = (a >> 13) & 3;
			const int pri = (a & 0x8000) ? 0x80 : 0x40;
			if (xpos == 0) spritemask |= 1;
			if (xpos > 0 && xpos < 0x40) spritemask |= 2;
			if (spritemask == 3)
				return;
			int yline = scanline - drawypos;
			const int ytile = yline >> 3;
			if (ytile < height) {
				yline &= 7;
				for (int xtile = 0; xtile < width; xtile++) {
					int xtile_code = xflip ? width - xtile - 1 : xtile;
					const int ytile_code = yflip ? height - ytile - 1 : ytile;
					const int row = yflip ? 7 - yline : yline;
					xtile_code *= height;
					const int code = (addr + xtile_code + ytile_code) % 2048;
					const uint32_t bits = tile_row(v, code, row);
					int xxx = (xpos + xtile * 8) & 0x1ff;
					for (int loopcount = 0; loopcount < 8; loopcount++) {
						const int px = xflip ? 7 - loopcount : loopcount;
						const uint8_t dat = (uint8_t)((bits >> (28 - 4 * px)) & 15);
						if (dat) {
							if (!s_sprite_line[xxx])
								s_sprite_line[xxx] = (uint8_t)(dat | (colour << 4) | pri);
							else
								v->sprite_collision = 1;
						}
						xxx = (xxx + 1) & 0x1ff;
						if (--maxpixels == 0)
							return;
					}
				}
			}
		}
		spritenum = link;
		maxsprites--;
	} while (maxsprites >= 0 && link != 0);
}

typedef struct { int code, row, xflip, colour, pri; uint32_t bits; } nametable_t;

static void get_nametable(const s315_5313_t *v, int tile_base, nametable_t *t, int vcolumn)
{
	const uint16_t d = VRAM(tile_base);
	t->xflip = (d & 0x0800) != 0;
	t->colour = (d & 0x6000) >> 13;
	t->pri = (d & 0x8000) >> 15;
	t->code = d & 0x7ff;
	t->row = (d & 0x1000) ? ((7 - vcolumn) & 7) : (vcolumn & 7);
	t->bits = tile_row(v, t->code, t->row);
}
static void draw_tile(const s315_5313_t *v, scratch_t *sc, const nametable_t *t, int start, int end, int *dpos, int is_fg)
{
	(void)v;
	int d = *dpos;
	if (start >= end)
		return;
	if (d < 0 || d + (end - start) > 320) {   // (never on the 320 columns; kept safe)
		*dpos += end - start;
		return;
	}
	const uint32_t bits = t->bits;
	const uint8_t col = (uint8_t)(t->colour << 4);
	for (int shift = start; shift < end; shift++, d++) {
		const int px = t->xflip ? 7 - shift : shift;
		const uint8_t dat = (uint8_t)((bits >> (28 - 4 * px)) & 15);
		if (!t->pri) {
			if (dat) s_video[d] = (uint32_t)(dat | col);
		} else if (is_fg) {
			if (dat) s_highpri[d] = (uint8_t)(dat | col | 0x80);
			else s_highpri[d] |= 0x80;
		} else
			s_highpri[d] = (uint8_t)(dat | col | 0x80);
	}
	*dpos = d;
}
static int vcolumn_tilebase(int *vcolumn, uint16_t base, int vscroll, int scanline, int vsize, int hsize, int hcolumn)
{
	*vcolumn = (vscroll + scanline) & (vsize * 8 - 1);
	return ((base >> 1) + (*vcolumn >> 3) * hsize + hcolumn) & 0x7fff;
}

static void render_planes(s315_5313_t *v, scratch_t *sc, int scanline)
{
	int hsize = 64, vsize = 64, numcolumns = 0, window_hsize = 0, window_vsize = 0;
	uint16_t base_w = 0;
	const int screenheight = REG01_240_LINE(v) ? 240 : 224;
	nametable_t tile;
	for (int x = 0; x < 320; x++)
		s_video[x] = (uint32_t)(REG07_BGCOLOUR(v) | 0x20000);
	memset(s_highpri, 0, sizeof(s_highpri));
	if (!REG01_DISP_ENABLE(v) || REG0_DISPLAY_DISABLE(v))
		return;
	const uint16_t base_a = (uint16_t)(REG02_PATTERN_ADDR_A(v) << 13), base_b = (uint16_t)(REG04_PATTERN_ADDR_B(v) << 13);
	const int size = REG10_HSCROLL_SIZE(v) | (REG10_VSCROLL_SIZE(v) << 4);
	if (get_hres(v) < 2) {
		numcolumns = 32; window_hsize = 32; window_vsize = 32;
		base_w = (uint16_t)((REG03_PATTERN_ADDR_W(v) & 0x1f) << 11);
	} else {
		numcolumns = 40; window_hsize = 64; window_vsize = 32;
		base_w = (uint16_t)((REG03_PATTERN_ADDR_W(v) & 0x1e) << 11);
	}
	int window_firstcol, window_lastcol, non_window_firstcol, non_window_lastcol, window_firstline, window_lastline;
	int window_is_bugged = 0;
	if (REG11_WINDOW_RIGHT(v)) {
		window_firstcol = REG11_WINDOW_HPOS(v) * 16;
		window_lastcol = numcolumns * 8;
		if (window_firstcol > window_lastcol) window_firstcol = window_lastcol;
		non_window_firstcol = 0;
		non_window_lastcol = window_firstcol;
	} else {
		window_firstcol = 0;
		window_lastcol = REG11_WINDOW_HPOS(v) * 16;
		if (window_lastcol > numcolumns * 8) window_lastcol = numcolumns * 8;
		non_window_firstcol = window_lastcol;
		non_window_lastcol = numcolumns * 8;
		if (window_lastcol != 0) window_is_bugged = 1;
	}
	if (REG12_WINDOW_DOWN(v)) {
		window_firstline = REG12_WINDOW_VPOS(v) * 8;
		window_lastline = screenheight;
		if (window_firstline > screenheight) window_firstline = screenheight;
	} else {
		window_firstline = 0;
		window_lastline = REG12_WINDOW_VPOS(v) * 8;
		if (window_lastline > screenheight) window_lastline = screenheight;
	}
	if (scanline >= window_firstline && scanline < window_lastline) {
		window_firstcol = 0; window_lastcol = numcolumns * 8;
		non_window_firstcol = 0; non_window_lastcol = 0;
	}
	const uint16_t hscroll_base = (uint16_t)(REG0D_HSCROLL_ADDR(v) << 10);
	switch (size) {
	case 0x00: hsize = 32; vsize = 32; break;
	case 0x01: hsize = 64; vsize = 32; break;
	case 0x03: hsize = 128; vsize = 32; break;
	case 0x10: hsize = 32; vsize = 64; break;
	case 0x11: hsize = 64; vsize = 64; break;
	case 0x30: hsize = 32; vsize = 128; break;
	}
	int hscroll_a = 0, hscroll_b = 0;
	switch (REG0B_HSCROLL_MODE(v)) {
	case 0: hscroll_a = VRAM((hscroll_base >> 1) + 0); hscroll_b = VRAM((hscroll_base >> 1) + 1); break;
	case 1: hscroll_a = VRAM((hscroll_base >> 1) + 0 + (scanline & 7) * 2); hscroll_b = VRAM((hscroll_base >> 1) + 1 + (scanline & 7) * 2); break;
	case 2: hscroll_a = VRAM((hscroll_base >> 1) + 0 + (scanline & ~7) * 2); hscroll_b = VRAM((hscroll_base >> 1) + 1 + (scanline & ~7) * 2); break;
	case 3: hscroll_a = VRAM((hscroll_base >> 1) + 0 + scanline * 2); hscroll_b = VRAM((hscroll_base >> 1) + 1 + scanline * 2); break;
	}
	const int vmode = REG0B_VSCROLL_MODE(v);
	// low priority B
	for (int column = 0; column < numcolumns / 2; column++) {
		int vcolumn, dpos = column * 16, tb, vs;
		const int part = 8 - (hscroll_b & 7);
		vs = vmode ? ((hscroll_b & 0xf) ? v->vsram[((column - 1) * 2 + 1) & 0x3f] : v->vsram[(column * 2 + 1) & 0x3f]) : v->vsram[1];
		tb = vcolumn_tilebase(&vcolumn, base_b, vs, scanline, vsize, hsize, ((column * 2 - 1) - (hscroll_b >> 3)) & (hsize - 1));
		get_nametable(v, tb, &tile, vcolumn);
		draw_tile(v, sc, &tile, part, 8, &dpos, 0);
		tb = vcolumn_tilebase(&vcolumn, base_b, vs, scanline, vsize, hsize, ((column * 2) - (hscroll_b >> 3)) & (hsize - 1));
		get_nametable(v, tb, &tile, vcolumn);
		draw_tile(v, sc, &tile, 0, 8, &dpos, 0);
		vs = vmode ? v->vsram[(column * 2 + 1) & 0x3f] : v->vsram[1];
		tb = vcolumn_tilebase(&vcolumn, base_b, vs, scanline, vsize, hsize, ((column * 2 + 1) - (hscroll_b >> 3)) & (hsize - 1));
		get_nametable(v, tb, &tile, vcolumn);
		draw_tile(v, sc, &tile, 0, part, &dpos, 0);
	}
	// A and the window
	for (int column = window_firstcol / 16; column < window_lastcol / 16; column++) {
		const int vcolumn = scanline & (window_vsize * 8 - 1);
		int dpos = column * 16;
		int tb = ((base_w >> 1) + (vcolumn >> 3) * window_hsize + ((column * 2) & (window_hsize - 1))) & 0x7fff;
		get_nametable(v, tb, &tile, vcolumn);
		draw_tile(v, sc, &tile, 0, 8, &dpos, 1);
		tb = ((base_w >> 1) + (vcolumn >> 3) * window_hsize + ((column * 2 + 1) & (window_hsize - 1))) & 0x7fff;
		get_nametable(v, tb, &tile, vcolumn);
		draw_tile(v, sc, &tile, 0, 8, &dpos, 1);
	}
	for (int column = non_window_firstcol / 16; column < non_window_lastcol / 16; column++) {
		int vcolumn, dpos = column * 16, tb, vs, hcolumn;
		const int part = 8 - (hscroll_a & 7);
		const int plain = !window_is_bugged || (hscroll_a & 0xf) == 0 || column > non_window_firstcol / 16;
		vs = vmode ? ((hscroll_a & 0xf) ? v->vsram[((column - 1) * 2 + 0) & 0x3f] : v->vsram[(column * 2 + 0) & 0x3f]) : v->vsram[0];
		hcolumn = plain ? ((column * 2 - 1) - (hscroll_a >> 3)) & (hsize - 1) : ((column * 2 + 1) - (hscroll_a >> 3)) & (hsize - 1);
		tb = vcolumn_tilebase(&vcolumn, base_a, vs, scanline, vsize, hsize, hcolumn);
		get_nametable(v, tb, &tile, vcolumn);
		draw_tile(v, sc, &tile, part, 8, &dpos, 1);
		if (plain)
			hcolumn = ((column * 2) - (hscroll_a >> 3)) & (hsize - 1);
		else
			hcolumn = ((hscroll_a & 0xf) < 8) ? ((column * 2) - (hscroll_a >> 3)) & (hsize - 1) : ((column * 2 + 2) - (hscroll_a >> 3)) & (hsize - 1);
		tb = vcolumn_tilebase(&vcolumn, base_a, vs, scanline, vsize, hsize, hcolumn);
		get_nametable(v, tb, &tile, vcolumn);
		draw_tile(v, sc, &tile, 0, 8, &dpos, 1);
		vs = vmode ? v->vsram[(column * 2 + 0) & 0x3f] : v->vsram[0];
		hcolumn = ((column * 2 + 1) - (hscroll_a >> 3)) & (hsize - 1);
		tb = vcolumn_tilebase(&vcolumn, base_a, vs, scanline, vsize, hsize, hcolumn);
		get_nametable(v, tb, &tile, vcolumn);
		draw_tile(v, sc, &tile, 0, part, &dpos, 1);
	}
	const int sh = REG0C_SHADOW_HIGLIGHT(v);
	for (int x = 0; x < 320; x++) {   // low priority sprites
		const uint8_t s = s_sprite_line[x + 128];
		if (!(s & 0x40))
			continue;
		if (!sh)
			s_video[x] = (uint32_t)((s & 0x3f) | 0x10000);
		else {
			const uint8_t d = s & 0x3f;
			if (d == 0x0e || d == 0x1e || d == 0x2e) s_video[x] = (uint32_t)(d | 0x4000 | 0x10000);
			else if (d == 0x3e) s_video[x] |= 0x8000;
			else if (d == 0x3f) s_video[x] |= 0x2000;
			else s_video[x] = (uint32_t)(d | 0x10000);
		}
	}
	for (int x = 0; x < 320; x++) {   // high priority A + B
		const uint8_t d = s_highpri[x];
		if (!(d & 0x80))
			continue;
		if (!sh) {
			if (d & 0x0f) s_video[x] = d & 0x3f;
		} else {
			if (d & 0x0f) s_video[x] = (uint32_t)((d & 0x3f) | 0x4000);
			else s_video[x] |= 0x4000;
		}
	}
	for (int x = 0; x < 320; x++) {   // high priority sprites
		const uint8_t s = s_sprite_line[x + 128];
		if (!(s & 0x80))
			continue;
		if (!sh)
			s_video[x] = (uint32_t)((s & 0x3f) | 0x10000);
		else {
			const uint8_t d = s & 0x3f;
			if (d == 0x3e) s_video[x] |= 0x8000;
			else if (d == 0x3f) s_video[x] |= 0x2000;
			else s_video[x] = (uint32_t)(d | 0x4000 | 0x10000);
		}
	}
}

int s315_5313_render_line(s315_5313_t *v, int vpos, uint16_t *raw)
{
	if (vpos < 0 || vpos >= v->visible_scanlines)
		return 0;
	scratch_t scratch, *sc = &scratch;
	render_sprites(v, sc, vpos);
	render_planes(v, sc, vpos);
	const int sh = REG0C_SHADOW_HIGLIGHT(v);
	for (int x = 0; x < 320; x++) {
		const uint32_t dat = s_video[x];
		uint16_t r = (dat & 0x20000) ? 0x000 : 0x100;
		if (!sh)
			r |= (uint16_t)((dat & 0x3f) | ((dat & 0x10000) ? 0x80 : 0x40));
		else
			switch (dat & 0x1e000) {
			case 0x00000: case 0x02000: case 0x06000: case 0x10000: case 0x12000: case 0x16000:
				r |= (uint16_t)(dat & 0x3f); break;
			case 0x04000: case 0x08000:
				r |= (uint16_t)((dat & 0x3f) | 0x40); break;
			case 0x14000: case 0x18000:
				r |= (uint16_t)((dat & 0x3f) | 0x80); break;
			case 0x0c000: case 0x1c000:
				r |= (uint16_t)((dat & 0x3f) | 0xc0); break;
			default: break;   // (MAME: random)
			}
		raw[x] = r;
	}
	return 1;
}
