/*  Data East MLC ("MotherLess Cassette"): Stadium Hero '96, Skull Fang, Hoops / Dunk Dream '95, Acchi Muite Hoi.
    Ported from MAME src/mame/dataeast/deco_mlc.cpp, deco_mlc_v.cpp (BSD-3-Clause, copyright-holders: Bryan McPhail).
    The 156 encrypted ARM (7 MHz, run at twice that as MAME's clock scale), a raster interrupt at a line the program
    sets, a YMZ280B (14 MHz), 93C46 EEPROM, Stadium Hero '96: the 146 protection. No tilemaps: the sprite chip draws
    blocks of tiles (sizes, zoom, a lookup ROM or RAM for the tiles, 8bpp from two blocks, shadow / highlight) a line
    at a time, from the clip windows and raster registers of that line. Avengers in Galactic Storm (SH-2) is not here.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/arm/arm.h"
#include "../../sound/ymz280b.h"
#include "../../machine/eeprom93c46.h"
#include "../../video/gfxlayout.h"
#include "../machine/deco156.h"
#include "../machine/deco146.h"
#include "../machine/armpoll.h"
#include "../../video/decoband.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CPU_CLOCK 14000000   // 42 MHz / 6, clock scale 2.0 (MAME init_mlc)
#define YMZ_CLOCK 14000000   // 42 MHz / 3
#define FRAME_MHZ 58000      // set_refresh_hz(58)
#define LINES 256            // set_size(40*8, 32*8)
#define VIS_Y0 8             // set_visarea(0, 319, 8, 247)
#define VIS_H 240
#define VIS_W 320
#define SPR_WORDS (0x3000 / 4)
#define BANDS 4

static int s_bpp, s_146, s_mono;          // the gfx layout's bits (4 / 5 / 6), Stadium Hero '96's 146, mono sound
static int s_gran, s_colors;              // the gfxdecode entry's granularity, colours
static uint32_t s_colour_mask, s_shadow_mask, s_shadow_shift;
static arm_t s_cpu;
static uint8_t *s_rom, *s_gfx2;
static uint32_t s_rom_size, s_gfx2_size;
static uint8_t (*s_ram_hp)[0x20000];            // 0x100000 (32-bit)
#define s_ram (*s_ram_hp)
static uint32_t s_irq_ram[0x20];          // 0x200000
static uint32_t s_clip_ram[0x20];         // 0x200080
static uint16_t s_spriteram[SPR_WORDS], s_spriteram_buf[SPR_WORDS];
static uint32_t (*s_vram_hp)[0x8000];           // 0x280000
#define s_vram (*s_vram_hp)
static uint16_t s_pal[0x800];             // 0x300000 (xBGR_555, the low halves)
static uint32_t s_vbl_i;
static int s_irq_target = -1;             // the raster interrupt's line (-1: none)
static rbx_gfx s_gfx;
static deco146_t s_prot;
static eeprom93c46_t s_eeprom;
static char s_nv_path[256];
static int s_port_in, s_port_in2, s_port_in3;
static int s_line;
static uint32_t s_frame;
static uint8_t s_volume;
static uint64_t s_cpu_frac, s_snd_frac;
static armpoll_t s_poll;
// the work RAM word each program's main loop polls for the raster interrupt's flag (0: none)
typedef struct { const char *set; uint32_t addr; } mlc_poll_t;
static const mlc_poll_t s_polls[] = {
	{ "stadhr96", 0x11573c }, { "stadhr96u", 0x11573c }, { "stadhr96j", 0x11573c },
	{ "skullfng", 0x100018 }, { "skullfngj", 0x100018 }, { "skullfnga", 0x100018 },
	{ "hoops96", 0x114c28 }, { "hoops95", 0x114c28 }, { "ddream95", 0x114c30 }, { "acchi", 0x100010 },
};
static uint32_t s_poll_addr;
static ymz280b_t s_ymz;
static uint32_t s_out_rate;
static int32_t *s_ymz_l, *s_ymz_r;
static int s_ymz_cap, s_ymz_n;
static uint32_t s_ymz_line0, s_ymz_line_n;
static int64_t s_line_c0, s_line_cn;
// what each visible line drew with (MAME VIDEO_UPDATE_SCANLINE: the registers as the line starts)
static uint32_t s_lclip[VIS_H][0x20], s_lirq[VIS_H][0x10];
static struct {
	uint16_t spriteram[SPR_WORDS];
	uint32_t vram[0x8000];
	uint32_t pens[0x800];
	uint32_t clip[VIS_H][0x20], irq[VIS_H][0x10];
	uint32_t frame;
} *s_snap;
static int s_565;

static uint32_t rd32le(const uint8_t *p) { return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24; }

// ---- sound: the YMZ280B rendered up to the CPU's time before each access ----
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
	int64_t d = s_cpu.cycles - s_line_c0;
	if (d < 0) d = 0;
	if (d > s_line_cn) d = s_line_cn;
	ymz_to((int)(s_ymz_line0 + (s_line_cn ? (uint32_t)(d * s_ymz_line_n / s_line_cn) : 0)));
}

// ---- bus ----
static uint32_t inputs_r(void)
{
	rbx_port_custom(s_port_in, 0x00800000u, eeprom_do(&s_eeprom) ? 0x00800000u : 0);
	return rbx_port_read(s_port_in);
}

static void irq_ram_w(uint32_t o, uint32_t data, uint32_t mask)
{
	s_irq_ram[o] = (s_irq_ram[o] & ~mask) | (data & mask);
	if (o == 0x10 / 4)   // IRQ ack
		arm_set_irq(&s_cpu, 0);
	else if (o == 0x14 / 4)   // the raster interrupt's line (from the next time the beam gets there)
		s_irq_target = s_irq_ram[o] == 0xffffffffu || s_irq_ram[o] >= LINES ? -1 : (int)s_irq_ram[o];
}

static uint32_t bus_r32(void *ctx, uint32_t a)
{
	(void)ctx;
	a &= 0x03fffffc;
	if (a < 0x100000)
		return rd32le(s_rom + (a & (s_rom_size - 1)));
	if (a < 0x120000) {
		const uint32_t v = rd32le(s_ram + (a & 0x1fffc));
		if (a == s_poll_addr)   // (the main loop's wait for the interrupt's flag: its RAM page comes here for it)
			armpoll_read(&s_poll, &s_cpu, v, s_rom, s_rom_size);
		return v;
	}
	if (a >= 0x200000 && a < 0x200080) {
		switch (a) {
		case 0x200070:   // (the vblank toggle: each read flips it)
			s_vbl_i ^= 0xffffffffu;
			return s_vbl_i;
		case 0x200074: return (uint32_t)s_line;   // scanline
		case 0x200000: case 0x200004: case 0x20007c: return 0xffffffffu;
		}
		return 0;
	}
	if (a >= 0x200080 && a < 0x200100) return s_clip_ram[(a - 0x200080) >> 2];
	if (a >= 0x204000 && a < 0x207000) return 0xffff0000u | s_spriteram[(a - 0x204000) >> 2];
	if (a >= 0x280000 && a < 0x2a0000) return s_vram[(a - 0x280000) >> 2];
	if (a >= 0x300000 && a < 0x308000) return s_pal[((a - 0x300000) >> 2) & 0x7ff];
	if (a == 0x400000) return inputs_r();
	if (a == 0x440000) return rbx_port_read(s_port_in2);
	if (a == 0x440004) return rbx_port_read(s_port_in3);
	if (a == 0x440008 || a == 0x44001c) return 0xffffffffu;
	if (a >= 0x600000 && a < 0x600008) {   // (umask32 0xff000000)
		ymz_sync();
		return (uint32_t)ymz280b_read(&s_ymz, (int)((a - 0x600000) >> 2)) << 24;
	}
	if (s_146 && a >= 0x70f000 && a < 0x710000)   // (umask32 0xffff0000; the address bitswap keeps the low bits)
		return (uint32_t)deco146_read(&s_prot, (uint16_t)(((a - 0x70f000) >> 2) * 2), NULL) << 16;
	return 0;
}

static void bus_w32m(uint32_t a, uint32_t data, uint32_t mask)
{
	a &= 0x03fffffc;
	if (a >= 0x100000 && a < 0x120000) {
		uint8_t *p = s_ram + (a & 0x1fffc);
		for (int i = 0; i < 4; i++)
			if (mask & (0xffu << (8 * i)))
				p[i] = (uint8_t)(data >> (8 * i));
		return;
	}
	if (a >= 0x200000 && a < 0x200080) { irq_ram_w((a - 0x200000) >> 2, data, mask); return; }
	if (a >= 0x200080 && a < 0x200100) {
		uint32_t *p = &s_clip_ram[(a - 0x200080) >> 2];
		*p = (*p & ~mask) | (data & mask);
		return;
	}
	if (a >= 0x204000 && a < 0x207000) {
		if (mask & 0xffff) {
			uint16_t *p = &s_spriteram[(a - 0x204000) >> 2];
			*p = (uint16_t)((*p & ~mask) | (data & mask & 0xffff));
		}
		return;
	}
	if (a >= 0x280000 && a < 0x2a0000) {
		uint32_t *p = &s_vram[(a - 0x280000) >> 2];
		*p = (*p & ~mask) | (data & mask);
		return;
	}
	if (a >= 0x300000 && a < 0x308000) {   // palette write16 (umask32 0x0000ffff)
		if (mask & 0xffff) {
			uint16_t *p = &s_pal[((a - 0x300000) >> 2) & 0x7ff];
			*p = (uint16_t)((*p & ~mask) | (data & mask & 0xffff));
		}
		return;
	}
	if (a == 0x500000) {   // eeprom_w: bits 8-15 the EEPROM lines, else bits 0-7 the master volume
		if (mask & 0xff00) {
			const uint32_t e = (data >> 8) & 0xff;
			eeprom_write_lines(&s_eeprom, (e >> 2) & 1, (e >> 1) & 1, e & 1);
		} else if (mask & 0xff)
			s_volume = (uint8_t)data;
		return;
	}
	if (a >= 0x600000 && a < 0x600008) {
		if (mask & 0xff000000u) {
			ymz_sync();
			ymz280b_write(&s_ymz, (int)((a - 0x600000) >> 2), (uint8_t)(data >> 24));
		}
		return;
	}
	if (s_146 && a >= 0x70f000 && a < 0x710000 && (mask & 0xffff0000u))
		deco146_write(&s_prot, (uint16_t)(((a - 0x70f000) >> 2) * 2), (uint16_t)(data >> 16), (uint16_t)(mask >> 16), NULL);
}

static void bus_w32(void *ctx, uint32_t a, uint32_t data) { (void)ctx; bus_w32m(a, data, 0xffffffffu); }
static uint8_t bus_r8(void *ctx, uint32_t a) { return (uint8_t)(bus_r32(ctx, a) >> (8 * (a & 3))); }
static void bus_w8(void *ctx, uint32_t a, uint8_t v)
{
	(void)ctx;
	const int sh = 8 * (int)(a & 3);
	bus_w32m(a, (uint32_t)v << sh, 0xffu << sh);
}

// ---- video (MAME deco_mlc_v.cpp) ----
typedef struct { int left, right; } xclip_t;

// one block of the sprite list as the frame's lines draw it: what does not depend on the line, worked out once a frame
typedef struct {
	int x, y, fx, fy, w, h, xoffs, yoffs, xscale, yscale, yend;
	int sprite, sprite2, use8bpp, block_is_index, indices_in_rom, hibits, tile_format;
	int clipper, raster_select, shadow_mode;
	uint32_t color;
} mlc_block_t;
static mlc_block_t (*s_blocks_hp)[SPR_WORDS / 8];
#define s_blocks (*s_blocks_hp)
static int s_nblocks;
static uint32_t s_dx[1024];   // (16 << 16) / width (the ARM of the Vita has no divide instruction)
static uint32_t s_count_mask; // the elements' count - 1 when a power of two, else 0

static inline uint32_t elem(uint32_t code) { return s_count_mask ? code & s_count_mask : code % s_gfx.count; }

static void draw_zoom_line(uint32_t *dest, uint8_t *pri, xclip_t clip, uint32_t code1, uint32_t code2, uint32_t color,
                           int flipx, int sx, int use8bpp, int scalex, int srcline, int shadow_mode, uint32_t alpha_mode)
{
	const int sprite_screen_width = (scalex * 16 + (sx & 0xffff)) >> 16;
	sx >>= 16;
	if (sprite_screen_width <= 0)
		return;
	int ex = sx + sprite_screen_width;
	if (ex > clip.right + 1)
		ex = clip.right + 1;
	const int sx0 = sx < clip.left ? clip.left : sx;
	if (ex <= sx0)
		return;
	int dx = sprite_screen_width < 1024 ? (int)s_dx[sprite_screen_width] : (16 << 16) / sprite_screen_width;
	int x_index_base = 0;
	if (flipx) {
		x_index_base = (sprite_screen_width - 1) * dx;
		dx = -dx;
	}
	x_index_base += (sx0 - sx) * dx;
	sx = sx0;
	const uint32_t *pal = s_snap->pens;
	const uint32_t pbase = (uint32_t)s_gran * (color & (uint32_t)(s_colors - 1));
	const uint8_t *src1 = s_gfx.pix + (size_t)elem(code1) * 256 + (size_t)srcline * 16;
	const uint8_t *src2 = s_gfx.pix + (size_t)elem(code2) * 256 + (size_t)srcline * 16;
	int x_index = x_index_base;
	if (shadow_mode && (alpha_mode & 0xc0)) {   // shadow / highlight: only marked
		for (int x = sx; x < ex; x++, x_index += dx)
			if (!(pri[x] & 0x80)) {
				int c = src1[x_index >> 16];
				if (use8bpp)
					c = (c << 4) | src2[x_index >> 16];
				if (c)
					pri[x] |= (uint8_t)shadow_mode;
			}
	} else if (!use8bpp)
		for (int x = sx; x < ex; x++, x_index += dx) {
			const int c = src1[x_index >> 16];
			if (c && !(pri[x] & 0x80)) {
				dest[x] = pal[(pbase + (uint32_t)c) & 0x7ff];
				pri[x] |= (uint8_t)(0x80 | shadow_mode);
			}
		}
	else
		for (int x = sx; x < ex; x++, x_index += dx)
			if (!(pri[x] & 0x80)) {
				const int c = (src1[x_index >> 16] << 4) | src2[x_index >> 16];
				if (c) {
					dest[x] = pal[(pbase + (uint32_t)c) & 0x7ff];
					pri[x] |= (uint8_t)(0x80 | shadow_mode);
				}
			}
}

// the frame's blocks from the sprite list (MAME draw_sprites' work that does not depend on the scanline)
static void mlc_blocks(void)
{
	const uint16_t *spriteram = s_snap->spriteram;
	const uint32_t *vram = s_snap->vram;
	const uint32_t frame = s_snap->frame;
	const uint32_t g2mask = s_gfx2_size - 1;
	int hibits = 0;   // (kept from the last RAM block, as MAME's)
	s_nblocks = 0;
	for (int offs = 0; offs < SPR_WORDS; offs += 8) {
		const int use8bpp = (offs + 8 < SPR_WORDS) && (spriteram[offs + 8 + 1] & 0x1000) && (spriteram[offs + 8 + 0] & 0x8000);
		if (use8bpp)
			offs += 8;
		if (!(spriteram[offs + 0] & 0x8000))
			continue;
		if ((spriteram[offs + 1] & 0x2000) && (frame & 1))
			continue;
		mlc_block_t *b = &s_blocks[s_nblocks];
		int y = spriteram[offs + 2] & 0x7ff, x = spriteram[offs + 3] & 0x7ff;
		if (x & 0x400) x = -(0x400 - (x & 0x3ff));
		if (y & 0x400) y = -(0x400 - (y & 0x3ff));
		b->x = x;
		b->y = y;
		b->fx = spriteram[offs + 1] & 0x8000;
		b->fy = spriteram[offs + 1] & 0x4000;
		uint32_t color = spriteram[offs + 1] & 0xff;
		const int raster_mode = (spriteram[offs + 1] >> 10) & 1;
		b->raster_select = raster_mode ? (spriteram[offs + 1] & 0x0180) >> 7 : 0;
		int clipper = (spriteram[offs + 1] >> 8) & 3;
		const int indx = spriteram[offs + 0] & 0x7fff;
		b->yscale = spriteram[offs + 4] & 0x3ff;
		b->xscale = spriteram[offs + 5] & 0x3ff;
		clipper = ((clipper & 2) >> 1) | ((clipper & 1) << 1);
		if ((spriteram[offs + 1] >> 10) & 2)
			clipper |= 4;
		b->clipper = clipper;
		b->shadow_mode = (int)((color & s_shadow_mask) >> s_shadow_shift);
		color &= s_colour_mask;
		if (use8bpp)
			color = spriteram[offs + 1 - 8] & 0x7f;
		b->color = color;
		int h, w, fx1, fy1;
		if (indx & 0x4000) {
			const uint8_t *p8 = s_gfx2 + (((uint32_t)indx * 8) & g2mask);
			if ((p8[5] & 1) && (frame & 1))
				continue;
			h = p8[1] & 0xf;
			w = p8[3] & 0xf;
			b->sprite = (p8[7] << 8) | p8[6] | ((p8[4] & 3) << 16);
			b->yoffs = p8[0];
			b->xoffs = p8[2];
			fy1 = (p8[1] & 0x10) >> 4;
			fx1 = (p8[3] & 0x10) >> 4;
			b->tile_format = p8[4] & 0x80;
			b->block_is_index = (p8[4] & 0xc0) != 0;
			b->indices_in_rom = 0;
		} else {
			const uint32_t *ip = vram + (indx & 0x1fff) * 4;
			if ((ip[2] & 0x0100) && (frame & 1))
				continue;
			h = (ip[0] >> 8) & 0xf;
			w = (ip[1] >> 8) & 0xf;
			b->sprite = (int)(((ip[2] & 3) << 16) | (ip[3] & 0xffff));
			b->block_is_index = (ip[2] & 0xc0) != 0;
			b->tile_format = (int)(ip[2] & 0x80);
			hibits = (int)((ip[2] & 0x3c) << 10);
			b->indices_in_rom = (int)(ip[2] & 3);
			b->yoffs = (int)(ip[0] & 0xff);
			b->xoffs = (int)(ip[1] & 0xff);
			fy1 = (int)((ip[0] & 0x1000) >> 12);
			fx1 = (int)((ip[1] & 0x1000) >> 12);
		}
		b->hibits = hibits;
		b->h = h ? h : 16;
		b->w = w ? w : 16;
		b->sprite2 = 0;
		if (use8bpp) {
			const int i2 = spriteram[offs + 0 - 8] & 0x7fff;
			if (i2 & 0x4000) {
				const uint8_t *p8 = s_gfx2 + (((uint32_t)i2 * 8) & g2mask);
				b->sprite2 = (p8[7] << 8) | p8[6];
			} else {
				const uint32_t *ip = vram + (i2 & 0x1fff) * 4;
				b->sprite2 = (int)(((ip[2] & 3) << 16) | (ip[3] & 0xffff));
			}
		}
		b->use8bpp = use8bpp;
		if (fx1) b->fx ^= 0x8000;
		if (fy1) b->fy ^= 0x4000;
		if (!b->yscale)   // (ratio 0: MAME skips the block)
			continue;
		// the lines below the block's top it covers: (bby << 8) / yscale < h * 16
		b->yend = (b->h * 16 * b->yscale + 255) >> 8;
		s_nblocks++;
	}
}

static void draw_sprites(int scanline, const uint32_t *clip_ram, const uint32_t *irq_ram, uint32_t *dest, uint8_t *pri)
{
	const uint32_t *vram = s_snap->vram;
	const uint32_t alpha_mode = irq_ram[0x04 / 4] & 0xc0;
	const uint32_t g2mask = s_gfx2_size - 1;
	for (int n = 0; n < s_nblocks; n++) {
		const mlc_block_t *b = &s_blocks[n];
		const int min_y = (int)clip_ram[b->clipper * 4 + 0], max_y = (int)clip_ram[b->clipper * 4 + 1];
		if (scanline < min_y || scanline > max_y)
			continue;
		int x = b->x, y = b->y, extra_x_scale = 0x100;
		if (b->raster_select) {   // the raster registers 6 / 9 / 12
			const int base = b->raster_select == 1 ? 6 : b->raster_select == 2 ? 9 : 12;
			int extra_y_off = (int)(irq_ram[base + 0] & 0x7ff), extra_x_off = (int)(irq_ram[base + 1] & 0x7ff);
			extra_x_scale = (int)(irq_ram[base + 2] & 0x3ff);
			if (extra_x_off & 0x400) extra_x_off -= 0x800;
			if (extra_y_off & 0x400) extra_y_off -= 0x800;
			x += extra_x_off;
			y += extra_y_off;
		}
		const int h = b->h, w = b->w, fx = b->fx, fy = b->fy;
		const int yscale_frac = b->yscale << 8;
		int ybase = y << 16;
		if (fy)
			ybase += (b->yoffs - 15) * yscale_frac - ((h - 1) * yscale_frac * 16);
		else
			ybase -= b->yoffs * yscale_frac;
		const int bby = scanline - (ybase >> 16);
		if (bby < 0 || bby >= b->yend)
			continue;
		int srcline = (bby << 16) / yscale_frac;
		const int by = srcline >> 4;
		if (by >= h)
			continue;
		xclip_t uc = { (int)clip_ram[b->clipper * 4 + 2], (int)clip_ram[b->clipper * 4 + 3] };
		if (uc.left < 0) uc.left = 0;
		if (uc.right > VIS_W - 1) uc.right = VIS_W - 1;
		srcline &= 0xf;
		if (fy)
			srcline = 15 - srcline;
		const int xscale = b->xscale * extra_x_scale, xinc = xscale * 16;
		int xbase = x << 16;
		if (fx)
			xbase += (b->xoffs - 15) * xscale - ((w - 1) * xinc);
		else
			xbase -= b->xoffs * xscale;
		if (!xscale)
			continue;
		const int row = fy ? h - 1 - by : by;
		for (int bx = 0; bx < w; bx++) {
			const int realxbase = xbase + bx * xinc;
			if ((realxbase >> 16) > uc.right)   // (a tile right of the window: the rest too as x grows)
				break;
			const int count = row * w + (fx ? w - 1 - bx : bx);
			int tile = b->sprite + count, tile2 = b->sprite2 + count, color_offset = 0;
			if (b->block_is_index) {
				const int hibits = b->hibits;
				if (b->indices_in_rom) {
					const uint8_t *ptr = s_gfx2 + (((uint32_t)tile * 2) & g2mask);
					tile = ptr[0] | (ptr[1] << 8);
					if (b->use8bpp) {
						const uint8_t *ptr2 = s_gfx2 + (((uint32_t)tile2 * 2) & g2mask);
						tile2 = ptr2[0] | (ptr2[1] << 8);
					} else
						tile2 = 0;
					if (b->tile_format) {
						color_offset = (tile & 0xf000) >> 12;
						tile = (tile & 0x0fff) | hibits;
						tile2 = (tile2 & 0x0fff) | hibits;
					} else {
						tile = (tile & 0xffff) | (hibits << 2);
						tile2 = (tile2 & 0xffff) | (hibits << 2);
					}
				} else {
					tile = (int)(vram[tile & 0x7fff] & 0xffff);
					if (b->tile_format) {
						color_offset = (tile & 0xf000) >> 12;
						tile = (tile & 0x0fff) | hibits;
					} else
						tile = (tile & 0xffff) | (hibits << 2);
					tile2 = 0;
				}
			}
			draw_zoom_line(dest, pri, uc, (uint32_t)tile, (uint32_t)tile2, b->color + (uint32_t)color_offset, fx != 0,
			               realxbase, b->use8bpp, xscale, srcline, b->shadow_mode, alpha_mode);
		}
	}
}

static void mlc_band(int band)
{
	const int r0 = VIS_H * band / BANDS, r1 = VIS_H * (band + 1) / BANDS;
	for (int y = r0; y < r1; y++) {
		uint32_t d[VIS_W];
		uint8_t pri[VIS_W];
		const uint32_t bg = s_snap->pens[0];   // (pen 0: Skull Fang level 2)
		for (int x = 0; x < VIS_W; x++)
			d[x] = bg;
		memset(pri, 0, sizeof(pri));
		draw_sprites(VIS_Y0 + y, s_snap->clip[y], s_snap->irq[y], d, pri);
		for (int x = 0; x < VIS_W; x++) {
			const uint8_t f = pri[x] & 3;
			if (f == 1)        // shadow
				d[x] = (d[x] & 0xfefefe) >> 1;
			else if (f == 2)   // highlight
				d[x] = ((d[x] & 0xfefefe) >> 1) | 0x808080;
		}
		if (rbx_video.out) {
			uint8_t *o = (uint8_t *)rbx_video.out + (size_t)y * rbx_video.out_pitch;
			if (s_565)
				for (int x = 0; x < VIS_W; x++)
					((uint16_t *)o)[x] = (uint16_t)((d[x] >> 8 & 0xf800) | (d[x] >> 5 & 0x07e0) | (d[x] >> 3 & 0x001f));
			else if (rbx_video.out_bpp == 4)
				for (int x = 0; x < VIS_W; x++)
					((uint32_t *)o)[x] = rbx_video.out_color((int)(d[x] >> 16) & 0xff, (int)(d[x] >> 8) & 0xff, (int)d[x] & 0xff);
			else
				for (int x = 0; x < VIS_W; x++)
					((uint16_t *)o)[x] = (uint16_t)rbx_video.out_color((int)(d[x] >> 16) & 0xff, (int)(d[x] >> 8) & 0xff, (int)d[x] & 0xff);
		}
	}
}
static decoband_t s_bands = { 0, 0, 0, 0, BANDS, mlc_band };
static int mlc_help(int helper) { return decoband_help(&s_bands, helper); }

static void mlc_render(void)
{
	if (!rbx_video.out)
		return;
	s_565 = rbx_video.out_bpp == 2 && rbx_video.out_color(0xff, 0, 0) == 0xf800 && rbx_video.out_color(0, 0xff, 0) == 0x07e0 &&
	        rbx_video.out_color(0, 0, 0xff) == 0x001f && rbx_video.out_color(0x84, 0x82, 0x21) == 0x8404;
	mlc_blocks();
	decoband_run(&s_bands);
	rbx_video.out_done = 1;
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap->spriteram, s_spriteram_buf, sizeof(s_spriteram_buf));
	memcpy(s_snap->vram, s_vram, sizeof(s_vram));
	for (int i = 0; i < 0x800; i++) {   // xBGR_555
		const int p = s_pal[i];
		int r = p & 31, g = (p >> 5) & 31, b = (p >> 10) & 31;
		r = (r << 3) | (r >> 2);
		g = (g << 3) | (g >> 2);
		b = (b << 3) | (b >> 2);
		s_snap->pens[i] = (uint32_t)(r << 16 | g << 8 | b);
	}
	memcpy(s_snap->clip, s_lclip, sizeof(s_lclip));
	memcpy(s_snap->irq, s_lirq, sizeof(s_lirq));
	s_snap->frame = s_frame;
	rbx_video_frame_done();
	// screen_vblank_mlc (rising edge): the sprites double buffered
	memcpy(s_spriteram_buf, s_spriteram, sizeof(s_spriteram));
}

// ---- init / frame ----
static void irq_ack(void *ctx, int line)
{
	(void)ctx;
	if (line == ARM_IRQ_LINE)   // (HOLD_LINE)
		arm_set_irq(&s_cpu, 0);
}

static void mlc_reset(void)
{
	memset(s_ram, 0, sizeof(s_ram));
	memset(s_irq_ram, 0, sizeof(s_irq_ram));
	memset(s_clip_ram, 0, sizeof(s_clip_ram));
	memset(s_spriteram, 0, sizeof(s_spriteram));
	memset(s_spriteram_buf, 0, sizeof(s_spriteram_buf));
	memset(s_vram, 0, sizeof(s_vram));
	memset(s_pal, 0, sizeof(s_pal));
	memset(s_lclip, 0, sizeof(s_lclip));
	memset(s_lirq, 0, sizeof(s_lirq));
	s_vbl_i = 0xffffffffu;
	s_irq_target = -1;
	s_volume = 0;
	ymz280b_reset(&s_ymz);
	s_ymz_n = 0;
	if (s_146)
		deco146_reset(&s_prot);
	arm_reset(&s_cpu);
	armpoll_reset(&s_poll);
	s_line = 0;
	s_frame = 0;
	s_cpu_frac = s_snd_frac = 0;
}

static void mlc_exit(void);

// MAME descramble_sound (as hvysmsh's)
static int descramble(uint8_t *rom, uint32_t size)
{
	uint8_t *buf = (uint8_t *)malloc(size);
	if (!buf)
		return -1;
	for (uint32_t x = 0; x < size; x++) {
		const uint32_t addr = (x & 0xe00000) | ((x & 1) << 20) | ((x >> 1) & 0xfffff);
		if (addr < size)
			buf[addr] = rom[x];
	}
	memcpy(rom, buf, size);
	free(buf);
	return 0;
}

static int mlc_init(const rbx_game *g)
{
	if (!s_blocks_hp && !(s_blocks_hp = calloc(1, sizeof(*s_blocks_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	if (!s_ram_hp && !(s_ram_hp = calloc(1, sizeof(*s_ram_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	if (!s_vram_hp && !(s_vram_hp = calloc(1, sizeof(*s_vram_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	uint32_t gsize, ysize;
	s_rom = rbx_region("maincpu", &s_rom_size);
	uint8_t *gfx = rbx_region("gfx1", &gsize), *ymz = rbx_region("ymz", &ysize);
	s_gfx2 = rbx_region("gfx2", &s_gfx2_size);
	if (!s_rom || !gfx || !ymz || !s_gfx2) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	const int acchi = !strcmp(g->machine, "acchi");
	s_146 = !strcmp(g->machine, "stadhr96");
	s_mono = !strcmp(g->machine, "mlc_5bpp");
	s_bpp = acchi || s_mono ? 5 : (s_146 || !strcmp(g->machine, "mlc_6bpp")) ? 6 : 4;
	if (deco156_decrypt(s_rom, s_rom_size) || (!acchi && descramble(ymz, ysize))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	{	// spritelayout_4bpp / 5bpp / 6bpp (rows of 32 bits, x bits 15..0), acchi's (RGN_FRAC(1,5), rows of 16)
		static uint32_t xo[16], yo[16];
		for (int i = 0; i < 16; i++) {
			xo[i] = (uint32_t)(15 - i);
			yo[i] = (uint32_t)(i * (acchi ? 16 : 32));
		}
		uint32_t pl[6], frac;
		if (acchi) {
			frac = gsize / 5;
			for (int i = 0; i < 5; i++)
				pl[i] = (uint32_t)i * frac * 8;
		} else if (s_bpp == 4) {
			frac = gsize / 2;
			const uint32_t h = frac * 8;
			pl[0] = h + 16; pl[1] = h; pl[2] = 16; pl[3] = 0;
		} else {
			frac = gsize / 3;
			const uint32_t t = frac * 8;
			if (s_bpp == 5) {
				pl[0] = 2 * t; pl[1] = t + 16; pl[2] = t; pl[3] = 16; pl[4] = 0;
			} else {
				pl[0] = 2 * t + 16; pl[1] = 2 * t; pl[2] = t + 16; pl[3] = t; pl[4] = 16; pl[5] = 0;
			}
		}
		const rbx_gfx_layout l = { 16, 16, s_bpp, pl, xo, yo, acchi ? 16 * 16 : 16 * 32 };
		if (rbx_gfx_decode(&s_gfx, &l, gfx, frac)) {
			rbx_fail("%s: out of memory (graphics)", g->set);
			mlc_exit();
			return -1;
		}
	}
	s_count_mask = (s_gfx.count & (s_gfx.count - 1)) ? 0 : s_gfx.count - 1;
	for (int i = 1; i < 1024; i++)
		s_dx[i] = (16u << 16) / (uint32_t)i;
	// the gfxdecode entry: 4bpp 256 colours, 5bpp 128 (acchi 64), 6bpp 64; video_start's masks
	s_gran = 1 << s_bpp;
	s_colors = s_bpp == 4 ? 256 : s_bpp == 5 ? (acchi ? 64 : 128) : 64;
	{
		const uint32_t max_color = 0x800u / (uint32_t)s_gran;
		s_colour_mask = max_color - 1;
		s_shadow_mask = max_color | (max_color << 1);
		s_shadow_shift = 11 - (uint32_t)s_bpp;
	}
	s_snap = calloc(1, sizeof(*s_snap));
	if (!s_snap) {
		rbx_fail("%s: out of memory", g->set);
		mlc_exit();
		return -1;
	}
	ymz280b_init(&s_ymz, YMZ_CLOCK, ymz, ysize, NULL, NULL);
	s_out_rate = ymz280b_rate(&s_ymz);
	s_ymz_cap = (int)(s_out_rate / 50) + 64;
	s_ymz_l = (int32_t *)calloc((size_t)s_ymz_cap, sizeof(int32_t));
	s_ymz_r = (int32_t *)calloc((size_t)s_ymz_cap, sizeof(int32_t));
	if (!s_ymz_l || !s_ymz_r || rbx_audio_setup_stereo(s_out_rate) || rbx_video_setup(VIS_W, VIS_H, 0x800)) {
		mlc_exit();
		return -1;
	}
	if (s_146) {
		deco146_init(&s_prot, 0);
		s_prot.magic_read_xor_on = 1;
	}
	s_nv_path[0] = 0;
	eeprom_init(&s_eeprom);
	{	// (a region named as the device gives its first contents: big-endian 16-bit words)
		uint32_t es;
		const uint8_t *e = rbx_region("eeprom", &es);
		for (uint32_t i = 0; e && i < 64 && 2 * i + 1 < es; i++)
			s_eeprom.data[i] = (uint16_t)(e[2 * i] << 8 | e[2 * i + 1]);
	}
	{
		extern const char *rbx_nvram_dir(void);
		const char *dir = rbx_nvram_dir();
		if (dir) {
			snprintf(s_nv_path, sizeof(s_nv_path), "%s%s.nv", dir, g->set);
			eeprom_load(&s_eeprom, s_nv_path);
		}
	}
	s_port_in = rbx_port_find("INPUTS");
	s_port_in2 = rbx_port_find("INPUTS2");
	s_port_in3 = rbx_port_find("INPUTS3");
	arm_init(&s_cpu);
	for (uint32_t a = 0; a < 0x100000 && a < s_rom_size; a += 0x1000)
		s_cpu.rpage[a >> 12] = s_rom + a;
	for (uint32_t a = 0; a < 0x20000; a += 0x1000)
		s_cpu.rpage[(0x100000 + a) >> 12] = s_cpu.wpage[(0x100000 + a) >> 12] = s_ram + a;
	s_poll_addr = 0;
	for (size_t i = 0; i < sizeof(s_polls) / sizeof(s_polls[0]); i++)
		if (!strcmp(s_polls[i].set, g->set))
			s_poll_addr = s_polls[i].addr & ~3u;
#ifdef RBX_ARM_PCPROF
	if (getenv("RBX_NOPOLL"))   // (profiling builds: the results without the skip, to compare)
		s_poll_addr = 0;
#endif
	if (s_poll_addr)   // (reads of that page: the handler)
		s_cpu.rpage[s_poll_addr >> 12] = NULL;
	s_cpu.read32 = bus_r32;
	s_cpu.write32 = bus_w32;
	s_cpu.read8 = bus_r8;
	s_cpu.write8 = bus_w8;
	s_cpu.ctx = NULL;
	s_cpu.irq_ack = irq_ack;
	mlc_reset();
	return 0;
}

static void mlc_exit(void)
{
	if (s_nv_path[0] && s_eeprom.dirty)
		eeprom_save(&s_eeprom, s_nv_path);
	s_nv_path[0] = 0;
	rbx_gfx_free(&s_gfx);
	free(s_snap);
	s_snap = NULL;
	free(s_ymz_l);
	free(s_ymz_r);
	s_ymz_l = s_ymz_r = NULL;
	free(s_blocks_hp);
	s_blocks_hp = NULL;
	free(s_ram_hp);
	s_ram_hp = NULL;
	free(s_vram_hp);
	s_vram_hp = NULL;
}

static void mlc_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	for (int line = 0; line < LINES; line++) {
		s_line = line;
		if (line == s_irq_target) {   // the raster interrupt (one shot: the program sets it again)
			s_irq_target = -1;
			arm_set_irq(&s_cpu, 1);
		}
		if (line >= VIS_Y0 && line < VIS_Y0 + VIS_H) {
			memcpy(s_lclip[line - VIS_Y0], s_clip_ram, sizeof(s_clip_ram));
			memcpy(s_lirq[line - VIS_Y0], s_irq_ram, sizeof(s_lirq[0]));
		} else if (line == VIS_Y0 + VIS_H)
			vblank_capture();
		const int lc = rbx_line_cycles(&s_cpu_frac, CPU_CLOCK, FRAME_MHZ, LINES);
		const int64_t end = s_cpu.cycles + lc;
		s_poll.slice_end = end;
		s_snd_frac += (uint64_t)s_out_rate * 1000;
		const uint64_t den = (uint64_t)FRAME_MHZ * LINES;
		s_ymz_line0 = (uint32_t)s_ymz_n;
		s_ymz_line_n = (uint32_t)rbx_div64_small(s_snd_frac, den);
		s_snd_frac -= (uint64_t)s_ymz_line_n * den;
		s_line_c0 = s_cpu.cycles;
		s_line_cn = lc;
		arm_run(&s_cpu, end);
		arm_break = 0;
		ymz_to((int)(s_ymz_line0 + s_ymz_line_n));
	}
	// the master volume (MAME: (255 - data) / 255 on both outputs); hoops: both channels to both speakers
	const int32_t gain = 255 - s_volume;
	for (int i = 0; i < s_ymz_n; i++) {
		int32_t l = s_ymz_l[i], r = s_ymz_r[i];
		if (s_mono)
			l = r = l + r;
		rbx_audio_put_stereo(l * gain / 255, r * gain / 255);
	}
	memset(s_ymz_l, 0, sizeof(int32_t) * (size_t)s_ymz_n);
	memset(s_ymz_r, 0, sizeof(int32_t) * (size_t)s_ymz_n);
	s_ymz_n = 0;
	s_frame++;
}

static int mlc_state(void *ctx, int load)
{
	uint8_t cpu[ARM_STATE_SIZE];
	if (!load)
		memcpy(cpu, &s_cpu, ARM_STATE_SIZE);
	uint64_t fr[2] = { s_cpu_frac, s_snd_frac };
	uint32_t misc[4] = { s_frame, s_vbl_i, (uint32_t)s_irq_target, s_volume };
	int rc = 0;
	rc |= rbx_state_io(ctx, "arm", cpu, ARM_STATE_SIZE);
	rc |= rbx_state_io(ctx, "ram", s_ram, sizeof(s_ram));
	rc |= rbx_state_io(ctx, "irqram", s_irq_ram, sizeof(s_irq_ram));
	rc |= rbx_state_io(ctx, "clipram", s_clip_ram, sizeof(s_clip_ram));
	rc |= rbx_state_io(ctx, "spr", s_spriteram, sizeof(s_spriteram));
	rc |= rbx_state_io(ctx, "sprb", s_spriteram_buf, sizeof(s_spriteram_buf));
	rc |= rbx_state_io(ctx, "vram", s_vram, sizeof(s_vram));
	rc |= rbx_state_io(ctx, "pal", s_pal, sizeof(s_pal));
	rc |= rbx_state_io(ctx, "eeprom", &s_eeprom, sizeof(s_eeprom));
	rc |= rbx_state_io(ctx, "ymz", &s_ymz, YMZ280B_STATE_SIZE);
	if (s_146)
		rc |= rbx_state_io(ctx, "prot", &s_prot, DECO146_STATE_SIZE);
	rc |= rbx_state_io(ctx, "frac", fr, sizeof(fr));
	rc |= rbx_state_io(ctx, "misc", misc, sizeof(misc));
	if (rc)
		return -1;
	if (load) {
		memcpy(&s_cpu, cpu, ARM_STATE_SIZE);
		s_cpu_frac = fr[0];
		s_snd_frac = fr[1];
		s_frame = misc[0];
		s_vbl_i = misc[1];
		s_irq_target = (int)misc[2];
		s_volume = (uint8_t)misc[3];
		armpoll_reset(&s_poll);
	}
	return 0;
}

static int mlc_save(void *ctx) { return mlc_state(ctx, 0); }
static int mlc_load(void *ctx) { return mlc_state(ctx, 1); }
static int mlc_supported(const char *machine)
{
	return !strcmp(machine, "stadhr96") || !strcmp(machine, "mlc_6bpp") || !strcmp(machine, "mlc_5bpp") ||
	       !strcmp(machine, "acchi");
}

const rbx_driver rbx_drv_deco_mlc = { "DECO MLC", mlc_init, mlc_exit, mlc_reset, mlc_frame, mlc_save, mlc_load,
                                      mlc_supported, mlc_render, mlc_help };
