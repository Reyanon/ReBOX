/*  Taito TC0150ROD road generator (tc0150rod.h), ported from MAME src/mame/taito/tc0150rod.cpp (BSD-3-Clause,
    copyright-holders: Nicola Salmoria). The drawing below follows MAME's draw() line by line, including its
    road A / road B priority scheme (see the notes in the MAME source); pixel words carry 0x8000 = transparent and
    0x7000 = pixel priority until the two roads are combined.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "tc0150rod.h"

#include <string.h>

void tc0150rod_init(tc0150rod_t *r, const uint8_t *gfx, uint32_t size)
{
	r->gfx = gfx;
	r->gfx_words = gfx ? size / 2 : 0;
	tc0150rod_reset(r);
}

void tc0150rod_reset(tc0150rod_t *r) { memset(r->ram, 0, sizeof(r->ram)); }

void tc0150rod_copy(tc0150rod_t *dst, const tc0150rod_t *src)
{
	memcpy(dst->ram, src->ram, sizeof(dst->ram));
	dst->gfx = src->gfx;
	dst->gfx_words = src->gfx_words;
}

// 2bpp pixel x_index (0..0x7ff) of road graphic tile: each word holds 8 pixels, low bits in the high byte
static inline int road_pixel(const tc0150rod_t *r, int tile, int x_index)
{
	uint32_t w = ((uint32_t)tile << 8) + (uint32_t)(x_index >> 3);
	if (w >= r->gfx_words)
		return 0;
	uint16_t g = (uint16_t)(r->gfx[w * 2] | r->gfx[w * 2 + 1] << 8);
	int b = 7 - (x_index & 7);
	return ((g >> (b + 8)) & 1) * 2 + ((g >> b) & 1);
}

void tc0150rod_draw(const tc0150rod_t *r, uint16_t *bm, uint8_t *prio, int pitch, int width, int sy0, int rows,
                    int y_offs, int palette_offs, int type, int road_trans, uint8_t low_priority, uint8_t high_priority)
{
	const int x_offs = 0xa7;   // increasing this shifts the road to the right
	uint16_t roada_line[512], roadb_line[512];
	const uint16_t *ram = r->ram;
	int road_ctrl = ram[0xfff];
	int screen_width = width > 512 ? 512 : width;
	int road_A_address = y_offs * 4 + ((road_ctrl & 0x0300) << 2);   // index into road RAM for road A
	int road_B_address = y_offs * 4 + ((road_ctrl & 0x0c00) << 0);   // and road B
	int priority_switch_line = (road_ctrl & 0x00ff) - y_offs;
	if (!r->gfx)
		return;

	for (int row = 0; row < rows; row++) {
		int y = sy0 + row;
		int line_needs_drawing = 0;
		int roadram_index = (road_A_address + y * 4) & 0xffc;
		int roadram2_index = (road_B_address + y * 4) & 0xffc;
		uint16_t *roada, *roadb;
		uint8_t priorities[6] = { 1, 1, 2, 3, 3, 4 };   // l-edge a, r-edge a, body a, l-edge b, r-edge b, body b

		for (int i = 0; i < screen_width; i++) {   // default transparency fill
			roada_line[i] = 0x8000;
			roadb_line[i] = 0x8000;
		}

		uint16_t roada_clipr = ram[roadram_index];
		uint16_t roada_clipl = ram[roadram_index + 1];
		uint16_t roada_bodyctrl = ram[roadram_index + 2];
		uint16_t roadb_clipr = ram[roadram2_index];
		uint16_t roadb_clipl = ram[roadram2_index + 1];
		uint16_t roadb_bodyctrl = ram[roadram2_index + 2];

		// not very logical, but seems to work (MAME)
		if (roada_bodyctrl & 0x2000) priorities[2] += 2;
		if (roadb_bodyctrl & 0x2000) priorities[2] += 1;
		if (roada_clipl & 0x2000) priorities[3] -= 1;
		if (roadb_clipl & 0x2000) priorities[3] -= 2;
		if (roada_clipr & 0x2000) priorities[4] -= 1;
		if (roadb_clipr & 0x2000) priorities[4] -= 2;
		if (priorities[4] == 0) priorities[4]++;   // fixes Aquajack LH edge dropping below background

		// ---------------- road A ----------------
		int palroffs = (roada_clipr & 0x1000) >> 11;
		int palloffs = (roada_clipl & 0x1000) >> 11;
		int xoffset = roada_bodyctrl & 0x7ff;
		int paloffs = (roada_bodyctrl & 0x1800) >> 11;
		int colbank = (ram[roadram_index + 3] & 0xf000) >> 10;
		int road_gfx_tilenum = ram[roadram_index + 3] & 0x3ff;
		int right_over = 0, left_over = 0;
		int road_center = 0x5ff - ((-xoffset + x_offs) & 0x7ff);
		int left_edge = road_center - (roada_clipl & 0x3ff);        // start pixel for left edge
		int right_edge = road_center + 1 + (roada_clipr & 0x3ff);   // start pixel for right edge
		if (roada_clipl || roada_clipr)
			line_needs_drawing = 1;
		int begin = left_edge + 1;   // main road line is drawn from begin to end - 1
		if (begin < 0)
			begin = 0;
		int end = right_edge;
		if (end > screen_width)
			end = screen_width;
		if (right_edge < 0) {
			right_over = -right_edge;
			right_edge = 0;
		}
		if (left_edge >= screen_width) {
			left_over = left_edge - screen_width + 1;
			left_edge = screen_width - 1;
		}
		int background_only = road_center > (screen_width - 2 + 1024 / 2);

		// main part of road A
		uint16_t color = (uint16_t)(((palette_offs + colbank + paloffs) << 4) + (type ? 1 : 4));
		uint16_t pri = (uint16_t)(priorities[2] << 12);
		int x_index = (-xoffset + x_offs + begin) & 0x7ff;
		roada = roada_line + screen_width - 1 - begin;
		if (line_needs_drawing && begin < end) {
			for (int i = begin; i < end; i++) {
				if (road_gfx_tilenum) {   // fixes Nightstr round C
					int pixel = road_pixel(r, road_gfx_tilenum, x_index);
					if (pixel || !road_trans) {
						if (type)
							pixel = (pixel - 1) & 3;
						*roada-- = (uint16_t)((color + pixel) | pri);
					} else
						*roada-- = 0xf000;   // priority transparency, fixes Bshark round 6 + Aquajack
				} else
					roada--;
				x_index = (x_index + 1) & 0x7ff;
			}
		}

		// 'left' edge of road A
		color = (uint16_t)(((palette_offs + colbank + palloffs) << 4) + (type ? 1 : 4));
		pri = (uint16_t)(priorities[0] << 12);
		if (background_only) {   // the edge is entirely off screen, but the background may need filling
			if (roada_clipl & 0x8000)
				for (int i = 0; i < screen_width; i++)
					roada_line[i] = (uint16_t)(color + (type ? 3 : 0));
		} else if (left_edge >= 0 && left_edge < screen_width) {
			x_index = (1024 / 2 - 1 - left_over) & 0x7ff;
			roada = roada_line + screen_width - 1 - left_edge;
			if (line_needs_drawing) {
				for (int i = left_edge; i >= 0; i--) {
					int pixel = road_pixel(r, road_gfx_tilenum, x_index);
					uint16_t pixpri = pixel == 0 ? 0 : pri;   // off edge has low priority
					if (pixel == 0 && !(roada_clipl & 0x8000))
						roada++;
					else {
						if (type)
							pixel = (pixel - 1) & 3;
						*roada++ = (uint16_t)((color + pixel) | pixpri);
					}
					x_index = (x_index - 1) & 0x7ff;
				}
			}
		}

		// 'right' edge of road A
		color = (uint16_t)(((palette_offs + colbank + palroffs) << 4) + (type ? 1 : 4));
		pri = (uint16_t)(priorities[1] << 12);
		if (right_edge < screen_width && right_edge >= 0) {
			x_index = (1024 / 2 + right_over) & 0x7ff;
			roada = roada_line + screen_width - 1 - right_edge;
			if (line_needs_drawing) {
				for (int i = right_edge; i < screen_width; i++) {
					int pixel = road_pixel(r, road_gfx_tilenum, x_index);
					uint16_t pixpri = pixel == 0 ? 0 : pri;
					if (pixel == 0 && !(roada_clipr & 0x8000))
						roada--;
					else {
						if (type)
							pixel = (pixel - 1) & 3;
						*roada-- = (uint16_t)((color + pixel) | pixpri);
					}
					x_index = (x_index + 1) & 0x7ff;
				}
			}
		}

		// ---------------- road B ----------------
		palroffs = (roadb_clipr & 0x1000) >> 11;
		palloffs = (roadb_clipl & 0x1000) >> 11;
		xoffset = roadb_bodyctrl & 0x7ff;
		paloffs = (roadb_bodyctrl & 0x1800) >> 11;
		colbank = (ram[roadram2_index + 3] & 0xf000) >> 10;
		road_gfx_tilenum = ram[roadram2_index + 3] & 0x3ff;
		right_over = 0;
		left_over = 0;
		road_center = 0x5ff - ((-xoffset + x_offs) & 0x7ff);
		left_edge = road_center - (roadb_clipl & 0x3ff);
		right_edge = road_center + 1 + (roadb_clipr & 0x3ff);
		int draw_top_road_line = 0;
		if ((roadb_clipl || roadb_clipr) && ((road_ctrl & 0x800) || type == 2)) {
			draw_top_road_line = 1;
			line_needs_drawing = 1;
		}
		begin = left_edge + 1;
		if (begin < 0)
			begin = 0;
		end = right_edge;
		if (end > screen_width)
			end = screen_width;
		if (right_edge < 0) {
			right_over = -right_edge;
			right_edge = 0;
		}
		if (left_edge >= screen_width) {
			left_over = left_edge - screen_width + 1;
			left_edge = screen_width - 1;
		}
		background_only = road_center > (screen_width - 2 + 1024 / 2);

		// main part of road B
		color = (uint16_t)(((palette_offs + colbank + paloffs) << 4) + (type ? 1 : 4));
		pri = (uint16_t)(priorities[5] << 12);
		x_index = (-xoffset + x_offs + begin) & 0x7ff;
		if (x_index > 0x3ff) {   // second half of the graphic holds the road body line
			roadb = roadb_line + screen_width - 1 - begin;
			if (draw_top_road_line && road_gfx_tilenum && begin < end) {
				for (int i = begin; i < end; i++) {
					int pixel = road_pixel(r, road_gfx_tilenum, x_index);
					if (pixel || !road_trans) {
						if (type)
							pixel = (pixel - 1) & 3;
						*roadb-- = (uint16_t)((color + pixel) | pri);
					} else
						*roadb-- = 0xf000;   // high priority transparency, fixes Aquajack
					x_index = (x_index + 1) & 0x7ff;
				}
			}
		}

		// 'left' edge of road B
		color = (uint16_t)(((palette_offs + colbank + palloffs) << 4) + (type ? 1 : 4));
		pri = (uint16_t)(priorities[3] << 12);
		if (background_only) {
			if ((roadb_clipl & 0x8000) && draw_top_road_line)
				for (int i = 0; i < screen_width; i++)
					roadb_line[i] = (uint16_t)(color + (type ? 3 : 0));
		} else if (left_edge >= 0 && left_edge < screen_width) {
			x_index = (1024 / 2 - 1 - left_over) & 0x7ff;
			roadb = roadb_line + screen_width - 1 - left_edge;
			if (draw_top_road_line) {
				for (int i = left_edge; i >= 0; i--) {
					int pixel = road_pixel(r, road_gfx_tilenum, x_index);
					uint16_t pixpri = pixel == 0 ? 0 : pri;
					if (pixel == 0 && !(roadb_clipl & 0x8000))   // background disabled
						roadb++;
					else {
						if (type)
							pixel = (pixel - 1) & 3;
						*roadb++ = (uint16_t)((color + pixel) | pixpri);
					}
					x_index--;
					if (x_index < 0)
						break;
				}
			}
		}

		// 'right' edge of road B
		color = (uint16_t)(((palette_offs + colbank + palroffs) << 4) + (type ? 1 : 4));
		pri = (uint16_t)(priorities[4] << 12);
		if (right_edge < screen_width && right_edge >= 0) {
			x_index = (1024 / 2 + right_over) & 0x7ff;
			roadb = roadb_line + screen_width - 1 - right_edge;
			if (draw_top_road_line) {
				for (int i = right_edge; i < screen_width; i++) {
					int pixel = road_pixel(r, road_gfx_tilenum, x_index);
					uint16_t pixpri = pixel == 0 ? 0 : pri;
					if (pixel == 0 && !(roadb_clipr & 0x8000))
						roadb--;
					else {
						if (type)
							pixel = (pixel - 1) & 3;
						*roadb-- = (uint16_t)((color + pixel) | pixpri);
					}
					x_index++;
					if (x_index > 0x3ff)
						break;
				}
			}
		}

		// combine the two lines by pixel priority, then draw (taitoic_drawscanline, transparent)
		if (line_needs_drawing) {
			uint16_t *d = bm + row * pitch;
			uint8_t *pp = prio + row * pitch;
			uint8_t p = y > priority_switch_line ? high_priority : low_priority;
			for (int i = 0; i < screen_width; i++) {
				uint16_t a = roada_line[i], b = roadb_line[i], px;
				if (a == 0x8000)
					px = b & 0x8fff;
				else if (b == 0x8000)
					px = a & 0x8fff;
				else
					px = ((b & 0x7000) > (a & 0x7000)) ? (b & 0x8fff) : (a & 0x8fff);
				if (px < 0x7fff) {
					d[i] = px;
					pp[i] |= p;
				}
			}
		}
	}
}
