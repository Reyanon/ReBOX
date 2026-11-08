/*  Motorola MC6845 CRTC (timing part), ported to C from MAME src/devices/video/mc6845.cpp (BSD-3-Clause,
    copyright-holders: Zsolt Vasvari). Registers, the per-line counter logic of handle_line_timer (raster / row /
    adjust counters, display enable, VSYNC with the MC6845's fixed 16-line width) and the screen geometry of
    recompute_parameters. The board calls mc6845_line() at the start of each scanline ((R0+1) character clocks).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_MC6845_H
#define RBX_MC6845_H

#include <stdint.h>

typedef struct {
	uint8_t addr_latch;
	uint8_t htotal, hdisp, hsync_pos, sync_width, vtotal, vadj, vdisp, vsync_pos, mode, max_ras;
	uint8_t cursor_start, cursor_end;
	uint16_t start_addr, cursor_addr;
	// counters
	uint16_t line_address;
	uint8_t raster, line, adjust_counter, adjust_active, line_enable, vsync_ff, vsync_width_counter;
	uint8_t vsync, de, valid;
	uint8_t pad_[2];
} mc6845_t;

void mc6845_reset(mc6845_t *c);
void mc6845_address_w(mc6845_t *c, uint8_t data);
void mc6845_register_w(mc6845_t *c, uint8_t data);
uint8_t mc6845_register_r(mc6845_t *c);
// new scanline: updates the counters, c->vsync and c->de (display enable at the start of the line)
void mc6845_line(mc6845_t *c);
static inline int mc6845_char_height(const mc6845_t *c) { return c->max_ras + 1; }
static inline int mc6845_vtotal_lines(const mc6845_t *c) { return (c->vtotal + 1) * (c->max_ras + 1) + c->vadj; }

#endif
