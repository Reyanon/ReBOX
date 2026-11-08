/*  Motorola MC6845 CRTC (timing part), ported to C from MAME src/devices/video/mc6845.cpp (BSD-3-Clause,
    copyright-holders: Zsolt Vasvari).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "mc6845.h"

#include <string.h>

static void recompute(mc6845_t *c)
{
	int ch = c->max_ras + 1;
	int htot = (c->htotal + 1) * 8, vtot = (c->vtotal + 1) * ch + c->vadj;
	int maxx = c->hdisp * 8 - 1, maxy = c->vdisp * ch - 1;
	int hsw = c->sync_width & 0x0f;
	if (!hsw)
		hsw = 16;
	int hs_on = c->hsync_pos * 8, hs_off = hs_on + hsw * 8;
	int vs_on = c->vsync_pos * ch;
	if (hs_off > htot)
		hs_off = htot;
	if (vs_on > vtot)
		vs_on = vtot;
	c->valid = htot > 0 && maxx < htot && vtot > 0 && maxy < vtot && hs_on <= htot && vs_on <= vtot && hs_on != hs_off;
}

void mc6845_reset(mc6845_t *c)
{
	memset(c, 0, sizeof(*c));
	// MAME device_start: large startup values
	c->htotal = 0xff;
	c->max_ras = 0x1f;
	c->vtotal = 0x7f;
	c->hsync_pos = 1;
	c->sync_width = 1;
	recompute(c);
}

void mc6845_address_w(mc6845_t *c, uint8_t data) { c->addr_latch = data & 0x1f; }

void mc6845_register_w(mc6845_t *c, uint8_t data)
{
	switch (c->addr_latch) {
	case 0x00: c->htotal = data; break;
	case 0x01: c->hdisp = data; break;
	case 0x02: c->hsync_pos = data; break;
	case 0x03: c->sync_width = data; break;
	case 0x04: c->vtotal = data & 0x7f; break;
	case 0x05: c->vadj = data & 0x1f; break;
	case 0x06: c->vdisp = data & 0x7f; break;
	case 0x07: c->vsync_pos = data & 0x7f; break;
	case 0x08: c->mode = data; break;
	case 0x09: c->max_ras = data & 0x1f; break;
	case 0x0a: c->cursor_start = data & 0x7f; break;
	case 0x0b: c->cursor_end = data & 0x1f; break;
	case 0x0c: c->start_addr = (uint16_t)(((data & 0x3f) << 8) | (c->start_addr & 0x00ff)); break;
	case 0x0d: c->start_addr = (uint16_t)(data | (c->start_addr & 0xff00)); break;
	case 0x0e: c->cursor_addr = (uint16_t)(((data & 0x3f) << 8) | (c->cursor_addr & 0x00ff)); break;
	case 0x0f: c->cursor_addr = (uint16_t)(data | (c->cursor_addr & 0xff00)); break;
	}
	recompute(c);
}

uint8_t mc6845_register_r(mc6845_t *c)
{
	switch (c->addr_latch) {
	case 0x0e: return (uint8_t)(c->cursor_addr >> 8);
	case 0x0f: return (uint8_t)c->cursor_addr;
	}
	return 0;   // write-only registers (light pen not connected)
}

static int match_line(mc6845_t *c)
{
	if (c->line == c->vdisp)
		c->line_enable = 0;
	if (c->line == c->vsync_pos) {
		c->vsync_width_counter = 0;
		c->vsync_ff = 1;
		return 1;
	}
	return 0;
}

void mc6845_line(mc6845_t *c)
{
	int new_vsync = c->vsync;
	if (c->vsync_ff) {
		c->vsync_width_counter = (c->vsync_width_counter + 1) & 0x0f;
		if (c->vsync_width_counter == 0) {   // MC6845: fixed 16-line VSYNC
			c->vsync_ff = 0;
			new_vsync = 0;
		}
	}
	if (c->raster == c->max_ras) {
		if (c->line == c->vtotal) {
			c->adjust_counter = 0;
			c->adjust_active = 1;
		}
		c->raster = 0;
		c->line = (c->line + 1) & 0x7f;
		c->line_address = (uint16_t)((c->line_address + c->hdisp) & 0x3fff);
		if (match_line(c))
			new_vsync = 1;
	} else {
		c->raster = (c->raster + 1) & 0x1f;
	}
	if (c->adjust_active) {
		if (c->adjust_counter == c->vadj) {
			c->adjust_active = 0;
			c->raster = 0;
			c->line = 0;
			c->line_address = c->start_addr;
			c->line_enable = 1;
		} else {
			c->adjust_counter = (c->adjust_counter + 1) & 0x1f;
		}
	}
	c->vsync = (uint8_t)new_vsync;
	c->de = c->line_enable;
}
