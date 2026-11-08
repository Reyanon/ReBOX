/*  Data East IRQ controller (raster, vblank and lightgun interrupts of the DECO 32 boards), after MAME
    src/mame/dataeast/deco_irq.cpp (BSD-3-Clause, copyright-holders: Dirk Best). The board calls deco_irq_line at the
    start of each screen line (MAME's scanline timer) and gets the lines' changes through its callback.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_DECO_IRQ_H
#define RBX_DECO_IRQ_H
#include <stdint.h>

enum { DECO_IRQ_RASTER1 = 0, DECO_IRQ_RASTER2 = 1 };

typedef struct {
	// ---- state (saved) ----
	uint8_t lightgun_irq, raster_irq, vblank_irq;
	uint8_t raster_target, raster_masked, raster_scanline;
	uint16_t lightgun_latch;
	// ---- configuration ----
	int vis_top, vis_bottom;    // the visible area's first and last line
	// the lines: raster 1 / 2, vblank, lightgun (NULL: not connected)
	void (*raster1)(void *user, int state), (*raster2)(void *user, int state);
	void (*vblank)(void *user, int state), (*lightgun)(void *user, int state);
	void *user;
} deco_irq_t;
#define DECO_IRQ_STATE_SIZE ((uint32_t)sizeof(uint8_t) * 6 + sizeof(uint16_t))

static inline void deco_irq_set(void (*f)(void *, int), void *user, int state) { if (f) f(user, state); }

static inline void deco_irq_reset(deco_irq_t *d)
{
	d->lightgun_irq = d->raster_irq = d->vblank_irq = 0;
	d->raster_target = DECO_IRQ_RASTER1;
	d->raster_masked = 1;
	d->raster_scanline = 0;
	d->lightgun_latch = 0;
}

// MAME scanline_callback at line y
static inline void deco_irq_line(deco_irq_t *d, int y)
{
	if (d->raster_scanline >= d->vis_top && d->raster_scanline <= d->vis_bottom && y == d->raster_scanline &&
	    !d->raster_masked) {
		d->raster_irq = 1;
		deco_irq_set(d->raster_target == DECO_IRQ_RASTER1 ? d->raster1 : d->raster2, d->user, 1);
	}
	if (d->lightgun_latch >= d->vis_top && d->lightgun_latch <= d->vis_bottom && y == d->lightgun_latch) {
		d->lightgun_irq = 1;
		deco_irq_set(d->lightgun, d->user, 1);
	}
	if (y == d->vis_bottom + 1) {
		d->vblank_irq = 1;
		deco_irq_set(d->vblank, d->user, 1);
	}
}

static inline void deco_irq_raster_ack(deco_irq_t *d)
{
	d->raster_irq = 0;
	deco_irq_set(d->raster1, d->user, 0);
	deco_irq_set(d->raster2, d->user, 0);
}

// the map (offsets 0 .. 3, byte wide): 0 control_w, 1 scanline r/w, 2 raster ack r / vblank ack w, 3 status r
static inline void deco_irq_w(deco_irq_t *d, int offset, uint8_t data)
{
	switch (offset & 3) {
	case 0:
		d->raster_target = (data >> 4) & 1;
		d->raster_masked = (data >> 1) & 1;
		if (d->raster_masked)
			deco_irq_raster_ack(d);
		break;
	case 1:
		d->raster_scanline = data;
		break;
	case 2:
		d->vblank_irq = 0;
		deco_irq_set(d->vblank, d->user, 0);
		break;
	}
}

// vblank / hblank: the screen's state now
static inline uint8_t deco_irq_r(deco_irq_t *d, int offset, int vblank, int hblank)
{
	switch (offset & 3) {
	case 1:
		return d->raster_scanline;
	case 2:
		deco_irq_raster_ack(d);
		return 0xff;
	case 3:
		return (uint8_t)(0x80 | d->lightgun_irq << 6 | d->raster_irq << 5 | d->vblank_irq << 4 | (vblank ? 2 : 0) |
		                 (hblank ? 1 : 0));
	}
	return 0xff;   // (offset 0: write only)
}

static inline void deco_irq_lightgun_ack(deco_irq_t *d)
{
	d->lightgun_irq = 0;
	deco_irq_set(d->lightgun, d->user, 0);
}
#endif
