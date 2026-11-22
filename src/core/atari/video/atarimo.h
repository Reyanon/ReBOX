/*  Atari motion objects (the MO hardware of the Atari raster games: Gauntlet, System 1 ...). Ported from MAME
    src/mame/atari/atarimo.cpp, atarimo.h (BSD-3-Clause, copyright-holders: Aaron Giles) and the drawing of
    src/devices/video/sprite.h (sprite16_device draw_async).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_ATARIMO_H
#define RBX_ATARIMO_H

#include <stdint.h>

#include "../../video/gfxlayout.h"

#define AMO_MAX_PER_BANK 1024
#define AMO_PRIORITY_SHIFT 12

// atari_motion_objects_config (the masks as 4 words; the code / color upper words are not used by the boards here)
typedef struct {
	uint8_t bankcount, linked, split, reverse, swapxy, nextneighbor;
	uint16_t slipheight;
	uint8_t slipoffset;
	uint16_t maxperline, palettebase;
	uint8_t transpen;
	uint16_t link[4], code[4], color[4], xpos[4], ypos[4], width[4], height[4], hflip[4], vflip[4], priority[4],
	         neighbor[4], absolute[4], special[4];
	uint16_t specialvalue;
} amo_config;

typedef struct { uint8_t word, shift; uint16_t mask; } amo_param;

typedef struct {
	amo_config c;
	amo_param linkm, codem, colorm, xposm, yposm, widthm, heightm, hflipm, vflipm, priom, neighm, absm, specm;
	int tilew, tileh, tilexshift, tileyshift;
	int bitmapw, bitmaph, bitmapxmask, bitmapymask;
	int entrycount, entrybits, slipshift, sliprammask;
	// lookups (MAME m_codelookup, m_colorlookup, m_gfxlookup): the board may change them after amo_init
	uint32_t *codelookup;
	int codesize;
	uint8_t colorlookup[256];
	uint8_t *gfxlookup;          // codesize / 256 entries: index into gfx[]
	const rbx_gfx *gfx[24];      // the gfx elements and their color granularity
	int granularity[24];
	// state (saved by the board: bank, xscroll, yscroll)
	int bank, xscroll, yscroll, xoffset;
	int last_xpos, next_xpos;
	uint16_t activelist[AMO_MAX_PER_BANK * 4];
	uint16_t *activelast;
} amo_t;

// device_start: the masks from the config; tile size of the gfx; 0 = ok (codelookup allocated)
int amo_init(amo_t *m, const amo_config *c, int tilew, int tileh);
void amo_exit(amo_t *m);
static inline void amo_set_xscroll(amo_t *m, int x) { m->xscroll = x & m->bitmapxmask; }
static inline void amo_set_yscroll(amo_t *m, int y) { m->yscroll = y & m->bitmapymask; }
// draw_async: clear the cliprect of bm (to 0xffff) and draw the objects into it. bm holds screen lines from 0 with
// the given pitch; [minx, maxx] x [miny, maxy] the cliprect (MAME's bitmap is cliprect.right + 1 by
// cliprect.bottom + 1, as draw_async wraps it)
void amo_draw(amo_t *m, const uint16_t *spriteram, const uint16_t *slipram, uint16_t *bm, int pitch, int minx, int maxx,
              int miny, int maxy);

#endif
