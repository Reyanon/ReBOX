/*  Data East DECO ACE (palette fading and alpha table), after MAME src/mame/dataeast/deco_ace.cpp (BSD-3-Clause,
    copyright-holders: Bryan McPhail, David Haywood): the buffered palette its DMA takes, the faded pens (0 .. 2047)
    and the raw ones (2048 .. 4095), the alpha levels of its RAM. The pens are worked out again when the board asks
    (deco_ace_pens: at the frame's capture), as only the drawing uses them.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_DECO_ACE_H
#define RBX_DECO_ACE_H
#include <stdint.h>
#include <string.h>

typedef struct {
	uint16_t ram[0x28];
	uint32_t palram[0x800];   // the CPU's palette RAM (0x00BBGGRR words)
	uint32_t palbuf[0x800];   // what the DMA took
	uint32_t pens[0x1000];
	int dirty;
} deco_ace_t;

static inline void deco_ace_reset(deco_ace_t *a)
{
	memset(a, 0, sizeof(*a));
}

// MAME palette_update (the pens from the taken palette and the fade registers 0x20 .. 0x26)
static inline void deco_ace_update(deco_ace_t *a)
{
	const int fr = a->ram[0x20] & 0xff, fg = a->ram[0x21] & 0xff, fb = a->ram[0x22] & 0xff;
	const int sr = a->ram[0x23] & 0xff, sg = a->ram[0x24] & 0xff, sb = a->ram[0x25] & 0xff;
	const uint16_t mode = a->ram[0x26];
	for (int i = 0; i < 0x800; i++) {
		int b = (a->palbuf[i] >> 16) & 0xff, g = (a->palbuf[i] >> 8) & 0xff, r = a->palbuf[i] & 0xff;
		a->pens[i + 0x800] = (uint32_t)(r << 16 | g << 8 | b);   // raw
		if (mode == 0x1000) {   // additive fade
			b = b + sb > 0xff ? 0xff : b + sb;
			g = g + sg > 0xff ? 0xff : g + sg;
			r = r + sr > 0xff ? 0xff : r + sr;
		} else {                // multiplicative fade (u8 of the sum)
			b = (uint8_t)(b + ((fb - b) * sb) / 255);
			g = (uint8_t)(g + ((fg - g) * sg) / 255);
			r = (uint8_t)(r + ((fr - r) * sr) / 255);
		}
		a->pens[i] = (uint32_t)(r << 16 | g << 8 | b);   // faded
	}
	a->dirty = 0;
}
// the pens up to date (call at the frame's capture)
static inline void deco_ace_pens(deco_ace_t *a)
{
	if (a->dirty)
		deco_ace_update(a);
}

static inline uint8_t deco_ace_alpha(const uint16_t *ram, uint8_t val)   // MAME get_alpha
{
	const int al = ram[val & 0x1f] & 0xff;
	if (al > 0x20)
		return 0x80;   // (TODO in MAME: a special blending command)
	const int r = 255 - (al << 3);
	return (uint8_t)(r < 0 ? 0 : r);
}

// ace_w (16-bit, offsets 0 .. 0x27): the fade registers mark the pens
static inline void deco_ace_w(deco_ace_t *a, int offset, uint16_t data, uint16_t mask)
{
	if (offset < 0 || offset >= 0x28)
		return;
	a->ram[offset] = (uint16_t)((a->ram[offset] & ~mask) | (data & mask));
	if (offset >= 0x20 && offset <= 0x26)
		a->dirty = 1;
}
// buffered_palette16_r / w (the 16-bit boards: even word the high half)
static inline uint16_t deco_ace_pal16_r(const deco_ace_t *a, int offset)
{
	const uint32_t p = a->palram[(offset >> 1) & 0x7ff];
	return (uint16_t)(offset & 1 ? p : p >> 16);
}
static inline void deco_ace_pal16_w(deco_ace_t *a, int offset, uint16_t data, uint16_t mask)
{
	uint32_t *p = &a->palram[(offset >> 1) & 0x7ff];
	if (offset & 1)
		*p = (*p & ~(uint32_t)mask) | (data & mask);
	else
		*p = (*p & ~((uint32_t)mask << 16)) | ((uint32_t)(data & mask) << 16);
}
static inline void deco_ace_dma(deco_ace_t *a)   // palette_dma_w
{
	memcpy(a->palbuf, a->palram, sizeof(a->palbuf));
	a->dirty = 1;
}
#endif
