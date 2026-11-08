/*  Taito TC0150ROD road generator, ported from MAME src/mame/taito/tc0150rod.cpp (BSD-3-Clause,
    copyright-holders: Nicola Salmoria) and taito_helper.cpp taitoic_drawscanline.
    Two roads (A and B, for forks) of one 2bpp line graphic per screen line, fetched from the road ROM, each line
    with left / right edges, body, palette offsets and priority bits. Used by Taito Z (contcirc, chasehq, enforce,
    bshark, sci, nightstr, aquajack, dblaxle, racingb).
    The RAM is plain host-order words (map it straight on the 68000 bus); a snapshot is a copy of the RAM.
    Drawing works on MAME screen coordinates: bitmap row r is screen line sy0 + r.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_TC0150ROD_H
#define RBX_TC0150ROD_H

#include <stdint.h>

#define TC0150ROD_RAM_BYTES 0x2000

typedef struct {
	uint16_t ram[TC0150ROD_RAM_BYTES / 2];
	const uint8_t *gfx;     // road ROM (little-endian 16-bit words, MAME ROM_REGION16_LE)
	uint32_t gfx_words;
} tc0150rod_t;

void tc0150rod_init(tc0150rod_t *r, const uint8_t *gfx, uint32_t size);
void tc0150rod_reset(tc0150rod_t *r);
// copy the drawing state (RAM) into a snapshot sharing the ROM
void tc0150rod_copy(tc0150rod_t *dst, const tc0150rod_t *src);
// MAME tc0150rod_device::draw over screen lines sy0 .. sy0 + rows - 1, width pixels (the full screen width):
// y_offs, palette_offs, type (0 / 1 / 2), road_trans as in MAME; pri |= low_priority above the priority switch
// line, high_priority below it
void tc0150rod_draw(const tc0150rod_t *r, uint16_t *bm, uint8_t *pri, int pitch, int width, int sy0, int rows,
                    int y_offs, int palette_offs, int type, int road_trans, uint8_t low_priority, uint8_t high_priority);

#endif
