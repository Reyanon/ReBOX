/*  Taito TC0480SCP tilemap generator, ported from MAME src/mame/taito/tc0480scp.cpp (BSD-3-Clause,
    copyright-holders: Nicola Salmoria) and taito_helper.cpp taitoic_drawscanline.
    Four 16x16 ROM tile layers (zoom, rowscroll; BG2/3 also row zoom and column scroll) and an 8x8 4bpp text
    layer with characters in RAM. Used by Taito F2 (footchmp, metalb, deadconx), Z (spacegun ...), gunbustr,
    slapshot, superchs, undrfire, groundfx.
    RAM is host-order words (map it on the 68000 bus); control writes go through tc0480scp_ctrl_w because the
    chip latches scroll values with the flip state of the write. A snapshot is tc0480scp_copy.
    Screen coordinates are MAME's: bitmap row r is screen line sy0 + r.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_TC0480SCP_H
#define RBX_TC0480SCP_H

#include <stdint.h>

#define TC0480SCP_RAM_BYTES 0x10000

typedef struct {
	uint16_t ram[TC0480SCP_RAM_BYTES / 2];
	uint16_t ctrl[0x18];
	int pri_reg;
	int bgscrollx[4], bgscrolly[4];
	int txscrollx, txscrolly;       // tilemap scroll of the text layer (MAME set_scrollx(0, -data))
	// configuration (MAME set_offsets / set_offsets_tx / set_offsets_flip / set_col_base)
	int x_offset, y_offset, text_xoffs, text_yoffs, flip_xoffs, flip_yoffs, col_base;
	const uint8_t *tiles, *tile_used;   // decoded 16x16 tiles, one byte per pixel
	uint32_t ntiles;
	uint8_t *owned;
} tc0480scp_t;

int tc0480scp_init(tc0480scp_t *c, const uint8_t *gfx, uint32_t size);
void tc0480scp_exit(tc0480scp_t *c);
void tc0480scp_reset(tc0480scp_t *c);
void tc0480scp_ctrl_w(tc0480scp_t *c, int offset, uint16_t data, uint16_t mask);
static inline uint16_t tc0480scp_ctrl_r(const tc0480scp_t *c, int offset) { return offset < 0x18 ? c->ctrl[offset] : 0; }
int tc0480scp_bg_priority(const tc0480scp_t *c);
void tc0480scp_copy(tc0480scp_t *dst, const tc0480scp_t *src);
// layer 0-3 = BG0-BG3, 4 = text; pen 0 transparent; pri |= priority where drawn. 320 pixels per row
void tc0480scp_draw(const tc0480scp_t *c, int layer, uint16_t *bm, uint8_t *pri, int pitch, int sy0, int rows,
                    uint8_t priority);
// state for save / load after the RAM and ctrl blocks are restored (recomputes the latched scrolls)
void tc0480scp_post_load(tc0480scp_t *c);

#endif
