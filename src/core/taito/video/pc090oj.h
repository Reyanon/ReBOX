/*  Taito PC090OJ sprite chip (256 16x16 sprites), ported from MAME src/mame/taito/pc090oj.cpp (BSD-3-Clause,
    copyright-holders: Nicola Salmoria). Used by Rastan, Rainbow Islands, Operation Wolf, the Asuka family ...
    Sprites are decoded at init to one byte per pixel; drawing reads only a pc090oj_regs snapshot.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_PC090OJ_H
#define RBX_PC090OJ_H

#include <stdint.h>

#define PC090OJ_ACTIVE_WORDS 0x400   // 0x800 bytes of sprite list

typedef struct {
	uint16_t ram[PC090OJ_ACTIVE_WORDS];   // (buffered) sprite list as drawn
	uint16_t ctrl;                        // bit 0: flip screen when 0
	uint16_t sprite_ctrl;                 // board bits (palette bank ...), see the colpri setup of the driver
} pc090oj_regs;

typedef struct {
	uint16_t ram[0x2000];                 // 16 KB sprite RAM
	uint16_t buffered[PC090OJ_ACTIVE_WORDS];
	uint16_t ctrl, sprite_ctrl;
	// configuration
	int x_offset, y_offset, use_buffer;
	int direct;                           // RAM mapped on the 68000 bus (no word_w): ctrl read from RAM
	int colors;                           // palette entries / 16
	uint8_t *tiles;                       // 256 bytes per sprite
	uint8_t *empty;                       // 1 = all pixels 0
	uint32_t ntiles;
} pc090oj_t;

int pc090oj_init(pc090oj_t *c, const uint8_t *gfx, uint32_t size, int palette_entries);   // gfx_16x16x4_packed_msb
void pc090oj_exit(pc090oj_t *c);
void pc090oj_reset(pc090oj_t *c);
void pc090oj_word_w(pc090oj_t *c, uint32_t offset, uint16_t data, uint16_t mask);
static inline uint16_t pc090oj_word_r(pc090oj_t *c, uint32_t offset) { return c->ram[offset & 0x1fff]; }
void pc090oj_eof(pc090oj_t *c);                    // MAME eof_callback (buffered sprite RAM)
void pc090oj_snapshot(const pc090oj_t *c, pc090oj_regs *s);

// draw from snapshot s into bm (screen lines y0..y0+h-1, x 0..w-1). colbank / pri_mask come from the driver's
// colpri callback; with use_pri the sprites are drawn in list order through the priority buffer pri (w x h) as
// MAME prio_transpen (hidden where (1 << pri) & pri_mask, pri set to 31), else in reverse order over everything
void pc090oj_draw(const pc090oj_t *c, const pc090oj_regs *s, uint16_t *bm, int pitch, uint8_t *pri,
                  int w, int y0, int h, int use_pri, uint32_t colbank, uint32_t pri_mask);

#endif
