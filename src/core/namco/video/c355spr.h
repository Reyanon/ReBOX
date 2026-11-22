/*  Namco 186/187 or C355/187 zooming sprites.
    Ported from MAME 0.289 src/mame/shared/namco_c355spr.cpp / namco_c355spr.h
    (BSD-3-Clause, copyright-holders: David Haywood, Phil Stroffolino, Bryan McPhail).
    This file is part of ReBOX. GPL-2.0-or-later.

    Graphics: MAME decodes the "c355spr" region with gfx_16x16x8_raw, i.e. the region already is one byte per pixel,
    256 bytes per 16x16 tile, 16 bytes a row. We read it raw (no decode, no copy); the caller keeps the region alive.
*/
#ifndef RBX_NAMCO_C355SPR_H
#define RBX_NAMCO_C355SPR_H

#include <stdint.h>
#include <stddef.h>

#define C355SPR_RAM_WORDS (0x20000 / 2)

typedef int (*c355spr_code2tile_cb)(void *user, int code);
/* palette word (sprite table attr 6) -> priority, or -1 = sprite disabled. NULL = MAME default ((pal >> 4) & 0xf). */
typedef int (*c355spr_pri_cb)(void *user, int pal_pri);
/* MAME's mix delegate. NULL = MAME default_mix (inlined in copy()). */
typedef int (*c355spr_mix_cb)(void *user, uint16_t *dest, uint8_t *destpri, uint16_t colbase, uint16_t src,
                              int srcpri, int pri);

typedef struct { int min_x, max_x, min_y, max_y; } c355spr_rect;   /* inclusive, like MAME's rectangle */

typedef struct {
    /* saved: RAM + position registers (C355SPR_SAVE_PTR / C355SPR_SAVE_SIZE) */
    uint16_t ram[2][C355SPR_RAM_WORDS];   /* [1] is only used with buffer > 1 */
    uint16_t position[4];
    uint16_t pad_[2];
    /* config (set before or after init, not saved) */
    int scrolloffs[2];
    int buffer;            /* 0: render in copy pri 0 (driver calls build_list_and_render); 1: at vblank; 2: + RAM copy */
    int alt_precision;
    int medium_res;        /* MAME's screen().height() > 384: 10-bit xscroll (System 21) */
    uint8_t transpen;      /* 255 */
    uint16_t colbase;      /* 0 */
    int colors;            /* 16 */
    int granularity;       /* 256 */
    c355spr_code2tile_cb code2tile;
    c355spr_pri_cb pri_cb;
    c355spr_mix_cb mix_cb;
    void *user;
    /* graphics (raw 16x16x8) */
    const uint8_t *gfx;
    uint32_t elements;
    /* render bitmap: (prio << 12) | (pen & 0xfff), 0xffff = empty */
    uint16_t *bitmap;
    int width, height;
} c355spr_t;

#define C355SPR_SAVE_PTR(s) ((void *)(s)->ram)
#define C355SPR_SAVE_SIZE (offsetof(c355spr_t, scrolloffs))

/* Sets MAME's defaults, allocates the width x height render bitmap. gfx = the "c355spr" region. 0 = ok. */
int c355spr_init(c355spr_t *s, int width, int height, const uint8_t *gfx, size_t gfx_size);
void c355spr_free(c355spr_t *s);
void c355spr_reset(c355spr_t *s);   /* RAM, position and bitmap cleared (MAME device_start state) */

uint16_t c355spr_spriteram_r(c355spr_t *s, uint32_t offset);   /* word offsets */
void c355spr_spriteram_w(c355spr_t *s, uint32_t offset, uint16_t data, uint16_t mem_mask);
uint16_t c355spr_position_r(c355spr_t *s, uint32_t offset);
void c355spr_position_w(c355spr_t *s, uint32_t offset, uint16_t data, uint16_t mem_mask);
/* visible = MAME's screen().visible_area(), used for the buffer > 0 render */
void c355spr_vblank(c355spr_t *s, int state, const c355spr_rect *visible);

void c355spr_build_list_and_render(c355spr_t *s, const c355spr_rect *clip);
/* MAME draw(): with pri 0 and buffer 0, builds and renders first, then copybitmap (ind16). */
void c355spr_draw(c355spr_t *s, uint16_t *dest, int dest_pitch, uint8_t *destpri, int pri_pitch,
                  const c355spr_rect *clip, int pri);
/* copybitmap for ind16 (pitches in elements) */
void c355spr_copy(c355spr_t *s, uint16_t *dest, int dest_pitch, uint8_t *destpri, int pri_pitch,
                  const c355spr_rect *clip, int pri);

/* MAME zdrawgfxzoom into the render bitmap (hsize / vsize only tested for 0) */
void c355spr_zdrawgfxzoom(c355spr_t *s, const c355spr_rect *clip, uint32_t code, uint32_t color, int flipx, int flipy,
                          int hpos, int vpos, int hsize, int vsize, uint8_t prival,
                          int sprite_screen_width, int sprite_screen_height);

#endif
