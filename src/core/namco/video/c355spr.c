/*  Namco 186/187 or C355/187 zooming sprites.
    Ported from MAME 0.289 src/mame/shared/namco_c355spr.cpp / namco_c355spr.h
    (BSD-3-Clause, copyright-holders: David Haywood, Phil Stroffolino, Bryan McPhail).
    This file is part of ReBOX. GPL-2.0-or-later.

    MAME builds a 256-entry sprite list (256 tiles each) and then renders it; nothing reads the list in between, so
    here each sprite is rendered as soon as it is built (same order, same arithmetic, no 1.3 MB list).
    Not done: the Data East access callbacks (set_read_sprite*), set_external_prifill, the rgb32 copybitmap.
*/
#include "c355spr.h"

#include <stdlib.h>
#include <string.h>

#define COMBINE(p, d, m) (*(p) = (uint16_t)((*(p) & ~(m)) | ((d) & (m))))

static inline int sext(int v, int bits)
{
    int m = 1 << (bits - 1);
    v &= (1 << bits) - 1;
    return (v ^ m) - m;
}

int c355spr_init(c355spr_t *s, int width, int height, const uint8_t *gfx, size_t gfx_size)
{
    memset(s, 0, sizeof(*s));
    s->transpen = 255;
    s->colbase = 0;
    s->colors = 16;
    s->granularity = 256;
    s->gfx = gfx;
    s->elements = (uint32_t)(gfx_size / 256);
    s->width = width;
    s->height = height;
    s->bitmap = (uint16_t *)malloc((size_t)width * height * sizeof(uint16_t));
    if (!s->bitmap)
        return -1;
    c355spr_reset(s);
    return 0;
}

void c355spr_free(c355spr_t *s)
{
    free(s->bitmap);
    s->bitmap = NULL;
}

void c355spr_reset(c355spr_t *s)
{
    memset(s->ram, 0, sizeof(s->ram));
    memset(s->position, 0, sizeof(s->position));
    if (s->bitmap)
        memset(s->bitmap, 0xff, (size_t)s->width * s->height * sizeof(uint16_t));
}

uint16_t c355spr_spriteram_r(c355spr_t *s, uint32_t offset)
{
    return s->ram[0][offset & (C355SPR_RAM_WORDS - 1)];
}

void c355spr_spriteram_w(c355spr_t *s, uint32_t offset, uint16_t data, uint16_t mem_mask)
{
    offset &= C355SPR_RAM_WORDS - 1;
    COMBINE(&s->ram[0][offset], data, mem_mask);
    /* update page 0 tables */
    if (offset >= 0x10000 / 2 && offset < 0x12000 / 2)
        COMBINE(&s->ram[0][offset - 0x10000 / 2], data, mem_mask);
    else if (offset >= 0x14000 / 2 && offset < 0x14400 / 2)
        COMBINE(&s->ram[0][offset - 0x12000 / 2], data, mem_mask);
}

uint16_t c355spr_position_r(c355spr_t *s, uint32_t offset)
{
    return s->position[offset & 3];
}

void c355spr_position_w(c355spr_t *s, uint32_t offset, uint16_t data, uint16_t mem_mask)
{
    COMBINE(&s->position[offset & 3], data, mem_mask);
}

void c355spr_vblank(c355spr_t *s, int state, const c355spr_rect *visible)
{
    if (state) {
        if (s->buffer > 0)
            c355spr_build_list_and_render(s, visible);
        if (s->buffer > 1)
            memcpy(s->ram[1], s->ram[0], sizeof(s->ram[0]));
    }
}

void c355spr_zdrawgfxzoom(c355spr_t *s, const c355spr_rect *clip, uint32_t code, uint32_t color, int flipx, int flipy,
                          int hpos, int vpos, int hsize, int vsize, uint8_t prival,
                          int sprite_screen_width, int sprite_screen_height)
{
    if (!hsize || !vsize) return;
    if (!s->gfx || !s->elements) return;

    if (sprite_screen_width && sprite_screen_height) {
        const uint32_t pal = s->colbase + (uint32_t)s->granularity * (color % (uint32_t)s->colors);
        const uint8_t *source_base = s->gfx + (size_t)(code % s->elements) * 256;
        const uint16_t pribits = (uint16_t)((prival & 0xf) << 12);
        const uint8_t transpen = s->transpen;

        /* compute sprite increment per screen pixel */
        int dx = (16 << 16) / sprite_screen_width;
        int dy = (16 << 16) / sprite_screen_height;
        int ex = hpos + sprite_screen_width;
        int ey = vpos + sprite_screen_height;
        int x_index_base, y_index;

        if (flipx) { x_index_base = (sprite_screen_width - 1) * dx; dx = -dx; }
        else x_index_base = 0;
        if (flipy) { y_index = (sprite_screen_height - 1) * dy; dy = -dy; }
        else y_index = 0;

        if (hpos < clip->min_x) { int pixels = clip->min_x - hpos; hpos += pixels; x_index_base += pixels * dx; }
        if (vpos < clip->min_y) { int pixels = clip->min_y - vpos; vpos += pixels; y_index += pixels * dy; }
        if (ex > clip->max_x + 1) ex = clip->max_x + 1;
        if (ey > clip->max_y + 1) ey = clip->max_y + 1;

        if (ex > hpos) {
            for (int y = vpos; y < ey; y++) {
                const uint8_t *source = source_base + (y_index >> 16) * 16;
                uint16_t *dest = s->bitmap + (size_t)y * s->width;
                int x_index = x_index_base;
                for (int x = hpos; x < ex; x++) {
                    const uint8_t c = source[x_index >> 16];
                    if (c != transpen)
                        dest[x] = (uint16_t)(pribits | ((pal + c) & 0xfff));
                    x_index += dx;
                }
                y_index += dy;
            }
        }
    }
}

/* get_single_sprite + render_sprites for one entry */
static void draw_single_sprite(c355spr_t *s, const uint16_t *ram, unsigned which, const c355spr_rect *cliprect)
{
    const uint16_t *tbl = ram + (which << 3);          /* 0x00000 sprite attr */
    const uint16_t *cliptbl = ram + 0x2400 / 2;        /* window attributes */
    const uint16_t *fmt;
    const uint16_t palette = tbl[6];
    const int priority = s->pri_cb ? s->pri_cb(s->user, palette) : ((palette >> 4) & 0xf);
    if (priority == -1)
        return;

    const unsigned fmtoff = tbl[0] & 0x7ff;
    const int offset = tbl[1];
    int hpos = tbl[2];
    int vpos = tbl[3];
    uint16_t hsize = tbl[4];
    uint16_t vsize = tbl[5];

    int xscroll = sext(s->position[1], 9);
    int yscroll = sext(s->position[0], 9);
    if (s->medium_res)
        xscroll = sext(s->position[1], 10);
    xscroll += s->scrolloffs[0];
    yscroll += s->scrolloffs[1];
    hpos -= xscroll;
    vpos -= yscroll;

    const int clipentry = (palette >> 8) & 0xf;
    const uint16_t *ce = cliptbl + (clipentry << 2);
    c355spr_rect clip;
    clip.min_x = ce[0] - xscroll;
    clip.max_x = ce[1] - xscroll;
    clip.min_y = ce[2] - yscroll;
    clip.max_y = ce[3] - yscroll;
    /* clip &= cliprect */
    if (clip.min_x < cliprect->min_x) clip.min_x = cliprect->min_x;
    if (clip.max_x > cliprect->max_x) clip.max_x = cliprect->max_x;
    if (clip.min_y < cliprect->min_y) clip.min_y = cliprect->min_y;
    if (clip.max_y > cliprect->max_y) clip.max_y = cliprect->max_y;

    hpos = sext(hpos & 0x7ff, 11);
    vpos = sext(vpos & 0x7ff, 11);

    fmt = ram + 0x4000 / 2 + (fmtoff << 2);
    int tile_index = fmt[0];
    const uint16_t format = fmt[1];
    const int dx = fmt[2] & 0x1ff;
    const int dy = fmt[3] & 0x1ff;
    int num_cols = (format >> 4) & 0xf;
    int num_rows = format & 0xf;

    if (num_cols == 0) num_cols = 0x10;
    const int flipx = (hsize & 0x8000) != 0;
    hsize &= 0x3ff;
    if (hsize == 0) return;
    uint32_t zoomx = ((uint32_t)hsize << 16) / (uint32_t)(num_cols * 16);
    int32_t dx_zoomed = (int32_t)(((uint32_t)(dx & 0xff) * zoomx + 0x8000) >> 16);
    if (dx & 0x100) dx_zoomed = -dx_zoomed;
    if (!flipx) hpos -= dx_zoomed;
    else hpos += dx_zoomed;

    if (num_rows == 0) num_rows = 0x10;
    const int flipy = (vsize & 0x8000) != 0;
    vsize &= 0x3ff;
    if (vsize == 0) return;
    uint32_t zoomy = ((uint32_t)vsize << 16) / (uint32_t)(num_rows * 16);
    int32_t dy_zoomed = (int32_t)(((uint32_t)(dy & 0xff) * zoomy + 0x8000) >> 16);
    if (dy & 0x100) dy_zoomed = -dy_zoomed;
    if (!flipy) vpos -= dy_zoomed;
    else vpos += dy_zoomed;

    const uint32_t color = palette & (s->colors - 1);
    /* 0x08000 tile table (indices past the end wrap inside the RAM; MAME would read past it) */

    uint32_t source_height_remaining = num_rows * 16;
    uint32_t screen_height_remaining = vsize;
    int y = vpos;
    for (int row = 0; row < num_rows; row++) {
        int tile_screen_height = (int)(16 * screen_height_remaining / source_height_remaining);
        zoomy = (screen_height_remaining << 16) / source_height_remaining;
        if (flipy) y -= tile_screen_height;
        uint32_t source_width_remaining = num_cols * 16;
        uint32_t screen_width_remaining = hsize;
        int x = hpos;
        for (int col = 0; col < num_cols; col++) {
            int tile_screen_width = (int)(16 * screen_width_remaining / source_width_remaining);
            zoomx = (screen_width_remaining << 16) / source_width_remaining;
            if (flipx) x -= tile_screen_width;
            const uint16_t tile = ram[(0x8000 / 2 + tile_index++) & (C355SPR_RAM_WORDS - 1)];
            if ((tile & 0x8000) == 0) {
                int ssw, ssh;
                if (s->alt_precision) {
                    int fx = (int)((uint32_t)x << 16), fy = (int)((uint32_t)y << 16);
                    ssw = ((fx + (int)(zoomx << 4)) >> 16) - (fx >> 16);
                    ssh = ((fy + (int)(zoomy << 4)) >> 16) - (fy >> 16);
                } else {
                    ssh = (int)(((int)zoomy * 16 + 0x8000) >> 16);
                    ssw = (int)(((int)zoomx * 16 + 0x8000) >> 16);
                }
                int t = tile;
                if (s->code2tile)
                    t = s->code2tile(s->user, t);
                c355spr_zdrawgfxzoom(s, &clip, (uint32_t)(t + offset), color, flipx, flipy, x, y,
                                     (int)zoomx, (int)zoomy, (uint8_t)priority, ssw, ssh);
            }
            if (!flipx) x += tile_screen_width;
            screen_width_remaining -= tile_screen_width;
            source_width_remaining -= 16;
        }
        if (!flipy) y += tile_screen_height;
        screen_height_remaining -= tile_screen_height;
        source_height_remaining -= 16;
    }
}

void c355spr_build_list_and_render(c355spr_t *s, const c355spr_rect *cliprect)
{
    const uint16_t *ram = s->ram[s->buffer > 1 ? 1 : 0];
    for (int y = cliprect->min_y; y <= cliprect->max_y; y++) {
        uint16_t *row = s->bitmap + (size_t)y * s->width;
        for (int x = cliprect->min_x; x <= cliprect->max_x; x++)
            row[x] = 0xffff;
    }
    for (int i = 0; i < 256; i++) {
        const uint16_t which = ram[0x2000 / 2 + i];
        draw_single_sprite(s, ram, which & 0xff, cliprect);
        if (which & 0x100) break;
    }
}

void c355spr_copy(c355spr_t *s, uint16_t *dest, int dest_pitch, uint8_t *destpri, int pri_pitch,
                  const c355spr_rect *clip, int pri)
{
    const uint16_t colbase = s->colbase;
    for (int y = clip->min_y; y <= clip->max_y; y++) {
        const uint16_t *src = s->bitmap + (size_t)y * s->width;
        uint16_t *d = dest + (size_t)y * dest_pitch;
        uint8_t *dp = destpri + (size_t)y * pri_pitch;
        if (!s->mix_cb) {
            /* default_mix */
            for (int x = clip->min_x; x <= clip->max_x; x++) {
                const uint16_t v = src[x];
                if (v != 0xffff && (v & 0xff) != 0xff)
                    d[x] = (uint16_t)(colbase + (v & 0xfff));
            }
        } else {
            for (int x = clip->min_x; x <= clip->max_x; x++) {
                const uint16_t v = src[x];
                if (v != 0xffff)
                    s->mix_cb(s->user, &d[x], &dp[x], colbase, (uint16_t)(v & 0xfff), (v >> 12) & 0xf, pri);
            }
        }
    }
}

void c355spr_draw(c355spr_t *s, uint16_t *dest, int dest_pitch, uint8_t *destpri, int pri_pitch,
                  const c355spr_rect *clip, int pri)
{
    if (pri == 0 && s->buffer == 0)
        c355spr_build_list_and_render(s, clip);
    c355spr_copy(s, dest, dest_pitch, destpri, pri_pitch, clip, pri);
}
