/*  Konami System GX: the board state shared by the driver (konamigx.c) and its video (konamigx_v.c).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_KONAMIGX_H
#define RBX_KONAMIGX_H

#include <stdint.h>
#include "../video/k056832.h"
#include "../video/k055673.h"
#include "../video/k053250.h"

// MAME's tile callbacks of the machines
enum { GX_TILECB_TYPE2, GX_TILECB_ALPHA, GX_TILECB_SALMNDR2,
       // (the Mystic Warriors boards, mystwarr_v.cpp: the layer's colour base from the K055555, no tile banks)
       GX_TILECB_MW, GX_TILECB_VIOSTORM, GX_TILECB_5BPP, GX_TILECB_4BPP, GX_TILECB_BISHI,
       GX_TILECB_PRE };   // (the board's pre_tilecb: the K053251 boards, gijoe ...)
// and the sprite callbacks
enum { GX_SPRCB_TYPE2, GX_SPRCB_DRAGOONJ, GX_SPRCB_SALMNDR2, GX_SPRCB_LE2,
       GX_SPRCB_MW, GX_SPRCB_METAMRPH, GX_SPRCB_GAIAPOLS, GX_SPRCB_MARTCHMP };

// the mixer's flags of a screen update (the tile alpha mix code; martchmp's blend mode)
// (GX_MIXMODE_LAYERS: bishi's screen update, the four layers in the K055555's order over the K054338's colour)
enum { GX_MIXMODE_ALPHA, GX_MIXMODE_MARTCHMP, GX_MIXMODE_LAYERS, GX_MIXMODE_PRE };

// the K053252 CCU's screen timing
typedef struct {
	uint8_t regs[16];
	int32_t hc, hfp, hbp, vc, vfp, vbp, hsw, vsw;
	int32_t lines, line_clocks;   // the frame: lines of line_clocks pixel clocks at clock
	uint32_t clock;
	int32_t vis_y0, vis_y1;       // the visible lines (vblank from vis_y1 + 1)
} gx_ccu_t;

typedef struct {
	// ---- the state saved as one block (up to state_end) ----
	uint8_t workram[0x20000];     // c00000 (host-order words)
	uint8_t palram[0x8000];       // d90000: 8192 xRGB_888 entries (host-order words, the high word first)
	uint16_t spriteram[0x800];    // the object DMA's copy of the sprite list
	uint8_t k55555[0x80];         // K055555 registers
	uint16_t k54338[0x10];        // K054338 registers
	uint8_t tilebanks[16];
	uint32_t fantjour_dma[8];     // (fantjour's DMA at db0000)
	gx_ccu_t ccu;
	uint8_t wrport1_0, wrport1_1, wrport2, rdport1_3, syncen, prev_pixel_clock;
	uint32_t ccu_clock;
	int64_t dma_due, cycles_done;
	uint64_t cyc_frac, snd_frac;
	uint32_t frame;
	int32_t line;
	uint8_t state_end;
	// ---- the chips (saved through their own state functions) and the configuration ----
	k056832_t k56832;
	k055673_t k55673;
	int tilecb, sprcb;
	int mw;            // a Mystic Warriors board (mystwarr.c): its tile ROM converted to the GX format
	int pal_entries;   // the palette's size (0: the GX's 8192; Mystic Warriors 2048)
	int spr_cbshift;   // (Mystic Warriors: the sprites' colour base = the K055555's palette index 4 << this)
	int spr_cbmask;    // (and masked with this when set: dadandrn, gaiapols 0x7f)
	int alpha_noinvert;   // (the K054338's alpha not inverted: martchmp)
	int mixmode;          // GX_MIXMODE_*
	int pal_bgr;          // (the palette xBGR_888: bishi)
	// ---- the pre-GX boards (GX_MIXMODE_PRE: MAME's bitmap_ind16 with the screen's priority bitmap, the K053247's
	// sprites by pdrawgfx; the board draws its frame with the gx_pre_* calls from its pre_draw) ----
	int pal555;           // (the palette xBGR_555 words)
	int pal555rgb;        // (xRGB_555 instead: dbz)
	void (*pre_tilecb)(int layer, int *code, int *color, int *flip, int *category);   // (flip: bit 0 x, bit 1 y)
	void (*pre_sprcb)(int *code, int *color, int *primask);
	void (*pre_capture)(void);   // (at vblank, the board's snapshot of what its drawing reads)
	void (*pre_draw)(void);      // (the frame: gx_pre_fill, gx_pre_tilemap, gx_pre_sprites)
	int pre_rgb;          // (MAME's bitmap_rgb32 instead: moo; the K054338's colour, shadows and alpha)
	int pre_screen_w;     // (the screen's width: the K053246's half-width sprites when flipped)
	// (the ind16 frame's palette adjusted, set by the board's drawing: set_pen_contrast of each block of 256 entries,
	// set_shadow_mode 1 (the shadow pens the highlights); pre_dim off: neither)
	int pre_dim, pre_highlight;
	float pre_contrast[8];
	float pre_shadow_factor;   // (set_shadow_factor: 0 the default 0.6)
	float pre_highlight_factor;   // (set_highlight_factor: 0 the default 1 / 0.6)
	int pre_shadow_red;         // (surpratk's palette_w: the shadow pens with only red at 0.6)
	int pre_k245_prishadow;     // (K053244 set_priority_shadows(true))
	k053250_t *k53250;    // (the sub layer 1 a K053250: metamrph; NULL none)
	// (the sub layer 1 a K053936 ROZ (K053936GP_0_zoom_draw) the board draws into the RGB frame: dadandrn, gaiapols;
	// frame: the visible area (VIS_W wide from VIS_X0, VIS_Y0), colorbase: the K055555's SUB1 palette index)
	void (*sub1_roz)(uint32_t *frame, int vis_x0, int vis_y0, int vis_w, int vis_h, int band_y0, int band_y1,
	                 const uint32_t *pens, int colorbase, int blend, int alpha);   // (band: the rows it may write)
	int bands;   // (the mixer's frame in this many bands drawn by the render thread and its helpers: gx_render_help)
	int pre_bands;   // (the pre-GX RGB drawing in this many bands, the board's draw recorded and replayed: moo)
	int help_worker_only;   // (the bands' helper only the idle first worker (rbx_aux_set_idle_help): not the busy
	                        // emulation thread nor CPU3's frontend helper, the DASP's core)
} gx_t;
extern gx_t gx;

// konamigx_v.c
int gx_video_init(void);
void gx_video_exit(void);
void gx_video_capture(void);
int gx_render_help(int helper);   // (rbx_driver.render_help: the bands of gx.bands)   // at vblank: what the next frame's drawing needs
void gx_render(void);
void gx_video_set_modes(int tilemode, int primode);
void gx_video_set_area(int x0, int w);   // the visible area (call before gx_video_init)   // (MAME video start: the tile mode, the mixer's priority mode)
void gx_video_set_y0(int y0);
void gx_video_set_height(int h);   // (the visible lines, 224 unless set after set_area: xexex 256)
// the pre-GX frame (from pre_draw): the black pen and priority 0, a layer with its priority code, the K053247's sprites
void gx_pre_fill(void);
void gx_pre_fill_pen(int pen);
void gx_pre_tilemap_opaque(int layer, int priority);   // (TILEMAP_DRAW_OPAQUE)
// the ind16 frame (the visible area, its width a line) and the priority bitmap: a board's own layers (dbz's ROZ)
uint16_t *gx_pre_frame16(void);
void gx_pre_call(void (*fn)(int arg, int y0, int y1), int arg);   // (a board's own layer over rows y0..y1 of the frame)
uint8_t *gx_pre_pri(void);   // (the ind16 frame filled with a pen: asterix's 0)
void gx_pre_tilemap(int layer, int priority);
void gx_pre_sprites(void);
// (the K053244/053245's sprites instead: its buffer (0x400 words) and registers, its offsets, its z rejection)
void gx_pre_sprites_053245(const uint16_t *buffer, const uint8_t *regs, int dx, int dy, int z_rejection);
// (a K051960's instead: its buffer (0x400 bytes), its control and shadow registers, the priority range (-1, -1: the
// priority bitmap), the board's callback (code, color, priority, shadow) and the shadows by priority)
void gx_pre_sprites_051960(const uint8_t *buffer, int control, int shadow_config, int min_priority, int max_priority,
                           void (*cb)(int *code, int *color, int *priority, int *shadow), int priority_shadows);
const k056832_t *gx_pre_k056832(void);
// (the RGB pre-GX frame: a layer blended at alpha, the K054338's update_all_shadows, its alpha level and registers)
void gx_pre_tilemap_alpha(int layer, int priority, int alpha);
void gx_pre_shadows(void);
void gx_pre_tilemap_cat(int layer, int priority, int all_categories, int category, int alpha);   // (TILEMAP_DRAW_CATEGORY)
void gx_pre_lvc(int colorbase, int priority);   // (the K053250 (gx.k53250) as a layer of the RGB frame)
int gx_pre_alpha_level(int blend);
const uint16_t *gx_pre_k054338(void);   // (the snapshot's K056832: its registers for the board's drawing)   // (the CCU's y offset: the visible area's first line, 16 unless set after set_area)

#endif
