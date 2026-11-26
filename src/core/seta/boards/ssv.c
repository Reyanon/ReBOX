/*  Seta / Sammy / Visco (SSV) System: an NEC V60, the Ensoniq ES5506 and the ST-0006 / ST-0007 sprite chip (every
    layer is a sprite, the 64 pixel high "tilemap sprite" rows too; shadows). Ported from MAME
    src/mame/seta/ssv.cpp, ssv.h, ssv_v.cpp (BSD-3-Clause, copyright-holders: Luca Elia, Roberto Zandona, David Haywood)
    and src/mame/seta/st0020.cpp (BSD-3-Clause, copyright-holders: Luca Elia, David Haywood: the ST-0020 of gdfs).
    The uPD96050 (ST010) of drifto94 / stmblade / twineag2 is src/core/cpu/upd7725, the ES5506 src/core/sound/es5506.
    The machine's devices after MAME: the uPD4701 trackball counter of eaglshot (src/devices/machine/upd4701.cpp,
    BSD-3-Clause, copyright-holders: AJR), the uPD7001 serial A/D converter of the sxyreact paddle (upd7001.cpp,
    BSD-3-Clause, copyright-holders: AJR) and the ADC0809 of the gdfs light guns (adc0808.cpp, BSD-3-Clause,
    copyright-holders: Dirk Best).
    Not run: jsk (its V810 sub CPU is not emulated here).
    The frame is drawn as MAME draws it, all at the vblank (MAME's screen_update), from a copy of the sprite RAM,
    the scroll registers and the palette, on the render side; a scroll write in the visible lines that changes a
    register ends a band first (MAME's partial update in scroll_w). gdfs and eaglshot (their sprites come from RAM)
    are drawn on the emulation thread at the vblank.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/v60/v60.h"
#include "../../cpu/upd7725/upd7725.h"
#include "../../machine/eeprom93c46.h"
#include "../../sound/es5506.h"
#include "../../video/decoband.h"
#include "../../video/zoomgfx.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CLOCK 16000000u              // (XTAL(48'000'000) / 3: the V60 and the ES5506)
#define LINES 262                    // (SSV_VTOTAL)
// a line in V60 cycles: 16 MHz * 0x1c6 / (42.954545 MHz / 6) = LINE_NUM / LINE_DEN
#define LINE_NUM 43584000000ull
#define LINE_DEN 42954545ull
#define FRAME_MHZ 60188u             // (60.1878 Hz)
#define PENS 0x8000
#define SPR_WORDS 0x20000
#define MAX_BANDS 32
#define BANDS 8
#define OUT_PERIOD 512               // (the output rate: 16 MHz / 512 = 31250 Hz, the ES5506's with 32 voices)

enum { K_DRIFTO94, K_GDFS, K_HYPREACT, K_HYPREAC2, K_JANJANS1, K_KEITHLCY, K_MEOSISM, K_MSLIDER, K_RYORIOH, K_SRMP4,
       K_SRMP7, K_SURVARTS, K_CAIRBLAD, K_SXYREACT, K_TWINEAG2, K_ULTRAX, K_EAGLSHOT };

typedef struct {
	const char *name;
	int kind;
	uint32_t rom;           // ssv_map(map, rom): the ROM at rom .. 0xffffff
	int vis_w, vis_h;
	int dsp;                // a uPD96050
	uint32_t nv_at, nv_size;   // the NVRAM (nvram_device::DEFAULT_ALL_0)
} machine_t;

static const machine_t s_machines[] = {
	{ "drifto94", K_DRIFTO94, 0xc00000, (0xcd - 0x25) * 2, 0x101 - 0x13, 1, 0x580000, 0x800 },
	{ "stmblade", K_DRIFTO94, 0xc00000, (0xd6 - 0x26) * 2, 0xfe - 0x0e, 1, 0x580000, 0x800 },
	{ "gdfs", K_GDFS, 0xc00000, (0xd5 - 0x2c) * 2, 0x102 - 0x12, 0, 0, 0 },
	{ "hypreact", K_HYPREACT, 0xf00000, (0xcb - 0x22) * 2, 0xfe - 0x0e, 0, 0, 0 },
	{ "hypreac2", K_HYPREAC2, 0xe00000, (0xcb - 0x22) * 2, 0xfe - 0x0e, 0, 0, 0 },
	{ "janjans1", K_JANJANS1, 0xc00000, (0xcb - 0x23) * 2, 0xfe - 0x0e, 0, 0, 0 },
	{ "keithlcy", K_KEITHLCY, 0xe00000, (0xcd - 0x25) * 2, 0x101 - 0x13, 0, 0, 0 },
	{ "pastelis", K_KEITHLCY, 0xe00000, (0xd6 - 0x26) * 2, 0xfe - 0x0e, 0, 0, 0 },
	{ "meosism", K_MEOSISM, 0xf00000, (0xd5 - 0x2c) * 2, 0xfe - 0x12, 0, 0x580000, 0x10000 },
	{ "mslider", K_MSLIDER, 0xf00000, (0xd6 - 0x26) * 2, 0xfe - 0x0e, 0, 0, 0 },
	{ "ryorioh", K_RYORIOH, 0xc00000, (0xcb - 0x23) * 2, 0xfe - 0x0e, 0, 0, 0 },
	{ "vasara", K_RYORIOH, 0xc00000, (0xcc - 0x24) * 2, 0xfe - 0x0e, 0, 0, 0 },
	{ "srmp4", K_SRMP4, 0xf00000, (0xd4 - 0x2c) * 2, 0x102 - 0x12, 0, 0, 0 },
	{ "srmp7", K_SRMP7, 0xc00000, (0xd4 - 0x2c) * 2, 0xfd - 0x0e, 0, 0, 0 },
	{ "survarts", K_SURVARTS, 0xf00000, (0xd4 - 0x2c) * 2, 0x102 - 0x12, 0, 0, 0 },
	{ "dynagear", K_SURVARTS, 0xf00000, (0xd4 - 0x2c) * 2, 0x102 - 0x12, 0, 0, 0 },
	{ "cairblad", K_CAIRBLAD, 0xe00000, (0xcb - 0x22) * 2, 0xfe - 0x0e, 0, 0x580000, 0x10000 },
	{ "sxyreact", K_SXYREACT, 0xe00000, (0xcb - 0x22) * 2, 0xfe - 0x0e, 0, 0x580000, 0x10000 },
	{ "sxyreac2", K_SXYREACT, 0xe00000, (0xcb - 0x23) * 2, 0xfe - 0x0e, 0, 0x580000, 0x10000 },
	{ "twineag2", K_TWINEAG2, 0xe00000, (0xd4 - 0x2c) * 2, 0x102 - 0x12, 1, 0, 0 },
	{ "ultrax", K_ULTRAX, 0xe00000, (0xd4 - 0x2c) * 2, 0x102 - 0x12, 0, 0, 0 },
	{ "eaglshot", K_EAGLSHOT, 0xf00000, (0xca - 0x2a) * 2, 0xf6 - 0x16, 0, 0xc00000, 0x800 },
};

enum { P_DSW1, P_DSW2, P_P1, P_P2, P_SYSTEM, P_ADD, P_SERVICE, P_PADDLE, P_TRACKX, P_TRACKY, P_GUNX1, P_GUNY1,
       P_GUNX2, P_GUNY2, P_KEY0, P_COUNT = P_KEY0 + 4 };
static const char *const s_port_tags[P_KEY0] = { "DSW1", "DSW2", "P1", "P2", "SYSTEM", "ADD_BUTTONS", "SERVICE",
	"PADDLE", "TRACKX", "TRACKY", "GUNX1", "GUNY1", "GUNX2", "GUNY2" };

enum { SV_PA_V60 = RBX_PA_DRV, SV_PA_DSP, SV_PA_SND, SV_PA_DRAW, SV_PA_END };
static const char *const s_pa_names[] = { "v60", "dsp", "es5506", "draw" };

static const machine_t *s_m;
static int s_kind;
static int s_sxyreac2;
static uint8_t *s_rom;
static uint32_t s_rom_size;
static uint8_t *s_gfx;               // the sprites, decoded in place: 16x8 tiles of 128 bytes, a byte a pixel
static uint32_t s_gfx_tiles;
static uint8_t *s_ram;               // 0x000000-0x00ffff
static uint8_t *s_ram2;              // the boards' more RAM (survarts 0x400000, srmp7 / twineag2 / ultrax 0x010000)
static uint32_t s_ram2_at, s_ram2_size, s_ram2_alloc;
static uint8_t *s_ram16;             // 0x160000-0x17ffff
static uint16_t *s_spr;              // 0x100000-0x13ffff (V60 order: little-endian words, host order here)
static uint32_t *s_pal;              // 0x140000-0x15ffff (xRGB_888 as two words: a 32-bit LE entry)
static uint8_t *s_nv;
static uint16_t *s_snap_spr;         // the drawing's copies
static uint32_t *s_snap_pal;
// (gdfs / eaglshot) the drawing's copies of the RAM graphics, their pages written since the last copy
static uint8_t *s_snap_st_gfx, *s_snap_ea_gfx;
static uint16_t *s_snap_st_spr, *s_snap_tmap;
static uint16_t s_snap_tmapscroll[2];
static uint32_t s_dirty_stgfx[1024 / 32], s_dirty_stspr[128 / 32], s_dirty_tmap[32 / 32], s_dirty_eagfx[1024 / 32];
static char s_nv_path[512];
static const uint8_t *s_es_rom[4];
static uint32_t s_es_size[4];
static int s_port[P_COUNT];
static uint32_t s_tile_code[16];
static int s_irq_ultrax, s_raster_irq;

static es5506_t s_es;
static upd96050_t s_dsp;
static eeprom93c46_t s_eeprom;
static char s_ee_path[512];

// gdfs: the ST-0020 and its tilemap
static uint8_t *s_st_gfx;            // gfxram (4 x 1 MB, a byte a pixel: 16x8 tiles of 128 bytes)
static uint16_t *s_st_spr;           // 0x80000 bytes
static uint16_t *s_tmapram;          // 0x400000-0x41ffff
static uint8_t *s_gd_ram;            // 0x420000-0x43ffff, 0x600000-0x600fff
static const uint8_t *s_st_rom;      // the blitter's source ("st0020_spr")
static uint32_t s_st_rom_size;
static const uint8_t *s_gd_tiles;    // "tiles": 16x16 8bpp raw
static uint32_t s_gd_tile_count;
static uint8_t *s_st_opaque;
static zoomgfx_t s_st_zg;

// eaglshot
static uint8_t *s_ea_gfx;            // gfxram: 16 x 0x40000 bytes, 16x8 tiles of 128 bytes
static const uint8_t *s_ea_rom;      // "gfxdata"
static uint32_t s_ea_rom_size;

// the saved state that is not memory
static struct {
	uint64_t cyc;                    // V60 cycles before the current slice
	uint64_t lines;                  // lines since the reset
	uint64_t line_start;             // the cycle the current line started at
	int line;                        // the line in the frame
	uint16_t scroll[0x40];           // 0x1c0000-0x1c007f
	uint16_t irqv[0x3a];             // 0x230000-0x230073
	uint16_t irq_enable, input_sel;
	uint8_t requested_int, enable_video, irq_line, pad0;
	uint32_t latches[8];
	uint32_t rand;
	// sound
	uint64_t snd_next, out_next;     // the next ES5506 sample's cycle, the next output sample's
	int32_t snd_l, snd_r;
	uint32_t srmp7_bank;
	// DSP
	int64_t dsp_done;                // its instructions run (at 10 MHz: 5 / 8 of the V60's cycles)
	// eaglshot
	uint8_t gfxrom_bank, ud_cs, ud_xy, ud_ul, ud_resetx, ud_resety, pad1[2];
	uint16_t ud_x, ud_y, ud_startx, ud_starty, ud_latchx, ud_latchy;
	// sxyreact (uPD7001)
	uint8_t ad_cs_active, ad_eoc_active, ad_oe, ad_sck, ad_si, ad_so, ad_dl, ad_sr, ad_mpx, pad2[3];
	uint64_t ad_conv_at, ad_oe_at;   // (0: not running)
	int paddle;
	// gdfs: ADC0809, ST-0020
	uint8_t adc_state, adc_start, adc_addr, adc_sar, adc_eoc, pad3[3];
	uint64_t adc_at;                 // the next step (0: idle)
	int gun[2][2];
	uint16_t tmapscroll[0x20];
	uint16_t st_regs[0x80];
	uint8_t st_bank, pad4[3];
} s_st;

// the bands of a frame (scroll writes in the visible lines) and the copy the drawing works from
typedef struct { int y1; uint16_t scroll[0x40]; } band_t;
static band_t s_bands[MAX_BANDS];
static int s_nbands, s_band_y;       // (bands this frame, the first line not in a band)
static band_t s_snap_bands[MAX_BANDS];
static int s_snap_nbands;
static int s_snap_enable;
static int s_pal_dirty;
static int s_in_exec;
static int s_irq_dirty;

static inline uint64_t cpu_now(void) { return s_st.cyc + (s_in_exec ? (uint64_t)v60_cycles_run() : 0); }

static inline uint16_t rd16(const uint8_t *p, uint32_t a) { return (uint16_t)(p[a] | p[a + 1] << 8); }
static inline void wr16(uint8_t *p, uint32_t a, uint16_t v, uint16_t mask)
{
	if (mask & 0x00ff) p[a] = (uint8_t)v;
	if (mask & 0xff00) p[a + 1] = (uint8_t)(v >> 8);
}
static inline uint16_t combine(uint16_t old, uint16_t v, uint16_t mask) { return (uint16_t)((old & ~mask) | (v & mask)); }

// ---------------------------------------------------------------------------------------------------- drawing
typedef struct {
	uint16_t *bm;
	int pitch;
	int min_x, max_x, min_y, max_y;
} clip_t;

typedef struct {
	const uint16_t *spr, *scroll;
	const uint8_t *gfx;
	uint32_t tiles;
	uint32_t shadow_mask, shadow_shift;
	int vis_max_y;
} drawctx_t;

typedef struct { uint8_t mask, shift; } drawmode_t;
static const drawmode_t BPP_MASK_TABLE[8] = {
	{ 0x3f, 0 }, { 0xff, 0 }, { 0x3f, 0 }, { 0xff, 0 }, { 0x0f, 0 }, { 0xf0, 4 }, { 0x3f, 0 }, { 0xff, 0 }
};

// a row of a 16x8 tile's pixels, columns c0..c1 (column k at x + k, or x + 15 - k flipped); d = the line at x.
// pen = (byte & mask) >> shift, 0 transparent, else (realcolor + pen) & 0x7fff
static inline void paint_row(uint16_t *d, const uint8_t *src, int c0, int c1, int flipx, uint32_t realcolor, uint32_t mask,
                             uint32_t shift)
{
	if (shift == 0 && realcolor + mask <= 0x7fff) {   // (no wrap past 0x7fff: the common case)
		const uint8_t m = (uint8_t)mask;
		const uint16_t rc = (uint16_t)realcolor;
		if (c0 == 0 && c1 == 15) {   // (the whole row, 4 pixels at a time: skipped when all transparent)
			const uint32_t m4 = m * 0x01010101u;
			for (int q = 0; q < 16; q += 4) {
				uint32_t w;
				memcpy(&w, src + q, 4);
				w &= m4;
				if (!w)
					continue;
				const uint16_t p0 = (uint16_t)(w & 0xff), p1 = (uint16_t)((w >> 8) & 0xff), p2 = (uint16_t)((w >> 16) & 0xff),
				               p3 = (uint16_t)(w >> 24);
				if (!flipx) {
					uint16_t *e = d + q;
					if (p0) e[0] = (uint16_t)(rc + p0);
					if (p1) e[1] = (uint16_t)(rc + p1);
					if (p2) e[2] = (uint16_t)(rc + p2);
					if (p3) e[3] = (uint16_t)(rc + p3);
				} else {
					uint16_t *e = d + 15 - q;
					if (p0) e[0] = (uint16_t)(rc + p0);
					if (p1) e[-1] = (uint16_t)(rc + p1);
					if (p2) e[-2] = (uint16_t)(rc + p2);
					if (p3) e[-3] = (uint16_t)(rc + p3);
				}
			}
			return;
		}
		if (!flipx) {
			for (int k = c0; k <= c1; k++) {
				const uint8_t pen = src[k] & m;
				if (pen)
					d[k] = (uint16_t)(rc + pen);
			}
		} else {
			uint16_t *e = d + 15;
			for (int k = c0; k <= c1; k++) {
				const uint8_t pen = src[k] & m;
				if (pen)
					e[-k] = (uint16_t)(rc + pen);
			}
		}
		return;
	}
	if (!flipx) {
		for (int k = c0; k <= c1; k++) {
			const uint32_t pen = (uint32_t)(src[k] & mask) >> shift;
			if (pen)
				d[k] = (uint16_t)((realcolor + pen) & 0x7fff);
		}
	} else {
		uint16_t *e = d + 15;
		for (int k = c0; k <= c1; k++) {
			const uint32_t pen = (uint32_t)(src[k] & mask) >> shift;
			if (pen)
				e[-k] = (uint16_t)((realcolor + pen) & 0x7fff);
		}
	}
}

// lines l0..l1 of a 16x8 tile at (base_sx, ty) (MAME's drawgfx_line for each, the tile looked up once)
static inline void drawgfx_tile(const drawctx_t *c, const clip_t *cl, int gfx, uint32_t code, uint32_t color, int flipx, int flipy,
                                int base_sx, int shadow, int ty, int l0, int l1)
{
	if (ty + l0 < cl->min_y)
		l0 = cl->min_y - ty;
	if (ty + l1 > cl->max_y)
		l1 = cl->max_y - ty;
	if (l0 > l1)
		return;
	// the columns inside the clip
	int c0 = 0, c1 = 15;
	if (!flipx) {
		if (base_sx < cl->min_x) c0 = cl->min_x - base_sx;
		if (base_sx + 15 > cl->max_x) c1 = cl->max_x - base_sx;
	} else {   // (column k at base_sx + 15 - k)
		if (base_sx + 15 > cl->max_x) c0 = base_sx + 15 - cl->max_x;
		if (base_sx < cl->min_x) c1 = base_sx + 15 - cl->min_x;
	}
	if (c0 > c1)
		return;
	const uint8_t *tile = c->gfx + (size_t)rbx_wrap(code, c->tiles) * 128;
	const uint32_t mask = BPP_MASK_TABLE[gfx & 7].mask, shift = BPP_MASK_TABLE[gfx & 7].shift;
	const uint32_t realcolor = 64 * (color & 511);
	for (int line = l0; line <= l1; line++) {
		const uint8_t *src = tile + (flipy ? 7 - line : line) * 16;
		uint16_t *d = cl->bm + (size_t)(ty + line) * cl->pitch + base_sx;
		if (shadow) {
			const uint32_t sm = c->shadow_mask, ss = c->shadow_shift;
			const int dx = flipx ? -1 : 1;
			d += flipx ? 15 - c0 : c0;
			for (int k = c0; k <= c1; k++, d += dx) {
				const uint32_t pen = (uint32_t)(src[k] & mask) >> shift;
				if (pen)
					*d = (uint16_t)(((*d & sm) | (pen << ss)) & 0x7fff);
			}
		} else
			paint_row(d, src, c0, c1, flipx, realcolor, mask, shift);
	}
}

static inline void drawgfx_line(const drawctx_t *c, const clip_t *cl, int gfx, uint32_t code, uint32_t color, int flipx, int flipy,
                                int base_sx, int shadow, int realline, int line)
{
	drawgfx_tile(c, cl, gfx, code, color, flipx, flipy, base_sx, shadow, realline - line, line, line);
}

static inline void get_tile(const drawctx_t *c, int x, int y, int size, int page, uint32_t *code, int *attr, int *flipx, int *flipy)
{
	const uint16_t *s3 = &c->spr[(page * (size * 4) + ((x & ((size - 1) & ~0xf)) << 2) + ((y & ((0x200 - 1) & ~0xf)) >> 3)) & 0x1ffff];
	uint32_t cd = s3[0];
	const int a = s3[1];
	cd += s_tile_code[(a & 0x3c00) >> 10];
	int fy = (a >> 14) & 1, fx = (a >> 15) & 1;
	const int s74 = c->scroll[0x74 / 2];
	if ((s74 & 0x1000) && !(s74 & 0x2000))
		fx = !fx;
	if ((s74 & 0x4000) && !(s74 & 0x2000))
		fy = !fy;
	*code = cd;
	*attr = a;
	*flipx = fx;
	*flipy = fy;
}

static void draw_row_64pixhigh(const drawctx_t *c, const clip_t *cliprect, int in_sy, int scrollreg)
{
	scrollreg &= 7;
	in_sy = (in_sy & 0x1ff) - (in_sy & 0x200);
	clip_t out = *cliprect;
	// outclip.set(0, 0x200, in_sy, in_sy + 0x40) and its checks against cliprect
	if (0 > cliprect->max_x || in_sy > cliprect->max_y || 0x200 < cliprect->min_x || in_sy + 0x40 < cliprect->min_y)
		return;
	out.min_x = 0 > cliprect->min_x ? 0 : cliprect->min_x;
	out.max_x = 0x200 < cliprect->max_x ? 0x200 : cliprect->max_x;
	out.min_y = in_sy > cliprect->min_y ? in_sy : cliprect->min_y;
	out.max_y = in_sy + 0x40 < cliprect->max_y ? in_sy + 0x40 : cliprect->max_y;
	const uint16_t *sc = c->scroll;
	const int mode = sc[scrollreg * 4 + 3];
	if ((mode & 0xe000) == 0)
		return;
	const int sizeshift = 8 + ((mode & 0xe000) >> 13);
	const int size = 1 << sizeshift;
	const int unknown = sc[scrollreg * 4 + 2];
	const int shadow = (mode >> 11) & 1;
	const int gfx = (mode & 0x0700) >> 8;
	const uint32_t mask = BPP_MASK_TABLE[gfx].mask, shift = BPP_MASK_TABLE[gfx].shift;
	const int s74 = sc[0x74 / 2];
	const int gflipx = (s74 & 0x1000) && !(s74 & 0x2000), gflipy = (s74 & 0x4000) && !(s74 & 0x2000);
	for (int line = out.min_y; line <= out.max_y; line++) {
		clip_t cl = out;
		cl.min_y = cl.max_y = line;
		int scrollx = sc[scrollreg * 4 + 0];
		int scrolly = sc[scrollreg * 4 + 1];
		const int page = (scrollx & 0x7fff) >> sizeshift;
		scrolly += in_sy;
		scrolly += ((sc[0x70 / 2] & 0x1ff) - (sc[0x70 / 2] & 0x200) + sc[0x6a / 2] + 2);
		if ((unknown & 0x05ff) == 0x0440) scrollx += -0x10;
		if ((unknown & 0x05ff) == 0x0401) scrollx += -0x20;
		const int realy = scrolly + (line - in_sy);
		if (mode & 0x1000) {
			const uint32_t base = (uint32_t)((mode & 0x00ff) * 0x400) / 2;
			scrollx += c->spr[(base + (uint32_t)(realy & 0x1ff)) & 0x1ffff];
		}
		int sx1 = 0 - (scrollx & 0xf);
		int x = scrollx;
		const int tileline = realy & 0xf;
		while (sx1 + 15 < cl.min_x) {   // (tiles left of the clip draw nothing)
			sx1 += 0x10;
			x += 0x10;
		}
		// the line's tiles (get_tile + draw_16x16_tile_line + drawgfx_line, what is the same along the line hoisted)
		uint16_t *dl = cl.bm + (size_t)line * cl.pitch;
		const int sbase = page * (size * 4) + ((realy & 0x1f0) >> 3);
		const int xmask = (size - 1) & ~0xf;
		for (int sx = sx1; sx <= cl.max_x; sx += 0x10, x += 0x10) {
			const uint16_t *s3 = &c->spr[(sbase + ((x & xmask) << 2)) & 0x1ffff];
			const int a = s3[1];
			const uint32_t code = s3[0] + s_tile_code[(a & 0x3c00) >> 10];
			const int flipy = ((a >> 14) & 1) ^ gflipy, flipx = ((a >> 15) & 1) ^ gflipx;
			const uint32_t realcode = flipy ? ((tileline & 8) ? code : code + 1) : ((tileline & 8) ? code + 1 : code);
			const int tl = tileline & 7;
			const uint8_t *src = c->gfx + (size_t)rbx_wrap(realcode, c->tiles) * 128 + (flipy ? 7 - tl : tl) * 16;
			int c0 = 0, c1 = 15;
			if (sx < cl.min_x || sx + 15 > cl.max_x) {
				if (!flipx) {
					if (sx < cl.min_x) c0 = cl.min_x - sx;
					if (sx + 15 > cl.max_x) c1 = cl.max_x - sx;
				} else {
					if (sx + 15 > cl.max_x) c0 = sx + 15 - cl.max_x;
					if (sx < cl.min_x) c1 = sx + 15 - cl.min_x;
				}
				if (c0 > c1)
					continue;
			}
			if (shadow) {
				const uint32_t sm = c->shadow_mask, ss = c->shadow_shift;
				const int dx = flipx ? -1 : 1;
				uint16_t *d = dl + sx + (flipx ? 15 - c0 : c0);
				for (int k = c0; k <= c1; k++, d += dx) {
					const uint32_t pen = (uint32_t)(src[k] & mask) >> shift;
					if (pen)
						*d = (uint16_t)(((*d & sm) | (pen << ss)) & 0x7fff);
				}
			} else
				paint_row(dl + sx, src, c0, c1, flipx, 64 * (uint32_t)(a & 511), mask, shift);
		}
	}
}

// ---- the frame's drawing as commands, made once a frame and sorted into the bands they touch (in their order), so
// a band does not walk the sprite lists again: a 16x8 tile, a 64 pixel high tilemap row, a zoomed ST-0020 tile
enum { CMD_TILE, CMD_ROW };
typedef struct { uint32_t code; int16_t x, y; uint16_t color; uint8_t kind, flags, sb, pad; } cmd_t;
typedef struct { uint32_t code, base, scalex, scaley; int16_t x, y; uint8_t flipx, flipy, pad[2]; } zcmd_t;
typedef struct { void *p; int n, cap, size; } cmdbuf_t;
static cmdbuf_t s_cmd[BANDS], s_zcmd[BANDS];
static int s_job_lo[BANDS + 1];

static void *cmd_push(cmdbuf_t *b)
{
	if (b->n == b->cap) {
		const int cap = b->cap ? b->cap * 2 : 1024;
		void *p = realloc(b->p, (size_t)cap * (size_t)b->size);
		if (!p)
			return NULL;
		b->p = p;
		b->cap = cap;
	}
	return (uint8_t *)b->p + (size_t)(b->n++) * (size_t)b->size;
}

// into the bands lines y0..y1 touch
static inline void emit(cmdbuf_t *bufs, int y0, int y1, const void *cmd, int size)
{
	for (int j = 0; j < BANDS; j++)
		if (y1 >= s_job_lo[j] && y0 < s_job_lo[j + 1]) {
			void *d = cmd_push(&bufs[j]);
			if (d)
				memcpy(d, cmd, (size_t)size);
		}
}

static int s_emit_sb;   // (the scroll band being made)
static void emit_row(const clip_t *cl, int in_sy, int scrollreg)
{
	const int y = (in_sy & 0x1ff) - (in_sy & 0x200);   // (draw_row_64pixhigh's lines: y .. y + 0x40)
	const int y0 = y > cl->min_y ? y : cl->min_y, y1 = y + 0x40 < cl->max_y ? y + 0x40 : cl->max_y;
	if (y0 > y1)
		return;
	const cmd_t c = { 0, (int16_t)scrollreg, (int16_t)in_sy, 0, CMD_ROW, 0, (uint8_t)s_emit_sb, 0 };
	emit(s_cmd, y0, y1, &c, sizeof(c));
}

static void draw_layer(const drawctx_t *c, const clip_t *cl, int nr)
{
	for (int sy = 0; sy <= c->vis_max_y; sy += 0x40)
		emit_row(cl, sy, nr);
}

static void draw_sprites(const drawctx_t *c, const clip_t *cl)
{
	const uint16_t *spr = c->spr, *sc = c->scroll;
	const int s74 = sc[0x74 / 2], s76 = sc[0x76 / 2], s7a = sc[0x7a / 2];
	const int use_local = (s76 >> 14) & 1;
	const int sprites_offsx = ((s74 & 0x7f) - (s74 & 0x80));
	const int sprites_offsy = -((sc[0x70 / 2] & 0x1ff) - (sc[0x70 / 2] & 0x200) + sc[0x6a / 2] + 1);
	for (int g = 0; g < 0x2000 / 2; g += 4) {
		const uint16_t *glob = spr + g;
		const int sprite = glob[1];
		if (sprite & 0x8000)
			break;
		uint32_t li = (uint32_t)(sprite & 0x7fff) * 4;
		const int tilemaps_offsy = ((spr[li + 3] & 0x1ff) - (spr[li + 3] & 0x200));
		const int mode = glob[0];
		const int local_num = mode & 0x001f;
		for (int count = 0; count <= local_num; count++, li += 4) {
			if (li >= SPR_WORDS)
				break;
			const uint16_t *loc = spr + li;
			int sx = loc[2];
			int sy = loc[3];
			int xnum = use_local ? (sx & 0x0c00) : (mode & 0x0c00);
			int ynum = use_local ? (sy & 0x0c00) : (mode & 0x0300) << 2;
			const int depth = use_local ? (sx & 0xf000) : (mode & 0xf000);
			if (loc[0] <= 7 && loc[1] == 0 && xnum == 0 && ynum == 0x0c00) {
				// a tilemap sprite
				const int scroll = loc[0];
				if (s76 & 0x1000)
					sy -= 0x20;
				else if (s7a & 0x0800) {
					if (s7a & 0x1000)
						sy -= tilemaps_offsy;
					else
						sy += tilemaps_offsy;
				}
				if (local_num != 0 && loc[0] != 0)
					emit_row(cl, sy, scroll);
			} else {
				uint32_t code = loc[0];
				const int attr = loc[1];
				code += s_tile_code[(attr & 0x3c00) >> 10];
				int flipy = (attr >> 14) & 1, flipx = (attr >> 15) & 1;
				if ((s74 & 0x1000) && !(s74 & 0x2000))
					flipx = !flipx;
				if ((s74 & 0x4000) && !(s74 & 0x2000))
					flipy = !flipy;
				const int gfx = (depth & 0x7000) >> 12;
				const int shadow = (depth >> 15) & 1;
				const int scrollreg = (mode & 0x00e0) >> 4;
				sx += glob[2] + sc[scrollreg + 0x40 / 2];
				sy += glob[3] + sc[scrollreg + 0x42 / 2];
				sx = (sx & 0x1ff) - (sx & 0x200);
				sy = (sy & 0x1ff) - (sy & 0x200);
				if (s74 & 0x4000) {
					sy = -sy;
					if (!(s74 & 0x8000))
						sy -= 0x10;
				}
				if (s74 & 0x1000)
					sx = -sx + 0x100;
				xnum = 1 << (xnum >> 10);
				ynum = 1 << (ynum >> 10);
				if (s7a == 0x7140) {
					sx = sprites_offsx + sx;
					sy = sprites_offsy - sy;
				} else if (s7a & 0x0800) {
					sx = sprites_offsx + sx - (xnum * 8);
					sy = sprites_offsy - sy - (ynum * 8) / 2;
				} else {
					sx = sprites_offsx + sx;
					sy = sprites_offsy - sy - (ynum * 8);
				}
				if (xnum == 2 && ynum == 4)
					code &= ~7u;
				// draw_sprites_tiles: the tiles column by column, a tile 8 lines
				const int xs = flipx ? xnum - 1 : 0, xe = flipx ? -1 : xnum, xi = flipx ? -1 : 1;
				const int ys = flipy ? ynum - 1 : 0, ye = flipy ? -1 : ynum, yi = flipy ? -1 : 1;
				for (int x = xs; x != xe; x += xi) {
					const int tx = sx + x * 16;
					if (tx > cl->max_x || tx + 15 < cl->min_x) {
						code += (uint32_t)ynum;
						continue;
					}
					for (int y = ys; y != ye; y += yi) {
						const int ty = sy + y * 8;
						if (ty <= cl->max_y && ty + 7 >= cl->min_y) {
							const cmd_t t = { code, (int16_t)tx, (int16_t)ty, (uint16_t)attr, CMD_TILE,
							                  (uint8_t)(gfx | shadow << 3 | flipx << 4 | flipy << 5), (uint8_t)s_emit_sb, 0 };
							emit(s_cmd, ty > cl->min_y ? ty : cl->min_y, ty + 7 < cl->max_y ? ty + 7 : cl->max_y, &t, sizeof(t));
						}
						code++;
					}
				}
			}
		}
	}
}

// screen_update for the lines of cl (MAME's cliprect), into cl->bm
// screen_update of a scroll band (MAME's cliprect: its lines): the context and clip it draws with, and its commands
static drawctx_t s_sbctx[MAX_BANDS];
static clip_t s_sbclip[MAX_BANDS];
static void sb_make(int sb, const drawctx_t *c0, const clip_t *cliprect, int enable)
{
	drawctx_t c = *c0;
	const uint16_t *sc = c.scroll;
	c.shadow_shift = (sc[0x76 / 2] & 0x0080) ? 15 - 4 : 15 - 2;
	c.shadow_mask = (1u << c.shadow_shift) - 1;
	clip_t clip = *cliprect;
	clip.min_x = (cliprect->max_x / 2 + sc[0x62 / 2]) * 2 - sc[0x64 / 2] * 2 + 2;
	clip.max_x = (cliprect->max_x / 2 + sc[0x62 / 2]) * 2 - sc[0x62 / 2] * 2 + 1;
	clip.min_y = (cliprect->max_y + sc[0x6a / 2]) - sc[0x6c / 2] + 1;
	clip.max_y = (cliprect->max_y + sc[0x6a / 2]) - sc[0x6a / 2];
	if (clip.min_x < cliprect->min_x) clip.min_x = cliprect->min_x;
	if (clip.max_x > cliprect->max_x) clip.max_x = cliprect->max_x;
	if (clip.min_y < cliprect->min_y) clip.min_y = cliprect->min_y;
	if (clip.max_y > cliprect->max_y) clip.max_y = cliprect->max_y;
	s_sbctx[sb] = c;
	s_sbclip[sb] = clip;
	if (!enable)
		return;
	if (clip.min_x > clip.max_x || clip.min_y > clip.max_y)
		return;
	s_emit_sb = sb;
	draw_layer(&c, &clip, 0);
	draw_sprites(&c, &clip);
}

// ---- gdfs: the ST-0020's zooming sprites and the tilemap
static void st0020_make(void)
{
	const uint16_t *spr = s_snap_st_spr;
	for (int pri = 0; pri <= 0xf; pri++) {
		const int priority = pri << 4;
		for (int i = 0; i < 0x2000 / 2; i += 4) {
			const uint16_t *s1 = spr + i;
			const int xoffs = s1[0], yoffs = s1[1], sprite = s1[2];
			int num = s1[3] % 0x101;
			if (sprite & 0x8000)
				break;
			const int spritebase = (sprite & 0x7fff) * 16 / 2;
			for (int s2 = 0; num > 0; num--, s2 += 16 / 2) {
				uint32_t code = spr[(spritebase + s2 + 0) & 0x3ffff];
				const uint16_t attr = spr[(spritebase + s2 + 1) & 0x3ffff];
				int sx = spr[(spritebase + s2 + 2) & 0x3ffff];
				int sy = spr[(spritebase + s2 + 3) & 0x3ffff];
				const uint16_t zoom = spr[(spritebase + s2 + 4) & 0x3ffff];
				const uint16_t size = spr[(spritebase + s2 + 5) & 0x3ffff];
				if (priority != (size & 0xf0))
					break;
				const int flipx = (attr >> 15) & 1, flipy = (attr >> 14) & 1;
				const uint32_t c11 = attr & 0x7ff;
				const uint32_t color = (c11 & 0x400) ? (c11 & 0x3ff) : (c11 & 0x3ff) * 4;
				int xnum = 1 << (size & 3);
				const int ynum = 1 << ((size >> 2) & 3);
				xnum = (xnum + 1) / 2;
				const int xs = flipx ? xnum - 1 : 0, xe = flipx ? -1 : xnum, xi = flipx ? -1 : 1;
				const int ys = flipy ? ynum - 1 : 0, ye = flipy ? -1 : ynum, yi = flipy ? -1 : 1;
				sx += xoffs;
				sy += yoffs;
				sx = (int)((uint32_t)sx << 22) >> 22;
				sy = (int)((uint32_t)sy << 22) >> 22;
				sy = -sy;
				sx <<= 16;
				sy <<= 16;
				const int xdim = ((((zoom >> 0) & 0xff) + 1) << 16) / xnum;
				const int ydim = ((((zoom >> 8) & 0xff) + 1) << 16) / ynum;
				int xscale = xdim / 16;
				int yscale = ydim / 8;
				if (xscale & 0xffff)
					xscale += (1 << 16) / 16;
				if (yscale & 0xffff)
					yscale += (1 << 16) / 8;
				// (zoom_transpen's color: color * 4, modulo its 2048 codes of 16 pens)
				const uint32_t base = ((color * 4) & 2047) * 16;
				// (zoomgfx_draw's lines: y .. y + dsth - 1, the band clips the rest)
				const int dsth = (int)(((uint32_t)yscale * 8u + 0x8000) >> 16);
				for (int x = xs; x != xe; x += xi)
					for (int y = ys; y != ye; y += yi) {
						const int dy = (sy + y * ydim) / 0x10000;
						const zcmd_t z = { code++, base, (uint32_t)xscale, (uint32_t)yscale, (int16_t)((sx + x * xdim) / 0x10000),
						                   (int16_t)dy, (uint8_t)flipx, (uint8_t)flipy, { 0, 0 } };
						if (dsth >= 1 && dy + dsth - 1 >= 0 && dy < s_m->vis_h)
							emit(s_zcmd, dy, dy + dsth - 1, &z, sizeof(z));
					}
			}
		}
	}
}

static void gdfs_tilemap_draw(const clip_t *cl)
{
	// 0x100 x 0x100 tiles of 16x16 (TILEMAP_SCAN_ROWS), pen 0 transparent, scroll from 0x44000c / 0x440010
	const int scrollx = s_snap_tmapscroll[0], scrolly = s_snap_tmapscroll[1];
	for (int y = cl->min_y; y <= cl->max_y; y++) {
		const int ty = (y + scrolly) & 0xfff;
		uint16_t *d = cl->bm + (size_t)y * cl->pitch;
		int x = cl->min_x;
		while (x <= cl->max_x) {
			const int tx = (x + scrollx) & 0xfff;
			const uint16_t tile = s_snap_tmap[(ty >> 4) * 0x100 + (tx >> 4)];
			const int fx = (tile >> 14) & 1, fy = (tile >> 15) & 1;
			const uint32_t code = rbx_wrap(tile, s_gd_tile_count);
			const int row = fy ? 15 - (ty & 15) : ty & 15;
			const uint8_t *src = s_gd_tiles + (size_t)code * 256 + row * 16;
			int col = tx & 15;
			int n = 16 - col;
			if (x + n - 1 > cl->max_x)
				n = cl->max_x - x + 1;
			if (n == 16 && !fx) {   // (a whole tile row: 4 pixels at a time skipped when all transparent)
				for (int q = 0; q < 16; q += 4) {
					uint32_t w4;
					memcpy(&w4, src + q, 4);
					if (!w4)
						continue;
					for (int k = q; k < q + 4; k++)
						if (src[k])
							d[x + k] = src[k];
				}
			} else if (!fx) {
				for (int k = 0; k < n; k++, col++)
					if (src[col])
						d[x + k] = src[col];
			} else {
				for (int k = 0; k < n; k++, col++) {
					const uint8_t pen = src[15 - col];
					if (pen)
						d[x + k] = pen;
				}
			}
			x += n;
		}
	}
}

// ---------------------------------------------------------------------------------------------------- the frame
// the frame in BANDS bands of lines, drawn apart by the render thread and its helpers (decoband): each band draws
// MAME's whole screen_update clipped to its lines, so the pixels are the same
// the frame's commands (render, before the bands)
static void frame_make(void)
{
	const int h = s_m->vis_h, w = s_m->vis_w;
	for (int j = 0; j <= BANDS; j++)
		s_job_lo[j] = h * j / BANDS;
	for (int j = 0; j < BANDS; j++)
		s_cmd[j].n = s_zcmd[j].n = 0;
	drawctx_t c;
	c.spr = s_snap_spr;
	c.gfx = s_kind == K_EAGLSHOT ? s_snap_ea_gfx : s_gfx;
	c.tiles = s_kind == K_EAGLSHOT ? 0x40000 * 16 / 128 : s_gfx_tiles;
	c.vis_max_y = h - 1;
	c.shadow_mask = c.shadow_shift = 0;
	int y0 = 0;
	for (int i = 0; i < s_snap_nbands; i++) {
		clip_t cl = { rbx_video.bitmap, rbx_video.pitch, 0, w - 1, y0, s_snap_bands[i].y1 };
		c.scroll = s_snap_bands[i].scroll;
		if (cl.max_y >= cl.min_y)
			sb_make(i, &c, &cl, s_snap_enable);
		y0 = s_snap_bands[i].y1 + 1;
	}
	if (s_kind == K_GDFS)
		st0020_make();
}

static void band_draw(int job)
{
	const int w = s_m->vis_w;
	const int ylo = s_job_lo[job], yhi = s_job_lo[job + 1] - 1;
	if (yhi < ylo)
		return;
	const unsigned prof = rbx_prof_enter(SV_PA_DRAW);
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	for (int y = ylo; y <= yhi; y++)   // (each scroll band's bitmap.fill(0, cliprect): every line)
		memset(bm + (size_t)y * pitch, 0, (size_t)w * 2);
	const cmd_t *cmd = (const cmd_t *)s_cmd[job].p;
	const int n = s_cmd[job].n;
	int sb = -1;
	clip_t cl = { bm, pitch, 0, -1, 0, -1 };
	const drawctx_t *c = NULL;
	for (int i = 0; i < n; i++) {
		const cmd_t *k = &cmd[i];
		if (k->sb != sb) {
			sb = k->sb;
			c = &s_sbctx[sb];
			cl = s_sbclip[sb];
			if (cl.min_y < ylo) cl.min_y = ylo;
			if (cl.max_y > yhi) cl.max_y = yhi;
		}
		if (cl.min_y > cl.max_y)
			continue;
		if (k->kind == CMD_TILE)
			drawgfx_tile(c, &cl, k->flags & 7, k->code, k->color, (k->flags >> 4) & 1, (k->flags >> 5) & 1, k->x,
			             (k->flags >> 3) & 1, k->y, 0, 7);
		else
			draw_row_64pixhigh(c, &cl, k->y, k->x);
	}
	if (s_kind == K_GDFS) {
		zoomgfx_target t = { bm, NULL, pitch, 0, ylo, w - 1, yhi, 0, 0 };
		const zcmd_t *z = (const zcmd_t *)s_zcmd[job].p;
		for (int i = 0; i < s_zcmd[job].n; i++)
			zoomgfx_draw(&s_st_zg, &t, z[i].code, z[i].base, z[i].flipx, z[i].flipy, z[i].x, z[i].y, z[i].scalex, z[i].scaley, 0, 0);
		clip_t tl = { bm, pitch, 0, w - 1, ylo, yhi };
		gdfs_tilemap_draw(&tl);
	}
	rbx_prof_leave(prof);
}

static decoband_t s_dband = { 0, 0, 0, 0, BANDS, band_draw };
static int ssv_help(int helper) { return decoband_help(&s_dband, helper); }

// the RAM the drawing reads that is too big to copy each frame (gdfs, eaglshot): written through the handlers, which
// mark its 4 KB pages; the marked pages are copied at the vblank
static inline void mark_dirty(uint32_t *bits, uint32_t off, uint32_t len)
{
	if (!len)
		return;
	for (uint32_t pg = off >> 12; pg <= (off + len - 1) >> 12; pg++)
		bits[pg >> 5] |= 1u << (pg & 31);
}

static void copy_dirty(uint8_t *dst, const uint8_t *src, uint32_t *bits, uint32_t size)
{
	for (uint32_t w = 0; w < (size >> 12) / 32; w++) {
		uint32_t m = bits[w];
		bits[w] = 0;
		while (m) {
			const int b = __builtin_ctz(m);
			m &= m - 1;
			const uint32_t pg = w * 32 + (uint32_t)b;
			memcpy(dst + (pg << 12), src + (pg << 12), 4096);
		}
	}
}

static void dirty_all(void)
{
	memset(s_dirty_stgfx, 0xff, sizeof(s_dirty_stgfx));
	memset(s_dirty_stspr, 0xff, sizeof(s_dirty_stspr));
	memset(s_dirty_tmap, 0xff, sizeof(s_dirty_tmap));
	memset(s_dirty_eagfx, 0xff, sizeof(s_dirty_eagfx));
}

// a scroll register write at the current line (scroll_w: m_screen->update_partial(m_screen->vpos() - 1))
static void partial_before_write(void)
{
	const int last = s_st.line - 1;
	if (s_st.line >= s_m->vis_h + 1 || last < s_band_y)
		return;
	const int y1 = last < s_m->vis_h - 1 ? last : s_m->vis_h - 1;
	if (s_nbands < MAX_BANDS - 1) {
		s_bands[s_nbands].y1 = y1;
		memcpy(s_bands[s_nbands].scroll, s_st.scroll, sizeof(s_st.scroll));
		s_nbands++;
	}
	s_band_y = y1 + 1;
}

static void capture(void)
{
	// the last band: the lines left, with the registers as they are now
	if (s_band_y <= s_m->vis_h - 1) {
		s_bands[s_nbands].y1 = s_m->vis_h - 1;
		memcpy(s_bands[s_nbands].scroll, s_st.scroll, sizeof(s_st.scroll));
		s_nbands++;
	}
	const unsigned prof = rbx_prof_enter(SV_PA_DRAW);
	rbx_video_wait_idle();
	if (memcmp(s_snap_pal, s_pal, PENS * 4)) {
		memcpy(s_snap_pal, s_pal, PENS * 4);
		s_pal_dirty = 1;
	}
	memcpy(s_snap_spr, s_spr, SPR_WORDS * 2);
	memcpy(s_snap_bands, s_bands, sizeof(band_t) * (size_t)s_nbands);
	s_snap_nbands = s_nbands;
	s_snap_enable = s_st.enable_video;
	if (s_kind == K_GDFS) {
		copy_dirty(s_snap_st_gfx, s_st_gfx, s_dirty_stgfx, 4 * 0x100000);
		copy_dirty((uint8_t *)s_snap_st_spr, (const uint8_t *)s_st_spr, s_dirty_stspr, 0x80000);
		copy_dirty((uint8_t *)s_snap_tmap, (const uint8_t *)s_tmapram, s_dirty_tmap, 0x20000);
		s_snap_tmapscroll[0] = s_st.tmapscroll[0x0c / 2];
		s_snap_tmapscroll[1] = s_st.tmapscroll[0x10 / 2];
	} else if (s_kind == K_EAGLSHOT)
		copy_dirty(s_snap_ea_gfx, s_ea_gfx, s_dirty_eagfx, 16 * 0x40000);
	rbx_video_frame_done();
	rbx_prof_leave(prof);
	s_nbands = 0;
	s_band_y = s_m->vis_h;   // (the rest of the frame: nothing more to draw)
}

static void ssv_render(void)
{
	if (s_pal_dirty) {
		for (int i = 0; i < PENS; i++)
			rbx_video.palette[i] = s_snap_pal[i] & 0xffffff;
		s_pal_dirty = 0;
	}
	frame_make();
	decoband_run(&s_dband);
}

// ---------------------------------------------------------------------------------------------------- sound
static void sound_bank_update(void)
{
	for (int b = 0; b < 4; b++) {
		const uint8_t *base = s_es_rom[b];
		uint32_t size = s_es_size[b];
		if (s_kind == K_SRMP7 && b >= 2 && base) {   // (srmp7_es5506_bank2 / 3_map: 0x400000 bytes of 2)
			const uint32_t off = (s_st.srmp7_bank & 1) * 0x400000u;
			size = size > off ? size - off : 0;
			if (size > 0x400000)
				size = 0x400000;
			base = size ? base + off : NULL;
		}
		es5506_set_bank(&s_es, b, base, size);
	}
}

static void sound_update(uint64_t now)
{
	if (s_st.snd_next > now)
		return;
	const unsigned prof = rbx_prof_enter(SV_PA_SND);
	while (s_st.snd_next <= now) {
		int32_t out[12];
		es5506_sample(&s_es, out);
		s_st.snd_l = out[0];
		s_st.snd_r = out[1];
		s_st.snd_next += (uint64_t)es5506_sample_clocks(&s_es);
		while (s_st.out_next < s_st.snd_next) {   // (the output at 31250 Hz: each takes the latest chip sample)
			// 20-bit samples (1 << 19 = 1.0) at 1.2 to each speaker
			rbx_audio_put_stereo((s_st.snd_l * 1229) >> 14, (s_st.snd_r * 1229) >> 14);
			s_st.out_next += OUT_PERIOD;
		}
	}
	rbx_prof_leave(prof);
}

// ---------------------------------------------------------------------------------------------------- the DSP
static void dsp_sync(void)
{
	const int64_t t = (int64_t)((cpu_now() * 5) >> 3);   // (10 MHz against 16)
	if (t > s_st.dsp_done) {
		const unsigned prof = rbx_prof_enter(SV_PA_DSP);
		upd96050_run(&s_dsp, t - s_st.dsp_done);
		s_st.dsp_done = t;
		rbx_prof_leave(prof);
	}
}

// ---------------------------------------------------------------------------------------------------- interrupts
// (set_input_line is synchronized in MAME: from a handler it takes effect after the current instruction, the slice
// cut there)
static void apply_irq(void)
{
	const int state = (s_st.requested_int & s_st.irq_enable) ? 1 : 0;
	if (state != s_st.irq_line) {
		s_st.irq_line = (uint8_t)state;
		v60_set_irq_line(state);
	}
}

static void update_irq_state(void)
{
	if (s_in_exec) {
		const int state = (s_st.requested_int & s_st.irq_enable) ? 1 : 0;
		s_irq_dirty = 1;
		if (state != s_st.irq_line)
			v60_abort_timeslice();
	} else
		apply_irq();
}

static uint8_t irq_callback(void)
{
	for (int i = 0; i <= 7; i++)
		if (s_st.requested_int & (1 << i))
			return (uint8_t)(s_st.irqv[i * (16 / 2)] & 7);
	return 0;
}

// ---------------------------------------------------------------------------------------------------- devices
// gdfs: the ADC0809 (1 MHz: 16 V60 cycles a clock)
enum { ADC_IDLE, ADC_START, ADC_READY, ADC_RUNNING };
static uint8_t adc_input(void)
{
	static const int ports[4] = { P_GUNX1, P_GUNY1, P_GUNX2, P_GUNY2 };
	const int a = s_st.adc_addr & 7;
	if (a > 3)
		return 0;
	const int p = a >> 1, axis = a & 1;
	(void)ports;
	const int v = s_st.gun[p][axis];
	return (uint8_t)(axis ? v : 0xff - v);   // (GUNX: PORT_INVERT)
}

static void adc_advance(uint64_t now)
{
	while (s_st.adc_at && s_st.adc_at <= now) {
		switch (s_st.adc_state) {
		case ADC_START:
			s_st.adc_state = ADC_READY;
			s_st.adc_at += 16;
			break;
		case ADC_READY:
			s_st.adc_state = ADC_RUNNING;
			s_st.adc_sar = adc_input();
			s_st.adc_eoc = 0;
			s_st.adc_at += 64 * 16;
			break;
		case ADC_RUNNING:
			s_st.adc_state = ADC_IDLE;
			s_st.adc_sar = adc_input();
			s_st.adc_eoc = 1;
			s_st.adc_at = 0;
			s_st.requested_int |= 1 << 6;   // (adc_int_w: reads the light gun)
			update_irq_state();
			break;
		default:
			s_st.adc_at = 0;
			break;
		}
	}
}

static void adc_start_w(int state, uint64_t now)
{
	if (s_st.adc_start == state)
		return;
	if (state) {
		s_st.adc_state = ADC_START;
		s_st.adc_at = now + 16;
	} else if (s_st.adc_state != ADC_IDLE)
		s_st.adc_at = now + 16;
	s_st.adc_start = (uint8_t)state;
}

// sxyreact: the uPD7001 (1.5 MHz)
#define AD_CLK(n) ((uint64_t)(n) * CLOCK / 1500000u)
static void ad_update(uint64_t now)
{
	if (s_st.ad_conv_at && now >= s_st.ad_conv_at) {
		s_st.ad_conv_at = 0;
		s_st.ad_sr = s_st.ad_mpx == 0 ? (uint8_t)s_st.paddle : 0;   // (a0_callback: PADDLE)
		s_st.ad_eoc_active = 1;
	}
	if (s_st.ad_oe_at && now >= s_st.ad_oe_at) {
		s_st.ad_oe_at = 0;
		s_st.ad_oe = 1;
	}
}

static void ad_cs_w(int state, uint64_t now)
{
	ad_update(now);
	if (!state && !s_st.ad_cs_active) {
		s_st.ad_conv_at = 0;
		s_st.ad_eoc_active = 0;
		s_st.ad_oe_at = now + AD_CLK(5);
	} else if (state && s_st.ad_cs_active) {
		s_st.ad_oe = 0;
		s_st.ad_oe_at = 0;
		s_st.ad_conv_at = now + AD_CLK(56);
	}
	s_st.ad_cs_active = (uint8_t)!state;
}

static void ad_sck_w(int state)
{
	if (s_st.ad_cs_active) {
		if (!state && s_st.ad_sck)
			s_st.ad_so = (s_st.ad_sr >> 7) & 1;
		else if (state && !s_st.ad_sck) {
			s_st.ad_sr = (uint8_t)((s_st.ad_sr << 1) | s_st.ad_si);
			if (s_st.ad_dl)
				s_st.ad_mpx = s_st.ad_sr & 3;
		}
	}
	s_st.ad_sck = (uint8_t)state;
}

// eaglshot: the uPD4701
static void ud_cs_w(int state)
{
	if (s_st.ud_cs != state) {
		s_st.ud_cs = (uint8_t)state;
		if (!state) {
			s_st.ud_latchx = (s_st.ud_x - s_st.ud_startx) & 0xfff;
			s_st.ud_latchy = (s_st.ud_y - s_st.ud_starty) & 0xfff;
		}
	}
}

// ---------------------------------------------------------------------------------------------------- the map
static inline uint16_t port(int p) { return s_port[p] >= 0 ? (uint16_t)rbx_port_read(s_port[p]) : 0; }
static inline uint16_t key(int k) { return s_port[P_KEY0 + k] >= 0 ? (uint16_t)rbx_port_read(s_port[P_KEY0 + k]) : 0xff; }

static uint16_t vblank_r(void)
{
	uint16_t ret = 0;
	const uint64_t now = cpu_now();
	if (s_st.line >= s_m->vis_h)
		ret |= 0x3000;
	const uint32_t hpos = (uint32_t)(((now - s_st.line_start) * 29324u) >> 16);   // (* 0x1c6 / a line's cycles)
	if ((int)hpos > s_m->vis_w - 1)
		ret |= 0x0800;
	return ret;
}

static uint16_t hypreact_input_r(void)
{
	const uint16_t sel = s_st.input_sel;
	uint16_t r = 0x3f;
	if (sel & 1) r &= key(0);
	if (sel & 2) r &= key(1);
	if (sel & 4) r &= key(2);
	if (sel & 8) r &= key(3);
	return r | 0x00c0;
}

static inline uint16_t bitrev6(uint16_t r)   // (bitswap<6>(r, 0, 1, 2, 3, 4, 5))
{
	uint16_t o = 0;
	for (int i = 0; i < 6; i++)
		o |= (uint16_t)(((r >> i) & 1) << (5 - i));
	return o;
}

static uint16_t srmp4_input_r(void)
{
	const uint16_t sel = s_st.input_sel;
	uint16_t r = 0x3f;
	if (sel & 2) r &= key(3);
	if (sel & 4) r &= key(2);
	if (sel & 8) r &= key(1);
	if (sel & 16) r &= key(0);
	return bitrev6(r & 0x3f) | 0xffc0;
}

static uint16_t srmp7_input_r(void)
{
	const uint16_t sel = s_st.input_sel;
	uint16_t r = 0x3f;
	if (sel & 2) r &= key(2);
	if (sel & 4) r &= key(1);
	if (sel & 8) r &= key(0);
	if (sel & 16) r &= key(3);
	return bitrev6(r & 0x3f) | 0x00c0;
}

static uint16_t base_read(uint32_t a, uint16_t mask)
{
	(void)mask;
	if (a >= 0x1c0000 && a < 0x1c0080)
		return a == 0x1c0000 ? vblank_r() : s_st.scroll[(a - 0x1c0000) >> 1];
	switch (a) {
	case 0x210002: return port(P_DSW1);
	case 0x210004: return port(P_DSW2);
	case 0x210008: return port(P_P1);
	case 0x21000a: return port(P_P2);
	case 0x21000c: return port(P_SYSTEM);
	}
	if (a >= 0x300000 && a < 0x300080) {
		if (!(mask & 0x00ff))
			return 0;
		sound_update(cpu_now());
		return es5506_read(&s_es, (int)((a - 0x300000) >> 1));
	}
	return 0;
}

static void lockout_w(uint16_t data, uint16_t mask)
{
	if (mask & 0x00ff)
		s_st.enable_video = (data >> 7) & 1;   // (the coin lockouts / counters: nothing to do here)
}

static void scroll_w(uint32_t i, uint16_t data, uint16_t mask)
{
	const uint16_t v = combine(s_st.scroll[i], data, mask);
	if (v == s_st.scroll[i])
		return;
	partial_before_write();
	s_st.scroll[i] = v;
}

static int base_write(uint32_t a, uint16_t data, uint16_t mask)
{
	if (a >= 0x1c0000 && a < 0x1c0080) {
		scroll_w((a - 0x1c0000) >> 1, data, mask);
		return 1;
	}
	if (a == 0x21000e) {
		lockout_w(data, mask);
		return 1;
	}
	if (a >= 0x230000 && a < 0x230072) {
		uint16_t *p = &s_st.irqv[(a - 0x230000) >> 1];
		*p = combine(*p, data, mask);
		return 1;
	}
	if (a >= 0x240000 && a < 0x240072) {   // (irq_ack_w)
		const int level = ((a - 0x240000) & 0x70) >> 4;
		s_st.requested_int &= (uint8_t)~(1 << level);
		update_irq_state();
		return 1;
	}
	if (a == 0x260000) {
		s_st.irq_enable = combine(s_st.irq_enable, data, mask);
		return 1;
	}
	if (a >= 0x300000 && a < 0x300080) {
		if (mask & 0x00ff) {
			sound_update(cpu_now());
			es5506_write(&s_es, (int)((a - 0x300000) >> 1), (uint8_t)data);
		}
		return 1;
	}
	return 0;
}

static uint16_t dsp_read(uint32_t a, uint16_t mask)
{
	if (a == 0x480000)
		return (mask & 0x00ff) ? (dsp_sync(), upd96050_data_r(&s_dsp)) : 0;
	if (a >= 0x482000 && a < 0x483000) {   // (dsp_r: umask16(0x00ff))
		if (!(mask & 0x00ff))
			return 0;
		dsp_sync();
		const uint32_t off = (a - 0x482000) >> 1;
		const uint16_t t = upd96050_dataram_r(&s_dsp, (uint16_t)(off / 2));
		return (uint16_t)((t >> ((off & 1) << 3)) & 0xff);
	}
	return 0;
}

static void dsp_write(uint32_t a, uint16_t data, uint16_t mask)
{
	if (a == 0x480000) {
		if (mask & 0x00ff) {
			dsp_sync();
			upd96050_data_w(&s_dsp, (uint8_t)data);
		}
	} else if (a >= 0x482000 && a < 0x483000 && (mask & 0x00ff)) {
		dsp_sync();
		const uint32_t off = (a - 0x482000) >> 1;
		const uint8_t d = (uint8_t)data;
		const int shift = (off & 1) << 3;
		upd96050_dataram_w(&s_dsp, (uint16_t)(off / 2), (uint16_t)(d << 8 | d), (uint16_t)(0xff << shift));
	}
}

static uint16_t st_gfx_r(uint32_t off)   // (byte offset in the 1 MB window)
{
	return rd16(s_st_gfx, s_st.st_bank * 0x100000u + off);
}

static void st0020_blit(void)
{
	const uint16_t *r = s_st.st_regs;
	const uint32_t src = (uint32_t)(r[0xc0 / 2] + (r[0xc2 / 2] << 16)) << 1;
	const uint32_t dst = (uint32_t)(r[0xc4 / 2] + (r[0xc6 / 2] << 16)) << 4;
	const uint32_t len = (uint32_t)r[0xc8 / 2] << 4;
	if (s_st_rom && (uint64_t)src + len <= s_st_rom_size && (uint64_t)dst + len <= 4 * 0x100000u)
	{
		memcpy(s_st_gfx + dst, s_st_rom + src, len);
		mark_dirty(s_dirty_stgfx, dst, len);
	}
}

static uint16_t main_read16(uint32_t a, uint16_t mask)
{
	switch (s_kind) {
	case K_DRIFTO94:
		if (a >= 0x480000 && a < 0x483000)
			return dsp_read(a, mask);
		if (a == 0x510000 || a == 0x520000) {   // (drifto94_unknown_r: machine().rand())
			s_st.rand = s_st.rand * 1103515245u + 12345u;
			return (uint16_t)(s_st.rand >> 8);
		}
		break;
	case K_TWINEAG2:
		if (a >= 0x480000 && a < 0x483000)
			return dsp_read(a, mask);
		break;
	case K_GDFS:
		if (a >= 0x440000 && a < 0x440040)
			return s_st.tmapscroll[(a - 0x440000) >> 1];
		if (a == 0x540000) {   // (eeprom_r)
			adc_advance(cpu_now());
			return (uint16_t)(s_st.adc_sar | (eeprom_do(&s_eeprom) << 8));
		}
		if (a >= 0x8c0000 && a < 0x8c0100)
			return 0;   // (regs_r)
		if (a >= 0x900000 && a < 0xa00000)
			return st_gfx_r(a - 0x900000);
		break;
	case K_HYPREACT:
		if (a == 0xc00000) return hypreact_input_r();
		if (a == 0xc00006) return s_st.input_sel;
		break;
	case K_HYPREAC2:
		if (a == 0x500000 || a == 0x500002) return hypreact_input_r();
		break;
	case K_JANJANS1:
		if (a == 0x800002) return srmp4_input_r();
		break;
	case K_SRMP4:
		if (a == 0xc0000a) return srmp4_input_r();
		break;
	case K_SRMP7:
		if (a == 0x300076) return 0x0080;   // (srmp7_irqv_r)
		if (a == 0x600000) return srmp7_input_r();
		break;
	case K_SURVARTS:
		if (a == 0x500008) return port(P_ADD);
		break;
	case K_SXYREACT:
		if (a == 0x500002) return port(P_SERVICE);
		if (a == 0x500004) {   // (dial_r on the low lane)
			ad_update(cpu_now());
			const int so = s_st.ad_oe ? s_st.ad_so : 1;
			const int eoc = s_st.ad_eoc_active ? 0 : 1;
			return (eoc && so) ? 0x80 : 0;
		}
		break;
	case K_EAGLSHOT:
		if (a >= 0x180000 && a < 0x1c0000)
			return rd16(s_ea_gfx, (s_st.scroll[0x76 / 2] & 0xf) * 0x40000u + (a - 0x180000));
		if (a >= 0xa00000 && a < 0xc00000) {
			const uint32_t off = s_st.gfxrom_bank * 0x200000u + (a - 0xa00000);
			return off + 1 < s_ea_rom_size ? rd16(s_ea_rom, off) : 0;
		}
		if (a == 0xd00000) {   // (upd4701 d_r on the low lane)
			if (s_st.ud_cs)
				return 0;
			uint16_t d = s_st.ud_xy ? s_st.ud_latchy : s_st.ud_latchx;
			return s_st.ud_ul ? (uint16_t)(d >> 8) : (uint16_t)(d & 0xff);
		}
		break;
	}
	return base_read(a, mask);
}

static void main_write16(uint32_t a, uint16_t data, uint16_t mask)
{
	switch (s_kind) {
	case K_DRIFTO94:
	case K_TWINEAG2:
		if (a >= 0x480000 && a < 0x483000) {
			dsp_write(a, data, mask);
			return;
		}
		break;
	case K_GDFS:
		if (a >= 0x440000 && a < 0x440040) {
			uint16_t *p = &s_st.tmapscroll[(a - 0x440000) >> 1];
			*p = combine(*p, data, mask);
			return;
		}
		if (a == 0x500000) {   // (eeprom_w)
			if (mask & 0xff00) {
				const uint64_t now = cpu_now();
				adc_advance(now);
				eeprom_write_lines(&s_eeprom, (data >> 12) & 1, (data >> 13) & 1, (data >> 14) & 1);
				s_st.adc_addr = (uint8_t)((data & 0x0700) >> 8);
				adc_start_w((data >> 11) & 1, now);
				if (s_st.adc_at && s_in_exec)
					v60_abort_timeslice();   // (the conversion's steps: run_until stops at them)
			}
			return;
		}
		if (a >= 0x400000 && a < 0x420000) {   // (tmapram: read straight, written here to mark it for the drawing)
			wr16((uint8_t *)s_tmapram, a - 0x400000, data, mask);
			mark_dirty(s_dirty_tmap, a - 0x400000, 2);
			return;
		}
		if (a >= 0x800000 && a < 0x880000) {   // (the ST-0020's sprite RAM, the same)
			wr16((uint8_t *)s_st_spr, a - 0x800000, data, mask);
			mark_dirty(s_dirty_stspr, a - 0x800000, 2);
			return;
		}
		if (a >= 0x8c0000 && a < 0x8c0100) {   // (regs_w)
			const uint32_t off = (a - 0x8c0000) >> 1;
			s_st.st_regs[off] = combine(s_st.st_regs[off], data, mask);
			if (off == 0x8a / 2 && (mask & 0x00ff))
				s_st.st_bank = s_st.st_regs[off] & 3;
			else if (off == 0xca / 2)
				st0020_blit();
			return;
		}
		if (a >= 0x900000 && a < 0xa00000) {
			const uint32_t off = s_st.st_bank * 0x100000u + (a - 0x900000);
			wr16(s_st_gfx, off, data, mask);
			mark_dirty(s_dirty_stgfx, off, 2);
			return;
		}
		break;
	case K_HYPREACT:
		if (a == 0x21000e) { lockout_w(data, mask); return; }
		if (a == 0xc00006) { s_st.input_sel = combine(s_st.input_sel, data, mask); return; }
		break;
	case K_HYPREAC2:
		if (a == 0x21000e) { lockout_w(data, mask); return; }
		if (a == 0x520000) { s_st.input_sel = combine(s_st.input_sel, data, mask); return; }
		break;
	case K_JANJANS1:
		if (a == 0x800000) { s_st.input_sel = combine(s_st.input_sel, data, mask); return; }
		break;
	case K_SRMP4:
		if (a == 0xc0000e) { s_st.input_sel = combine(s_st.input_sel, data, mask); return; }
		break;
	case K_SRMP7:
		if (a == 0x21000e) { lockout_w(data, mask); return; }
		if (a == 0x580000) {
			if (mask & 0x00ff) {
				sound_update(cpu_now());
				s_st.srmp7_bank = data & 1;
				sound_bank_update();
			}
			return;
		}
		if (a == 0x680000) { s_st.input_sel = combine(s_st.input_sel, data, mask); return; }
		break;
	case K_CAIRBLAD:
		if (a == 0x21000e) { lockout_w(data, mask); return; }
		break;
	case K_SXYREACT:
		if (a == 0x21000e) { lockout_w(data, mask); return; }
		if (a == 0x520000 && (mask & 0x00ff)) {   // (dial_w)
			const uint64_t now = cpu_now();
			ad_cs_w((data >> 5) & 1, now);
			ad_sck_w((data >> 6) & 1);
			return;
		}
		break;
	case K_EAGLSHOT:
		if (a == 0x21000e) { lockout_w(data, mask); return; }
		if (a >= 0x180000 && a < 0x1c0000) {
			const uint32_t off = (s_st.scroll[0x76 / 2] & 0xf) * 0x40000u + (a - 0x180000);
			wr16(s_ea_gfx, off, data, mask);
			mark_dirty(s_dirty_eagfx, off, 2);
			return;
		}
		if (a == 0x800000 && (mask & 0x00ff)) {   // (gfxrom_bank_w)
			const uint8_t d = (uint8_t)data;
			s_st.gfxrom_bank = d < 6 ? d : 6;
			return;
		}
		if (a == 0x900000 && (mask & 0x00ff)) {   // (trackball_w)
			const uint8_t d = (uint8_t)data;
			ud_cs_w(!((d >> 6) & 1));
			s_st.ud_ul = (d >> 5) & 1;
			s_st.ud_xy = (d >> 4) & 1;
			const int rx = (d >> 3) & 1, ry = (d >> 2) & 1;
			if (rx != s_st.ud_resetx) { s_st.ud_resetx = (uint8_t)rx; if (rx) s_st.ud_startx = s_st.ud_x; }
			if (ry != s_st.ud_resety) { s_st.ud_resety = (uint8_t)ry; if (ry) s_st.ud_starty = s_st.ud_y; }
			return;
		}
		break;
	}
	base_write(a, data, mask);
}

// ---------------------------------------------------------------------------------------------------- run
static void run_until(uint64_t target)
{
	rbx_prof_set(SV_PA_V60);
	while (s_st.cyc < target) {
		uint64_t next = target;
		if (s_st.adc_at && s_st.adc_at > s_st.cyc && s_st.adc_at < next)
			next = s_st.adc_at;
		uint64_t n = next - s_st.cyc;
		if (n > 0x10000000u)
			n = 0x10000000u;
		s_in_exec = 1;
		const int ran = v60_execute((int)n);
		s_in_exec = 0;
		s_st.cyc += (uint64_t)(ran > 0 ? ran : 0);
		if (ran <= 0 && n > 0)
			s_st.cyc++;   // (never stuck)
		if (s_irq_dirty) {
			s_irq_dirty = 0;
			apply_irq();
		}
		if (s_st.adc_at && s_st.cyc >= s_st.adc_at)
			adc_advance(s_st.cyc);
	}
	rbx_prof_set(RBX_PA_DRIVER);
}

static void read_controls(void)
{
	if (s_kind == K_SXYREACT) {   // (PORT_MINMAX(0, 0xcf) PORT_KEYDELTA(15), no centering)
		const int k = s_st.paddle + rbx_analog_dpad(1, RBX_IPT_PADDLE, 15);
		s_st.paddle = k < 0 ? 0 : k > 0xcf ? 0xcf : k;
	} else if (s_kind == K_EAGLSHOT) {   // (upd4701 update: the trackball's movement this frame)
		const int dx = rbx_analog_dpad(1, RBX_IPT_TRACKBALL_X, 30);
		const int dy = rbx_analog_dpad(1, RBX_IPT_TRACKBALL_Y, 30);
		if (!s_st.ud_resetx && dx) s_st.ud_x = (uint16_t)(s_st.ud_x + dx);
		if (!s_st.ud_resety && dy) s_st.ud_y = (uint16_t)(s_st.ud_y + dy);
	} else if (s_kind == K_GDFS) {
		for (int p = 0; p < 2; p++) {
			int x = s_st.gun[p][0] + rbx_analog(p + 1, RBX_IPT_LIGHTGUN_X);
			int y = s_st.gun[p][1] + rbx_analog(p + 1, RBX_IPT_LIGHTGUN_Y);
			s_st.gun[p][0] = x < 0 ? 0 : x > 0xff ? 0xff : x;
			s_st.gun[p][1] = y < 0 ? 0 : y > 0xff ? 0xff : y;
		}
	}
}

static void ssv_frame(void)
{
	read_controls();
	for (int line = 0; line < LINES; line++) {
		s_st.line = line;
		s_st.line_start = s_st.cyc;
		if (line == 0) {
			s_nbands = 0;
			s_band_y = 0;
			if (s_irq_ultrax) {
				s_st.requested_int |= 1 << 1;
				update_irq_state();
			}
		} else if (line == 120 && s_raster_irq) {
			s_st.requested_int |= 1 << 2;
			update_irq_state();
		} else if (line == 240) {
			s_st.requested_int |= 1 << 3;
			update_irq_state();
		}
		if (line == s_m->vis_h)
			capture();
		s_st.lines++;
		run_until(rbx_div64_small(s_st.lines * LINE_NUM, LINE_DEN));
	}
	sound_update(s_st.cyc);
	if (s_m->dsp)
		dsp_sync();
}

// ---------------------------------------------------------------------------------------------------- init / reset
static void ssv_exit(void);

static void nv_save(void)
{
	if (s_nv_path[0] && s_nv)
		rbx_file_put_now(s_nv_path, s_nv, s_m->nv_size);
	if (s_kind == K_GDFS && s_ee_path[0])
		eeprom_save(&s_eeprom, s_ee_path);
}

static void ssv_reset(void)
{
	rbx_video_wait_idle();
	memset(s_ram, 0, 0x10000);
	if (s_ram2)
		memset(s_ram2, 0, s_ram2_size);
	memset(s_ram16, 0, 0x20000);
	memset(s_spr, 0, SPR_WORDS * 2);
	memset(s_pal, 0, PENS * 4);
	const int enable = 1;   // (init_ssv: enable_video(1))
	memset(&s_st, 0, sizeof(s_st));
	s_st.enable_video = enable;
	s_st.ud_cs = 1;
	s_st.ad_sck = 1;
	s_st.ad_si = s_kind == K_SXYREACT && !s_sxyreac2 ? 0 : 1;   // (sxyreact: si_w(0), dl_w(1))
	s_st.ad_dl = s_kind == K_SXYREACT && !s_sxyreac2;
	s_st.adc_sar = 0xff;
	s_st.adc_eoc = 1;
	s_st.snd_next = 0;
	s_st.out_next = OUT_PERIOD;
	for (int p = 0; p < 2; p++)
		s_st.gun[p][0] = s_st.gun[p][1] = 0x80;
	s_nbands = 0;
	s_band_y = 0;
	if (s_kind == K_GDFS) {
		memset(s_st_gfx, 0, 4 * 0x100000);
		memset(s_st_spr, 0, 0x80000);
		memset(s_tmapram, 0, 0x20000);
		memset(s_gd_ram, 0, 0x21000);
	}
	if (s_kind == K_EAGLSHOT)
		memset(s_ea_gfx, 0, 16 * 0x40000);
	dirty_all();
	es5506_init(&s_es, 1);
	sound_bank_update();
	if (s_m->dsp)
		upd96050_reset(&s_dsp);
	v60_init();
	v60_reset();
}

// the sprite ROM (layout_16x8x8: 4 quarters of 2 planes each, 16x8 tiles) to a byte a pixel, in place: the quarters'
// 32-byte pieces of a tile brought together (a 4 x N transposition by its cycles), then each tile decoded
static int decode_gfx(uint8_t *g, uint32_t size)
{
	const uint32_t n = size / 128, blocks = n * 4;
	uint8_t *seen = (uint8_t *)calloc((blocks + 7) / 8, 1);
	if (!seen)
		return -1;
	uint8_t tmp[32];
	for (uint32_t start = 0; start < blocks; start++) {
		if (seen[start >> 3] & (1 << (start & 7)))
			continue;
		// position d gets the source block (d & 3) * n + (d >> 2)
		memcpy(tmp, g + (size_t)start * 32, 32);
		uint32_t cur = start;
		for (;;) {
			seen[cur >> 3] |= (uint8_t)(1 << (cur & 7));
			const uint32_t src = (cur & 3) * n + (cur >> 2);
			if (src == start) {
				memcpy(g + (size_t)cur * 32, tmp, 32);
				break;
			}
			memcpy(g + (size_t)cur * 32, g + (size_t)src * 32, 32);
			cur = src;
		}
	}
	free(seen);
	for (uint32_t t = 0; t < n; t++) {
		uint8_t *p = g + (size_t)t * 128, out[128];
		for (int y = 0; y < 8; y++)
			for (int x = 0; x < 16; x++) {
				uint8_t pen = 0;
				const int byte = x < 8 ? 0 : 2, bit = 7 - (x & 7);
				for (int q = 0; q < 4; q++) {
					const uint8_t *r = p + q * 32 + y * 4 + byte;
					pen |= (uint8_t)(((r[0] >> bit) & 1) << (2 * q));
					pen |= (uint8_t)(((r[1] >> bit) & 1) << (2 * q + 1));
				}
				out[y * 16 + x] = pen;
			}
		memcpy(p, out, 128);
	}
	return 0;
}

static void map_main(void)
{
	const machine_t *m = s_m;
	v60_bus_clear();
	v60_bus_map(0x000000, 0x00ffff, s_ram, 0x10000, 1, 1);
	if (s_kind == K_MSLIDER)   // (the RAM mirror)
		v60_bus_map(0x010000, 0x01ffff, s_ram, 0x10000, 1, 1);
	if (s_ram2)
		v60_bus_map(s_ram2_at, s_ram2_at + s_ram2_size - 1, s_ram2, s_ram2_alloc, 1, 1);
	v60_bus_map(0x100000, 0x13ffff, s_spr, 0x40000, 1, 1);
	v60_bus_map(0x140000, 0x15ffff, s_pal, 0x20000, 1, 1);
	v60_bus_map(0x160000, 0x17ffff, s_ram16, 0x20000, 1, 1);
	if (s_nv)
		v60_bus_map(m->nv_at, m->nv_at + (m->nv_size < 0x1000 ? 0xfff : m->nv_size - 1), s_nv, m->nv_size < 0x1000 ? 0x1000 : m->nv_size, 1, 1);
	if (s_kind == K_GDFS) {
		v60_bus_map(0x400000, 0x41ffff, s_tmapram, 0x20000, 1, 0);
		v60_bus_map(0x420000, 0x43ffff, s_gd_ram, 0x20000, 1, 1);
		v60_bus_map(0x600000, 0x600fff, s_gd_ram + 0x20000, 0x1000, 1, 1);
		v60_bus_map(0x800000, 0x87ffff, s_st_spr, 0x80000, 1, 0);
	}
	const uint32_t span = 0x1000000u - m->rom;
	v60_bus_map(m->rom, 0xffffff, s_rom, s_rom_size < span ? s_rom_size : span, 1, 0);
	v60_bus.read16 = main_read16;
	v60_bus.write16 = main_write16;
	v60_bus.irq_cycle = irq_callback;
}

static int ssv_init(const rbx_game *g)
{
	char mname[32];
	snprintf(mname, sizeof(mname), "%s", g->machine);
	char *colon = strchr(mname, ':');
	const char *init = colon ? colon + 1 : "ssv";
	if (colon)
		*colon = 0;
	s_m = NULL;
	for (size_t i = 0; i < sizeof(s_machines) / sizeof(s_machines[0]); i++)
		if (!strcmp(s_machines[i].name, mname))
			s_m = &s_machines[i];
	if (!s_m) {
		rbx_fail("%s: machine %s is not emulated", g->set, g->machine);
		return -1;
	}
	const machine_t *m = s_m;
	s_kind = m->kind;
	s_sxyreac2 = !strcmp(mname, "sxyreac2");
	// init_ssv / init_ssv_tilescram (sexy, eaglshot) / irq1 / pastelis
	const int tilescram = !strcmp(init, "ssv_tilescram") || !strcmp(init, "sexy") || !strcmp(init, "eaglshot");
	for (int i = 0; i < 16; i++)
		s_tile_code[i] = tilescram ? (uint32_t)i << 16
		                           : (uint32_t)(((i & 1) << 3 | (i & 2) << 1 | (i & 4) >> 1 | (i & 8) >> 3) << 16);
	s_irq_ultrax = !strcmp(init, "ssv_irq1");
	s_raster_irq = !strcmp(init, "pastelis");

	s_rom = rbx_region("maincpu", &s_rom_size);
	if (!s_rom)
		return -1;
	uint32_t gsize = 0;
	if (s_kind == K_EAGLSHOT) {
		s_ea_rom = rbx_region("gfxdata", &s_ea_rom_size);
		if (!s_ea_rom)
			return -1;
		s_gfx = NULL;
		s_gfx_tiles = 0;
	} else {
		s_gfx = rbx_region("sprites", &gsize);
		if (!s_gfx)
			return -1;
		s_gfx_tiles = gsize / 128;
		if (decode_gfx(s_gfx, gsize)) {
			rbx_fail("%s: out of memory", g->set);
			return -1;
		}
	}
	for (int b = 0; b < 4; b++) {
		char name[16];
		snprintf(name, sizeof(name), "ensoniq.%d", b);
		s_es_rom[b] = rbx_region_opt(name, &s_es_size[b]);
		if (!s_es_rom[b])
			s_es_size[b] = 0;
	}

	s_ram = (uint8_t *)calloc(1, 0x10000);
	s_ram16 = (uint8_t *)calloc(1, 0x20000);
	s_spr = (uint16_t *)calloc(1, SPR_WORDS * 2 + 0x100);
	s_pal = (uint32_t *)calloc(PENS, 4);
	s_snap_pal = (uint32_t *)calloc(PENS, 4);
	s_ram2_at = s_ram2_size = 0;
	switch (s_kind) {
	case K_SURVARTS: s_ram2_at = 0x400000; s_ram2_size = 0x40000; break;
	case K_SRMP7: s_ram2_at = 0x010000; s_ram2_size = 0x41000; break;   // (0x010000-0x050faf)
	case K_TWINEAG2: case K_ULTRAX: s_ram2_at = 0x010000; s_ram2_size = 0x30000; break;
	}
	s_ram2_alloc = 0x1000;   // (v60_bus_map repeats a power of two)
	while (s_ram2_alloc < s_ram2_size)
		s_ram2_alloc <<= 1;
	s_ram2 = s_ram2_size ? (uint8_t *)calloc(1, s_ram2_alloc) : NULL;
	s_nv = m->nv_size ? (uint8_t *)calloc(1, m->nv_size < 0x1000 ? 0x1000 : m->nv_size) : NULL;
	int bad = !s_ram || !s_ram16 || !s_spr || !s_pal || !s_snap_pal || (s_ram2_size && !s_ram2) || (m->nv_size && !s_nv);
	bad |= !(s_snap_spr = (uint16_t *)calloc(1, SPR_WORDS * 2 + 0x100));
	if (s_kind == K_GDFS) {
		s_st_gfx = (uint8_t *)calloc(1, 4 * 0x100000);
		s_st_spr = (uint16_t *)calloc(1, 0x80000);
		s_tmapram = (uint16_t *)calloc(1, 0x20000);
		s_gd_ram = (uint8_t *)calloc(1, 0x21000);
		s_st_opaque = (uint8_t *)malloc(0x400000 / 128);
		s_snap_st_gfx = (uint8_t *)calloc(1, 4 * 0x100000);
		s_snap_st_spr = (uint16_t *)calloc(1, 0x80000);
		s_snap_tmap = (uint16_t *)calloc(1, 0x20000);
		bad |= !s_st_gfx || !s_st_spr || !s_tmapram || !s_gd_ram || !s_st_opaque || !s_snap_st_gfx || !s_snap_st_spr || !s_snap_tmap;
		if (!bad) {
			memset(s_st_opaque, 1, 0x400000 / 128);
			s_st_zg.pix = s_snap_st_gfx;
			s_st_zg.opaque = s_st_opaque;
			s_st_zg.count = 0x400000 / 128;
			s_st_zg.w = 16;
			s_st_zg.h = 8;
		}
		s_st_rom = rbx_region("st0020_spr", &s_st_rom_size);
		uint32_t tsize = 0;
		s_gd_tiles = rbx_region("tiles", &tsize);
		s_gd_tile_count = tsize / 256;
		if (!s_st_rom || !s_gd_tiles)
			bad = 1;
	}
	if (s_kind == K_EAGLSHOT)
		bad |= !(s_ea_gfx = (uint8_t *)calloc(1, 16 * 0x40000)) || !(s_snap_ea_gfx = (uint8_t *)calloc(1, 16 * 0x40000));
	if (m->dsp) {
		uint32_t psize, dsize;
		const uint8_t *prg = rbx_region("dspprg", &psize), *data = rbx_region("dspdata", &dsize);
		if (!prg || !data)
			bad = 1;
		else
			upd96050_init(&s_dsp, prg, psize, data, dsize);
	}
	if (bad || rbx_audio_setup_stereo(CLOCK / OUT_PERIOD) || rbx_video_setup(m->vis_w, m->vis_h, PENS)) {
		rbx_fail("%s: out of memory", g->set);
		ssv_exit();
		return -1;
	}
	rbx_audio.frame_mhz = FRAME_MHZ;
	for (int j = 0; j < BANDS; j++) {
		s_cmd[j].size = sizeof(cmd_t);
		s_zcmd[j].size = sizeof(zcmd_t);
	}
	rbx_prof_name(RBX_PA_DRV, s_pa_names, SV_PA_END - RBX_PA_DRV);
	s_pal_dirty = 1;
	for (int i = 0; i < P_KEY0; i++)
		s_port[i] = rbx_port_find(s_port_tags[i]);
	for (int i = 0; i < 4; i++) {
		char tag[8];
		snprintf(tag, sizeof(tag), "KEY%d", i);
		s_port[P_KEY0 + i] = rbx_port_find(tag);
	}

	// the battery backed RAM, kept as MAME's <set>.nv; gdfs's EEPROM as <set>.eeprom
	extern const char *rbx_nvram_dir(void);
	const char *dir = rbx_nvram_dir();
	s_nv_path[0] = s_ee_path[0] = 0;
	if (s_nv && dir) {
		snprintf(s_nv_path, sizeof(s_nv_path), "%s%s.nv", dir, g->set);
		FILE *f = fopen(s_nv_path, "rb");
		if (f) {
			if (fread(s_nv, 1, m->nv_size, f) != m->nv_size)
				memset(s_nv, 0, m->nv_size);
			fclose(f);
		}
	}
	if (s_kind == K_GDFS) {
		eeprom_init(&s_eeprom);
		if (dir) {
			snprintf(s_ee_path, sizeof(s_ee_path), "%s%s.eeprom", dir, g->set);
			eeprom_load(&s_eeprom, s_ee_path);
		}
	}

	ssv_reset();
	map_main();
	return 0;
}

static void ssv_exit(void)
{
	rbx_video_wait_idle();
	if (s_m)
		nv_save();
	v60_release();
	free(s_ram);
	free(s_ram2);
	free(s_ram16);
	free(s_spr);
	free(s_pal);
	free(s_snap_pal);
	free(s_snap_spr);
	free(s_snap_st_gfx);
	for (int j = 0; j < BANDS; j++) {
		free(s_cmd[j].p);
		free(s_zcmd[j].p);
		memset(&s_cmd[j], 0, sizeof(s_cmd[j]));
		memset(&s_zcmd[j], 0, sizeof(s_zcmd[j]));
	}
	free(s_snap_st_spr);
	free(s_snap_tmap);
	free(s_snap_ea_gfx);
	s_snap_st_gfx = s_snap_ea_gfx = NULL;
	s_snap_st_spr = s_snap_tmap = NULL;
	free(s_nv);
	free(s_st_gfx);
	free(s_st_spr);
	free(s_tmapram);
	free(s_gd_ram);
	free(s_st_opaque);
	free(s_ea_gfx);
	s_ram = s_ram2 = s_ram16 = s_nv = s_st_gfx = s_gd_ram = s_st_opaque = s_ea_gfx = NULL;
	s_spr = s_snap_spr = s_st_spr = s_tmapram = NULL;
	s_pal = s_snap_pal = NULL;
	s_nv_path[0] = s_ee_path[0] = 0;
	memset(&s_st_zg, 0, sizeof(s_st_zg));
	s_m = NULL;
}

// ---------------------------------------------------------------------------------------------------- save states
static int ssv_state(void *ctx, int load)
{
	rbx_video_wait_idle();
	v60_state_t vs;
	if (!load)
		v60_get_state(&vs);
	int rc = 0;
	rc |= rbx_state_io(ctx, "v60", &vs, sizeof(vs));
	rc |= rbx_state_io(ctx, "ram", s_ram, 0x10000);
	if (s_ram2)
		rc |= rbx_state_io(ctx, "ram2", s_ram2, s_ram2_size);
	rc |= rbx_state_io(ctx, "ram16", s_ram16, 0x20000);
	rc |= rbx_state_io(ctx, "spr", s_spr, SPR_WORDS * 2);
	rc |= rbx_state_io(ctx, "pal", s_pal, PENS * 4);
	if (s_nv)
		rc |= rbx_state_io(ctx, "nv", s_nv, s_m->nv_size);
	rc |= rbx_state_io(ctx, "st", &s_st, sizeof(s_st));
	rc |= rbx_state_io(ctx, "es5506", &s_es, ES5506_STATE_SIZE);
	rc |= rbx_state_io(ctx, "bands", s_bands, sizeof(s_bands));
	int nb[2] = { s_nbands, s_band_y };
	rc |= rbx_state_io(ctx, "nbands", nb, sizeof(nb));
	if (s_m->dsp)
		rc |= rbx_state_io(ctx, "dsp", &s_dsp, UPD96050_STATE_SIZE);
	if (s_kind == K_GDFS) {
		rc |= rbx_state_io(ctx, "stgfx", s_st_gfx, 4 * 0x100000);
		rc |= rbx_state_io(ctx, "stspr", s_st_spr, 0x80000);
		rc |= rbx_state_io(ctx, "tmap", s_tmapram, 0x20000);
		rc |= rbx_state_io(ctx, "gdram", s_gd_ram, 0x21000);
		rc |= rbx_state_io(ctx, "eeprom", &s_eeprom, sizeof(s_eeprom));
	}
	if (s_kind == K_EAGLSHOT)
		rc |= rbx_state_io(ctx, "eagfx", s_ea_gfx, 16 * 0x40000);
	if (rc)
		return -1;
	if (load) {
		v60_set_state(&vs);
		s_nbands = nb[0];
		s_band_y = nb[1];
		sound_bank_update();
		if (s_m->dsp)
			s_dsp.writes++;   // (the idle loop check starts again)
		s_pal_dirty = 1;
		memset(s_snap_pal, 0xff, PENS * 4);
		dirty_all();
	}
	return 0;
}

static int ssv_save(void *ctx) { return ssv_state(ctx, 0); }
static int ssv_load(void *ctx) { return ssv_state(ctx, 1); }
static int ssv_supported(const char *machine)
{
	char mname[32];
	snprintf(mname, sizeof(mname), "%s", machine);
	char *colon = strchr(mname, ':');
	if (colon)
		*colon = 0;
	for (size_t i = 0; i < sizeof(s_machines) / sizeof(s_machines[0]); i++)
		if (!strcmp(s_machines[i].name, mname))
			return 1;
	return 0;   // (jsk: the V810)
}

const rbx_driver rbx_drv_ssv = { "Seta SSV", ssv_init, ssv_exit, ssv_reset, ssv_frame, ssv_save, ssv_load, ssv_supported,
                                 ssv_render, ssv_help };
