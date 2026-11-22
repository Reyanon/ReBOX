/*  Williams / Midway T-unit with the ADPCM sound board (Mortal Kombat T-unit, NBA Jam, NBA Jam T.E., Judge Dredd),
    ported to C from MAME src/mame/williams/midtunit.cpp, midtunit_m.cpp, midtunit.h, midtunit_v.cpp,
    midtunit_v.h, midtunit_v.ipp (BSD-3-Clause, copyright-holders: Alex Pasadyn, Zsolt Vasvari, Ernesto Corvi,
    Aaron Giles; thanks-to: Kurt Mahan). Mortal Kombat II: the DCS board (machine/dcs.c), mk2_state.
    The Wolf unit (rbx_drv_midwunit: the same video device as midwunit_video_device, DCS 8K) from MAME
    src/mame/williams/midwunit.cpp, midwunit_m.cpp, midwunit.h (BSD-3-Clause, copyright-holders: Aaron Giles,
    Ernesto Corvi) and the serial PIC simulation and emulation (midway_serial_pic_device,
    midway_serial_pic_emu_device: a PIC16C57) of src/mame/williams/midwayic.cpp (BSD-3-Clause,
    copyright-holders: Aaron Giles).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/tms34010/tms34010.h"
#include "../../cpu/pic16c5x/pic16c5x.h"
#include "../machine/williams_snd.h"
#include "../machine/dcs.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

enum { DMA_LRSKIP = 0, DMA_COMMAND, DMA_OFFSETLO, DMA_OFFSETHI, DMA_XSTART, DMA_YSTART, DMA_WIDTH, DMA_HEIGHT,
       DMA_PALETTE, DMA_COLOR, DMA_SCALE_X, DMA_SCALE_Y, DMA_TOPCLIP, DMA_BOTCLIP, DMA_UNKNOWN_E, DMA_CONFIG,
       DMA_LEFTCLIP, DMA_RIGHTCLIP };
enum { PIXEL_SKIP = 0, PIXEL_COLOR = 1, PIXEL_COPY = 2 };
#define XPOSMASK 0x3ff
#define YPOSMASK 0x1ff
#define CLOCK 50000000u
#define NPENS 32768

enum { PROT_MK = 1, PROT_NBAJAM, PROT_NBAJAMTE, PROT_JDREDD };

static const uint8_t mk_prot_values[] = {
	0x13, 0x27, 0x0f, 0x1f, 0x3e, 0x3d, 0x3b, 0x37, 0x2e, 0x1c, 0x38, 0x31, 0x22, 0x05, 0x0a, 0x15,
	0x2b, 0x16, 0x2d, 0x1a, 0x34, 0x28, 0x10, 0x21, 0x03, 0x06, 0x0c, 0x19, 0x32, 0x24, 0x09, 0x13,
	0x27, 0x0f, 0x1f, 0x3e, 0x3d, 0x3b, 0x37, 0x2e, 0x1c, 0x38, 0x31, 0x22, 0x05, 0x0a, 0x15, 0x2b,
	0x16, 0x2d, 0x1a, 0x34, 0x28, 0x10, 0x21, 0x03, 0xff };
static const uint32_t nbajam_prot_values[128] = {
	0x21283b3b, 0x2439383b, 0x31283b3b, 0x302b3938, 0x31283b3b, 0x302b3938, 0x232f2f2f, 0x26383b3b,
	0x21283b3b, 0x2439383b, 0x312a1224, 0x302b1120, 0x312a1224, 0x302b1120, 0x232d283b, 0x26383b3b,
	0x2b3b3b3b, 0x2e2e2e2e, 0x39383b1b, 0x383b3b1b, 0x3b3b3b1b, 0x3a3a3a1a, 0x2b3b3b3b, 0x2e2e2e2e,
	0x2b39383b, 0x2e2e2e2e, 0x393a1a18, 0x383b1b1b, 0x3b3b1b1b, 0x3a3a1a18, 0x2b39383b, 0x2e2e2e2e,
	0x01202b3b, 0x0431283b, 0x11202b3b, 0x1021283b, 0x11202b3b, 0x1021283b, 0x03273b3b, 0x06302b39,
	0x09302b39, 0x0c232f2f, 0x19322e06, 0x18312a12, 0x19322e06, 0x18312a12, 0x0b31283b, 0x0e26383b,
	0x03273b3b, 0x06302b39, 0x11202b3b, 0x1021283b, 0x13273938, 0x12243938, 0x03273b3b, 0x06302b39,
	0x0b31283b, 0x0e26383b, 0x19322e06, 0x18312a12, 0x1b332f05, 0x1a302b11, 0x0b31283b, 0x0e26383b,
	0x21283b3b, 0x2439383b, 0x31283b3b, 0x302b3938, 0x31283b3b, 0x302b3938, 0x232f2f2f, 0x26383b3b,
	0x21283b3b, 0x2439383b, 0x312a1224, 0x302b1120, 0x312a1224, 0x302b1120, 0x232d283b, 0x26383b3b,
	0x2b3b3b3b, 0x2e2e2e2e, 0x39383b1b, 0x383b3b1b, 0x3b3b3b1b, 0x3a3a3a1a, 0x2b3b3b3b, 0x2e2e2e2e,
	0x2b39383b, 0x2e2e2e2e, 0x393a1a18, 0x383b1b1b, 0x3b3b1b1b, 0x3a3a1a18, 0x2b39383b, 0x2e2e2e2e,
	0x01202b3b, 0x0431283b, 0x11202b3b, 0x1021283b, 0x11202b3b, 0x1021283b, 0x03273b3b, 0x06302b39,
	0x09302b39, 0x0c232f2f, 0x19322e06, 0x18312a12, 0x19322e06, 0x18312a12, 0x0b31283b, 0x0e26383b,
	0x03273b3b, 0x06302b39, 0x11202b3b, 0x1021283b, 0x13273938, 0x12243938, 0x03273b3b, 0x06302b39,
	0x0b31283b, 0x0e26383b, 0x19322e06, 0x18312a12, 0x1b332f05, 0x1a302b11, 0x0b31283b, 0x0e26383b };
static const uint32_t nbajamte_prot_values[128] = {
	0x00000000, 0x04081020, 0x08102000, 0x0c183122, 0x10200000, 0x14281020, 0x18312204, 0x1c393326,
	0x20000001, 0x24081021, 0x28102000, 0x2c183122, 0x30200001, 0x34281021, 0x38312204, 0x3c393326,
	0x00000102, 0x04081122, 0x08102102, 0x0c183122, 0x10200000, 0x14281020, 0x18312204, 0x1c393326,
	0x20000103, 0x24081123, 0x28102102, 0x2c183122, 0x30200001, 0x34281021, 0x38312204, 0x3c393326,
	0x00010204, 0x04091224, 0x08112204, 0x0c193326, 0x10210204, 0x14291224, 0x18312204, 0x1c393326,
	0x20000001, 0x24081021, 0x28102000, 0x2c183122, 0x30200001, 0x34281021, 0x38312204, 0x3c393326,
	0x00010306, 0x04091326, 0x08112306, 0x0c193326, 0x10210204, 0x14291224, 0x18312204, 0x1c393326,
	0x20000103, 0x24081123, 0x28102102, 0x2c183122, 0x30200001, 0x34281021, 0x38312204, 0x3c393326,
	0x00000000, 0x01201028, 0x02213018, 0x03012030, 0x04223138, 0x05022110, 0x06030120, 0x07231108,
	0x08042231, 0x09243219, 0x0a251229, 0x0b050201, 0x0c261309, 0x0d060321, 0x0e072311, 0x0f273339,
	0x10080422, 0x1128140a, 0x1229343a, 0x13092412, 0x142a351a, 0x150a2532, 0x160b0502, 0x172b152a,
	0x180c2613, 0x192c363b, 0x1a2d160b, 0x1b0d0623, 0x1c2e172b, 0x1d0e0703, 0x1e0f2733, 0x1f2f371b,
	0x20100804, 0x2130182c, 0x2231381c, 0x23112834, 0x2432393c, 0x25122914, 0x26130924, 0x2733190c,
	0x28142a35, 0x29343a1d, 0x2a351a2d, 0x2b150a05, 0x2c361b0d, 0x2d160b25, 0x2e172b15, 0x2f373b3d,
	0x30180c26, 0x31381c0e, 0x32393c3e, 0x33192c16, 0x343a3d1e, 0x351a2d36, 0x361b0d06, 0x373b1d2e,
	0x381c2e17, 0x393c3e3f, 0x3a3d1e0f, 0x3b1d0e27, 0x3c3e1f2f, 0x3d1e0f07, 0x3e1f2f37, 0x3f3f3f1f };
static const uint8_t jdredd_prot_values_10740[] = {
	0x14,0x2A,0x15,0x0A,0x25,0x32,0x39,0x1C, 0x2E,0x37,0x3B,0x1D,0x2E,0x37,0x1B,0x0D,
	0x26,0x33,0x39,0x3C,0x1E,0x2F,0x37,0x3B, 0x3D,0x3E,0x3F,0x1F,0x2F,0x17,0x0B,0x25,
	0x32,0x19,0x0C,0x26,0x33,0x19,0x2C,0x16, 0x2B,0x15,0x0A,0x05,0x22,0x00 };
static const uint8_t jdredd_prot_values_13240[] = { 0x28 };
static const uint8_t jdredd_prot_values_76540[] = { 0x04,0x08 };
static const uint8_t jdredd_prot_values_77760[] = {
	0x14,0x2A,0x14,0x2A,0x35,0x2A,0x35,0x1A, 0x35,0x1A,0x2D,0x1A,0x2D,0x36,0x2D,0x36,
	0x1B,0x36,0x1B,0x36,0x2C,0x36,0x2C,0x18, 0x2C,0x18,0x31,0x18,0x31,0x22,0x31,0x22,
	0x04,0x22,0x04,0x08,0x04,0x08,0x10,0x08, 0x10,0x20,0x10,0x20,0x00,0x20,0x00,0x00,
	0x00,0x00,0x01,0x00,0x01,0x02,0x01,0x02, 0x05,0x02,0x05,0x0B,0x05,0x0B,0x16,0x0B,
	0x16,0x2C,0x16,0x2C,0x18,0x2C,0x18,0x31, 0x18,0x31,0x22,0x31,0x22,0x04,0x22,0x04,
	0x08,0x04,0x08,0x10,0x08,0x10,0x20,0x10, 0x20,0x00,0x00 };
static const uint8_t jdredd_prot_values_80020[] = {
	0x3A,0x1D,0x2E,0x37,0x00,0x00,0x2C,0x1C, 0x39,0x33,0x00,0x00,0x00,0x00,0x00,0x00 };
static const struct { uint32_t off; const uint8_t *t; uint8_t n; } s_jdredd_tabs[] = {
	{ 0x1074, jdredd_prot_values_10740, sizeof(jdredd_prot_values_10740) },
	{ 0x1324, jdredd_prot_values_13240, sizeof(jdredd_prot_values_13240) },
	{ 0x7654, jdredd_prot_values_76540, sizeof(jdredd_prot_values_76540) },
	{ 0x7776, jdredd_prot_values_77760, sizeof(jdredd_prot_values_77760) },
	{ 0x8002, jdredd_prot_values_80020, sizeof(jdredd_prot_values_80020) },
};

// the configuration
static int s_prot, s_mkturbo, s_dcs;   // s_dcs: mk2_state (DCS sound, the large graphics ROM)
static int s_wolf, s_wwfmania;        // the Wolf unit (s_dcs too)
static int s_picemu, s_umk3;          // wunit_picemu (the PIC16C57), init_umk3 / init_umk3r11 (the palette hack)
// midway_serial_pic_emu_device (saved apart from S: the PIC's state and these)
static pic16c5x_t s_pic;
static struct { uint8_t command, data_out, clk, status; uint64_t t; } s_pe;
static uint32_t s_cmos_words;
static int s_year;
static uint8_t *s_gfx;
static uint32_t s_gfx_size;

// memory (heap, ADR-040)
static uint16_t *s_ram;        // 0x40000 words
static uint16_t *s_cmos;       // 0x2000 words
static uint16_t *s_palram;     // 0x8000 words
static uint16_t *s_vram;       // 0x80000 words
static uint16_t *s_maincpu;
static uint32_t *s_pal;
static uint16_t *s_fb;         // 512 x 512
static char s_nv_path[512];
static int s_port[4];

// a blit's parameters (dma_state_t)
typedef struct {
	uint32_t offset;
	int32_t xpos, ypos, width, height;
	uint16_t palette, color;
	uint8_t yflip, preskip, postskip, pad;
	int32_t topclip, botclip, leftclip, rightclip, startskip, endskip;
	uint16_t xstep, ystep;
} dma_params;

// state (saved)
static struct {
	uint16_t control, dma_register[18];
	uint32_t gfxbank_offset[2];
	uint8_t videobank_select, cmos_write_enable, fake_sound_state, mk_prot_index;
	uint16_t nbajam_prot_queue[5];
	uint8_t nbajam_prot_index, jdredd_tab, jdredd_prot_index, dma_pending;
	dma_params dma;
	uint64_t dma_at, ticks, cyc_frac, cyc_target, line_tick0, line_cyc0;
	uint16_t mk2_prot_data;
	// the Wolf unit
	uint16_t iodata[8];
	uint8_t ioshuffle[16];
	uint8_t pic_data[16], pic_buff, pic_idx, pic_status;
	uint32_t rand_seed;
} S;

// ---- the DMA blitter (midtunit_v.cpp: one routine with the template's parameters as arguments) ----
static inline uint32_t extract(uint32_t o, uint32_t m)
{
	uint32_t a = o >> 3;
	uint32_t w = a + 1 < s_gfx_size ? (uint32_t)(s_gfx[a] | (s_gfx[a + 1] << 8)) : a < s_gfx_size ? s_gfx[a] : 0;
	return (w >> (o & 7)) & m;
}

static inline __attribute__((always_inline)) void dma_draw_t(const dma_params *D, const int BitsPerPixel, const int XFlip, const int Skip,
	const int Scale, const int Zero, const int NonZero)
{
	int const height = D->height << 8;
	uint32_t offset = D->offset;
	uint16_t pal = D->palette;
	uint16_t color = pal | D->color;
	int sy = D->ypos;
	int iy = 0;
	int ty;
	int const mask = (1 << BitsPerPixel) - 1;
	int const xstep = Scale ? D->xstep : 0x100;
	int pre = 0, post = 0;
	while (iy < height) {
		int const startskip = D->startskip << 8;
		int endskip = D->endskip << 8;
		int width = D->width << 8;
		int sx = D->xpos;
		int ix = 0;
		int tx;
		uint32_t o = offset;
		uint16_t *d;
		(void)endskip;
		if (Skip) {
			uint8_t const value = (uint8_t)extract(o, 0xff);
			o += 8;
			pre = (value & 0x0f) << (D->preskip + 8);
			tx = pre / xstep;
			if (XFlip)
				sx = (sx - tx) & XPOSMASK;
			else
				sx = (sx + tx) & XPOSMASK;
			ix += tx * xstep;
			post = ((value >> 4) & 0x0f) << (D->postskip + 8);
			width -= post;
			endskip -= post;
		}
		if (sy < D->topclip || sy > D->botclip)
			goto clipy;
		if (ix < startskip) {
			tx = ((startskip - ix) / xstep) * xstep;
			ix += tx;
			o += (uint32_t)((tx >> 8) * BitsPerPixel);
		}
		if ((width >> 8) > D->width - D->endskip)
			width = (D->width - D->endskip) << 8;
		d = &s_vram[sy * 512];
		// (an unscaled row: the same pixels without the checks that cannot fail; sy <= YPOSMASK and sx <= XPOSMASK
		// keep d[sx] inside s_vram, the source checked once for the whole row)
		if (!Scale && ix < width) {
			const int n = (width - ix + 0xff) >> 8;
			const int lc = D->leftclip, rc = D->rightclip;
			if (lc > rc)
				goto clipy;
			if (((o + (uint32_t)(n - 1) * BitsPerPixel) >> 3) + 1 < s_gfx_size) {
				const uint8_t *const g = s_gfx;
				// the clip as a span of i when sx does not wrap in the row (the same pixels, no test per pixel)
				int i0 = 0, i1 = n, chk = 1;
				if (XFlip ? sx - (n - 1) >= 0 : sx + (n - 1) <= XPOSMASK) {
					const int a = XFlip ? sx - rc : lc - sx, b = XFlip ? sx - lc : rc - sx;
					i0 = a > 0 ? a : 0;
					i1 = b + 1 < n ? b + 1 : n;
					if (i0 >= i1)
						goto clipy;
					o += (uint32_t)(i0 * BitsPerPixel);
					sx = XFlip ? sx - i0 : sx + i0;
					chk = 0;
				}
				for (int i = i0; i < i1; i++) {
					if (!chk || (sx >= lc && sx <= rc)) {
						const uint32_t a = o >> 3;
						const int pixel = (int)((((uint32_t)g[a] | ((uint32_t)g[a + 1] << 8)) >> (o & 7)) & (uint32_t)mask);
						uint16_t *const p = &d[sx];
						if (Zero == NonZero) {
							if (Zero == PIXEL_COLOR)
								*p = color;
							else if (Zero == PIXEL_COPY)
								*p = (uint16_t)(pixel | pal);
						} else if (pixel) {
							if (NonZero == PIXEL_COLOR) *p = color;
							else if (NonZero == PIXEL_COPY) *p = (uint16_t)(pixel | pal);
						} else {
							if (Zero == PIXEL_COLOR) *p = color;
							else if (Zero == PIXEL_COPY) *p = pal;
						}
					}
					sx = XFlip ? (sx - 1) & XPOSMASK : (sx + 1) & XPOSMASK;
					o += (uint32_t)BitsPerPixel;
				}
				goto clipy;
			}
		}
		while (ix < width) {
			if (sx >= D->leftclip && sx <= D->rightclip) {
				// (sx can pass 511 within the clip: MAME writes into the next line; kept inside the buffer here)
				uint16_t *p = &d[sx];
				if (p < s_vram + 0x80000) {
					if (Zero == NonZero) {
						if (Zero == PIXEL_COLOR)
							*p = color;
						else if (Zero == PIXEL_COPY)
							*p = (uint16_t)(extract(o, (uint32_t)mask) | pal);
					} else {
						int pixel = (int)extract(o, (uint32_t)mask);
						if (pixel) {
							if (NonZero == PIXEL_COLOR) *p = color;
							else if (NonZero == PIXEL_COPY) *p = (uint16_t)(pixel | pal);
						} else {
							if (Zero == PIXEL_COLOR) *p = color;
							else if (Zero == PIXEL_COPY) *p = pal;
						}
					}
				}
			}
			if (XFlip)
				sx = (sx - 1) & XPOSMASK;
			else
				sx = (sx + 1) & XPOSMASK;
			if (!Scale) {
				ix += 0x100;
				o += (uint32_t)BitsPerPixel;
			} else {
				tx = ix >> 8;
				ix += xstep;
				tx = (ix >> 8) - tx;
				o += (uint32_t)(BitsPerPixel * tx);
			}
		}
	clipy:
		if (D->yflip)
			sy = (sy - 1) & YPOSMASK;
		else
			sy = (sy + 1) & YPOSMASK;
		if (!Scale) {
			iy += 0x100;
			width = D->width;
			if (Skip) {
				offset += 8;
				width -= (pre + post) >> 8;
				if (width > 0) offset += (uint32_t)(width * BitsPerPixel);
			} else
				offset += (uint32_t)(width * BitsPerPixel);
		} else {
			ty = iy >> 8;
			iy += D->ystep;
			ty = (iy >> 8) - ty;
			if (!Skip)
				offset += (uint32_t)(ty * D->width * BitsPerPixel);
			else if (ty--) {
				o = offset + 8;
				width = D->width - ((pre + post) >> 8);
				if (width > 0) o += (uint32_t)(width * BitsPerPixel);
				while (ty--) {
					uint8_t const value = (uint8_t)extract(o, 0xff);
					o += 8;
					pre = (value & 0x0f) << D->preskip;
					post = ((value >> 4) & 0x0f) << D->postskip;
					width = D->width - pre - post;
					if (width > 0) o += (uint32_t)(width * BitsPerPixel);
				}
				offset = o;
			}
		}
	}
}

// the routines by (zero, nonzero) with bpp, xflip, skip and scale as constants per routine group
#define DMA_FN(name, Z, NZ) \
	static void name(const dma_params *D, int bpp, int xf, int skip, int scale) { \
		static const int bits[8] = { 8, 1, 2, 3, 4, 5, 6, 7 }; \
		const int b = bits[bpp]; \
		if (!scale) { \
			if (!skip) { if (xf) dma_draw_t(D, b, 1, 0, 0, Z, NZ); else dma_draw_t(D, b, 0, 0, 0, Z, NZ); } \
			else { if (xf) dma_draw_t(D, b, 1, 1, 0, Z, NZ); else dma_draw_t(D, b, 0, 1, 0, Z, NZ); } \
		} else { \
			if (!skip) { if (xf) dma_draw_t(D, b, 1, 0, 1, Z, NZ); else dma_draw_t(D, b, 0, 0, 1, Z, NZ); } \
			else { if (xf) dma_draw_t(D, b, 1, 1, 1, Z, NZ); else dma_draw_t(D, b, 0, 1, 1, Z, NZ); } \
		} \
	}
DMA_FN(draw_p0, PIXEL_COPY, PIXEL_SKIP)
DMA_FN(draw_p1, PIXEL_SKIP, PIXEL_COPY)
DMA_FN(draw_c0, PIXEL_COLOR, PIXEL_SKIP)
DMA_FN(draw_c1, PIXEL_SKIP, PIXEL_COLOR)
DMA_FN(draw_p0p1, PIXEL_COPY, PIXEL_COPY)
DMA_FN(draw_c0c1, PIXEL_COLOR, PIXEL_COLOR)
DMA_FN(draw_c0p1, PIXEL_COLOR, PIXEL_COPY)
DMA_FN(draw_p0c1, PIXEL_COPY, PIXEL_COLOR)
typedef void (*draw_fn)(const dma_params *D, int bpp, int xf, int skip, int scale);
// INIT_TEMPLATED_DMA_DRAW_GROUP: the routine of command bits 0-3
static const draw_fn s_draw[16] = { NULL, draw_p0, draw_p1, draw_p0p1, draw_c0, draw_c0, draw_c0p1, draw_c0p1,
                                    draw_c1, draw_p0c1, draw_c1, draw_p0c1, draw_c0c1, draw_c0c1, draw_c0c1, draw_c0c1 };

// ---- the blits on the second worker thread (ADR-057; off: RBX_TU_NODMATHREAD=1) ----
// The blitter only writes s_vram from its parameters and the graphics ROM, so a blit is queued at its command and drawn
// by rbx_aux2's job while the TMS34010 runs on: its timing (dma_at, the interrupt, the status bit) does not depend on
// the drawing. Everything that reads or writes s_vram waits for the queue first (the CPU's VRAM reads and writes, the
// shift register transfers, the state, the exit), except a scanline's copy, which waits only when a queued blit may
// touch its row. So s_vram is the same at every look as when the blits are drawn at their commands.
#define DQ_N 1024
typedef struct {
	dma_params p;
	draw_fn f;
	uint8_t bpp, xf, skip, scale;
	uint16_t row0, rows;   // the rows it may write: row0 .. row0 + rows - 1 (mod 512)
} dma_job_t;
static dma_job_t *s_dq;
static unsigned s_dq_head, s_dq_tail;   // (head: queued by the emulation thread; tail: drawn, by the job)
static int s_dq_on;

static __thread int t_emu;   // (the emulation thread: RBX_TU_DMASTAT=1 counts the blits it drew itself)
static unsigned s_ds_jobs, s_ds_here, s_ds_sync[3];   // (syncs: VRAM / shift register / state, a row, a full queue)
static void dma_stat(void)
{
	fprintf(stderr, "tu dma: %u blits, %u drawn by the emulation thread; syncs: vram %u, row %u, full %u\n", s_ds_jobs,
		s_ds_here, s_ds_sync[0], s_ds_sync[1], s_ds_sync[2]);
}
static void dma_run_job(const dma_job_t *j)
{
	s_ds_here += t_emu;
	j->f(&j->p, j->bpp, j->xf, j->skip, j->scale);
}

static void dma_job(void)
{
	unsigned t = __atomic_load_n(&s_dq_tail, __ATOMIC_RELAXED);
	for (;;) {
		const unsigned h = __atomic_load_n(&s_dq_head, __ATOMIC_ACQUIRE);
		if (t == h)
			break;
		do {
			dma_run_job(&s_dq[t % DQ_N]);
			t++;
			__atomic_store_n(&s_dq_tail, t, __ATOMIC_RELEASE);
		} while (t != h);
	}
}

static inline int dq_busy(void) { return __atomic_load_n(&s_dq_tail, __ATOMIC_ACQUIRE) != s_dq_head; }
// all the queued blits drawn
static inline void dma_flush(void)
{
	if (s_dq_on && dq_busy())
		s_ds_sync[0]++, rbx_aux2_sync();
}
// the queued blits drawn when one may write VRAM row `row` (a scanline's source)
static void dma_flush_row(unsigned row)
{
	if (!s_dq_on)
		return;
	for (unsigned t = __atomic_load_n(&s_dq_tail, __ATOMIC_ACQUIRE); t != s_dq_head; t++) {
		const dma_job_t *j = &s_dq[t % DQ_N];
		if (((row - j->row0) & YPOSMASK) < j->rows) {
			s_ds_sync[1]++, rbx_aux2_sync();
			return;
		}
	}
}

static void dma_w(uint32_t offset, uint16_t data)
{
	static const uint8_t register_map[2][16] = {
		{ 0,1,2,3,4,5,6,7,8,9,10,11,16,17,14,15 },
		{ 0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15 } };
	int const regbank = (S.dma_register[DMA_CONFIG] >> 5) & 1;
	int pixels = 0;
	int regnum = register_map[regbank][offset & 15];
	S.dma_register[regnum] = data;
	if (regnum != DMA_COMMAND)
		return;
	int const command = S.dma_register[DMA_COMMAND];
	tms34010_set_irq(0, 0);
	if (!(command & 0x8000))
		return;
	int const bpp = (command >> 12) & 7;
	S.dma.xpos = S.dma_register[DMA_XSTART] & XPOSMASK;
	S.dma.ypos = S.dma_register[DMA_YSTART] & YPOSMASK;
	S.dma.width = S.dma_register[DMA_WIDTH] & 0x3ff;
	S.dma.height = S.dma_register[DMA_HEIGHT] & 0x3ff;
	S.dma.palette = S.dma_register[DMA_PALETTE] & 0x7f00;
	S.dma.color = S.dma_register[DMA_COLOR] & 0xff;
	S.dma.yflip = (uint8_t)((command & 0x20) >> 5);
	S.dma.preskip = (uint8_t)((command >> 8) & 3);
	S.dma.postskip = (uint8_t)((command >> 10) & 3);
	S.dma.xstep = S.dma_register[DMA_SCALE_X] ? S.dma_register[DMA_SCALE_X] : 0x100;
	S.dma.ystep = S.dma_register[DMA_SCALE_Y] ? S.dma_register[DMA_SCALE_Y] : 0x100;
	S.dma.topclip = S.dma_register[DMA_TOPCLIP] & 0x1ff;
	S.dma.botclip = S.dma_register[DMA_BOTCLIP] & 0x1ff;
	S.dma.leftclip = S.dma_register[DMA_LEFTCLIP] & 0x3ff;
	S.dma.rightclip = S.dma_register[DMA_RIGHTCLIP] & 0x3ff;
	uint32_t gfxoffset = S.dma_register[DMA_OFFSETLO] | ((uint32_t)S.dma_register[DMA_OFFSETHI] << 16);
	if ((command & 0x0f) == 0x0c)
		gfxoffset = 0;
	if (!s_dcs && gfxoffset >= 0x2000000)   // (!m_gfx_rom_large)
		gfxoffset -= 0x2000000;
	if (gfxoffset >= 0xf8000000)
		gfxoffset -= 0xf8000000;
	if (gfxoffset < 0x10000000) {
		S.dma.offset = gfxoffset;
		if (command & 0x40) {
			S.dma.startskip = S.dma_register[DMA_LRSKIP] & 0xff;
			S.dma.endskip = S.dma_register[DMA_LRSKIP] >> 8;
		} else {
			S.dma.startskip = 0;
			S.dma.endskip = S.dma_register[DMA_LRSKIP];
		}
		const draw_fn f = s_draw[command & 0x0f];
		const int xf = (command >> 4) & 1, skip = (command >> 7) & 1;
		const int scale = !(S.dma.xstep == 0x100 && S.dma.ystep == 0x100);
		if (f && S.dma.height > 0) {
			if (!s_dq_on)
				f(&S.dma, bpp, xf, skip, scale);
			else {
				if (s_dq_head - __atomic_load_n(&s_dq_tail, __ATOMIC_ACQUIRE) >= DQ_N)
					s_ds_sync[2]++, rbx_aux2_sync();
				dma_job_t *j = &s_dq[s_dq_head % DQ_N];
				s_ds_jobs++;
				j->p = S.dma;
				j->f = f;
				j->bpp = (uint8_t)bpp;
				j->xf = (uint8_t)xf;
				j->skip = (uint8_t)skip;
				j->scale = (uint8_t)scale;
				// the rows: one per iteration of the row loop (up or down from ypos), and the next one (sx past 511)
				const unsigned n = scale ? ((unsigned)S.dma.height * 256u + S.dma.ystep - 1) / S.dma.ystep
				                         : (unsigned)S.dma.height;
				if (n >= 511) {
					j->row0 = 0;
					j->rows = 512;
				} else {
					j->row0 = (uint16_t)((S.dma.yflip ? (unsigned)S.dma.ypos - (n - 1) : (unsigned)S.dma.ypos) & YPOSMASK);
					j->rows = (uint16_t)(n + 1);
				}
				__atomic_store_n(&s_dq_head, s_dq_head + 1, __ATOMIC_RELEASE);
				rbx_aux2_kick();
			}
		}
		if (!scale) {
			pixels = S.dma.width * S.dma.height;
		} else {
			if (S.dma.xstep && S.dma.ystep)
				pixels = ((S.dma.width << 8) / S.dma.xstep) * ((S.dma.height << 8) / S.dma.ystep);
			else
				pixels = 0;
		}
	}
	// m_dma_timer->adjust(41 ns * pixels)
	if (pixels < 0) pixels = 0;
	S.dma_at = tms34010_total_cycles() + (((uint64_t)pixels * 41u * (CLOCK / 8) + 999999999u) / 1000000000u);
	S.dma_pending = 1;
	tms34010_abort();
}

static void dma_done(void)
{
	S.dma_pending = 0;
	S.dma_register[DMA_COMMAND] &= ~0x8000;
	tms34010_set_irq(0, 1);
}

// ---- the bus ----
static void vram_w(uint32_t offset, uint16_t data)
{
	dma_flush();
	offset *= 2;
	if (S.videobank_select) {
		s_vram[offset] = (uint16_t)((data & 0xff) | ((S.dma_register[DMA_PALETTE] & 0xff) << 8));
		s_vram[offset + 1] = (uint16_t)(((data >> 8) & 0xff) | (S.dma_register[DMA_PALETTE] & 0xff00));
	} else {
		s_vram[offset] = (uint16_t)((s_vram[offset] & 0xff) | ((data & 0xff) << 8));
		s_vram[offset + 1] = (uint16_t)((s_vram[offset + 1] & 0xff) | (data & 0xff00));
	}
}
static uint16_t vram_r(uint32_t offset)
{
	dma_flush();
	offset *= 2;
	if (S.videobank_select)
		return (uint16_t)((s_vram[offset] & 0x00ff) | (s_vram[offset + 1] << 8));
	return (uint16_t)((s_vram[offset] >> 8) | (s_vram[offset + 1] & 0xff00));
}

static inline uint8_t pal5bit(uint16_t v) { v &= 0x1f; return (uint8_t)((v << 3) | (v >> 2)); }
static void paletteram_w(uint32_t offset, uint16_t data)
{
	s_palram[offset] = data;
	s_pal[offset] = (uint32_t)pal5bit(data >> 10) << 16 | (uint32_t)pal5bit(data >> 5) << 8 | pal5bit(data);
}

static void control_w(uint16_t data)
{
	S.control = data;
	// gfx bank select is bit 7 (with the large ROMs of the DCS games)
	S.gfxbank_offset[0] = (!(S.control & 0x0080) || !s_dcs) ? 0x000000 : 0x800000;
	S.videobank_select = (S.control >> 5) & 1;
}

// ---- the Wolf unit ----
static void wu_control_w(uint16_t data)
{
	S.control = data;
	// gfx bank select is bits 8-9
	S.gfxbank_offset[0] = 0x800000 * ((S.control >> 8) & 3);
	// video bank select is unknown
	S.videobank_select = (S.control >> 11) & 1;
}

// running_machine::rand
static uint32_t machine_rand(void)
{
	S.rand_seed = 1664525u * S.rand_seed + 1013904223u;
	return (S.rand_seed >> 16) | (S.rand_seed << 16);
}

// midway_serial_pic_device (the simulation): generate_serial_data
static void pic_generate_serial_data(int upper)
{
	int year = s_year, month = 12, day = 11;
	uint32_t serial_number, temp;
	uint8_t serial_digit[9];
	serial_number = 123450;
	serial_number += (uint32_t)upper * 1000000;
	serial_number += 0x06 & 0x0f;   // (SERIAL_DIGIT: its default)
	serial_digit[0] = (serial_number / 100000000) % 10;
	serial_digit[1] = (serial_number / 10000000) % 10;
	serial_digit[2] = (serial_number / 1000000) % 10;
	serial_digit[3] = (serial_number / 100000) % 10;
	serial_digit[4] = (serial_number / 10000) % 10;
	serial_digit[5] = (serial_number / 1000) % 10;
	serial_digit[6] = (serial_number / 100) % 10;
	serial_digit[7] = (serial_number / 10) % 10;
	serial_digit[8] = (serial_number / 1) % 10;
	S.pic_data[12] = machine_rand() & 0xff;
	S.pic_data[13] = machine_rand() & 0xff;
	S.pic_data[14] = 0;
	S.pic_data[15] = 0;
	temp = 0x174 * (year - 1980) + 0x1f * (month - 1) + day;
	S.pic_data[10] = (temp >> 8) & 0xff;
	S.pic_data[11] = temp & 0xff;
	temp = serial_digit[4] + serial_digit[7] * 10 + serial_digit[1] * 100;
	temp = (temp + 5 * S.pic_data[13]) * 0x1bcd + 0x1f3f0;
	S.pic_data[7] = temp & 0xff;
	S.pic_data[8] = (temp >> 8) & 0xff;
	S.pic_data[9] = (temp >> 16) & 0xff;
	temp = serial_digit[6] + serial_digit[8] * 10 + serial_digit[0] * 100 + serial_digit[2] * 10000;
	temp = (temp + 2 * S.pic_data[13] + S.pic_data[12]) * 0x107f + 0x71e259;
	S.pic_data[3] = temp & 0xff;
	S.pic_data[4] = (temp >> 8) & 0xff;
	S.pic_data[5] = (temp >> 16) & 0xff;
	S.pic_data[6] = (temp >> 24) & 0xff;
	temp = serial_digit[5] * 10 + serial_digit[3] * 100;
	temp = (temp + S.pic_data[12]) * 0x245 + 0x3d74;
	S.pic_data[0] = temp & 0xff;
	S.pic_data[1] = (temp >> 8) & 0xff;
	S.pic_data[2] = (temp >> 16) & 0xff;
}

static void pic_write(uint8_t data)
{
	// status seems to reflect the clock bit
	S.pic_status = (data >> 4) & 1;
	// on the falling edge, clock the next data byte through
	if (!S.pic_status) {
		// the self-test writes 1F, 0F, and expects to read an F in the low 4 bits
		if (data & 0x0f)
			S.pic_buff = data & 0xf;
		else
			S.pic_buff = S.pic_data[S.pic_idx++ % sizeof(S.pic_data)];
	}
}

// midway_serial_pic_emu_device: PORTA command in, PORTB data out, PORTC bit 7 clock in, bit 6 status out
static uint8_t pe_read_port(void *ctx, int port)
{
	(void)ctx;
	if (port == 0)
		return s_pe.command;
	if (port == 2)
		return (uint8_t)(s_pe.clk << 7);   // read_c (bit 2: the RTC data, not fitted)
	return 0;
}
static void pe_write_port(void *ctx, int port, uint8_t data, uint8_t mem_mask)
{
	(void)ctx; (void)mem_mask;
	if (port == 1)
		s_pe.data_out = data;
	else if (port == 2)
		s_pe.status = (data >> 6) & 1;     // write_c
}
// run the PIC (4 MHz: 1 MHz cycles) up to the main CPU's time (6.25 MHz cycles)
static void pe_sync(void)
{
	uint64_t t = tms34010_total_cycles() * 4 / 25;
	if (t > s_pe.t) {
		pic16c5x_execute(&s_pic, (int)(t - s_pe.t));
		s_pe.t = t;
	}
}

static void wu_reset_shuffle(void)
{
	for (int i = 0; i < 16; i++)
		S.ioshuffle[i] = (uint8_t)(i % 8);
}

static void wwfmania_io_0_w(uint16_t data)
{
	wu_reset_shuffle();
	switch (data) {
	case 1: S.ioshuffle[4] = 0; S.ioshuffle[8] = 1; S.ioshuffle[1] = 2; S.ioshuffle[9] = 3; S.ioshuffle[2] = 4; break;
	case 2: S.ioshuffle[8] = 0; S.ioshuffle[2] = 1; S.ioshuffle[4] = 2; S.ioshuffle[6] = 3; S.ioshuffle[1] = 4; break;
	case 3: S.ioshuffle[1] = 0; S.ioshuffle[8] = 1; S.ioshuffle[2] = 2; S.ioshuffle[10] = 3; S.ioshuffle[5] = 4; break;
	case 4: S.ioshuffle[2] = 0; S.ioshuffle[4] = 1; S.ioshuffle[1] = 2; S.ioshuffle[7] = 3; S.ioshuffle[8] = 4; break;
	}
}

static void sound_sync(void)
{
	uint64_t c = tms34010_total_cycles();
	uint64_t t = S.line_tick0 + (c > S.line_cyc0 ? (c - S.line_cyc0) * 64000000u / CLOCK : 0);
	if (t > S.ticks)
		t = S.ticks;
	if (s_dcs)
		dcs_run(t * 5 / 4);   // (8 MHz ticks to 10 MHz ADSP cycles)
	else
		wms_snd_run(t);
}

// the protection handlers' ranges (installed over the control register at 0x01b00000)
static int prot_range(uint32_t a, uint32_t *off)
{
	switch (s_prot) {
	case PROT_MK:
		if (a >= 0x01b00000 && a < 0x01b70000) { *off = (a - 0x01b00000) >> 4; return 1; }
		break;
	case PROT_NBAJAM:
		if (a >= 0x01b14020 && a < 0x01b25040) { *off = (a - 0x01b14020) >> 4; return 1; }
		break;
	case PROT_NBAJAMTE:
		if (a >= 0x01b15f40 && a < 0x01b37f60) { *off = (a - 0x01b15f40) >> 4; return 1; }
		if (a >= 0x01b95f40 && a < 0x01bb7f60) { *off = (a - 0x01b95f40) >> 4; return 1; }
		break;
	case PROT_JDREDD:
		if (a >= 0x01b00000 && a < 0x01c00000) { *off = (a - 0x01b00000) >> 4; return 1; }
		break;
	}
	return 0;
}

static uint16_t prot_r(uint32_t off)
{
	(void)off;
	switch (s_prot) {
	case PROT_MK:
		if (S.mk_prot_index >= sizeof(mk_prot_values))
			S.mk_prot_index = 0;
		return (uint16_t)(mk_prot_values[S.mk_prot_index++] << 9);
	case PROT_NBAJAM:
	case PROT_NBAJAMTE: {
		uint16_t const result = S.nbajam_prot_queue[S.nbajam_prot_index];
		if (S.nbajam_prot_index < 4)
			S.nbajam_prot_index++;
		return result;
	}
	case PROT_JDREDD: {
		uint16_t result = 0xffff;
		if (S.jdredd_tab && S.jdredd_prot_index < s_jdredd_tabs[S.jdredd_tab - 1].n)
			result = (uint16_t)(s_jdredd_tabs[S.jdredd_tab - 1].t[S.jdredd_prot_index++] << 9);
		return result;
	}
	}
	return 0xffff;
}

static void prot_w(uint32_t off, uint16_t data)
{
	switch (s_prot) {
	case PROT_MK: {
		int const first_val = (data >> 9) & 0x3f;
		unsigned i;
		for (i = 0; i < sizeof(mk_prot_values); i++)
			if (mk_prot_values[i] == first_val) {
				S.mk_prot_index = (uint8_t)i;
				break;
			}
		if (i == sizeof(mk_prot_values))
			S.mk_prot_index = 0;
		break;
	}
	case PROT_NBAJAM:
	case PROT_NBAJAMTE: {
		uint32_t const protval = (s_prot == PROT_NBAJAM ? nbajam_prot_values : nbajamte_prot_values)[(off >> 6) & 0x7f];
		S.nbajam_prot_queue[0] = data;
		S.nbajam_prot_queue[1] = (uint16_t)(((protval >> 24) & 0xff) << 9);
		S.nbajam_prot_queue[2] = (uint16_t)(((protval >> 16) & 0xff) << 9);
		S.nbajam_prot_queue[3] = (uint16_t)(((protval >> 8) & 0xff) << 9);
		S.nbajam_prot_queue[4] = (uint16_t)(((protval >> 0) & 0xff) << 9);
		S.nbajam_prot_index = 0;
		break;
	}
	case PROT_JDREDD:
		for (unsigned i = 0; i < sizeof(s_jdredd_tabs) / sizeof(s_jdredd_tabs[0]); i++)
			if (s_jdredd_tabs[i].off == off) {
				S.jdredd_prot_index = 0;
				S.jdredd_tab = (uint8_t)(i + 1);
			}
		break;
	}
}

static uint16_t bus_read(uint32_t a)
{
	uint32_t off;
	if (a < 0x00400000)
		return vram_r(a >> 4);
	if (a >= 0x01400000 && a < 0x01420000)
		return s_cmos[(a - 0x01400000) >> 4];
	if (a >= 0x01600000 && a < 0x01600040) {
		int p = s_port[(a - 0x01600000) >> 4];
		return p >= 0 ? (uint16_t)rbx_port_read(p) : 0xffff;
	}
	if (a >= 0x01800000 && a < 0x01880000)
		return s_palram[(a - 0x01800000) >> 4];
	if (a >= 0x01a80000 && a < 0x01a80100) {
		uint32_t o = (a - 0x01a80000) >> 4;
		if (o == 0)
			o = 1;
		return S.dma_register[o];
	}
	if (prot_range(a, &off))
		return prot_r(off);
	if (s_dcs) {
		if (a >= 0x01d00000 && a < 0x01d00020) {   // dcs_state_r
			sound_sync();
			return (uint16_t)(dcs_control_r() >> 4);
		}
		if (a >= 0x01d01020 && a < 0x01d01040) {   // dcs_r
			int empty;
			sound_sync();
			uint16_t d = dcs_data_r(&empty);
			if (empty)   // (polling: eat some cycles)
				tms34010_eat(4444);
			return d & 0xff;
		}
		if ((a >= 0x01a190e0 && a < 0x01a19100) || (a >= 0x01a3d0c0 && a < 0x01a3d100))   // mk2_prot_r
			return S.mk2_prot_data;
		if (a >= 0x01a191c0 && a < 0x01a191e0)   // mk2_prot_shift_r
			return S.mk2_prot_data >> 1;
		if ((a >= 0x01d9d1e0 && a < 0x01d9d200) || (a >= 0x01def920 && a < 0x01def940))   // mk2_prot_const_r
			return 2;
	}
	if (a >= 0x01d00000 && a < 0x01d00020) {   // sound_state_r
		if (S.fake_sound_state) {
			S.fake_sound_state--;
			return 0;
		}
		return 0xffff;
	}
	if (a >= 0x01d01020 && a < 0x01d01040)
		return 0xffff;
	if (a >= 0x02000000 && a < 0x08000000) {   // midtunit_gfxrom_r
		uint32_t offset = (a - 0x02000000) >> 4;
		uint32_t base = S.gfxbank_offset[(offset >> 21) & 1];
		offset = base + (offset & 0x01fffff) * 2;
		return offset + 1 < s_gfx_size ? (uint16_t)(s_gfx[offset] | (s_gfx[offset + 1] << 8)) : 0;
	}
	if (a >= 0xff800000 || (a >= 0x1f800000 && a < 0x20000000)) {
		if (s_mkturbo && a >= 0xfffff400 && a < 0xfffff410)
			return (uint16_t)rand();
		return s_maincpu[(a & 0x007fffff) >> 4];
	}
	return 0xffff;   // (unmap_value_high)
}

static void bus_write(uint32_t a, uint16_t v)
{
	uint32_t off;
	if (a < 0x00400000) { vram_w(a >> 4, v); return; }
	if (a >= 0x01400000 && a < 0x01420000) { s_cmos[(a - 0x01400000) >> 4] = v; S.cmos_write_enable = 0; return; }
	if (a >= 0x01480000 && a < 0x01500000) { S.cmos_write_enable = 1; return; }
	if (a >= 0x01800000 && a < 0x01880000) { paletteram_w((a - 0x01800000) >> 4, v); return; }
	if (a >= 0x01a80000 && a < 0x01a80100) { dma_w((a - 0x01a80000) >> 4, v); return; }
	if (prot_range(a, &off)) { prot_w(off, v); return; }
	if ((a >= 0x01b00000 && a < 0x01b00020) || (a >= 0x01f00000 && a < 0x01f00020)) { control_w(v); return; }
	if (s_dcs) {
		if ((a >= 0x00f20c60 && a < 0x00f20c80) || (a >= 0x00f42820 && a < 0x00f42840)) {   // mk2_prot_w
			S.mk2_prot_data = v;
			return;
		}
		if (a >= 0x01d01020 && a < 0x01d01040) {   // dcs_w
			if (a < 0x01d01030)
				return;
			sound_sync();
			dcs_reset_w((v & 0x100) != 0);
			dcs_data_w(v & 0xff);
			return;
		}
	}
	if (a >= 0x01d01020 && a < 0x01d01040) {   // sound_w
		if (a < 0x01d01030)
			return;
		sound_sync();
		wms_snd_reset_write((~v & 0x100) != 0);
		wms_snd_write(v & 0xff);
		S.fake_sound_state = 128;
		return;
	}
	tms34010_unmapped();
}

// tms34010_config.pure_read: the reads of bus_read / wu_bus_read without an effect (the sound board, the PIC, the
// protection and mkturbo's random number are not)
static int tu_pure_read(uint32_t a)
{
	uint32_t off;
	if (a < 0x00400000 || (a >= 0x01400000 && a < 0x01420000) || (a >= 0x01600000 && a < 0x01600040) ||
		(a >= 0x01800000 && a < 0x01880000) || (a >= 0x01a80000 && a < 0x01a80100))
		return 1;
	if (prot_range(a, &off) || (a >= 0x01d00000 && a < 0x01d01040))
		return 0;
	return !(s_mkturbo && a >= 0xfffff400 && a < 0xfffff410);
}
static int wu_pure_read(uint32_t a)
{
	if (a < 0x00400000 || (a >= 0x01400000 && a < 0x01460000))
		return 1;
	if (a >= 0x01600000 && a < 0x01680020)
		return 0;
	if (a >= 0x01800000 && a < 0x01880000)
		return S.ioshuffle[((a - 0x01800000) >> 4) % 16] != 4;
	return 1;
}

static void umk3_palette_hack(void) { tms34010_eat(100); }   // (umk3_palette_hack_w after the RAM write)

static uint16_t wu_bus_read(uint32_t a)
{
	if (a < 0x00400000)
		return vram_r(a >> 4);
	if (a >= 0x01400000 && a < 0x01460000)
		return s_cmos[(a - 0x01400000) >> 4];
	if (a >= 0x01600000 && a < 0x01600020) {   // security_r
		if (s_picemu) {
			pe_sync();
			return s_pe.data_out;
		}
		S.pic_status = 1;
		return S.pic_buff;
	}
	if (a >= 0x01680000 && a < 0x01680020) {   // sound_r
		int empty;
		sound_sync();
		uint16_t d = dcs_data_r(&empty);
		if (empty)
			tms34010_eat(4444);
		return d & 0xff;
	}
	if (a >= 0x01800000 && a < 0x01880000) {   // io_r
		uint32_t offset = S.ioshuffle[((a - 0x01800000) >> 4) % 16];
		if (offset < 4) {
			int p = s_port[offset];
			return p >= 0 ? (uint16_t)rbx_port_read(p) : 0xffff;
		}
		if (offset == 4) {
			sound_sync();
			if (s_picemu)
				pe_sync();
			return (uint16_t)(((s_picemu ? s_pe.status : S.pic_status) << 12) | dcs_control_r());
		}
		return 0xffff;
	}
	if (a >= 0x01880000 && a < 0x01900000)
		return s_palram[(a - 0x01880000) >> 4];
	if ((a & ~0x00080000u) >= 0x01a00000 && (a & ~0x00080000u) < 0x01a00100) {
		uint32_t o = ((a & ~0x00080000u) - 0x01a00000) >> 4;
		if (o == 0)
			o = 1;
		return S.dma_register[o];
	}
	if (a >= 0x01b00000 && a < 0x01b00020)
		return S.control;
	if (a >= 0x02000000 && a < 0x07000000) {   // midwunit_gfxrom_r
		uint32_t offset = S.gfxbank_offset[0] + ((a - 0x02000000) >> 4) * 2;
		return offset + 1 < s_gfx_size ? (uint16_t)(s_gfx[offset] | (s_gfx[offset + 1] << 8)) : 0;
	}
	if (a >= 0xff800000)
		return s_maincpu[(a & 0x007fffff) >> 4];
	return 0;
}

static void wu_bus_write(uint32_t a, uint16_t v)
{
	if (a < 0x00400000) { vram_w(a >> 4, v); return; }
	if (a >= 0x01000000 && a < 0x01100000) {   // the main RAM's first page unmapped for writes on umk3
		s_ram[(a - 0x01000000) >> 4] = v;
		if (a >= 0x0106a060 && a < 0x0106a0a0)   // umk3_palette_hack_w
			tms34010_eat(100);
		return;
	}
	if (a >= 0x01400000 && a < 0x01460000) {
		if (S.cmos_write_enable) {
			s_cmos[(a - 0x01400000) >> 4] = v;
			S.cmos_write_enable = 0;
		}
		return;
	}
	if (a >= 0x01480000 && a < 0x01500000) { S.cmos_write_enable = 1; return; }
	if (a >= 0x01600000 && a < 0x01600020) {   // security_w
		if (a < 0x01600010) {
			if (s_picemu) {
				pe_sync();
				s_pe.command = v & 0x0f;
				s_pe.clk = (v >> 4) & 1;
			} else
				pic_write((uint8_t)v);
		}
		return;
	}
	if (a >= 0x01680000 && a < 0x01680020) {   // sound_w
		if (a < 0x01680010) {
			sound_sync();
			dcs_data_w(v & 0xff);
		}
		return;
	}
	if (a >= 0x01800000 && a < 0x01880000) {   // io_w
		if (s_wwfmania && a < 0x01800010) {
			wwfmania_io_0_w(v);
			return;
		}
		uint32_t offset = S.ioshuffle[((a - 0x01800000) >> 4) % 16] % 8;
		if (offset == 1) {
			// bit 4 reset sound CPU
			sound_sync();
			dcs_reset_w((~v & 0x10) != 0);
			// bit 5 (active low) reset security chip
			if (s_picemu) {
				if (!(v & 0x20)) {   // reset_w: reset while 0
					pe_sync();
					pic16c5x_reset(&s_pic);
				}
			} else if (v & 0x20) {
				S.pic_idx = 0;
				S.pic_status = 0;
				S.pic_buff = 0;
			}
		}
		S.iodata[offset] = v;
		return;
	}
	if (a >= 0x01880000 && a < 0x01900000) { paletteram_w((a - 0x01880000) >> 4, v); return; }
	if ((a & ~0x00080000u) >= 0x01a00000 && (a & ~0x00080000u) < 0x01a00100) {
		dma_w(((a & ~0x00080000u) - 0x01a00000) >> 4, v);
		return;
	}
	if (a >= 0x01b00000 && a < 0x01b00020) { wu_control_w(v); return; }
	tms34010_unmapped();
}

static void to_shiftreg(uint32_t address, uint16_t *shiftreg)
{
	dma_flush();
	uint32_t i = (address >> 3) & 0x7ffff;
	if (i > 0x80000 - 1024) i = 0x80000 - 1024;
	memcpy(shiftreg, &s_vram[i], 2 * 512 * sizeof(uint16_t));
}
static void from_shiftreg(uint32_t address, uint16_t *shiftreg)
{
	dma_flush();
	uint32_t i = (address >> 3) & 0x7ffff;
	if (i > 0x80000 - 1024) i = 0x80000 - 1024;
	memcpy(&s_vram[i], shiftreg, 2 * 512 * sizeof(uint16_t));
}

static void scanline_update(int scanline, const tms34010_display_params *params, uint16_t *dest)
{
	(void)scanline;
	dma_flush_row((unsigned)params->rowaddr & YPOSMASK);
	uint16_t const *const src = &s_vram[(params->rowaddr << 9) & 0x3fe00];
	int coladdr = params->coladdr << 1;
	int hs = params->hsblnk > 2048 ? 2048 : params->hsblnk;
	// (the runs up to the row's end: coladdr wraps at 512)
	for (int x = params->heblnk; x < hs;) {
		const int c = coladdr & 0x1ff;
		const int n = 512 - c < hs - x ? 512 - c : hs - x;
		const uint16_t *s = src + c;
		uint16_t *d = dest + x;
		for (int i = 0; i < n; i++)
			d[i] = s[i] & 0x7fff;
		x += n;
		coladdr += n;
	}
}

// ---- machine ----
static int tu_supported(const char *m) { return !strncmp(m, "tunit_adpcm", 11) || !strncmp(m, "mk2:", 4); }
static int wu_supported(const char *m) { return !strncmp(m, "wunit_picsim:", 13) || !strncmp(m, "wunit_picemu:", 13); }
static void tu_exit(void);
static void tu_reset(void);

static int tu_init(const rbx_game *g)
{
	const char *init = strchr(g->machine, ':');
	init = init ? init + 1 : "";
	memset(&S, 0, sizeof(S));
	s_nv_path[0] = 0;
	s_mkturbo = !strcmp(init, "mkturbo");
	s_wolf = !strncmp(g->machine, "wunit", 5);
	s_wwfmania = s_wolf && !strcmp(init, "wwfmania");
	s_picemu = !strncmp(g->machine, "wunit_picemu", 12);
	s_umk3 = s_wolf && (!strcmp(init, "umk3") || !strcmp(init, "umk3r11"));
	memset(&s_pe, 0, sizeof(s_pe));
	s_dcs = s_wolf || !strncmp(g->machine, "mk2:", 4);
	s_cmos_words = s_wolf ? 0x6000 : 0x2000;
	s_year = g->year ? atoi(g->year) : 1995;
	S.gfxbank_offset[1] = 0x400000;
	s_prot = s_dcs ? 0 : !strncmp(init, "mk", 2) ? PROT_MK : !strcmp(init, "nbajam") ? PROT_NBAJAM :
	         !strcmp(init, "nbajamte") ? PROT_NBAJAMTE : !strcmp(init, "jdreddp") ? PROT_JDREDD : 0;
	s_ram = (uint16_t *)calloc(0x40000, 2);
	s_cmos = (uint16_t *)calloc(s_cmos_words, 2);
	s_palram = (uint16_t *)calloc(0x8000, 2);
	s_vram = (uint16_t *)calloc(0x80000, 2);
	s_pal = (uint32_t *)calloc(NPENS + 1, 4);
	s_fb = (uint16_t *)calloc(512 * 512, 2);
	s_dq = (dma_job_t *)calloc(DQ_N, sizeof(dma_job_t));
	s_dq_head = s_dq_tail = 0;
	if (!s_ram || !s_cmos || !s_palram || !s_vram || !s_pal || !s_fb || !s_dq) {
		rbx_fail("%s: out of memory", g->set);
		tu_exit();
		return -1;
	}
	uint32_t msz, csz, osz;
	s_maincpu = (uint16_t *)rbx_region("maincpu", &msz);
	s_gfx = rbx_region("video", &s_gfx_size);
	if (s_picemu) {
		uint32_t psz;
		uint8_t *picrom = rbx_region("serial_security:pic", &psz);
		if (!picrom || psz < 0x1000) {
			rbx_fail("%s: no security PIC", g->set);
			tu_exit();
			return -1;
		}
		pic16c5x_init(&s_pic, PIC16C57, picrom, psz, NULL);
		s_pic.read_port = pe_read_port;
		s_pic.write_port = pe_write_port;
	}
	if (s_dcs) {
		uint8_t *dcsrom = rbx_region("dcs", &csz);
		if (!s_maincpu || msz < 0x100000 || !s_gfx || !dcsrom || dcs_init(s_wolf ? DCS_8K : DCS_2K, dcsrom, csz)) {
			tu_exit();
			return -1;
		}
		dcs_set_async(1);   // (the board on the worker thread beside the TMS34010)
	} else {
	uint8_t *snd = rbx_region("adpcm:cpu", &csz);
	uint8_t *oki = rbx_region("adpcm:oki", &osz);
	if (!s_maincpu || msz < 0x100000 || !s_gfx || !snd || !oki) {
		tu_exit();
		return -1;
	}
	if (wms_snd_init(WMS_SND_ADPCM, snd, csz, oki, osz)) {
		tu_exit();
		return -1;
	}
	}
	if (s_prot == PROT_MK) wms_snd_hidden_ram(0xfb9c, 0xfbc6);
	else if (s_prot == PROT_NBAJAM) wms_snd_hidden_ram(0xfbaa, 0xfbd4);
	else if (s_prot == PROT_NBAJAMTE) wms_snd_hidden_ram(0xfbec, 0xfc16);
	else if (s_prot == PROT_JDREDD) wms_snd_hidden_ram(0xfbcf, 0xfbf9);

	static const char *const tags[2][4] = { { "IN0", "IN1", "IN2", "DSW" }, { "IN0", "IN1", "DSW", "IN2" } };
	for (int i = 0; i < 4; i++)
		s_port[i] = rbx_port_find(tags[s_wolf][i]);

	tms34010_config tc;
	memset(&tc, 0, sizeof(tc));
	tc.clock = CLOCK;
	tc.pixclock = s_wolf ? 8000000 : 8000000 / 2;
	tc.pixperclock = s_wolf ? 1 : 2;
	tc.htotal = 506; tc.hbend = s_wolf ? 101 : 100; tc.hbstart = s_wolf ? 501 : 500;
	tc.vtotal = 289; tc.vbend = 20; tc.vbstart = 274;
	tc.read = s_wolf ? wu_bus_read : bus_read;
	tc.write = s_wolf ? wu_bus_write : bus_write;
	tc.pure_read = s_wolf ? wu_pure_read : tu_pure_read;
	tc.to_shiftreg = to_shiftreg;
	tc.from_shiftreg = from_shiftreg;
	tc.scanline = scanline_update;
	tc.black_pen = NPENS;
	tc.bitmap = s_fb;
	tc.pitch = 512;
	tms34010_init(&tc);
	tms34010_map(0x01000000, 0x013fffff, s_ram, s_ram);
	if (s_umk3)   // init_umk3: a write handler over 0x0106a060-0x0106a09f (a tap on the direct page; RBX_UMK3_NOTAP=1:
		              // the page through wu_bus_write as before)
		if (getenv("RBX_UMK3_NOTAP"))
			tms34010_map(0x01000000, 0x010fffff, s_ram, NULL);
		else
			tms34010_write_watch(0x0106a060, 0x0106a09f, umk3_palette_hack);
	if (!s_wolf)
		tms34010_map(0x1f800000, 0x1fffffff, s_maincpu, NULL);
	tms34010_map(0xff800000, s_mkturbo ? 0xffefffff : 0xffffffff, s_maincpu, NULL);
	if (!s_wolf)
		tms34010_code_rom(0x1f800000, 0x1fffffff);
	tms34010_code_rom(0xff800000, s_mkturbo ? 0xffefffff : 0xffffffff);
	if (rbx_audio_setup(s_dcs ? dcs_rate() : wms_snd_rate()) || rbx_video_setup(512, 400, NPENS + 1)) {
		tu_exit();
		return -1;
	}
	{
		extern const char *rbx_nvram_dir(void);
		const char *dir = rbx_nvram_dir();
		if (dir) {
			snprintf(s_nv_path, sizeof(s_nv_path), "%s%s.nv", dir, g->set);
			FILE *f = fopen(s_nv_path, "rb");
			if (f) {
				if (fread(s_cmos, 2, s_cmos_words, f) != s_cmos_words)
					memset(s_cmos, 0, s_cmos_words * 2);
				fclose(f);
			}
		}
	}
	s_dq_on = !(getenv("RBX_TU_NODMATHREAD") && atoi(getenv("RBX_TU_NODMATHREAD")));
	if (s_dq_on) {
		static int stat;
		t_emu = 1;
		if (!stat && getenv("RBX_TU_DMASTAT") && atoi(getenv("RBX_TU_DMASTAT"))) {
			stat = 1;
			atexit(dma_stat);
		}
		rbx_aux2_set_job(dma_job);
		rbx_aux2_set_alone(1);   // (CPU3 on the Vita when it is open: CPU1 runs the sound board)
		// (the worker looks for the next blit this long before it sleeps: asleep, the emulation thread finds the queue
		// undrawn at its next sync and draws it itself; RBX_TU_DMASPIN=us)
		rbx_aux2_set_spin(getenv("RBX_TU_DMASPIN") ? atoi(getenv("RBX_TU_DMASPIN")) : 500);
	}
	tu_reset();
	return 0;
}

static void tu_exit(void)
{
	if (s_dq_on) {
		rbx_aux2_set_job(NULL);   // (syncs first: the queued blits drawn)
		s_dq_on = 0;
	}
	if (s_nv_path[0] && s_cmos) {
		FILE *f = fopen(s_nv_path, "wb");
		if (f) {
			fwrite(s_cmos, 2, s_cmos_words, f);
			fclose(f);
		}
	}
	if (s_dcs)
		dcs_exit();
	else
		wms_snd_exit();
	free(s_ram); s_ram = NULL;
	free(s_cmos); s_cmos = NULL;
	free(s_palram); s_palram = NULL;
	free(s_vram); s_vram = NULL;
	free(s_pal); s_pal = NULL;
	free(s_fb); s_fb = NULL;
	free(s_dq); s_dq = NULL;
	tms34010_release();   // (the JIT's block table, ADR-200)
	s_nv_path[0] = 0;
}

static void tu_reset(void)
{
	dma_flush();
	S.dma_pending = 0;
	S.ticks = S.cyc_frac = S.cyc_target = 0;
	tms34010_reset();
	if (s_dcs) {
		dcs_reset();
		dcs_reset_w(0);
		dcs_reset_w(1);
		if (s_wolf) {
			wu_reset_shuffle();
			if (s_picemu) {   // the PIC16C57 device_reset
				pic16c5x_reset(&s_pic);
				s_pe.t = 0;
			} else {
			// the serial PIC's device_reset (MAME's random numbers from the start)
			S.rand_seed = 0x9d14abd7u;
			pic_generate_serial_data(528);
			}
		}
	} else {
		wms_snd_reset();
		wms_snd_reset_write(1);
		wms_snd_reset_write(0);
	}
	memset(s_fb, 0, 512 * 512 * 2);
}

static void run_main(uint64_t target)
{
	for (;;) {
		uint64_t now = tms34010_total_cycles();
		if (S.dma_pending && S.dma_at <= now) {
			dma_done();
			continue;
		}
		if (now >= target)
			break;
		uint64_t lim = target;
		if (S.dma_pending && S.dma_at < lim)
			lim = S.dma_at;
		tms34010_execute((int)(lim - now));
	}
}

static void tu_frame(void)
{
	tms34010_screen_t *scr = &tms34010_screen;
	do {
		int len = tms34010_line_start();
		S.line_tick0 = S.ticks;
		S.line_cyc0 = S.cyc_target;
		S.ticks += (uint64_t)len;
		S.cyc_frac += (uint64_t)len * CLOCK;
		uint64_t n = S.cyc_frac / 64000000u;
		S.cyc_frac -= n * 64000000u;
		S.cyc_target += n;
		run_main(S.cyc_target);
		if (s_dcs)
			dcs_run(S.ticks * 5 / 4);
		else
			wms_snd_run(S.ticks);
	} while (!tms34010_line_end());
	// (the PIC is brought up to the host's time at its accesses and at the frame's end only: its inputs change only
	// at the accesses and pic16c5x_execute carries the overrun, so it runs the same instructions as line by line)
	if (s_picemu)
		pe_sync();
	rbx_audio.frame_mhz = (uint32_t)(8000000ull * 1000u / ((uint64_t)scr->width * (uint64_t)scr->height));

	rbx_video_wait_idle();
	int w = scr->max_x - scr->min_x + 1, h = scr->max_y - scr->min_y + 1;
	if (w > 512) w = 512;
	if (h > 400) h = 400;
	rbx_video.width = w;
	rbx_video.height = h;
	for (int y = 0; y < h; y++)
		memcpy(rbx_video.bitmap + (size_t)y * rbx_video.pitch, s_fb + (size_t)y * 512, (size_t)w * 2);
	memcpy(rbx_video.palette, s_pal, (size_t)(NPENS + 1) * 4);
	rbx_video.palette_used = NPENS + 1;
	rbx_video_frame_done();
	// the board's samples of the frame done: last, so that the sound worker's lag overlaps the wait for the render
	// thread and the copy above (they do not touch the sound board; its queue holds the frame's runs already)
	if (s_dcs)
		dcs_frame_end();
	else
		wms_snd_frame_end();
}

static int tu_state(void *ctx, int load)
{
	dma_flush();
	if (rbx_state_io(ctx, "tuS", &S, sizeof(S)) || rbx_state_io(ctx, "turam", s_ram, 0x40000 * 2) ||
		rbx_state_io(ctx, "tucmos", s_cmos, s_cmos_words * 2) || rbx_state_io(ctx, "tupal", s_palram, 0x8000 * 2) ||
		rbx_state_io(ctx, "tuvram", s_vram, 0x80000 * 2) || tms34010_state(ctx, load) ||
		(s_dcs ? dcs_state(ctx, load) : wms_snd_state(ctx, load)) ||
		(s_picemu && (rbx_state_io(ctx, "tupic", &s_pic, pic16c5x_state_size()) || rbx_state_io(ctx, "tupe", &s_pe, sizeof(s_pe)))))
		return -1;
	if (load)
		for (uint32_t i = 0; i < 0x8000; i++)
			paletteram_w(i, s_palram[i]);
	return 0;
}
static int tu_save(void *ctx) { return tu_state(ctx, 0); }
static int tu_load(void *ctx) { return tu_state(ctx, 1); }

const rbx_driver rbx_drv_midtunit = { "Midway T-Unit", tu_init, tu_exit, tu_reset, tu_frame, tu_save, tu_load,
                                      tu_supported, NULL, NULL };
const rbx_driver rbx_drv_midwunit = { "Midway Wolf Unit", tu_init, tu_exit, tu_reset, tu_frame, tu_save, tu_load,
                                      wu_supported, NULL, NULL };
