/*  Halley's Comet / Ben Bero Beh hardware, ported from MAME src/mame/taito/halleys.cpp
    (BSD-3-Clause, copyright-holders: Phil Stroffolino, Acho A. Tang).
    MC6809E main CPU (19.968 MHz / 12) through cpu/m6809, Z80 sound CPU (3 MHz) with four YM2149 (1.5 MHz, three on
    the sound map; the fourth one is not mapped in MAME either). The MB1551x blitter, its six 256x256 render layers,
    the palette tricks, the scanline interrupt scheme (FIRQ at lines 56 / 112 with the vector swap, coin NMI at 168),
    the program ROM decryption and MAME's collision / SRAM test hacks are carried over from MAME as they are.
    The frame is composed at vblank from the render layers (MAME screen_update) into the snapshot; render() only
    copies it out.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m6809/m6809.h"
#include "../../cpu/z80/z80.h"
#include "../../sound/ay8910.h"

#include <stdlib.h>
#include <string.h>

#define GAME_BENBEROB 0
#define GAME_HALLEYS  1

#define SCREEN_WIDTH_L2 8
#define SCREEN_HEIGHT 256
#define VIS_MINX 0
#define VIS_MAXX 255
#define VIS_MINY 8
#define VIS_MAXY (255 - 8)
#define MAX_LAYERS 6
#define MAX_SPRITES 256

#define SP_2BACK 0x100
#define SP_ALPHA 0x200
#define SP_COLLD 0x300
#define BG_MONO  0x400
#define BG_RGB   0x500
#define PALETTE_SIZE 0x600
#define BLACK_PEN PALETTE_SIZE   // MAME palette black_pen()

#define SCREEN_WIDTH (1 << SCREEN_WIDTH_L2)
#define SCREEN_SIZE (SCREEN_HEIGHT << SCREEN_WIDTH_L2)
#define SCREEN_BYTEWIDTH_L2 (SCREEN_WIDTH_L2 + 1)
#define SCREEN_BYTEWIDTH (1 << SCREEN_BYTEWIDTH_L2)
#define CLIP_SKIP (VIS_MINY * SCREEN_WIDTH + VIS_MINX)
#define CLIP_W (VIS_MAXX - VIS_MINX + 1)
#define CLIP_H (VIS_MAXY - VIS_MINY + 1)

#define LINES 256
#define MAIN_CLOCK 1664000u     // 19.968 MHz / 12
#define AUDIO_CLOCK 3000000u
#define RATE_MHZ 59500u         // 59.50 Hz
#define SND_IRQ_PERIOD 81920    // 6 MHz / (4*16*16*10*16) in 3 MHz Z80 clocks
#define GFX_PAD 0x20000         // reads past the unpacked planes (MAME reads out of bounds there)

typedef struct {
	z80 z;
	int irq;
} zcpu;

static int s_game_id;
static uint16_t s_collision_detection;
static uint8_t *s_cpu1_base;        // decrypted program region (also read by the blitter and the vector fetch)
static uint8_t *s_audio_rom;
static uint32_t s_audio_size;
static const uint8_t *s_prom;
static uint8_t *s_gfx_alloc, *s_gfx_plane02, *s_gfx_plane13;
static uint8_t *s_gfx1_base;
static uint16_t *s_layer_alloc, *s_render_layer[MAX_LAYERS];
static uint32_t s_internal_palette[PALETTE_SIZE];
static uint32_t s_rgb[PALETTE_SIZE + 1];
static uint8_t s_paletteram[0x20];
static uint8_t s_blitter_ram[0x1000 + 0x20];
static uint8_t s_io_ram[0xc0];
static uint8_t s_work_ram[0xf00];
static uint8_t s_audio_ram[0x800];
static uint8_t s_collision_list[MAX_SPRITES];

typedef struct {
	int32_t blitter_busy, collision_count, stars_enabled, mVectorType, sndnmi_mask, firq_level;
	int64_t blitter_reset_at;
	uint8_t soundlatch;
	uint8_t pad[7];
	uint64_t main_frac, audio_frac;
	int64_t main_target;
	int32_t snd_irq_cnt;
	int32_t pad2;
} misc_t;
static misc_t s_st;

static m6809_t s_main;
static zcpu s_audio;
static ay8910_t s_ay[3];   // ay2, ay3, ay4 (ay1 is not mapped)
static int32_t *s_mix;
static int s_mix_cap, s_mix_n;
static uint32_t s_ay_done;
static unsigned long s_audio_frame_start;

static uint16_t *s_snap;   // composed frame, CLIP_W x CLIP_H
static uint32_t s_snap_pal[PALETTE_SIZE + 1];

static int s_port_in[4], s_port_dsw[4];

// ---- sound ----
static void ay_run_to(uint32_t pos)
{
	if (pos <= s_ay_done)
		return;
	uint32_t n = pos - s_ay_done;
	int got = 0;
	for (int i = 0; i < 3; i++)
		got = ay8910_render(&s_ay[i], n, s_mix + s_mix_n, s_mix_cap - s_mix_n);
	s_mix_n += got;
	s_ay_done = pos;
}

static void ay_sync(void) { ay_run_to((uint32_t)((s_audio.z.cyc - s_audio_frame_start) / 2)); }

static void sndnmi_msk_w(void *u, uint8_t data) { (void)u; s_st.sndnmi_mask = data & 1; }

static uint8_t audio_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x4000) return a < s_audio_size ? s_audio_rom[a] : 0;
	if (a < 0x4800) return s_audio_ram[a - 0x4000];
	if (a == 0x4801 || a == 0x4803 || a == 0x4805) { ay_sync(); return ay8910_data_r(&s_ay[(a - 0x4800) >> 1]); }
	if (a == 0x5000) return s_st.soundlatch;
	if (a >= 0xe000 && a < 0xf000) return a < s_audio_size ? s_audio_rom[a] : 0;
	return 0;
}

static void audio_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0x4000 && a < 0x4800) { s_audio_ram[a - 0x4000] = v; return; }
	if (a >= 0x4800 && a <= 0x4805) { ay_sync(); ay8910_write(&s_ay[(a - 0x4800) >> 1], a & 1, v); }
}

static uint8_t no_in(z80 *z, uint8_t p) { (void)z; (void)p; return 0xff; }
static void no_out(z80 *z, uint8_t p, uint8_t v) { (void)z; (void)p; (void)v; }

static void audio_cpu_reset(void)
{
	unsigned long cyc = s_audio.z.cyc;
	z80_init(&s_audio.z);
	s_audio.z.read_byte = audio_read;
	s_audio.z.write_byte = audio_write;
	s_audio.z.port_in = no_in;
	s_audio.z.port_out = no_out;
	s_audio.z.int_data = 0xff;
	s_audio.z.cyc = cyc;
	s_audio.irq = 0;
}

// ---- MB1551x blitter (MAME halleys_state::blit) ----
#define ACTIVE 0x02
#define IGNORE_0 0x01
#define COLOR_ON 0x04
#define MIRROR_X 0x08
#define MIRROR_Y 0x10
#define GROUP    0x80
#define PENCOLOR 0x0f
#define BANKBIT1 0x80
#define COMMAND  0x0f
#define BGLAYER  0x10
#define BANKBIT0 0x40
#define PLANE    0x80
#define EFX1      0x8
#define HORIZBAR  0xe
#define STARPASS1 0x3
#define STARPASS2 0xd
#define TILEPASS1 0x5
#define FLIP_X     0x0100
#define FLIP_Y     0x0200
#define SINGLE_PEN 0x0400
#define RGB_MASK   0x0800
#define AD_HIGH    0x1000
#define SY_HIGH    0x2000
#define S1_IDLE    0x4000
#define S1_REV     0x8000
#define S2_IDLE    0x10000
#define S2_REV     0x20000
#define BACKMODE   0x40000
#define PPCD_ON    0x100000
#define HALLEYS_SPLIT 0xf4
#define XMASK (SCREEN_WIDTH - 1)
#define YMASK (SCREEN_HEIGHT - 1)
#define GFX_HI 0x10000

static void blit(int offset)
{
	static const uint8_t penxlat[16] = { 0x03,0x07,0x0b,0x0f,0x02,0x06,0x0a,0x0e,0x01,0x05,0x09,0x0d,0x00,0x04,0x08,0x0c };
	static const uint8_t rgbmask[16] = { 0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xfc,0xff,0xff,0xff,0xff,0xff };
	static const uint8_t tyremap[8] = { 0, 5, 9, 13, 16, 20, 24, 28 };

	uint8_t *param = s_blitter_ram + offset, *src_base;
	uint16_t *dst_base;
	int stptr, mode, color, code, y, x, h, w, src1, src2;
	int status, flags, group, command, bank, layer, pen0, pen1;
	int yclip, xclip, src_yskip, src_xskip, dst_skip, hclip, wclip, src_dy, src_dx;
	uint32_t *pal_ptr;
	const uint8_t *src1_ptr, *src2_ptr;
	uint16_t *dst_ptr, *edi;
	int eax, ebx, ecx, edx;
	uint16_t ax;
	uint8_t al, ah;

	stptr = (int)param[0x2] << 8 | (int)param[0x3];
	mode = (int)param[0x5];
	color = (int)param[0x6];
	code = (int)param[0x7];

	layer = (code >> 3 & 2) | (code >> 7 & 1);
	offset >>= 4;
	status = stptr ? s_cpu1_base[stptr] : ACTIVE;
	flags = mode;
	group = mode & GROUP;
	command = code & COMMAND;
	if (s_game_id == GAME_HALLEYS) {
		if (!layer) flags |= PPCD_ON;
		if (offset >= HALLEYS_SPLIT) flags |= AD_HIGH;
		else if (offset & 1) memcpy(param - 0x10, param, 0x10);
		else group = param[0x15] & GROUP;
		// HACK: force engine flame in group zero to increase collision accuracy
		if (offset == 0x1a || offset == 0x1b) group = 0;
	} else if (offset & 1)
		memcpy(param - 0x10, param, 0x10);

	y = (int)param[0x8];
	x = (int)param[0x9];
	h = (int)param[0xa];
	w = (int)param[0xb] + 1;
	if (!h) return;

	src1 = (int)param[0xc] << 8 | (int)param[0xd];
	src2 = (int)param[0xe] << 8 | (int)param[0xf];
	flags |= src1 & (S1_IDLE | S1_REV);
	flags |= src2 << 2 & (S2_IDLE | S2_REV);
	src1 &= 0x3fff;
	src2 &= 0x3fff;
	bank = ((code & BANKBIT0) | (color & BANKBIT1)) << 8;
	pal_ptr = s_internal_palette;

	if (!(code & BGLAYER || command & 7)) {
		if (flags & MIRROR_Y) { flags |= FLIP_Y; y -= (h - 1); }
		if (flags & MIRROR_X) { flags |= FLIP_X; x -= (w - 1); }
		if (y > VIS_MAXY || (y + h) <= VIS_MINY) return;
		if (x > VIS_MAXX || (x + w) <= VIS_MINX) return;

		yclip = y; xclip = x; hclip = h; wclip = w;
		src_yskip = src_xskip = 0;
		if (yclip < VIS_MINY) { src_yskip = VIS_MINY - yclip; yclip = VIS_MINY; hclip -= src_yskip; }
		if (yclip + hclip > VIS_MAXY + 1) { hclip = VIS_MAXY + 1 - yclip; }
		if (xclip < VIS_MINX) { src_xskip = VIS_MINX - xclip; xclip = VIS_MINX; wclip -= src_xskip; }
		if (xclip + wclip > VIS_MAXX + 1) { wclip = VIS_MAXX + 1 - xclip; }
		dst_skip = (yclip << SCREEN_WIDTH_L2) + xclip;

		eax = 0;
		if (flags & (S1_REV | S2_REV)) { flags ^= FLIP_Y | FLIP_X; eax -= w * h - 8; }
		if (flags & FLIP_Y) {
			eax += w * (h - 1);
			src_yskip = -src_yskip;
			src_dy = (flags & FLIP_X) ? -w + wclip : -w - wclip;
		} else
			src_dy = (flags & FLIP_X) ? w + wclip : w - wclip;
		if (flags & FLIP_X) {
			eax += w - 1;
			src_xskip = -src_xskip;
			src_dx = -1;
		} else
			src_dx = 1;

		src1_ptr = s_gfx_plane02 + ((bank + src1) << 3) + eax;
		src2_ptr = s_gfx_plane13 + ((bank + src2) << 3) + eax;
		if (!(flags & (S1_IDLE | S2_IDLE))) {
			eax = src_yskip * w + src_xskip;
			src1_ptr += eax;
			src2_ptr += eax;
		} else
			src_dy = src_dx = 0;

		dst_ptr = s_render_layer[layer] + dst_skip;

		pen0 = code >> 3 & 0x10;
		pen1 = 0;
		if (command == EFX1) { flags |= BACKMODE; pen0 |= SP_2BACK; }
		if (src1 == src2) {
			flags |= SINGLE_PEN;
			eax = (int)penxlat[color & PENCOLOR];
			if (eax) pen1 = pen0 + eax;
		} else if (color & PENCOLOR)
			flags |= RGB_MASK;

#define BLOCK_WIPE_COMMON { \
	uint16_t *d_ = dst_ptr; \
	ecx = hclip; \
	do { memset(d_, 0, (size_t)wclip * 2); d_ += SCREEN_WIDTH; } while (--ecx); \
}

		if ((flags & (SINGLE_PEN | RGB_MASK | COLOR_ON)) == COLOR_ON) {
			// multi-pen block or transparent blit
			if (!(flags & IGNORE_0)) BLOCK_WIPE_COMMON
			dst_ptr += wclip;
			ecx = wclip = -wclip;
			edx = src_dx;
			if (!(flags & PPCD_ON)) {
				al = ah = (uint8_t)pen0;
				if (!(flags & BACKMODE)) {
					do {
						do {
							al |= *src1_ptr; src1_ptr += edx;
							al |= *src2_ptr; src2_ptr += edx;
							if (al & 0xf) { dst_ptr[ecx] = (uint16_t)al; al = ah; }
						} while (++ecx);
						ecx = wclip; src1_ptr += src_dy; src2_ptr += src_dy; dst_ptr += SCREEN_WIDTH;
					} while (--hclip);
				} else {
					do {
						do {
							al |= *src1_ptr; src1_ptr += edx;
							al |= *src2_ptr; src2_ptr += edx;
							if (al & 0xf) { dst_ptr[ecx] = (uint16_t)al | SP_2BACK; al = ah; }
						} while (++ecx);
						ecx = wclip; src1_ptr += src_dy; src2_ptr += src_dy; dst_ptr += SCREEN_WIDTH;
					} while (--hclip);
				}
				return;
			}
			ax = 0;
			if (group) {
				do {
					do {
						al = *src1_ptr; src1_ptr += edx;
						al |= *src2_ptr; src2_ptr += edx;
						if (al & 0xf) dst_ptr[ecx] = (uint16_t)al | SP_COLLD;   // collision flag on group one pixels
					} while (++ecx);
					ecx = wclip; src1_ptr += src_dy; src2_ptr += src_dy; dst_ptr += SCREEN_WIDTH;
				} while (--hclip);
			} else {
				do {
					do {
						al = *src1_ptr; src1_ptr += edx;
						al |= *src2_ptr; src2_ptr += edx;
						if (al & 0xf) { ax |= dst_ptr[ecx]; dst_ptr[ecx] = (uint16_t)al; }   // combine collision flags
					} while (++ecx);
					ecx = wclip; src1_ptr += src_dy; src2_ptr += src_dy; dst_ptr += SCREEN_WIDTH;
				} while (--hclip);
			}
			if (status & ACTIVE && ax & SP_COLLD) {
				s_collision_list[s_st.collision_count & (MAX_SPRITES - 1)] = (uint8_t)offset;
				s_st.collision_count++;
			}
		} else if ((flags & (RGB_MASK | COLOR_ON)) == RGB_MASK + COLOR_ON) {
			// multi-pen, RGB masked block or transparent blit
			if (!(flags & IGNORE_0)) BLOCK_WIPE_COMMON
			dst_ptr += wclip;
			ecx = wclip = -wclip;
			al = ah = (uint8_t)pen0;
			uint32_t m = rgbmask[color & PENCOLOR] | 0xffffff00u;
			do {
				do {
					al |= *src1_ptr; src1_ptr += src_dx;
					al |= *src2_ptr; src2_ptr += src_dx;
					if (al & 0xf) { edx = al; al = ah; dst_ptr[ecx] = (uint16_t)(pal_ptr[edx] & m); }
				} while (++ecx);
				ecx = wclip; src1_ptr += src_dy; src2_ptr += src_dy; dst_ptr += SCREEN_WIDTH;
			} while (--hclip);
		} else if ((flags & (SINGLE_PEN | COLOR_ON)) == SINGLE_PEN + COLOR_ON) {
			// single-pen block or transparent blit
			if (!(flags & IGNORE_0)) BLOCK_WIPE_COMMON
			dst_ptr += wclip;
			ebx = hclip;
			ecx = wclip = -wclip;
			edx = src_dx;
			ax = (uint16_t)pen1;
			do {
				do {
					if (*src1_ptr) dst_ptr[ecx] = ax;
					src1_ptr += edx;
				} while (++ecx);
				ecx = wclip; src1_ptr += src_dy; dst_ptr += SCREEN_WIDTH;
			} while (--ebx);
		} else if ((flags & (IGNORE_0 | COLOR_ON)) == IGNORE_0) {
			// transparent wipe
			dst_ptr += wclip;
			wclip = -wclip;
			ecx = wclip;
			edx = src_dx;
			if (flags & PPCD_ON && !group) {
				// preserve collision flags when wiping group zero objects
				do {
					do {
						al = *src1_ptr; ah = *src2_ptr;
						src1_ptr += edx; src2_ptr += edx;
						if (al | ah) dst_ptr[ecx] &= SP_COLLD;
					} while (++ecx);
					ecx = wclip; src1_ptr += src_dy; src2_ptr += src_dy; dst_ptr += SCREEN_WIDTH;
				} while (--hclip);
			} else {
				do {
					do {
						al = *src1_ptr; ah = *src2_ptr;
						src1_ptr += edx; src2_ptr += edx;
						if (al | ah) dst_ptr[ecx] = 0;
					} while (++ecx);
					ecx = wclip; src1_ptr += src_dy; src2_ptr += src_dy; dst_ptr += SCREEN_WIDTH;
				} while (--hclip);
			}
		} else if ((flags & (IGNORE_0 | COLOR_ON)) == 0)
			BLOCK_WIPE_COMMON   // block wipe
		return;
	}

	// reject illegal blits and adjust parameters
	if (command) {
		if (h > 8) return;
		if (y >= 0xf8) { y -= 0xf8; flags |= SY_HIGH; }
		else if (y >= 8) return;
		if (command != TILEPASS1) { h <<= 5; y <<= 5; }
	}

	src_dx = (flags & S1_IDLE) ? 0 : 1;
	if (flags & S1_REV) src_dx = -src_dx;
	src_base = s_gfx1_base + bank;
	if (command == STARPASS1 || command == STARPASS2) layer = (layer & 1) + 4;
	dst_base = s_render_layer[layer];

#define WARP_WIPE_COMMON { \
	ebx = y + h; \
	ecx = (x + w - SCREEN_WIDTH) << 1; \
	for (edx = y; edx < ebx; edx++) { \
		dst_ptr = dst_base + ((edx & YMASK) << SCREEN_WIDTH_L2); \
		if (ecx > 0) { memset(dst_ptr, 0, (size_t)ecx); eax = SCREEN_WIDTH - x; } else eax = w; \
		memset(dst_ptr + x, 0, (size_t)eax << 1); \
	} \
}

	if ((flags & (IGNORE_0 | COLOR_ON)) == 0) {
		// specialized block wipe
		if (!command && y == 0xff && h == 1) {
			// wipe the star layer
			y = 0; h = SCREEN_HEIGHT;
			dst_base = s_render_layer[(layer & 1) + 4];
			WARP_WIPE_COMMON
			s_st.stars_enabled = ~layer & 1;
		} else {
			// wipe background and chain-wipe the sprite layer when the command is zero
			WARP_WIPE_COMMON
			if (!command) { dst_base = s_render_layer[layer & 1]; WARP_WIPE_COMMON }
		}
	} else if (command == STARPASS1 && flags & COLOR_ON) {
		// gray shaded star map with 16x16 cells (MAME: speculative)
#define RORB(R, N) ((((R) >> (N)) | ((R) << (8 - (N)))) & 0xff)
#define C2S(X, Y, O) (((((Y) + ((O) >> 4)) & YMASK) << SCREEN_WIDTH_L2) + (((X) + ((O) & 0xf)) & XMASK))
		src2_ptr = src_base + src2 + GFX_HI;
		for (yclip = y + h; y < yclip; y += 16)
			for (xclip = x + w; x < xclip; x += 16) {
				edx = *src2_ptr;
				src2_ptr++;
				if (edx) {
					ax = (uint16_t)*(src2_ptr - 0x100 - 1);
					ebx = ax;
					ax |= BG_MONO;
					if (edx & 0x01) { dst_base[C2S(x, y, ebx)] = ax; }
					if (edx & 0x02) { ecx = RORB(ebx, 1); dst_base[C2S(x, y, ecx)] = ax; }
					if (edx & 0x04) { ecx = RORB(ebx, 2); dst_base[C2S(x, y, ecx)] = ax; }
					if (edx & 0x08) { ecx = RORB(ebx, 3); dst_base[C2S(x, y, ecx)] = ax; }
					if (edx & 0x10) { ecx = RORB(ebx, 4); dst_base[C2S(x, y, ecx)] = ax; }
					if (edx & 0x20) { ecx = RORB(ebx, 5); dst_base[C2S(x, y, ecx)] = ax; }
					if (edx & 0x40) { ecx = RORB(ebx, 6); dst_base[C2S(x, y, ecx)] = ax; }
					if (edx & 0x80) { ecx = RORB(ebx, 7); dst_base[C2S(x, y, ecx)] = ax; }
				}
			}
		s_st.stars_enabled = layer & 1;
#undef RORB
#undef C2S
	} else if (command == STARPASS2 && flags & COLOR_ON) {
		// stars color modifier
		edx = SCREEN_WIDTH - x - w;
		w = -w;
		ax = (uint16_t)(~src_base[src2] & 0xff);
		for (yclip = y + h; y < yclip; y++) {
			edi = dst_base + ((y & YMASK) << SCREEN_WIDTH_L2);
			if (edx < 0) {
				ecx = edx;
				dst_ptr = edi - edx;
				do { if (dst_ptr[ecx]) dst_ptr[ecx] ^= ax; } while (++ecx);
				ecx = x - SCREEN_WIDTH;
			} else
				ecx = w;
			dst_ptr = edi + x - ecx;
			do { if (dst_ptr[ecx]) dst_ptr[ecx] ^= ax; } while (++ecx);
		}
	} else if (command == TILEPASS1 && flags & COLOR_ON) {
		// RGB 8x8 bit-packed transparent tile (MAME: positioning not understood)
		if (y & 1) x -= 8;
		y = tyremap[y] << 3;
		if (flags & SY_HIGH) y += 8;
		if (y > 0xf8) return;
		if (code & PLANE) return;   // unemulated in MAME
		if (flags & IGNORE_0) { w = 8; h = 8; WARP_WIPE_COMMON }
		src1_ptr = src_base + src1;
		dst_ptr = s_render_layer[2] + (y << SCREEN_WIDTH_L2);
		src1_ptr += 8;
		edx = -8;
		if (x <= 0xf8) {
			dst_ptr += x;
			do {
				ax = (uint16_t)src1_ptr[edx];
				al = src1_ptr[edx + 0x10000];
				ax |= BG_RGB;
				if (al & 0x01) dst_ptr[0] = ax;
				if (al & 0x02) dst_ptr[1] = ax;
				if (al & 0x04) dst_ptr[2] = ax;
				if (al & 0x08) dst_ptr[3] = ax;
				if (al & 0x10) dst_ptr[4] = ax;
				if (al & 0x20) dst_ptr[5] = ax;
				if (al & 0x40) dst_ptr[6] = ax;
				if (al & 0x80) dst_ptr[7] = ax;
				dst_ptr += SCREEN_WIDTH;
			} while (++edx);
		} else {
#define WARPMASK ((SCREEN_WIDTH << 1) - 1)
			do {
				ecx = x & WARPMASK;
				ax = (uint16_t)src1_ptr[edx];
				al = src1_ptr[edx + 0x10000];
				ax |= BG_RGB;
				for (int b = 0; b < 8; b++) {
					if (al & (1 << b)) dst_ptr[ecx] = ax;
					ecx++; ecx &= WARPMASK;
				}
				dst_ptr += SCREEN_WIDTH;
			} while (++edx);
#undef WARPMASK
		}
	} else if (command == HORIZBAR && flags & COLOR_ON && !(layer & 1)) {
		// RGB horizontal bar (foreground only)
#define WARP_LINE_COMMON { \
	if (ecx & 1) { ecx--; *dst_ptr = (uint16_t)eax; dst_ptr++; } \
	dst_ptr += ecx; ecx = -ecx; \
	while (ecx) { dst_ptr[ecx] = (uint16_t)eax; dst_ptr[ecx + 1] = (uint16_t)eax; ecx += 2; } \
}
		const uint8_t *tmp;
		src1_ptr = src_base + src1;
		src2_ptr = src_base + src2;
		hclip = y + h;
		if (!(flags & S2_IDLE)) {
			// double source mode
			src2_ptr += GFX_HI;
			wclip = x + w;
			w = 32;
		} else {
			// single source mode
			if (color & 4) src1_ptr += GFX_HI;
			wclip = 32;
		}
		for (xclip = x; xclip < wclip; xclip += 32) {
			xclip &= XMASK;
			edx = xclip + w - SCREEN_WIDTH;
			for (yclip = y; yclip < hclip; yclip++) {
				eax = *src1_ptr;
				src1_ptr += src_dx;
				if (!eax) continue;
				eax = eax | BG_RGB;
				edi = dst_base + ((yclip & YMASK) << SCREEN_WIDTH_L2);
				if (edx > 0) {
					ecx = edx;
					dst_ptr = edi;
					WARP_LINE_COMMON
					ecx = SCREEN_WIDTH - xclip;
				} else
					ecx = w;
				dst_ptr = edi + xclip;
				WARP_LINE_COMMON
			}
			tmp = src1_ptr; src1_ptr = src2_ptr; src2_ptr = tmp;
		}
#undef WARP_LINE_COMMON
	}
}

// Ben Bero Beh color backdrop
static void bgtile_w(int offset, uint8_t data)
{
	s_cpu1_base[0x1f00 + offset] = data;
	offset -= 0x18;
	if ((unsigned)offset >= 191) return;
	int yskip = offset / 48, xskip = offset % 48;
	if (xskip > 43) return;
	yskip = yskip * 48 + 24;
	xskip = xskip * 5 + 2;
	uint16_t *edi = s_render_layer[2] + (yskip << SCREEN_WIDTH_L2) + xskip + (48 << SCREEN_WIDTH_L2);
	int ecx = -(48 << SCREEN_WIDTH_L2);
	uint16_t ax = (uint16_t)(data | BG_RGB);
	do { edi[ecx] = edi[ecx + 1] = edi[ecx + 2] = edi[ecx + 3] = edi[ecx + 4] = ax; } while (ecx += SCREEN_WIDTH);
}

static void blitter_w(int offset, uint8_t data)
{
	int i = offset & 0xf;
	s_blitter_ram[offset] = data;
	if (i == 0) blit(offset);
	if (s_game_id == GAME_BENBEROB) {
		if (i == 0 || (i == 4 && !data)) {
			s_st.blitter_busy = 0;
			if (s_st.firq_level) m6809_set_firq(&s_main, 1);   // make up delayed FIRQs
		} else {
			s_st.blitter_busy = 1;
			s_st.blitter_reset_at = s_main.cycles + 100;      // free the blitter if no updates in 100 cycles
		}
	}
}

// ---- palette ----
static uint32_t rgb(uint32_t r, uint32_t g, uint32_t b) { return r << 16 | g << 8 | b; }

static void decode_rgb(uint32_t *r, uint32_t *g, uint32_t *b, int addr, int data)
{
	int latch1 = s_paletteram[addr];
	int latch2 = (data >> 4 & 0x0f) | (data << 1 & 0x10) | (data << 3 & 0x20) | (data << 5 & 0x40) | (data << 7 & 0x80);
	int bit0 = latch1 >> 5 & 0x01, bit1 = latch1 >> 3 & 0x02, bit2, bit3, bit4;
	bit2 = latch1 >> 5 & 0x04; bit3 = latch1 >> 3 & 0x08; bit4 = latch2 >> 2 & 0x10;
	*r = s_prom[0x00 + (bit0 | bit1 | bit2 | bit3 | bit4)];
	bit2 = latch1 >> 0 & 0x04; bit3 = latch1 >> 0 & 0x08; bit4 = latch2 >> 3 & 0x10;
	*g = s_prom[0x20 + (bit0 | bit1 | bit2 | bit3 | bit4)];
	bit2 = latch1 << 2 & 0x04; bit3 = latch1 << 2 & 0x08; bit4 = latch2 << 1 & 0x10;
	*b = s_prom[0x40 + (bit0 | bit1 | bit2 | bit3 | bit4)];
}

static void paletteram_w(int offset, uint8_t data)
{
	s_paletteram[offset] = data;
	uint32_t d = data, j = d | BG_RGB;
	s_internal_palette[offset] = j;
	s_internal_palette[offset + SP_2BACK] = j;
	s_internal_palette[offset + SP_ALPHA] = j;
	s_internal_palette[offset + SP_COLLD] = j;
	uint32_t i = d >> 6 & 0x03;
	uint32_t r = (d >> 2 & 0x0c) | i; r = r << 4 | r;
	uint32_t g = (d & 0x0c) | i; g = g << 4 | g;
	uint32_t b = (d << 2 & 0x0c) | i; b = b << 4 | b;
	uint32_t c = rgb(r, g, b);
	s_rgb[offset] = s_rgb[offset + SP_2BACK] = s_rgb[offset + SP_ALPHA] = s_rgb[offset + SP_COLLD] = c;
	decode_rgb(&r, &g, &b, offset, 0);
	s_rgb[offset + 0x20] = rgb(r, g, b);
}

static void palette_init(void)
{
	memset(s_internal_palette, 0, sizeof(s_internal_palette));
	memset(s_rgb, 0, sizeof(s_rgb));
	for (uint32_t i = 0; i < 16; i++) {
		uint32_t d = (i << 6 & 0xc0) | (i << 2 & 0x30) | (i & 0x0c) | (i >> 2 & 0x03) | BG_RGB;
		for (uint32_t count = 0; count < 16; count++) {
			uint32_t r = i << 4, g = r + count + BG_MONO;
			r += i;
			s_internal_palette[g] = d;
			s_rgb[g] = rgb(r, r, r);
		}
	}
	for (uint32_t d = 0; d < 256; d++) {
		uint32_t j = d + BG_RGB;
		s_internal_palette[j] = j;
		uint32_t i = d >> 6 & 0x03;
		uint32_t r = (d >> 2 & 0x0c) | i, g = (d & 0x0c) | i, b = (d << 2 & 0x0c) | i;
		s_rgb[j] = rgb(r * 17, g * 17, b * 17);
	}
}

// ---- main CPU ----
static uint8_t coin_lockout_r(void)
{
	int inp = (int)rbx_port_read(s_port_in[0]);
	int result = ((int)rbx_port_read(s_port_dsw[3]) & 0x20) >> 5;
	if (inp & 0x80) result |= 0x02;
	if (inp & 0x40) result |= 0x04;
	return (uint8_t)result;
}

static uint8_t main_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x1000) {
		int i = a & 0xf;
		return (i == 0 || i == 4) ? 1 : s_blitter_ram[a];
	}
	if (a < 0xf000) return s_cpu1_base[a];
	if (a < 0xff00) return s_work_ram[a - 0xf000];
	if (a < 0xffc0) {
		switch (a) {
		case 0xff66:   // HACK: collision detection bypass (Halley's Comet only)
			if (s_game_id == GAME_HALLEYS && s_main.pc == s_collision_detection) {
				if (s_st.collision_count) { s_st.collision_count--; return s_collision_list[s_st.collision_count]; }
				return 0;
			}
			return s_io_ram[0x66];
		case 0xff71:   // HACK: trick the SRAM test on startup
			return (s_game_id == GAME_HALLEYS && s_main.pc == 0x8017) ? 0x55 : 0;
		case 0xff80: case 0xff81: case 0xff82: case 0xff83:
			return (uint8_t)rbx_port_read(s_port_in[a - 0xff80]);
		case 0xff90: case 0xff91: case 0xff92: case 0xff93:
			return (uint8_t)rbx_port_read(s_port_in[a - 0xff90]);
		case 0xff94: return coin_lockout_r();
		case 0xff95: return (uint8_t)rbx_port_read(s_port_dsw[0]);
		case 0xff96: return (uint8_t)rbx_port_read(s_port_dsw[1]);
		case 0xff97: return (uint8_t)rbx_port_read(s_port_dsw[2]);
		}
		return s_io_ram[a - 0xff00];
	}
	if (a < 0xffe0) return s_paletteram[a - 0xffc0];
	return s_cpu1_base[0xffe0 + ((a - 0xffe0) ^ (s_st.mVectorType << 4))];   // vector_r
}

static void main_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a < 0x1000) { blitter_w(a, v); return; }
	if (a >= 0x1f00 && a < 0x2000) { bgtile_w(a - 0x1f00, v); return; }
	if (a < 0xf000) return;
	if (a < 0xff00) { s_work_ram[a - 0xf000] = v; return; }
	if (a < 0xffc0) {
		s_io_ram[a - 0xff00] = v;
		if (a == 0xff8a) {   // soundcommand_w
			s_st.soundlatch = v;
			s_audio.z.nmi_pending = 1;
		} else if (a == 0xff9c) {   // firq_ack_w
			if (s_st.firq_level) s_st.firq_level--;
			m6809_set_firq(&s_main, 0);
		}
		return;
	}
	if (a < 0xffe0) paletteram_w(a - 0xffc0, v);
}

// ---- scanline interrupts ----
static void scanline_cb(int scanline)
{
	if (s_game_id == GAME_HALLEYS) {
		switch (scanline) {
		case 248: s_st.collision_count = 0; break;
		case 56 * 3: m6809_set_nmi(&s_main, 1); m6809_set_nmi(&s_main, 0); break;
		case 56 * 2: s_st.mVectorType = 1; m6809_set_firq(&s_main, 1); break;
		case 56: s_st.mVectorType = 0; m6809_set_firq(&s_main, 1); break;
		}
	} else {
		if (s_st.blitter_busy && s_main.cycles >= s_st.blitter_reset_at)
			s_st.blitter_busy = 0;   // blitter_reset timer
		switch (scanline) {
		case 56 * 3: m6809_set_nmi(&s_main, 1); m6809_set_nmi(&s_main, 0); break;
		case 56 * 2:
		case 56 * 1:
			// FIRQ must not happen while the blitter is being updated
			if (!s_st.blitter_busy) m6809_set_firq(&s_main, 1); else s_st.firq_level++;
			break;
		}
	}
}

// ---- video ----
static void copy_scroll(uint16_t *bm, const uint16_t *source, int sx, int sy, int opaque)
{
	sx = -sx & 0xff;
	sy = -sy & 0xff;
	int rcw = CLIP_W - sx;
	if (rcw < 0) rcw = 0;
	int bch = CLIP_H - sy;
	if (bch < 0) bch = 0;
	const uint16_t *src_base = source + CLIP_SKIP + sy * SCREEN_WIDTH;
	for (int y = 0; y < CLIP_H; y++) {
		if (y == bch)
			src_base = source + CLIP_SKIP;
		uint16_t *dest = bm + y * CLIP_W;
		if (opaque) {
			memcpy(dest, src_base + sx, 2 * (size_t)rcw);
			memcpy(dest + rcw, src_base, 2 * (size_t)(CLIP_W - rcw));
		} else {
			const uint16_t *src = src_base + sx;
			int x;
			for (x = 0; x < rcw; x++) { uint16_t p = src[x]; if (p) dest[x] = p; }
			for (src = src_base - rcw; x < CLIP_W; x++) { uint16_t p = src[x]; if (p) dest[x] = p; }
		}
		src_base += SCREEN_WIDTH;
	}
}

static void copy_fixed_xp(uint16_t *bm, const uint16_t *source)
{
	const uint16_t *src = source + CLIP_SKIP;
	for (int y = 0; y < CLIP_H; y++, src += SCREEN_WIDTH, bm += CLIP_W)
		for (int x = 0; x < CLIP_W; x++)
			if (src[x]) bm[x] = src[x];
}

static void copy_fixed_2b(uint16_t *bm, const uint16_t *source)
{
	const uint16_t *src = source + CLIP_SKIP;
	for (int y = 0; y < CLIP_H; y++, src += SCREEN_WIDTH, bm += CLIP_W)
		for (int x = 0; x < CLIP_W; x++) {
			uint16_t p = src[x];
			if ((p && !(p & SP_2BACK)) || !bm[x]) bm[x] = p;
		}
}

static void fill(uint16_t *bm, uint16_t v)
{
	for (int i = 0; i < CLIP_W * CLIP_H; i++) bm[i] = v;
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	uint16_t *bm = s_snap;
	const uint8_t *io = s_io_ram;
	if (s_game_id == GAME_HALLEYS) {   // screen_update_halleys
		if (s_st.stars_enabled) {
			copy_scroll(bm, s_render_layer[5], io[0x9a], io[0x8e], 1);
			copy_scroll(bm, s_render_layer[4], io[0xa3], io[0xa2], 0);
		} else
			fill(bm, BLACK_PEN);
		copy_scroll(bm, s_render_layer[2], io[0xa3], io[0xa2], 0);
		copy_fixed_2b(bm, s_render_layer[1]);
		copy_fixed_xp(bm, s_render_layer[0]);
	} else {   // screen_update_benberob
		if (io[0xa0] & 0x80)
			copy_scroll(bm, s_render_layer[2], io[0xa3], io[0xa2], 1);
		else
			fill(bm, BLACK_PEN);
		copy_fixed_xp(bm, s_render_layer[1]);
		copy_fixed_xp(bm, s_render_layer[0]);
	}
	memcpy(s_snap_pal, s_rgb, sizeof(s_snap_pal));
	rbx_video_frame_done();
}

static void hl_render(void)
{
	memcpy(rbx_video.palette, s_snap_pal, sizeof(s_snap_pal));
	for (int y = 0; y < CLIP_H; y++)
		memcpy(rbx_video.bitmap + y * rbx_video.pitch, s_snap + y * CLIP_W, CLIP_W * 2);
}

// ---- init / reset / frame ----
static void hl_exit(void)
{
	free(s_gfx_alloc);
	free(s_gfx1_base);
	free(s_layer_alloc);
	free(s_mix);
	free(s_snap);
	s_gfx_alloc = s_gfx1_base = NULL;
	s_layer_alloc = NULL;
	s_mix = NULL;
	s_snap = NULL;
}

static void hl_reset(void)
{
	// (the palette / video RAM too: a game that has not written it yet showed the last game's, as at start it is 0)
	memset(s_paletteram, 0, sizeof(s_paletteram));
	memset(&s_st, 0, sizeof(s_st));
	memset(s_io_ram, 0xff, sizeof(s_io_ram));
	memset(s_layer_alloc, 0, (size_t)SCREEN_SIZE * MAX_LAYERS * 2);
	memset(s_work_ram, 0, sizeof(s_work_ram));
	memset(s_audio_ram, 0, sizeof(s_audio_ram));
	memset(s_blitter_ram, 0, sizeof(s_blitter_ram));
	memset(s_collision_list, 0, sizeof(s_collision_list));
	m6809_init(&s_main);
	s_main.read = main_read;
	s_main.write = main_write;
	for (int p = 0x10; p < 0xf0; p++)
		if (p != 0x1f) s_main.rpage[p] = s_cpu1_base + p * 256;
	s_main.rpage[0x1f] = s_cpu1_base + 0x1f00;
	for (int p = 0xf0; p < 0xff; p++)
		s_main.rpage[p] = s_main.wpage[p] = s_work_ram + (p - 0xf0) * 256;
	m6809_reset(&s_main);
	memset(&s_audio, 0, sizeof(s_audio));
	audio_cpu_reset();
	for (int i = 0; i < 3; i++)
		ay8910_reset(&s_ay[i]);
	s_ay_done = 0;
	s_mix_n = 0;
	s_audio_frame_start = s_audio.z.cyc;
}

static uint16_t bitswap16(int v, const int *b)
{
	int r = 0;
	for (int i = 0; i < 16; i++) r |= ((v >> b[i]) & 1) << (15 - i);
	return (uint16_t)r;
}

static int hl_init(const rbx_game *g)
{
	uint32_t msize, gsize, psize;
	s_cpu1_base = rbx_region("maincpu", &msize);
	s_audio_rom = rbx_region("audiocpu", &s_audio_size);
	uint8_t *gfx = rbx_region("gfx1", &gsize);
	s_prom = rbx_region("proms", &psize);
	if (!s_cpu1_base || msize < 0x10000 || !s_audio_rom || !gfx || gsize < 0x20000 || !s_prom || psize < 0x60) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	if (!strcmp(g->set, "benberob")) {
		s_game_id = GAME_BENBEROB;
		s_collision_detection = 0;
	} else {
		s_game_id = GAME_HALLEYS;
		s_collision_detection = !strcmp(g->set, "halleyscjp") ? 0xb038 : !strcmp(g->set, "halleysc87") ? 0xb10d : 0xb114;
	}
	s_gfx_alloc = (uint8_t *)calloc(0x100000 + 2 * GFX_PAD, 1);
	s_gfx1_base = (uint8_t *)calloc(0x20000 + GFX_PAD, 1);
	s_layer_alloc = (uint16_t *)calloc((size_t)SCREEN_SIZE * MAX_LAYERS + 0x1000, 2);
	s_snap = (uint16_t *)calloc(CLIP_W * CLIP_H, 2);
	if (!s_gfx_alloc || !s_gfx1_base || !s_layer_alloc || !s_snap) {
		hl_exit();
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	s_gfx_plane02 = s_gfx_alloc + GFX_PAD;
	s_gfx_plane13 = s_gfx_plane02 + 0x80000;
	for (int i = 0; i < MAX_LAYERS; i++)
		s_render_layer[i] = s_layer_alloc + SCREEN_SIZE * i;

	// decrypt the main program ROM (MAME init_common)
	static const int abits[16] = { 15, 14, 13, 12, 11, 10, 1, 0, 4, 5, 6, 3, 7, 8, 9, 2 };
	uint8_t *buf = s_gfx1_base, *rom = s_cpu1_base;
	for (int i = 0; i < 0x10000; i++) {
		uint8_t v = rom[bitswap16(i, abits)];
		buf[i] = (uint8_t)((v & 1) << 7 | (v >> 7 & 1) << 6 | (v >> 6 & 1) << 5 | (v >> 5 & 1) << 4 |
		                   (v >> 1 & 1) << 3 | (v >> 4 & 1) << 2 | (v >> 2 & 1) << 1 | (v >> 3 & 1));
	}
	memcpy(rom, buf, 0x10000);

	// swap the graphics ROM addresses and unpack each pixel
	uint8_t *p = s_gfx_plane02;
	for (int i = 0xffff; i >= 0; i--) {
		uint8_t al = gfx[i], ah = gfx[i + 0x10000], dl, dh;
		s_gfx1_base[0xffff - i] = al;
		s_gfx1_base[0x1ffff - i] = ah;
		p[0] = dl = (uint8_t)((al & 1) | (ah << 2 & 4)); dl <<= 1;
		p[1] = dh = (uint8_t)((al >> 1 & 1) | (ah << 1 & 4)); dh <<= 1;
		p[0 + 0x80000] = dl; p[1 + 0x80000] = dh;
		p[2] = dl = (uint8_t)((al >> 2 & 1) | (ah & 4)); dl <<= 1;
		p[3] = dh = (uint8_t)((al >> 3 & 1) | (ah >> 1 & 4)); dh <<= 1;
		p[2 + 0x80000] = dl; p[3 + 0x80000] = dh;
		p[4] = dl = (uint8_t)((al >> 4 & 1) | (ah >> 2 & 4)); dl <<= 1;
		p[5] = dh = (uint8_t)((al >> 5 & 1) | (ah >> 3 & 4)); dh <<= 1;
		p[4 + 0x80000] = dl; p[5 + 0x80000] = dh;
		p[6] = dl = (uint8_t)((al >> 6 & 1) | (ah >> 4 & 4)); dl <<= 1;
		p[7] = dh = (uint8_t)((al >> 7) | (ah >> 5 & 4)); dh <<= 1;
		p[6 + 0x80000] = dl; p[7 + 0x80000] = dh;
		p += 8;
	}
	palette_init();
	memset(s_paletteram, 0, sizeof(s_paletteram));

	static const char *in_tags[4] = { "IN0", "IN1", "IN2", "IN3" }, *dsw_tags[4] = { "DSW1", "DSW2", "DSW3", "DSW4" };
	for (int i = 0; i < 4; i++) {
		s_port_in[i] = rbx_port_find(in_tags[i]);
		s_port_dsw[i] = rbx_port_find(dsw_tags[i]);
	}
	// YM2149 at 6 MHz / 4, MAME default (legacy) output, route 0.15
	for (int i = 0; i < 3; i++)
		ay8910_init(&s_ay[i], AY8910_TYPE_YM2149, 1500000, AY8910_LEGACY_OUTPUT, 38, 4, 1);
	s_ay[2].port_b_write = sndnmi_msk_w;
	s_mix_cap = (int)(ay8910_rate(&s_ay[0]) / 50) + 64;
	s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	if (!s_mix || rbx_audio_setup(ay8910_rate(&s_ay[0])) || rbx_video_setup(CLIP_W, CLIP_H, 0x800)) {
		hl_exit();
		return -1;
	}
	hl_reset();
	return 0;
}

static void hl_frame(void)
{
	rbx_audio.frame_mhz = RATE_MHZ;   // (the board's frame rate: a frame's sound is played in a 60 Hz frame, as its picture)
	for (int line = 0; line < LINES; line++) {
		scanline_cb(line);
		if (line == VIS_MAXY + 1)
			vblank_capture();
		s_st.main_target += rbx_line_cycles(&s_st.main_frac, MAIN_CLOCK, RATE_MHZ, LINES);
		m6809_run_until(&s_main, s_st.main_target);
		int n = rbx_line_cycles(&s_st.audio_frac, AUDIO_CLOCK, RATE_MHZ, LINES);
		unsigned long end = s_audio.z.cyc + (unsigned long)n;
		while ((long)(s_audio.z.cyc - end) < 0) {
			s_audio.z.int_pending = s_audio.irq ? 1 : 0;
			z80_step(&s_audio.z);
			if (s_audio.irq && !s_audio.z.int_pending)
				s_audio.irq = 0;   // acknowledged (HOLD_LINE)
		}
		s_st.snd_irq_cnt += n;
		if (s_st.snd_irq_cnt >= SND_IRQ_PERIOD) {
			s_st.snd_irq_cnt -= SND_IRQ_PERIOD;
			s_audio.irq = 1;
		}
		ay_sync();
	}
	for (int i = 0; i < s_mix_n; i++)
		rbx_audio_put(s_mix[i]);
	memset(s_mix, 0, sizeof(int32_t) * (size_t)s_mix_n);
	s_mix_n = 0;
	s_ay_done = 0;
	s_audio_frame_start = s_audio.z.cyc;
}

static int hl_state(void *ctx, int load)
{
	int rc = 0;
	m6809_t m = s_main;
	rc |= rbx_state_io(ctx, "m6809", &m, M6809_STATE_SIZE);
	zcpu z = s_audio;
	rc |= rbx_state_io(ctx, "z80", &z, sizeof(z));
	rc |= rbx_state_io(ctx, "misc", &s_st, sizeof(s_st));
	rc |= rbx_state_io(ctx, "blit", s_blitter_ram, sizeof(s_blitter_ram));
	rc |= rbx_state_io(ctx, "io", s_io_ram, sizeof(s_io_ram));
	rc |= rbx_state_io(ctx, "wram", s_work_ram, sizeof(s_work_ram));
	rc |= rbx_state_io(ctx, "aram", s_audio_ram, sizeof(s_audio_ram));
	rc |= rbx_state_io(ctx, "pal", s_paletteram, sizeof(s_paletteram));
	rc |= rbx_state_io(ctx, "bgtile", s_cpu1_base + 0x1f00, 0x100);
	rc |= rbx_state_io(ctx, "coll", s_collision_list, sizeof(s_collision_list));
	rc |= rbx_state_io(ctx, "layers", s_layer_alloc, (uint32_t)SCREEN_SIZE * MAX_LAYERS * 2);
	rc |= rbx_state_io(ctx, "ay2", &s_ay[0], ay8910_state_size());
	rc |= rbx_state_io(ctx, "ay3", &s_ay[1], ay8910_state_size());
	rc |= rbx_state_io(ctx, "ay4", &s_ay[2], ay8910_state_size());
	if (rc)
		return -1;
	if (load) {
		memcpy(&s_main, &m, M6809_STATE_SIZE);
		z.z.read_byte = audio_read;
		z.z.write_byte = audio_write;
		z.z.port_in = no_in;
		z.z.port_out = no_out;
		z.z.userdata = NULL;
		s_audio = z;
		s_audio_frame_start = s_audio.z.cyc;
		s_ay_done = 0;
		s_mix_n = 0;
		for (int i = 0; i < 0x20; i++)
			paletteram_w(i, s_paletteram[i]);
	}
	return 0;
}

static int hl_save(void *ctx) { return hl_state(ctx, 0); }
static int hl_load(void *ctx) { return hl_state(ctx, 1); }
static int hl_supported(const char *machine) { return !strcmp(machine, "halleys") || !strcmp(machine, "benberob"); }

const rbx_driver rbx_drv_halleys = { "Halley's Comet", hl_init, hl_exit, hl_reset, hl_frame, hl_save, hl_load, hl_supported, hl_render };
