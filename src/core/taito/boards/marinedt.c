/*  Marine Date, ported from MAME src/mame/taito/marinedt.cpp (license BSD-3-Clause, copyright-holders:
    Angelo Salese). Memory and I/O maps, the vblank IRQ (HOLD_LINE), the screen timing (set_raw 9.987 MHz / 2,
    328 x 263), the guessed sea gradient bitmap, the flipped tilemap, the two 32x32 object frame buffers and the
    PC3259 object / tile collision reads (MAME's experimental narrowed head box) follow MAME, including its
    imperfections (MAME flags the game NOT_WORKING / IMPERFECT_COLORS). Sound: MAME has no emulation of the discrete
    sound board (bgm_w / sfx_w are empty), so the driver outputs silence.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../rbx_core.h"
#include "../../cpu/z80/z80.h"

#include <stdlib.h>
#include <string.h>

// MAME set_raw(9.987 MHz / 2, 328, 0, 256, 263, 32, 256)
#define LINES 263
#define VIS_Y0 32
#define VIS_Y1 255
#define SCREEN_W 256
#define CPU_LINE 164            // 9.987 MHz / 4 = 2.49675 MHz, a line is 328 pixels at 4.9935 MHz
#define AUDIO_RATE 48000
#define FRAME_MHZ 57890         // 4993500 / (328 * 263) in milli-Hz

static uint8_t *s_rom;
static uint32_t s_rom_size;
static uint8_t s_ram[0x400], s_vram[0x400];
static z80 s_z;
static int s_irq;
static uint8_t s_in_select, s_layer_en, s_screen_flip, s_sea_bank;
static uint8_t s_obj_offs[2], s_obj_x[2], s_obj_y[2];
static uint8_t s_track[4];   // P1_TRACKX, P2_TRACKX, P1_TRACKY, P2_TRACKY counters
static uint64_t s_afrac;
// video
static uint8_t *s_chars, *s_objs[2];   // 256 8x8 3bpp; 16 32x32 2bpp per object ROM
static uint32_t s_pal_rgb[128];
static uint16_t *s_sea[2];             // 512 x 512 sea gradient bitmaps (MAME init_seabitmap)
typedef struct {
	uint8_t vram[0x400];
	uint8_t layer_en, screen_flip, sea_bank;
	uint8_t offs[2], x[2], y[2];
} snap_t;
static snap_t s_snap;
// ports
static int s_port_system, s_port_dsw1, s_port_dsw2;

// ---- objects (the MAME obj frame buffers, drawn in screen coordinates inside the visible area) ----
static int obj_pix(int which, uint8_t offs, uint8_t ox, uint8_t oy, int x, int y)
{
	if (x < 0 || x >= SCREEN_W || y < VIS_Y0 || y > VIS_Y1)
		return 0;
	int dx = x - ox, dy = y - oy;
	if (dx < 0 || dx >= 32 || dy < 0 || dy >= 32)
		return 0;
	int tile = ((offs & 4) << 1) | ((offs & 0x38) >> 3);
	int color = offs & 3;
	int fx = (offs >> 6) & 1, fy = !((offs >> 7) & 1);
	uint8_t p = s_objs[which][tile * 1024 + (fy ? 31 - dy : dy) * 32 + (fx ? 31 - dx : dx)];
	return p ? (which == 0 ? 48 : 32) + color * 4 + p : 0;
}

// tilemap pixmap (flipped in x and y unless screen_flip), 256 x 256
static int tile_pix(int x, int y)
{
	if (x < 0 || x > 255 || y < 0 || y > 255)
		return 0;
	if (!s_screen_flip) {
		x = 255 - x;
		y = 255 - y;
	}
	return s_chars[s_vram[(y >> 3) * 32 + (x >> 3)] * 64 + (y & 7) * 8 + (x & 7)];
}

static uint32_t obj_to_obj_collision(void)
{
	if ((s_layer_en & 3) != 3)
		return 0;
	for (int y = 0; y < 32; y++)
		for (int x = 0; x < 32; x++) {
			int resx = s_obj_x[0] + x, resy = s_obj_y[0] + y;
			if ((obj_pix(0, s_obj_offs[0], s_obj_x[0], s_obj_y[0], resx, resy) & 3) == 0)
				continue;
			if (obj_pix(1, s_obj_offs[1], s_obj_x[1], s_obj_y[1], resx, resy) != 0)
				return (uint32_t)(((resy / 8) * 32) | (((resx / 8) - 1) & 0x1f));
		}
	return 0;
}

static uint32_t obj_to_layer_collision(void)
{
	if (!(s_layer_en & 1))
		return 0;
	// MAME: experimental, narrowed to the internal 8x8 head
	for (int y = 12; y < 21; y++)
		for (int x = 12; x < 21; x++) {
			uint16_t resx = (uint16_t)(s_obj_x[0] + x), resy = (uint16_t)(s_obj_y[0] + y);
			if ((obj_pix(0, s_obj_offs[0], s_obj_x[0], s_obj_y[0], resx, resy) & 3) == 0)
				continue;
			if (!s_screen_flip)
				resy = (uint16_t)(resy - 32);
			if (tile_pix(resx, resy) != 0) {
				uint32_t r = (uint32_t)(((resy / 8) * 32) | (((resx / 8) - 1) & 0x1f));
				return s_screen_flip ? r : r ^ 0x3ff;
			}
		}
	return 0;
}

static uint8_t pc3259_r(int reg)
{
	uint32_t rest = obj_to_layer_collision(), reso = obj_to_obj_collision();
	switch (reg) {
	case 0: return (uint8_t)((rest & 0xf) | (reso & 0xf) << 4);
	case 1: return (uint8_t)(((rest & 0xf0) >> 4) | ((reso & 0xf0) >> 4) << 4);
	case 2: return (uint8_t)(((rest & 0x300) >> 8) | ((reso & 0x300) >> 8) << 4);
	default: return (uint8_t)((reso != 0) << 7 | (rest != 0) << 3);
	}
}

// ---- CPU ----
static uint8_t port8(int p) { return p >= 0 ? (uint8_t)rbx_port_read(p) : 0; }

static uint8_t main_read(void *u, uint16_t a)
{
	(void)u;
	a &= 0x7fff;   // A15 not decoded
	if (a < 0x4000) return a < s_rom_size ? s_rom[a] : 0xff;
	if (a < 0x4800) return s_ram[a & 0x3ff];
	if (a < 0x5000) return s_vram[a & 0x3ff];
	return 0;
}
static void main_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	a &= 0x7fff;
	if (a >= 0x4000 && a < 0x4800) s_ram[a & 0x3ff] = v;
	else if (a >= 0x4800 && a < 0x5000) s_vram[a & 0x3ff] = v;
}

static uint8_t io_in(z80 *z, uint8_t p)
{
	(void)z;
	p &= 0x0f;
	if ((p & 3) == 2)
		return pc3259_r(p >> 2);
	switch (p) {
	case 0x00: return port8(s_port_dsw1);
	case 0x01: return s_track[s_in_select & 3];
	case 0x03: return port8(s_port_system);
	case 0x04: return port8(s_port_dsw2);
	}
	return 0;
}

static void obj_w(int which, int offset, uint8_t data)
{
	switch (offset) {
	case 0: s_obj_offs[which] = data; break;
	case 1: s_obj_x[which] = (uint8_t)(data + 4); break;
	case 2: s_obj_y[which] = (uint8_t)(data + 1); break;
	}
}

static void io_out(z80 *z, uint8_t p, uint8_t v)
{
	(void)z;
	p &= 0x0f;
	if (p >= 0x02 && p <= 0x04) { obj_w(0, p - 2, v); return; }
	if (p >= 0x08 && p <= 0x0b) { obj_w(1, p - 8, v); return; }
	switch (p) {
	case 0x0d: s_layer_en = v & 0xf; s_sea_bank = (v >> 4) & 1; break;
	case 0x0f:
		s_in_select = (uint8_t)((v & 0xc) >> 2);
		s_screen_flip = (v >> 1) & 1;
		break;
	// 0x05 bgm_w, 0x06 sfx_w: discrete sound, not emulated in MAME
	}
}

// ---- video ----
static void md_render(void)
{
	const snap_t *s = &s_snap;
	memcpy(rbx_video.palette, s_pal_rgb, sizeof(s_pal_rgb));
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	const int tflip = !s->screen_flip;   // flip_screen_set(!m_screen_flip)
	for (int sy = VIS_Y0; sy <= VIS_Y1; sy++) {
		uint16_t *d = bm + (sy - VIS_Y0) * pitch;
		if (s->layer_en & 8) {
			// copybitmap(sea, flipx = flipy = !screen_flip, at -256, -224 when flipped)
			const uint16_t *sea = s_sea[s->sea_bank];
			int srcy = tflip ? 287 - sy : sy;
			for (int sx = 0; sx < SCREEN_W; sx++)
				d[sx] = sea[srcy * 512 + (tflip ? 255 - sx : sx)];
		} else {
			memset(d, 0, SCREEN_W * sizeof(uint16_t));
		}
		// MAME flips a tilemap around the centre of the visible area (y extent 32 + 255 + 1)
		int ty = tflip ? 287 - sy : sy;
		for (int sx = 0; sx < SCREEN_W; sx++) {
			int tx = tflip ? 255 - sx : sx;
			uint8_t p = s_chars[s->vram[(ty >> 3) * 32 + (tx >> 3)] * 64 + (ty & 7) * 8 + (tx & 7)];
			if (p)
				d[sx] = p;
		}
		for (int w = 0; w < 2; w++) {
			if (!(s->layer_en & (1 << w)))
				continue;
			int dy = sy - s->y[w];
			if (dy < 0 || dy >= 32)
				continue;
			for (int k = 0; k < 32; k++) {
				int sx = s->x[w] + k;
				if (sx >= SCREEN_W)
					break;
				int v = obj_pix(w, s->offs[w], s->x[w], s->y[w], sx, sy);
				if (v)
					d[sx] = (uint16_t)v;
			}
		}
	}
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap.vram, s_vram, sizeof(s_vram));
	s_snap.layer_en = s_layer_en;
	s_snap.screen_flip = s_screen_flip;
	s_snap.sea_bank = s_sea_bank;
	for (int i = 0; i < 2; i++) {
		s_snap.offs[i] = s_obj_offs[i];
		s_snap.x[i] = s_obj_x[i];
		s_snap.y[i] = s_obj_y[i];
	}
	rbx_video_frame_done();
}

// ---- init / frame ----
static void md_reset(void)
{
	memset(s_ram, 0, sizeof(s_ram));
	memset(s_vram, 0, sizeof(s_vram));
	z80_init(&s_z);
	s_z.read_byte = main_read;
	s_z.write_byte = main_write;
	s_z.port_in = io_in;
	s_z.port_out = io_out;
	s_z.int_data = 0xff;
	s_irq = 0;
	s_in_select = s_layer_en = s_screen_flip = s_sea_bank = 0;
	memset(s_obj_offs, 0, sizeof(s_obj_offs));
	memset(s_obj_x, 0, sizeof(s_obj_x));
	memset(s_obj_y, 0, sizeof(s_obj_y));
	memset(s_track, 0, sizeof(s_track));
	s_afrac = 0;
}

static void md_exit(void)
{
	free(s_chars);
	free(s_objs[0]);
	free(s_objs[1]);
	free(s_sea[0]);
	free(s_sea[1]);
	s_chars = s_objs[0] = s_objs[1] = NULL;
	s_sea[0] = s_sea[1] = NULL;
}

static int md_init(const rbx_game *g)
{
	uint32_t cs = 0, o1 = 0, o2 = 0, ps = 0;
	s_rom = rbx_region("ipl", &s_rom_size);
	uint8_t *chars = rbx_region("gfx1", &cs), *obj[2] = { rbx_region("gfx2", &o1), rbx_region("gfx3", &o2) };
	uint8_t *rprom = rbx_region("proms", &ps);
	if (!s_rom || !chars || cs < 0x1800 || !obj[0] || o1 < 0x1000 || !obj[1] || o2 < 0x1000 || !rprom || ps < 0x80) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	s_chars = (uint8_t *)malloc(256 * 64);
	s_objs[0] = (uint8_t *)malloc(16 * 1024);
	s_objs[1] = (uint8_t *)malloc(16 * 1024);
	s_sea[0] = (uint16_t *)calloc(512 * 512, sizeof(uint16_t));
	s_sea[1] = (uint16_t *)calloc(512 * 512, sizeof(uint16_t));
	if (!s_chars || !s_objs[0] || !s_objs[1] || !s_sea[0] || !s_sea[1]) {
		md_exit();
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	// charlayout: planes RGN_FRAC(0,3) (msb), (1,3), (2,3); x STEP8(7,-1): pixel x is bit x
	for (int t = 0; t < 256; t++)
		for (int y = 0; y < 8; y++)
			for (int x = 0; x < 8; x++) {
				int v = 0;
				for (int p = 0; p < 3; p++)
					v = v << 1 | ((chars[p * 0x800 + t * 8 + y] >> x) & 1);
				s_chars[t * 64 + y * 8 + x] = (uint8_t)v;
			}
	// objlayout: 32x32, planes { 0, 4 }, x groups of 4 pixels from the last 32-byte column, y one byte a row
	for (int w = 0; w < 2; w++)
		for (int t = 0; t < 16; t++)
			for (int y = 0; y < 32; y++)
				for (int x = 0; x < 32; x++) {
					uint32_t bit = (uint32_t)t * 2048 + (uint32_t)(32 * 8 * (7 - (x >> 2)) + (x & 3)) +
					               (uint32_t)(y < 16 ? y * 8 : 128 + (y - 16) * 8);
					int v = 0;
					for (int p = 0; p < 2; p++) {
						uint32_t b = bit + (uint32_t)(p * 4);
						v = v << 1 | ((obj[w][b >> 3] >> (7 - (b & 7))) & 1);
					}
					s_objs[w][t * 1024 + y * 32 + x] = (uint8_t)v;
				}
	// palette (ROMREGION_INVERT on the PROMs)
	uint8_t prom[0x80];
	for (int i = 0; i < 0x80; i++)
		prom[i] = (uint8_t)~rprom[i];
	for (int i = 0; i < 64; i++) {
		uint8_t c = prom[i];
		int r = 0x55 * (c & 1) + 0xaa * ((c >> 1) & 1);
		if (!((c >> 2) & 1)) r /= 2;
		int gg = 0x55 * ((c >> 3) & 1) + 0xaa * ((c >> 4) & 1);
		int b = 0x55 * ((c >> 5) & 1) + 0xaa * ((c >> 6) & 1);
		if (!((c >> 7) & 1)) b /= 2;
		s_pal_rgb[i] = (uint32_t)(r << 16 | gg << 8 | b);
	}
	for (int i = 0; i < 32; i++) {
		int b = prom[i + 0x60];
		s_pal_rgb[64 + 31 - i] = (uint32_t)b;
		s_pal_rgb[64 + 63 - i] = (uint32_t)(0xff0000 | b);
	}
	// sea gradient (clip 32-256 x 32-256)
	for (int y = 32; y <= 256; y++)
		for (int x = 32; x <= 256; x++) {
			int pen = 0x48 + (x - 32) / 8;
			if (pen > 0x5f) pen = 0x5f;
			s_sea[0][y * 512 + x] = (uint16_t)pen;
			s_sea[1][y * 512 + x] = (uint16_t)(pen + 0x20);
		}
	s_port_system = rbx_port_find("SYSTEM");
	s_port_dsw1 = rbx_port_find("DSW1");
	s_port_dsw2 = rbx_port_find("DSW2");
	if (rbx_audio_setup(AUDIO_RATE) || rbx_video_setup(SCREEN_W, VIS_Y1 - VIS_Y0 + 1, 128)) {
		md_exit();
		return -1;
	}
	md_reset();
	return 0;
}

static void md_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;   // (the board's frame rate: a frame's sound is played in a 60 Hz frame, as its picture)
	// trackballs (X reversed): the analog input, or the d-pad
	for (int p = 0; p < 2; p++) {
		uint32_t in = rbx_input_bits(p + 1);
		int dx = rbx_analog(p + 1, RBX_IPT_TRACKBALL_X), dy = rbx_analog(p + 1, RBX_IPT_TRACKBALL_Y);
		if (!dx) dx = ((in & RBX_IN_RIGHT) ? 4 : 0) - ((in & RBX_IN_LEFT) ? 4 : 0);
		if (!dy) dy = ((in & RBX_IN_DOWN) ? 4 : 0) - ((in & RBX_IN_UP) ? 4 : 0);
		s_track[p] = (uint8_t)(s_track[p] - dx);
		s_track[2 + p] = (uint8_t)(s_track[2 + p] + dy);
	}
	for (int line = 0; line < LINES; line++) {
		if (line == VIS_Y1 + 1) {
			vblank_capture();
			s_irq = 1;   // irq0_line_hold
		}
		unsigned long end = s_z.cyc + CPU_LINE;
		while ((long)(s_z.cyc - end) < 0) {
			s_z.int_pending = s_irq ? 1 : 0;
			z80_step(&s_z);
			if (s_irq && !s_z.int_pending)
				s_irq = 0;
		}
	}
	int n = rbx_line_cycles(&s_afrac, AUDIO_RATE, FRAME_MHZ, 1);   // silence
	for (int i = 0; i < n; i++)
		rbx_audio_put(0);
}

static int md_state(void *ctx, int load)
{
	z80 z = s_z;
	int rc = 0;
	rc |= rbx_state_io(ctx, "z80", &z, sizeof(z));
	rc |= rbx_state_io(ctx, "ram", s_ram, sizeof(s_ram));
	rc |= rbx_state_io(ctx, "vram", s_vram, sizeof(s_vram));
	uint8_t regs[14] = { s_in_select, s_layer_en, s_screen_flip, s_sea_bank, s_obj_offs[0], s_obj_offs[1],
		s_obj_x[0], s_obj_x[1], s_obj_y[0], s_obj_y[1], s_track[0], s_track[1], s_track[2], s_track[3] };
	rc |= rbx_state_io(ctx, "regs", regs, sizeof(regs));
	rc |= rbx_state_io(ctx, "irq", &s_irq, sizeof(s_irq));
	if (rc)
		return -1;
	if (load) {
		z.read_byte = main_read;
		z.write_byte = main_write;
		z.port_in = io_in;
		z.port_out = io_out;
		z.userdata = NULL;
		s_z = z;
		s_in_select = regs[0]; s_layer_en = regs[1]; s_screen_flip = regs[2]; s_sea_bank = regs[3];
		s_obj_offs[0] = regs[4]; s_obj_offs[1] = regs[5]; s_obj_x[0] = regs[6]; s_obj_x[1] = regs[7];
		s_obj_y[0] = regs[8]; s_obj_y[1] = regs[9];
		memcpy(s_track, regs + 10, 4);
	}
	return 0;
}

static int md_save(void *ctx) { return md_state(ctx, 0); }
static int md_load(void *ctx) { return md_state(ctx, 1); }
static int md_supported(const char *machine) { return !strcmp(machine, "marinedt"); }

const rbx_driver rbx_drv_marinedt = { "Marine Date", md_init, md_exit, md_reset, md_frame, md_save, md_load,
                                      md_supported, md_render };
