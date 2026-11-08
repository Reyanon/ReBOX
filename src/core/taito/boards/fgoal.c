/*  Taito Field Goal, ported from MAME src/mame/taito/fgoal.cpp (license BSD-3-Clause, copyright-holders:
    Stefan Jokisch) and src/devices/machine/mb14241.cpp (BSD-3-Clause, Fabio Priuli).
    MC6800 at 10.065 MHz / 10 (cpu/m6800 core, M6801_TYPE_6800), 256 x 263 at 60 Hz (256 x 240 visible).
    The interrupt timer of MAME fires at scanlines 0, 128 and 256: it asserts IRQ (cleared by reading 0xfc-0xff)
    and, on a coin press, NMI (cleared by reading 0xf8-0xfb). IN1 bit 7 is 128V.
    Video: 1 bpp bitmap; lit pixels take the 64x64 4 bpp overlay PROM picture (zoomed x4, colour set by the
    player and port 0x8002) and unlit ones the background picture; the 8x8 ball has a fixed colour. The overlay is
    flipped for player 2 in a cocktail cabinet. The PROMs are ROM_LOAD_NIB_LOW / HIGH into a ROMREGION_INVERT
    region: the two nibble files are combined and inverted here.
    Paddles: rbx_analog(PADDLE) deltas, PORT_REVERSE, range 1-254 (the game freezes at 0 / 255); without a device
    the d-pad left / right turns the paddle.
    Sound: none (MAME_NO_SOUND in MAME: the discrete sound board is not emulated there either). The sound latch
    writes are ignored except the player select bit.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../rbx_core.h"
#include "../../cpu/m6800/m6801.h"

#include <stdlib.h>
#include <string.h>

#define CPU_CLOCK (10065000 / 10)
#define VTOTAL 263
#define VIS_Y0 16
#define VIS_W 256
#define VIS_H 240
#define AUDIO_RATE 48000

static m6801 s_cpu;
static uint8_t *s_rom;
static uint32_t s_rom_size;
static uint8_t s_ram[0x400], s_vram[0x4000];
static uint8_t s_tile[64 * 64];   // overlay picture, 4 bpp, 1 byte per pixel
static uint8_t s_prom[0x100];
static uint8_t s_xpos, s_ypos, s_color, s_player, s_row, s_col, s_prev_coin;
static uint16_t s_shift_data;
static uint8_t s_shift_count;
static int s_line;
static int s_paddle[2];
static int s_port_in0, s_port_in1;
static uint64_t s_frac, s_afrac;

typedef struct {
	uint8_t vram[0x2000];
	uint8_t xpos, ypos, color, player, flip;
} snap_t;
static snap_t s_snap;

// ---- MB14241 ----
static void shift_count_w(uint8_t d) { s_shift_count = (uint8_t)(~d & 7); }
static void shift_data_w(uint8_t d) { s_shift_data = (uint16_t)((s_shift_data >> 8) | ((uint16_t)d << 7)); }
static uint8_t shift_result_r(void) { return (uint8_t)((s_shift_data >> s_shift_count) & 0xff); }

static uint8_t in1_r(void)
{
	return (uint8_t)((rbx_port_read(s_port_in1) & 0x7f) | (s_line & 0x80));   // 128V
}

static uint16_t vram_address(void) { return (uint16_t)(0x4000 | (s_row << 5) | (s_col >> 3)); }

// ---- memory ----
static uint8_t mem_r(void *u, uint16_t a)
{
	(void)u;
	if (a < 0xf0) return s_ram[a];
	if (a < 0x100) {
		switch (a) {
		case 0xf0: return s_row;
		case 0xf1: return (uint8_t)s_paddle[s_player];
		case 0xf2: return (uint8_t)rbx_port_read(s_port_in0);
		case 0xf3: return in1_r();
		case 0xf4: return (uint8_t)(vram_address() >> 8);
		case 0xf5: return (uint8_t)(vram_address() & 0xff);
		case 0xf6: return shift_result_r();
		case 0xf7: {
			uint8_t v = shift_result_r(), r = 0;
			for (int i = 0; i < 8; i++)
				r |= (uint8_t)(((v >> i) & 1) << (7 - i));
			return r;
		}
		case 0xf8: case 0xf9: case 0xfa: case 0xfb:
			m6801_set_irq_line(&s_cpu, M6801_NMI_LINE, 0);
			return 0;
		default:
			m6801_set_irq_line(&s_cpu, M6801_IRQ1_LINE, 0);
			return 0;
		}
	}
	if (a < 0x400) return s_ram[a];
	if (a >= 0x4000 && a < 0x8000) return s_vram[a - 0x4000];
	if ((a >= 0xa000 && a < 0xc000) || a >= 0xd000) return s_rom[a];
	return 0xff;
}

static void mem_w(void *u, uint16_t a, uint8_t d)
{
	(void)u;
	if (a < 0xf0) { s_ram[a] = d; return; }
	if (a < 0x100) {
		switch (a) {
		case 0xf0: case 0xf2: s_row = d; shift_data_w(0); break;
		case 0xf1: case 0xf3: s_col = d; shift_count_w(d); break;
		case 0xf4: case 0xf5: case 0xf6: case 0xf7: shift_data_w(d); break;
		case 0xfc: case 0xfd: case 0xfe: case 0xff: s_player = d & 1; break;   // sound2_w
		default: break;                                                    // sound1_w
		}
		return;
	}
	if (a < 0x400) s_ram[a] = d;
	else if (a >= 0x4000 && a < 0x8000) s_vram[a - 0x4000] = d;
	else if (a == 0x8000) s_ypos = d;
	else if (a == 0x8001) s_xpos = d;
	else if (a == 0x8002) s_color = d & 3;
}

// ---- video ----
static void fg_render(void)
{
	const snap_t *s = &s_snap;
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	const uint16_t fgbase = (uint16_t)(((s->player << 2) | s->color) * 16);
	for (int y = VIS_Y0; y < 256; y++) {
		uint16_t *row = bm + (y - VIS_Y0) * pitch;
		const uint8_t *v = s->vram + y * 32;
		// overlay row (zoomed x4; flipped at (0, 16) for player 2 in cocktail)
		int ty = s->flip ? (255 - (y - 16)) >> 2 : y >> 2;
		const uint8_t *trow = s_tile + ty * 64;
		int ball = y >= s->ypos && y < s->ypos + 8;
		for (int x = 0; x < 256; x++) {
			int tx = s->flip ? (255 - x) >> 2 : x >> 2;
			uint8_t pix = trow[tx];
			if ((v[x >> 3] >> (x & 7)) & 1)
				row[x] = (ball && x >= s->xpos && x < s->xpos + 8) ? 128 + 16 : (uint16_t)(fgbase + pix);
			else
				row[x] = (uint16_t)(0x80 + pix);
		}
	}
}

static int intensity(int bits)
{
	int v = 0x2e;
	if (bits & 1) v += 0x27;
	if (bits & 2) v += 0xaa;
	return v;
}

// ---- controls ----
static void update_paddles(void)
{
	for (int p = 0; p < 2; p++) {
		int d = rbx_analog(p + 1, RBX_IPT_PADDLE);
		if (!d) {
			uint32_t in = rbx_input(p + 1);
			if (in & RBX_IN_LEFT) d = -5;
			if (in & RBX_IN_RIGHT) d = 5;
		}
		int v = s_paddle[p] - d;   // PORT_REVERSE
		s_paddle[p] = v < 1 ? 1 : v > 254 ? 254 : v;
	}
}

// ---- machine ----
static void fg_reset(void)
{
	memset(s_ram, 0, sizeof(s_ram));
	memset(s_vram, 0, sizeof(s_vram));
	s_xpos = s_ypos = s_color = s_player = s_row = s_col = s_prev_coin = 0;
	s_shift_data = 0;
	s_shift_count = 0;
	s_paddle[0] = s_paddle[1] = 0x80;
	s_frac = s_afrac = 0;
	m6801_init(&s_cpu);
	m6801_reset(&s_cpu);
}

static void fg_exit(void) {}

static int fg_init(const rbx_game *g)
{
	uint32_t tsize, lsize, hsize;
	s_rom = rbx_region("maincpu", &s_rom_size);
	const uint8_t *tiles = rbx_region("tiles", &tsize);
	const uint8_t *lo = rbx_region("proms:nib_low", &lsize);
	const uint8_t *hi = rbx_region("proms:nib_high", &hsize);
	if (!s_rom || s_rom_size < 0x10000 || !tiles || tsize < 0x1000 || !lo || !hi || lsize < 0x100 || hsize < 0x100) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	// ROM_LOAD_NIB_LOW / HIGH, then ROMREGION_INVERT
	for (int i = 0; i < 0x100; i++)
		s_prom[i] = (uint8_t)~(((hi[i] & 0x0f) << 4) | (lo[i] & 0x0f));
	// gfxlayout: 64x64, 1 byte per pixel, planes at bit offsets 4-7 (MSB-first numbering): the low nibble
	for (int i = 0; i < 64 * 64; i++)
		s_tile[i] = tiles[i] & 0x0f;
	s_port_in0 = rbx_port_find("IN0");
	s_port_in1 = rbx_port_find("IN1");
	if (rbx_video_setup(VIS_W, VIS_H, 256) || rbx_audio_setup(AUDIO_RATE))
		return -1;
	for (int i = 0; i < 128; i++) {
		uint8_t c = s_prom[0x80 | i] & 63;
		rbx_video.palette[i] = (uint32_t)(intensity(c >> 4) << 16 | intensity(c >> 2) << 8 | intensity(c));
	}
	for (int i = 0; i < 8; i++) {
		rbx_video.palette[128 + i] = 0x2e802e;
		rbx_video.palette[136 + i] = 0x2e2e2e;
	}
	rbx_video.palette[144] = (uint32_t)(intensity(0x38 >> 4) << 16 | intensity(0x38 >> 2) << 8 | intensity(0x38));
	memset(&s_cpu, 0, sizeof(s_cpu));
	s_cpu.read = mem_r;
	s_cpu.write = mem_w;
	s_cpu.type = M6801_TYPE_6800;
	fg_reset();
	return 0;
}

static void fg_frame(void)
{
	update_paddles();
	for (int line = 0; line < VTOTAL; line++) {
		s_line = line;
		if (line == 0 || line == 128 || line == 256) {
			// interrupt_callback
			int coin = rbx_port_read(s_port_in1) & 2;
			m6801_set_irq_line(&s_cpu, M6801_IRQ1_LINE, 1);
			if (!coin && s_prev_coin)
				m6801_set_irq_line(&s_cpu, M6801_NMI_LINE, 1);
			s_prev_coin = (uint8_t)coin;
		}
		if (line == 256) {
			rbx_video_wait_idle();
			memcpy(s_snap.vram, s_vram, sizeof(s_snap.vram));
			s_snap.xpos = s_xpos;
			s_snap.ypos = s_ypos;
			s_snap.color = s_color;
			s_snap.player = s_player;
			s_snap.flip = s_player == 1 && (rbx_port_read(s_port_in1) & 0x40);
			rbx_video_frame_done();
		}
		m6801_execute(&s_cpu, rbx_line_cycles(&s_frac, CPU_CLOCK, 60000, VTOTAL));
		// no sound: silence at the output rate
		s_afrac += AUDIO_RATE;
		while (s_afrac >= 60 * VTOTAL) {
			s_afrac -= 60 * VTOTAL;
			rbx_audio_put(0);
		}
	}
}

static int fg_state(void *ctx, int load)
{
	int rc = 0;
	rc |= rbx_state_io(ctx, "cpu", m6801_state_data(&s_cpu), m6801_state_size());
	rc |= rbx_state_io(ctx, "ram", s_ram, sizeof(s_ram));
	rc |= rbx_state_io(ctx, "vram", s_vram, sizeof(s_vram));
	uint8_t m[8] = { s_xpos, s_ypos, s_color, s_player, s_row, s_col, s_prev_coin, s_shift_count };
	rc |= rbx_state_io(ctx, "misc", m, sizeof(m));
	rc |= rbx_state_io(ctx, "shift", &s_shift_data, sizeof(s_shift_data));
	rc |= rbx_state_io(ctx, "paddle", s_paddle, sizeof(s_paddle));
	rc |= rbx_state_io(ctx, "frac", &s_frac, sizeof(s_frac));
	if (rc)
		return -1;
	if (load) {
		s_xpos = m[0]; s_ypos = m[1]; s_color = m[2]; s_player = m[3] & 1; s_row = m[4]; s_col = m[5];
		s_prev_coin = m[6]; s_shift_count = m[7];
	}
	return 0;
}

static int fg_save(void *ctx) { return fg_state(ctx, 0); }
static int fg_load(void *ctx) { return fg_state(ctx, 1); }
static int fg_supported(const char *machine) { return !strcmp(machine, "fgoal"); }

const rbx_driver rbx_drv_fgoal = { "Field Goal", fg_init, fg_exit, fg_reset, fg_frame, fg_save, fg_load,
                                   fg_supported, fg_render };
