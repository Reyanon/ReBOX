/*  KiKi KaiKai / Kick and Run, ported from MAME src/mame/taito/kikikai.cpp (BSD-3-Clause, copyright-holders:
    Ernesto Corvi, Nicola Salmoria). Memory maps, the bank switch, the MC6801U4 interface (kikikai / kicknrun), the
    bootleg 68705P3 interface (knightb / mexico86, MAME's guesswork), the sub board CPU of Kick and Run and the
    sprite-only video (both screen_update variants) follow MAME. Sound: YM2203 (rbx_ym).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/z80/z80.h"
#include "../../cpu/m6800/m6801.h"
#include "../../cpu/m6805/m68705.h"
#include "../../sound/rbx_ym.h"

#include <stdlib.h>
#include <string.h>

enum { KK_KIKIKAI, KK_KICKNRUN, KK_MEXICO86, KK_KNIGHTB, KK_KNIGHTBA };

// MAME set_raw(24 MHz / 4, 384, 0, 256, 264, 16, 240): the Z80s run at the pixel clock (384 clocks a line)
#define LINES 264
#define VIS_Y0 16
#define VIS_Y1 239
#define SCREEN_W 256
#define MAIN_LINE 384
#define SUB_LINE 256        // 4 MHz
#define YM_LINE 192         // 3 MHz
#define M6801_LINE 48       // 3 MHz / 4
#define M68705_LINE 64      // 4 MHz / 4
#define SLICES 4

typedef struct {
	z80 z;
	int irq;
	int reset;
} zcpu;

static int s_kind, s_has_sub, s_vector_irq;
static uint8_t *s_main_rom, *s_audio_rom, *s_sub_rom;
static uint32_t s_main_size, s_audio_size, s_sub_size;
static zcpu s_main, s_audio, s_sub;
static uint8_t s_mainram[0x2800], s_sharedram[0x100], s_ram_e900[0x700], s_subram[0x800], s_sub_work[0x800];
static uint8_t s_audio_ram[0x1800];
static int s_bank, s_charbank;
// MC6801U4
static m6801 s_m6801;
static uint8_t s_port1_out, s_port2_out, s_port3_out, s_port4_out, s_port3_in;
// 68705P3 (bootlegs)
static m68705_t s_m68705;
static uint8_t s_pa_out, s_pb_out, s_latch, s_address;
static int s_mcu_reset;
// sound
static int s_ym_irq;
static int32_t *s_mix;
static int s_mix_cap, s_mix_n;
static uint32_t s_ym_done, s_ym_slice_start;
static unsigned long s_cpu_start;
// video
static uint8_t *s_tiles;
static uint32_t s_pal_rgb[256];
static uint8_t s_snap_ram[0x2800];
static int s_snap_charbank;
// ports
static int s_p_in[8], s_p_dsw0, s_p_dsw1;

static uint8_t port8(int p) { return (uint8_t)rbx_port_read(p); }

// ---- sound ----
static void ym_run_to(uint32_t pos)
{
	if (pos <= s_ym_done)
		return;
	s_mix_n += rbx_ym_advance(0, pos - s_ym_done, s_mix + s_mix_n, s_mix_cap - s_mix_n);
	s_ym_done = pos;
}
static void ym_sync(void)
{
	uint32_t done = (uint32_t)(s_audio.z.cyc - s_cpu_start);
	if (done > MAIN_LINE / SLICES)
		done = MAIN_LINE / SLICES;
	ym_run_to(s_ym_slice_start + done / 2);   // (YM at half the CPU clock)
}
static void ym_irq(int state) { s_ym_irq = state; }
static uint8_t ym_pa(void) { return port8(s_p_dsw0); }
static uint8_t ym_pb(void) { return port8(s_p_dsw1); }

// ---- MC6801U4 ----
static uint8_t m6801_read(void *u, uint16_t a) { (void)u; (void)a; return 0xff; }
static void m6801_write(void *u, uint16_t a, uint8_t v) { (void)u; (void)a; (void)v; }
static uint8_t m6801_port_r(void *u, int port)
{
	(void)u;
	if (port == 1) return (uint8_t)~port8(s_p_in[0]);
	if (port == 3) return s_port3_in;
	return 0xff;
}
static void m6801_port_w(void *u, int port, uint8_t data, uint8_t ddr)
{
	(void)u; (void)ddr;
	switch (port) {
	case 1: s_port1_out = data; break;
	case 2:
		// bit 2: clock, latch on high -> low
		if ((s_port2_out & 0x04) && (~data & 0x04)) {
			int address = s_port4_out;
			if (data & 0x10) {
				s_port3_in = (data & 0x01) ? s_sharedram[address] : port8(s_p_in[(address & 1) + 1]);
				m6801_set_irq_line(&s_m6801, M6801_IS3_LINE, 1);
				m6801_set_irq_line(&s_m6801, M6801_IS3_LINE, 0);
			} else
				s_sharedram[address] = s_port3_out;
		}
		s_port2_out = data;
		break;
	case 3: s_port3_out = data; break;
	case 4: s_port4_out = data; break;
	}
}

// ---- 68705P3 (mexico86 / knightb) ----
static uint8_t m68705_port_r(void *u, int n, uint8_t mask)
{
	(void)u; (void)mask;
	return n == 2 ? port8(s_p_in[0]) : 0xff;
}
static void m68705_port_w(void *u, int n, uint8_t data, uint8_t mask)
{
	(void)u;
	if (n == 0) {
		s_pa_out = data;
		return;
	}
	if (n != 1)
		return;
	uint8_t pa = (uint8_t)(s_pa_out & ((s_pb_out & 1) ? 0xff : s_latch));
	if ((mask & 8) && !(data & 8) && (s_pb_out & 8)) {
		if (s_pb_out & 0x10)
			s_latch = (s_pb_out & 4) ? s_sharedram[s_address] : port8(s_p_in[(s_address & 1) + 1]);
		else
			s_sharedram[s_address] = pa;
	}
	m68705_port_input(&s_m68705, 0, ((mask & 1) && !(data & 1)) ? s_latch : 0xff);
	if ((mask & 2) && !(data & 2) && (s_pb_out & 2))
		s_address = pa;
	if ((mask & 0x20) && (data & 0x20) && !(s_pb_out & 0x20)) {
		s_main.irq = 1;
		m68705_set_irq_line(&s_m68705, 0);
	}
	s_pb_out = data;
}

// ---- main CPU ----
static void audio_reset_cpu(void);

static void f008_w(uint8_t data)
{
	int ar = !(data & 4);
	if (ar && !s_audio.reset)
		audio_reset_cpu();
	s_audio.reset = ar;
	int mr = !(data & 2);
	if (s_kind == KK_KIKIKAI || s_kind == KK_KICKNRUN)
		m6801_set_reset(&s_m6801, mr);
	else if (s_kind == KK_MEXICO86 || s_kind == KK_KNIGHTB) {
		if (mr)
			m68705_set_irq_line(&s_m68705, 0);   // (the line is raised again by the next vblank, an edge for the core)
		m68705_set_reset_line(&s_m68705, mr);
	}
	s_mcu_reset = mr;
}

static uint8_t main_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x8000) return s_main_rom[a];
	if (a < 0xc000) {
		uint32_t o = 0x8000u + 0x4000u * (uint32_t)s_bank + (a - 0x8000u);
		return o < s_main_size ? s_main_rom[o] : 0xff;
	}
	if (a < 0xe800) return s_mainram[a - 0xc000];
	if (s_kind == KK_KNIGHTBA) {
		if (a < 0xf000) return s_ram_e900[(a - 0xe800) % sizeof(s_ram_e900)];
		if (a == 0xf010) return port8(s_p_in[3]);
		if (a == 0xf019) return port8(s_p_in[1]);
		return 0;
	}
	if (a < 0xe900) return s_sharedram[a & 0xff];
	if (a < 0xf000) return s_ram_e900[a - 0xe900];
	if (a == 0xf010) return port8(s_p_in[3]);
	if (a >= 0xf800) return s_subram[a & 0x7ff];
	return 0;
}

static void main_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a < 0xc000) return;
	if (a < 0xe800) { s_mainram[a - 0xc000] = v; return; }
	if (s_kind == KK_KNIGHTBA) {
		if (a < 0xf000) { s_ram_e900[(a - 0xe800) % sizeof(s_ram_e900)] = v; return; }
		if (a == 0xf000) { s_bank = v & 7; s_charbank = (v >> 5) & 1; }
		return;
	}
	if (a < 0xe900) { s_sharedram[a & 0xff] = v; return; }
	if (a < 0xf000) { s_ram_e900[a - 0xe900] = v; return; }
	if (a == 0xf000) { s_bank = v & 7; s_charbank = (v >> 5) & 1; return; }
	if (a == 0xf008) { f008_w(v); return; }
	if (a >= 0xf800) { s_subram[a & 0x7ff] = v; return; }
}

// ---- sound CPU ----
static uint8_t audio_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x8000) return a < s_audio_size ? s_audio_rom[a] : 0xff;
	if (a < 0xa800) return s_mainram[a - 0x8000];
	if (a < 0xc000) return s_audio_ram[a - 0xa800];
	if (a < 0xc002) { ym_sync(); return rbx_ym_read(0, a & 1); }
	return 0xff;
}
static void audio_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a < 0x8000) return;
	if (a < 0xa800) { s_mainram[a - 0x8000] = v; return; }
	if (a < 0xc000) { s_audio_ram[a - 0xa800] = v; return; }
	if (a < 0xc002) { ym_sync(); rbx_ym_write(0, a & 1, v); }
}

// ---- sub CPU (kicknrun / mexico86) ----
static uint8_t sub_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x4000) return a < s_sub_size ? s_sub_rom[a] : 0xff;
	if (a < 0x4800) return s_sub_work[a & 0x7ff];
	if (a >= 0x8000 && a < 0x8800) return s_subram[a & 0x7ff];
	if (a >= 0xc000 && a <= 0xc003) return port8(s_p_in[4 + (a & 3)]);
	return 0xff;
}
static void sub_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0x4000 && a < 0x4800) s_sub_work[a & 0x7ff] = v;
	else if (a >= 0x8000 && a < 0x8800) s_subram[a & 0x7ff] = v;
}

static uint8_t no_in(z80 *z, uint8_t p) { (void)z; (void)p; return 0xff; }
static void no_out(z80 *z, uint8_t p, uint8_t v) { (void)z; (void)p; (void)v; }

static void zcpu_setup(zcpu *c, uint8_t (*rd)(void *, uint16_t), void (*wr)(void *, uint16_t, uint8_t))
{
	unsigned long cyc = c->z.cyc;
	int reset = c->reset;
	z80_init(&c->z);
	c->z.read_byte = rd;
	c->z.write_byte = wr;
	c->z.port_in = no_in;
	c->z.port_out = no_out;
	c->z.int_data = 0xff;
	c->z.cyc = cyc;
	c->irq = 0;
	c->reset = reset;
}
static void audio_reset_cpu(void) { zcpu_setup(&s_audio, audio_read, audio_write); }

// level: -1 = HOLD_LINE (c->irq, cleared on acknowledge), else the line level
static void zcpu_run(zcpu *c, int cycles, int level)
{
	unsigned long end = c->z.cyc + (unsigned long)cycles;
	if (c->reset) {
		c->z.cyc = end;
		return;
	}
	while ((long)(c->z.cyc - end) < 0) {
		int line = level >= 0 ? level : c->irq;
		c->z.int_pending = line ? 1 : 0;
		if (c == &s_main && s_vector_irq)
			c->z.int_data = s_sharedram[0];   // MAME mcram_vect_r
		z80_step(&c->z);
		if (level < 0 && c->irq && !c->z.int_pending)
			c->irq = 0;   // acknowledged (also mcram_vect_r clears the line)
	}
}

// ---- video ----
static void draw_tile(int code, int color, int flipx, int x, int y)
{
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	const uint8_t *src = s_tiles + (code & 8191) * 64;
	for (int ty = 0; ty < 8; ty++) {
		int dy = y + ty;
		if (dy < VIS_Y0 || dy > VIS_Y1) continue;
		uint16_t *d = bm + (dy - VIS_Y0) * pitch;
		const uint8_t *r = src + ty * 8;
		for (int tx = 0; tx < 8; tx++) {
			int dx = x + tx;
			if (dx >= SCREEN_W) continue;
			uint8_t c = r[flipx ? 7 - tx : tx];
			if (c != 15)
				d[dx] = (uint16_t)(color * 16 + c);
		}
	}
}

static void render_kicknrun(void)
{
	const uint8_t *ram = s_snap_ram;
	int sx = 0;
	for (int offs = 0x1500; offs < 0x2000; offs += 4) {
		if (offs >= 0x1800 && offs < 0x1980) continue;
		if (offs >= 0x19c0) continue;
		if (!(ram[offs] | ram[offs + 1] | ram[offs + 2] | ram[offs + 3])) continue;
		int gfx_num = ram[offs + 1], gfx_attr = ram[offs + 3];
		int gfx_offs, height;
		if (!(gfx_num & 0x80)) {
			gfx_offs = ((gfx_num & 0x1f) * 0x80) + ((gfx_num & 0x60) >> 1) + 12;
			height = 2;
		} else {
			gfx_offs = (gfx_num & 0x3f) * 0x80;
			height = 32;
		}
		if ((gfx_num & 0xc0) == 0xc0)
			sx += 16;
		else
			sx = ram[offs + 2];
		int sy = 256 - height * 8 - ram[offs];
		for (int xc = 0; xc < 2; xc++)
			for (int yc = 0; yc < height; yc++) {
				int goffs = gfx_offs + xc * 0x40 + yc * 2;
				int code = ram[goffs] + ((ram[goffs + 1] & 7) << 8) + ((ram[goffs + 1] & 0x80) << 4) + (s_snap_charbank << 12);
				int color = ((ram[goffs + 1] & 0x38) >> 3) + ((gfx_attr & 2) << 2);
				draw_tile(code, color, ram[goffs + 1] & 0x40, (sx + xc * 8) & 0xff, (sy + yc * 8) & 0xff);
			}
	}
}

static void render_kikikai(void)
{
	const uint8_t *ram = s_snap_ram;
	int sx = 0;
	for (int offs = 0x1500; offs < 0x1800; offs += 4) {
		if (!(ram[offs] | ram[offs + 1] | ram[offs + 2] | ram[offs + 3])) continue;
		int ty = ram[offs], gfx_num = ram[offs + 1], tx = ram[offs + 2];
		int gfx_offs, height;
		if (gfx_num & 0x80) {
			gfx_offs = (gfx_num & 0x3f) << 7;
			height = 32;
			if (gfx_num & 0x40) sx += 16;
			else sx = tx;
		} else {
			if (!(ty && tx)) continue;
			gfx_offs = ((gfx_num & 0x1f) << 7) + ((gfx_num & 0x60) >> 1) + 12;
			height = 2;
			sx = tx;
		}
		int sy = 256 - (height << 3) - ty;
		height <<= 1;
		for (int yc = 0; yc < height; yc += 2) {
			int y = (sy + (yc << 2)) & 0xff;
			int goffs = gfx_offs + yc;
			draw_tile(ram[goffs] + ((ram[goffs + 1] & 0x1f) << 8), (ram[goffs + 1] & 0xe0) >> 5, 0, sx & 0xff, y);
			goffs += 0x40;
			draw_tile(ram[goffs] + ((ram[goffs + 1] & 0x1f) << 8), (ram[goffs + 1] & 0xe0) >> 5, 0, (sx + 8) & 0xff, y);
		}
	}
}

static void kk_render(void)
{
	for (int i = 0; i < 256; i++)
		rbx_video.palette[i] = s_pal_rgb[i];
	rbx_video.palette[256] = 0;   // black pen
	const int kik = s_kind == KK_KIKIKAI || s_kind == KK_KNIGHTB || s_kind == KK_KNIGHTBA;
	const uint16_t fill = kik ? 256 : 255;
	uint16_t *bm = rbx_video.bitmap;
	for (int y = 0; y < rbx_video.height; y++)
		for (int x = 0; x < SCREEN_W; x++)
			bm[y * rbx_video.pitch + x] = fill;
	if (kik)
		render_kikikai();
	else
		render_kicknrun();
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap_ram, s_mainram, sizeof(s_mainram));
	s_snap_charbank = s_charbank;
	rbx_video_frame_done();
}

// ---- init / frame ----
static int kind_of(const char *m)
{
	if (!strcmp(m, "kikikai")) return KK_KIKIKAI;
	if (!strcmp(m, "kicknrun")) return KK_KICKNRUN;
	if (!strcmp(m, "mexico86")) return KK_MEXICO86;
	if (!strcmp(m, "knightb")) return KK_KNIGHTB;
	if (!strcmp(m, "knightba")) return KK_KNIGHTBA;
	return -1;
}

static void kk_reset(void)
{
	// (first: the sound syncs read them, from a CPU write or a port callback of a chip reset below, before a frame
	// sets them; the last game's values made a render of billions of clocks, the next game of the board hung)
	s_cpu_start = 0;
	memset(s_mainram, 0, sizeof(s_mainram));
	memset(s_sharedram, 0, sizeof(s_sharedram));
	memset(s_ram_e900, 0, sizeof(s_ram_e900));
	memset(s_subram, 0, sizeof(s_subram));
	memset(s_sub_work, 0, sizeof(s_sub_work));
	memset(s_audio_ram, 0, sizeof(s_audio_ram));
	memset(&s_main, 0, sizeof(s_main));
	memset(&s_audio, 0, sizeof(s_audio));
	memset(&s_sub, 0, sizeof(s_sub));
	zcpu_setup(&s_main, main_read, main_write);
	zcpu_setup(&s_audio, audio_read, audio_write);
	zcpu_setup(&s_sub, sub_read, sub_write);
	s_sub.reset = !s_has_sub || (port8(s_p_dsw1) & 0x80);
	s_bank = 0;
	s_charbank = 0;
	s_port1_out = s_port2_out = s_port3_out = s_port4_out = s_port3_in = 0;
	s_pa_out = s_pb_out = 0xff;
	s_latch = s_address = 0;
	s_mcu_reset = 0;
	if (s_kind == KK_KIKIKAI || s_kind == KK_KICKNRUN) {
		m6801_init(&s_m6801);
		m6801_reset(&s_m6801);
	} else if (s_kind != KK_KNIGHTBA)
		m68705_reset(&s_m68705);
	s_ym_irq = 0;
	rbx_ym_reset(0);
	s_ym_done = 0;
	s_mix_n = 0;
}

static void kk_exit(void)
{
	free(s_tiles);
	s_tiles = NULL;
	free(s_mix);
	s_mix = NULL;
	rbx_ym_exit(0);
}

static int kk_init(const rbx_game *g)
{
	s_kind = kind_of(g->machine);
	if (s_kind < 0) {
		rbx_fail("%s: machine %s is not emulated", g->set, g->machine);
		return -1;
	}
	s_has_sub = s_kind == KK_KICKNRUN || s_kind == KK_MEXICO86;
	s_vector_irq = s_kind != KK_KNIGHTBA;
	uint32_t gsize, psize;
	s_main_rom = rbx_region("maincpu", &s_main_size);
	s_audio_rom = rbx_region("audiocpu", &s_audio_size);
	s_sub_rom = rbx_region("sub", &s_sub_size);
	uint8_t *gfx = rbx_region("gfx1", &gsize), *prom = rbx_region("proms", &psize);
	if (!s_main_rom || s_main_size < 0x8000 || !s_audio_rom || !gfx || gsize < 0x40000 || !prom || psize < 0x300 ||
	    (s_has_sub && !s_sub_rom)) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	if (s_kind == KK_KIKIKAI || s_kind == KK_KICKNRUN) {
		uint32_t msize;
		uint8_t *mrom = rbx_region("mcu", &msize);
		if (!mrom || msize < 0x1000) {
			rbx_fail("%s: MCU ROM missing", g->set);
			return -1;
		}
		memset(&s_m6801, 0, sizeof(s_m6801));
		s_m6801.read = m6801_read;
		s_m6801.write = m6801_write;
		s_m6801.port_r = m6801_port_r;
		s_m6801.port_w = m6801_port_w;
		s_m6801.rom = mrom;
		s_m6801.rom_base = 0xf000;
		s_m6801.rom_size = 0x1000;
		s_m6801.type = M6801_TYPE_6801U4;
	} else if (s_kind != KK_KNIGHTBA) {
		uint32_t msize;
		uint8_t *mrom = rbx_region("mcu", &msize);
		memset(&s_m68705, 0, sizeof(s_m68705));
		if (!mrom || msize < 0x800 || m68705_init(&s_m68705, M68705_P3, mrom, 0x800)) {
			rbx_fail("%s: MCU ROM missing", g->set);
			return -1;
		}
		s_m68705.port_r[2] = m68705_port_r;
		s_m68705.port_w[0] = m68705_port_w;
		s_m68705.port_w[1] = m68705_port_w;
	}
	for (int i = 0; i < 256; i++)   // RGB_444_PROMS
		s_pal_rgb[i] = (uint32_t)((prom[i] & 15) * 17 << 16 | (prom[0x100 + i] & 15) * 17 << 8 | (prom[0x200 + i] & 15) * 17);
	// ROMREGION_INVERT, then charlayout: 8192 8x8 tiles, planes { 0x20000*8, +4, 0, 4 }, x { 3,2,1,0, 11,10,9,8 }
	for (uint32_t i = 0; i < gsize; i++)
		gfx[i] ^= 0xff;
	s_tiles = (uint8_t *)malloc(8192 * 64);
	if (!s_tiles) {
		kk_exit();
		return -1;
	}
	static const int xo[8] = { 3, 2, 1, 0, 11, 10, 9, 8 };
	const uint32_t planes[4] = { 0x20000 * 8, 0x20000 * 8 + 4, 0, 4 };
	for (uint32_t t = 0; t < 8192; t++)
		for (int y = 0; y < 8; y++)
			for (int x = 0; x < 8; x++) {
				int v = 0;
				for (int p = 0; p < 4; p++) {
					uint32_t b = t * 128 + planes[p] + (uint32_t)xo[x] + (uint32_t)y * 16;
					v = v << 1 | ((gfx[b >> 3] >> (7 - (b & 7))) & 1);
				}
				s_tiles[t * 64 + y * 8 + x] = (uint8_t)v;
			}
	static const char *const tags[8] = { "IN0", "IN1", "IN2", "IN3", "IN4", "IN5", "IN6", "IN7" };
	for (int i = 0; i < 8; i++)
		s_p_in[i] = rbx_port_find(tags[i]);
	s_p_dsw0 = rbx_port_find("DSW0");
	s_p_dsw1 = rbx_port_find("DSW1");
	rbx_ym_config yc;
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM2203;
	yc.clock = 3000000;
	yc.irq = ym_irq;
	yc.port_a_read = ym_pa;
	yc.port_b_read = ym_pb;
	yc.gain[0] = yc.gain[1] = yc.gain[2] = 77;   // 0.30
	yc.gain[3] = 256;
	if (rbx_ym_init(0, &yc)) {
		rbx_fail("%s: sound init failed", g->set);
		kk_exit();
		return -1;
	}
	s_mix_cap = (int)(rbx_ym_rate(0) / 50) + 64;
	s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	if (!s_mix || rbx_audio_setup(rbx_ym_rate(0)) || rbx_video_setup(SCREEN_W, VIS_Y1 - VIS_Y0 + 1, 512)) {
		kk_exit();
		return -1;
	}
	kk_reset();
	return 0;
}

static void kk_frame(void)
{
	for (int i = 0; i < LINES; i++) {
		int line = (VIS_Y1 + 1 + i) % LINES;
		if (line == VIS_Y1 + 1) {
			vblank_capture();
			if (s_kind != KK_MEXICO86 && s_kind != KK_KNIGHTB)
				s_main.irq = 1;    // kikikai_interrupt (ASSERT, cleared by the acknowledge) / irq0_line_hold
			// (mexico86 / knightb: the main IRQ comes only from the 68705)
			if (!s_audio.reset)
				s_audio.irq = 1;   // irq0_line_hold
			if (s_has_sub && !s_sub.reset)
				s_sub.irq = 1;
			if (s_kind == KK_KIKIKAI || s_kind == KK_KICKNRUN)
				m6801_set_irq_line(&s_m6801, M6801_IRQ1_LINE, 1);   // screen vblank line
			else if (s_kind != KK_KNIGHTBA && !s_m68705.st.in_reset)
				m68705_set_irq_line(&s_m68705, 1);   // (MAME skips vblank_int while the CPU is held in reset)
		} else if (line == VIS_Y0 && (s_kind == KK_KIKIKAI || s_kind == KK_KICKNRUN))
			m6801_set_irq_line(&s_m6801, M6801_IRQ1_LINE, 0);
		for (int s = 0; s < SLICES; s++) {
			s_ym_slice_start = s_ym_done;
			zcpu_run(&s_main, MAIN_LINE / SLICES, -1);
			s_cpu_start = s_audio.z.cyc;
			// the sound CPU takes the vblank IRQ (HOLD_LINE) and the YM2203 IRQ
			if (!s_audio.reset) {
				unsigned long end = s_audio.z.cyc + MAIN_LINE / SLICES;
				while ((long)(s_audio.z.cyc - end) < 0) {
					s_audio.z.int_pending = (s_audio.irq || s_ym_irq) ? 1 : 0;
					z80_step(&s_audio.z);
					if (s_audio.irq && !s_audio.z.int_pending)
						s_audio.irq = 0;
				}
			} else
				s_audio.z.cyc += MAIN_LINE / SLICES;
			if (s_has_sub)
				zcpu_run(&s_sub, SUB_LINE / SLICES, -1);
			if (s_kind == KK_KIKIKAI || s_kind == KK_KICKNRUN)
				m6801_execute(&s_m6801, M6801_LINE / SLICES);
			else if (s_kind != KK_KNIGHTBA)
				m68705_execute(&s_m68705, M68705_LINE / SLICES);
			ym_run_to(s_ym_slice_start + YM_LINE / SLICES);
		}
	}
	for (int i = 0; i < s_mix_n; i++)
		rbx_audio_put(s_mix[i]);
	memset(s_mix, 0, sizeof(int32_t) * (size_t)s_mix_n);
	s_mix_n = 0;
	s_ym_done = 0;
}

static int zcpu_state(void *ctx, const char *tag, zcpu *c, int load)
{
	zcpu t = *c;
	if (rbx_state_io(ctx, tag, &t, sizeof(t)))
		return -1;
	if (load) {
		t.z.read_byte = c->z.read_byte;
		t.z.write_byte = c->z.write_byte;
		t.z.port_in = c->z.port_in;
		t.z.port_out = c->z.port_out;
		t.z.userdata = c->z.userdata;
		*c = t;
	}
	return 0;
}

static int kk_state(void *ctx, int load)
{
	int rc = 0;
	rc |= zcpu_state(ctx, "main", &s_main, load);
	rc |= zcpu_state(ctx, "audio", &s_audio, load);
	rc |= zcpu_state(ctx, "sub", &s_sub, load);
	rc |= rbx_state_io(ctx, "mainram", s_mainram, sizeof(s_mainram));
	rc |= rbx_state_io(ctx, "shared", s_sharedram, sizeof(s_sharedram));
	rc |= rbx_state_io(ctx, "e900", s_ram_e900, sizeof(s_ram_e900));
	rc |= rbx_state_io(ctx, "subram", s_subram, sizeof(s_subram));
	rc |= rbx_state_io(ctx, "subwork", s_sub_work, sizeof(s_sub_work));
	rc |= rbx_state_io(ctx, "aram", s_audio_ram, sizeof(s_audio_ram));
	if (s_kind == KK_KIKIKAI || s_kind == KK_KICKNRUN)
		rc |= rbx_state_io(ctx, "m6801", m6801_state_data(&s_m6801), m6801_state_size());
	else if (s_kind != KK_KNIGHTBA)
		rc |= rbx_state_io(ctx, "m68705", &s_m68705.st, sizeof(s_m68705.st));
	rc |= rbx_ym_state(0, ctx, load);
	int32_t misc[16] = { s_bank, s_charbank, s_port1_out, s_port2_out, s_port3_out, s_port4_out, s_port3_in, s_pa_out,
		s_pb_out, s_latch, s_address, s_mcu_reset, s_ym_irq, 0, 0, 0 };
	rc |= rbx_state_io(ctx, "misc", misc, sizeof(misc));
	if (rc)
		return -1;
	if (load) {
		s_bank = misc[0]; s_charbank = misc[1]; s_port1_out = (uint8_t)misc[2]; s_port2_out = (uint8_t)misc[3];
		s_port3_out = (uint8_t)misc[4]; s_port4_out = (uint8_t)misc[5]; s_port3_in = (uint8_t)misc[6];
		s_pa_out = (uint8_t)misc[7]; s_pb_out = (uint8_t)misc[8]; s_latch = (uint8_t)misc[9]; s_address = (uint8_t)misc[10];
		s_mcu_reset = misc[11]; s_ym_irq = misc[12];
	}
	return 0;
}

static int kk_save(void *ctx) { return kk_state(ctx, 0); }
static int kk_load(void *ctx) { return kk_state(ctx, 1); }
static int kk_supported(const char *machine) { return kind_of(machine) >= 0; }

const rbx_driver rbx_drv_kikikai = { "KiKi KaiKai", kk_init, kk_exit, kk_reset, kk_frame, kk_save, kk_load, kk_supported, kk_render };
