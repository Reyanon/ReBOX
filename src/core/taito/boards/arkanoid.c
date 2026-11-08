/*  Arkanoid hardware (Arkanoid, Tournament Arkanoid, bootlegs, Hexa, Brixian, Tetris D.R. Korea), ported from MAME
    src/mame/taito/arkanoid.cpp, arkanoid_m.cpp and arkanoid_v.cpp (BSD-3-Clause, copyright-holders: Brad Oliver,
    Stephane Humbert). Memory maps, machine configurations, the bootleg protection kludges and the video follow MAME.
    MCU: MC68705P5 / P3 through machine/taito68705 (arkanoid_68705p5 / p3). Sound: AY-3-8910 / YM2149 (sound/ay8910).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/z80/z80.h"
#include "../machine/taito68705.h"
#include "../../sound/ay8910.h"

#include <stdlib.h>
#include <string.h>

enum { AK_MCU_P5, AK_MCU_P3, AK_MCU_P3_AY, AK_BOOTLEG, AK_BOOTLEG_AY, AK_CRUISIN5, AK_HEXA, AK_BRIXIAN };
enum { BL_NONE, BL_ARKANGC, BL_ARKANGC2, BL_ARKBLOCK, BL_ARKBLOC2, BL_ARKGCBL, BL_PADDLE2, BL_BLOCK2 };

typedef struct {
	const char *name;
	int kind;
} machine_t;

static const machine_t s_machines[] = {
	{ "arkanoid", AK_MCU_P5 },
	{ "p3mcu", AK_MCU_P3 },
	{ "p3mcuay", AK_MCU_P3_AY },
	{ "bootleg", AK_BOOTLEG },
	{ "aysnd", AK_BOOTLEG_AY },
	{ "cruisin5", AK_CRUISIN5 },
	{ "hexa", AK_HEXA },
	{ "brixian", AK_BRIXIAN },
};

// MAME set_raw(12 MHz / 2, 384, 0, 256, 264, 16, 240): the Z80 runs at the pixel clock
#define LINES 264
#define VIS_Y0 16
#define VIS_Y1 239
#define SCREEN_W 256
#define MAIN_LINE 384
#define AY_LINE 192        // 3 MHz input clock
#define MCU_LINE 48        // 3 MHz / 4
#define SLICES 2

static const machine_t *s_m;
static int s_has_mcu, s_bootleg_id, s_tetrsark;
static uint8_t *s_rom;
static uint32_t s_rom_size;
static uint8_t s_ram[0x800], s_vram[0x800], s_spr[0x7c0 + 0x40];   // c000-c7ff, e000-e7ff, e800-efff
static z80 s_z;
static int s_irq;
static int s_gfxbank, s_palbank, s_paddle_select, s_flipx, s_flipy, s_hexa_bank;
static uint8_t s_bootleg_cmd;
static taito68705_t s_mcu;
static ay8910_t s_ay;
static int s_dial[2];
// video
static uint8_t *s_tiles;
static uint32_t s_pal_rgb[512];
static int s_pal_n;
static uint8_t s_snap_vram[0x800], s_snap_spr[0x40];
static int s_snap_gfxbank, s_snap_palbank, s_snap_flipx, s_snap_flipy;
// ports
static int s_port_system, s_port_system2, s_port_buttons, s_port_dsw, s_port_unused, s_port_inputs;
// sound
static int32_t *s_mix;
static int s_mix_cap, s_mix_n;
static uint32_t s_ay_done, s_slice_start, s_slice_len;
static unsigned long s_cpu_start;

static uint8_t port8(int p) { return (uint8_t)rbx_port_read(p); }

// ---- sound ----
static void ay_run_to(uint32_t pos)
{
	if (pos <= s_ay_done)
		return;
	s_mix_n += ay8910_render(&s_ay, pos - s_ay_done, s_mix + s_mix_n, s_mix_cap - s_mix_n);
	s_ay_done = pos;
}
static void ay_sync(void)
{
	uint32_t done = (uint32_t)(s_z.cyc - s_cpu_start);
	if (done > MAIN_LINE / SLICES)
		done = MAIN_LINE / SLICES;
	ay_run_to(s_slice_start + s_slice_len * done / (MAIN_LINE / SLICES));
}
static uint8_t ay_port_a(void *u) { (void)u; return port8(s_m->kind >= AK_HEXA ? s_port_inputs : s_port_unused); }
static uint8_t ay_port_b(void *u) { (void)u; return port8(s_port_dsw); }

// ---- inputs ----
static uint8_t input_mux_r(void) { return (uint8_t)s_dial[s_paddle_select ? 1 : 0]; }
static uint8_t mcu_portb_r(void *u) { (void)u; return input_mux_r(); }

static uint8_t system_r(void)
{
	if (s_has_mcu) {
		// bits 6-7: host and MCU semaphores, active low
		int v = (taito68705_host_semaphore_r(&s_mcu) ? 0 : 1) | (taito68705_mcu_semaphore_r(&s_mcu) ? 0 : 2);
		rbx_port_custom(s_port_system, 0xc0, (uint32_t)v << 6);
	}
	return port8(s_port_system);
}

// ---- bootleg kludges (MAME arkanoid_m.cpp) ----
static uint8_t bootleg_f000_r(void)
{
	if (s_bootleg_id == BL_BLOCK2 && (s_bootleg_cmd == 0x05 || s_bootleg_cmd == 0x0a))
		return s_bootleg_cmd;
	return 0;
}

static uint8_t bootleg_f002_r(void)
{
	if (s_bootleg_id == BL_ARKGCBL) {
		switch (s_bootleg_cmd) {
		case 0x8a: return 0xa5;
		case 0xff: return 0xe2;
		}
	} else if (s_bootleg_id == BL_PADDLE2) {
		switch (s_bootleg_cmd) {
		case 0x24: return 0x9b;
		case 0x36: return 0x2d;
		case 0x38: return 0xf3;
		case 0x8a: return 0xa5;
		case 0xc3: return 0x1d;
		case 0xe3: return 0x61;
		case 0xf7: return 0x00;
		case 0xff: return 0xe2;
		}
	}
	return 0;
}

static void bootleg_d018_w(uint8_t data)
{
	// only the commands MAME keeps (cmd = data); every other write clears the command
	const uint16_t pc = s_z.pc;
	s_bootleg_cmd = 0;
	switch (s_bootleg_id) {
	case BL_BLOCK2:
		if ((data == 0x05 && pc == 0x0363) || (data == 0x0a && pc == 0x0372))
			s_bootleg_cmd = data;
		break;
	case BL_ARKGCBL:
		if ((data == 0x8a && pc == 0x9661) || (data == 0xff && pc == 0x9670))
			s_bootleg_cmd = data;
		break;
	case BL_PADDLE2:
		if ((data == 0x24 && pc == 0xbd7a) || (data == 0x36 && pc == 0x7c4c) || (data == 0x38 && pc == 0x7b87) ||
		    (data == 0x8a && pc == 0x9661) || (data == 0xc3 && pc == 0xbd8a) || (data == 0xe3 && pc == 0x67e9) ||
		    (data == 0xf7 && pc == 0x0349) || (data == 0xff && pc == 0x9670))
			s_bootleg_cmd = data;
		break;
	}
}

static uint8_t bootleg_d008_r(void)
{
	uint8_t paddle = input_mux_r(), v = 0;
	switch (s_bootleg_id) {
	case BL_ARKANGC2:
	case BL_BLOCK2:
		v = 0x02;
		break;
	case BL_ARKBLOC2:
		v = paddle < 0x40 ? 0x20 : 0;
		break;
	case BL_ARKGCBL:
		v = (uint8_t)(0x02 | (paddle < 0x40 ? 0x20 : 0));
		break;
	case BL_PADDLE2:
		v = (uint8_t)(0x0f | (paddle < 0x40 ? 0x20 : 0));
		break;
	}
	return v;
}

// ---- Z80 read map: pages read straight from memory (the same bytes main_read returns) ----
static const uint8_t *s_map[256];

static void map_range(uint32_t start, uint32_t end, const uint8_t *mem)
{
	for (uint32_t a = start; a < end; a += 0x100)
		s_map[a >> 8] = mem ? mem + (a - start) : NULL;
}

static void update_map(void)
{
	memset(s_map, 0, sizeof(s_map));
	for (uint32_t a = 0; a < 0xc000; a += 0x100) {
		uint32_t o = a;
		if (a >= 0x8000 && s_m->kind == AK_HEXA)
			o = 0x10000u + 0x4000u * (uint32_t)s_hexa_bank + (a - 0x8000u);
		if (o + 0x100 <= s_rom_size)
			s_map[a >> 8] = s_rom + o;
	}
	map_range(0xc000, 0xc800, s_ram);
	if (s_m->kind <= AK_MCU_P3_AY)
		map_range(0xc800, 0xd000, s_ram);
	map_range(0xe000, 0xe800, s_vram);
	map_range(0xe800, 0xf000, s_spr);
	s_z.read_map = s_map;
}

// ---- main CPU ----
static void d008_w(uint8_t data)
{
	s_flipx = data & 1;
	s_flipy = (data >> 1) & 1;
	if (s_m->kind == AK_HEXA) {
		if (s_hexa_bank != ((data >> 4) & 1)) {
			s_hexa_bank = (data >> 4) & 1;
			update_map();
		}
		s_gfxbank = (data >> 5) & 1;
		return;
	}
	if (s_m->kind != AK_BRIXIAN)
		s_paddle_select = data & 4;
	s_gfxbank = (data >> 5) & 1;
	s_palbank = (data >> 6) & 1;
	if (s_has_mcu && !s_tetrsark)
		taito68705_reset_w(&s_mcu, !(data & 0x80));
}

static uint8_t main_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x8000) return a < s_rom_size ? s_rom[a] : 0xff;
	if (a < 0xc000) {
		if (s_m->kind == AK_HEXA) {
			uint32_t o = 0x10000u + 0x4000u * (uint32_t)s_hexa_bank + (a - 0x8000u);
			return o < s_rom_size ? s_rom[o] : 0xff;
		}
		return a < s_rom_size ? s_rom[a] : 0xff;
	}
	if (a < 0xd000) {
		if (a < 0xc800 || s_m->kind <= AK_MCU_P3_AY)   // (mirror 0x800 on the MCU boards)
			return s_ram[a & 0x7ff];
		return 0;
	}
	if (a < 0xe000) {
		if (s_m->kind >= AK_HEXA)
			return a == 0xd001 ? (ay_sync(), ay8910_data_r(&s_ay)) : 0;
		if (s_m->kind <= AK_MCU_P3_AY) {
			switch (a & 0x1c) {
			case 0x00:
			case 0x04:
				if (a & 1) { ay_sync(); return ay8910_data_r(&s_ay); }
				return 0;
			case 0x08: return port8(s_port_system2);
			case 0x0c: return system_r();
			case 0x10:
			case 0x14: return port8(s_port_buttons);
			default: return taito68705_data_r(&s_mcu);
			}
		}
		switch (a) {   // bootleg map
		case 0xd001: ay_sync(); return ay8910_data_r(&s_ay);
		case 0xd008: return s_bootleg_id != BL_NONE ? bootleg_d008_r() : 0;
		case 0xd00c: return port8(s_port_system);
		case 0xd010: return port8(s_port_buttons);
		case 0xd018: return input_mux_r();
		}
		return 0;
	}
	if (a < 0xe800) return s_vram[a - 0xe000];
	if (a < 0xf000) return s_spr[a - 0xe800];
	if (s_bootleg_id != BL_NONE) {
		if (a == 0xf000) return bootleg_f000_r();
		if (a == 0xf002) return bootleg_f002_r();
	}
	return 0;
}

static void main_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a < 0xc000) return;
	if (a < 0xd000) {
		if (a < 0xc800 || s_m->kind <= AK_MCU_P3_AY)
			s_ram[a & 0x7ff] = v;
		return;
	}
	if (a < 0xe000) {
		if (s_m->kind >= AK_HEXA) {
			if (a == 0xd000 || a == 0xd001) { ay_sync(); ay8910_write(&s_ay, a & 1, v); }
			else if (a == 0xd008) d008_w(v);
			return;
		}
		if (s_m->kind <= AK_MCU_P3_AY) {
			switch (a & 0x18) {
			case 0x00: ay_sync(); ay8910_write(&s_ay, a & 1, v); return;
			case 0x08: d008_w(v); return;
			case 0x10: return;   // watchdog
			default: taito68705_data_w(&s_mcu, v); return;
			}
		}
		switch (a) {   // bootleg map
		case 0xd000:
		case 0xd001: ay_sync(); ay8910_write(&s_ay, a & 1, v); return;
		case 0xd008: d008_w(v); return;
		case 0xd018: if (s_bootleg_id != BL_NONE) bootleg_d018_w(v); return;
		}
		return;
	}
	if (a < 0xe800) { s_vram[a - 0xe000] = v; return; }
	if (a < 0xf000) { s_spr[a - 0xe800] = v; return; }
}

static uint8_t no_in(z80 *z, uint8_t p) { (void)z; (void)p; return 0xff; }
static void no_out(z80 *z, uint8_t p, uint8_t v) { (void)z; (void)p; (void)v; }

// ---- video ----
static void ak_render(void)
{
	memcpy(rbx_video.palette, s_pal_rgb, sizeof(uint32_t) * (size_t)s_pal_n);
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch, h = rbx_video.height;
	const int fx = s_snap_flipx, fy = s_snap_flipy;
	// background: 32x32 tiles, scan rows; MAME flips the whole tilemap with the screen
	for (int row = 0; row < 32; row++) {
		int sy = fy ? (31 - row) * 8 : row * 8;
		if (sy + 8 <= VIS_Y0 || sy > VIS_Y1)
			continue;
		for (int col = 0; col < 32; col++) {
			int offs = (row * 32 + col) * 2;
			int code = s_snap_vram[offs + 1] + ((s_snap_vram[offs] & 0x07) << 8) + 2048 * s_snap_gfxbank;
			int color = ((s_snap_vram[offs] & 0xf8) >> 3) + 32 * s_snap_palbank;
			const uint8_t *src = s_tiles + (code & 0xfff) * 64;
			uint16_t base = (uint16_t)(color * 8);
			int sx = fx ? (31 - col) * 8 : col * 8;
			for (int y = 0; y < 8; y++) {
				int dy = sy + y - VIS_Y0;
				if (dy < 0 || dy >= h)
					continue;
				const uint8_t *r = src + (fy ? 7 - y : y) * 8;
				uint16_t *d = bm + dy * pitch + sx;
				for (int x = 0; x < 8; x++)
					d[x] = (uint16_t)(base + r[fx ? 7 - x : x]);
			}
		}
	}
	if (s_m->kind >= AK_HEXA)
		return;   // MAME screen_update_hexa: no sprites
	for (int offs = 0; offs < 0x40; offs += 4) {
		int sx = s_snap_spr[offs], sy = 248 - s_snap_spr[offs + 1];
		if (fx) sx = 248 - sx;
		if (fy) sy = 248 - sy;
		int code = s_snap_spr[offs + 3] + ((s_snap_spr[offs + 2] & 0x03) << 8) + 1024 * s_snap_gfxbank;
		int color = ((s_snap_spr[offs + 2] & 0xf8) >> 3) + 32 * s_snap_palbank;
		for (int half = 0; half < 2; half++) {
			int c = (2 * code + half) & 0xfff;
			int ty = half ? sy : sy + (fy ? 8 : -8);
			const uint8_t *src = s_tiles + c * 64;
			uint16_t base = (uint16_t)(color * 8);
			for (int y = 0; y < 8; y++) {
				int dy = ty + y - VIS_Y0;
				if (dy < 0 || dy >= h)
					continue;
				const uint8_t *r = src + (fy ? 7 - y : y) * 8;
				uint16_t *d = bm + dy * pitch;
				for (int x = 0; x < 8; x++) {
					int dx = sx + x;
					uint8_t p = r[fx ? 7 - x : x];
					if (p && dx >= 0 && dx < SCREEN_W)
						d[dx] = (uint16_t)(base + p);
				}
			}
		}
	}
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap_vram, s_vram, sizeof(s_vram));
	memcpy(s_snap_spr, s_spr, sizeof(s_snap_spr));
	s_snap_gfxbank = s_gfxbank;
	s_snap_palbank = s_palbank;
	s_snap_flipx = s_flipx;
	s_snap_flipy = s_flipy;
	rbx_video_frame_done();
}

// ---- init / frame ----
static const machine_t *find_machine(const char *name)
{
	for (size_t i = 0; i < sizeof(s_machines) / sizeof(s_machines[0]); i++)
		if (!strcmp(s_machines[i].name, name))
			return &s_machines[i];
	return NULL;
}

static uint8_t *s_protdata;

static void ak_reset(void)
{
	// (first: the sound syncs read them, from a CPU write or a port callback of a chip reset below, before a frame
	// sets them; the last game's values made a render of billions of clocks, the next game of the board hung)
	s_slice_start = s_slice_len = s_cpu_start = 0;
	memset(s_ram, 0, sizeof(s_ram));
	memset(s_vram, 0, sizeof(s_vram));
	memset(s_spr, 0, sizeof(s_spr));
	if (s_m->kind == AK_BRIXIAN && s_protdata)
		memcpy(s_ram + 0x600, s_protdata, 0x200);   // MAME init_brixian
	z80_init(&s_z);
	s_z.read_byte = main_read;
	s_z.write_byte = main_write;
	s_z.port_in = no_in;
	s_z.port_out = no_out;
	s_z.int_data = 0xff;
	s_irq = 0;
	s_gfxbank = s_palbank = s_paddle_select = s_flipx = s_flipy = s_hexa_bank = 0;
	update_map();
	s_bootleg_cmd = 0;
	if (s_has_mcu)
		taito68705_reset(&s_mcu);
	ay8910_reset(&s_ay);
	s_ay_done = 0;
	s_mix_n = 0;
}

static void ak_exit(void)
{
	free(s_tiles);
	s_tiles = NULL;
	free(s_mix);
	s_mix = NULL;
}

static int bootleg_id(const char *set)
{
	static const struct { const char *set; int id; } t[] = {
		{ "arkangc", BL_ARKANGC }, { "arkaboot", BL_ARKANGC }, { "arkangc2", BL_ARKANGC2 }, { "arkblock", BL_ARKBLOCK },
		{ "arkbloc2", BL_ARKBLOC2 }, { "arkbloc3", BL_BLOCK2 }, { "block2", BL_BLOCK2 }, { "arkgcbl", BL_ARKGCBL },
		{ "arkgcbla", BL_ARKGCBL }, { "paddle2", BL_PADDLE2 },
	};
	for (size_t i = 0; i < sizeof(t) / sizeof(t[0]); i++)
		if (!strcmp(t[i].set, set))
			return t[i].id;
	return BL_NONE;
}

static int ak_init(const rbx_game *g)
{
	s_m = find_machine(g->machine);
	if (!s_m) {
		rbx_fail("%s: machine %s is not emulated", g->set, g->machine);
		return -1;
	}
	uint32_t gsize = 0, psize = 0;
	s_rom = rbx_region("maincpu", &s_rom_size);
	uint8_t *gfx = rbx_region("gfx1", &gsize);
	uint8_t *prom = rbx_region("proms", &psize);
	if (!s_rom || !gfx || !prom || gsize < 0x18000 || psize < 0x300) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	s_bootleg_id = bootleg_id(g->set);
	s_tetrsark = !strncmp(g->set, "tetrsark", 8);
	if (!strcmp(g->set, "tetrsark"))   // MAME init_tetrsark
		for (uint32_t i = 0; i < 0x8000 && i < s_rom_size; i++)
			s_rom[i] ^= 0x94;
	if (s_bootleg_id == BL_BLOCK2) {   // MAME init_block2: unscramble the tiles
		uint8_t *buf = (uint8_t *)malloc(0x18000);
		if (!buf) {
			rbx_fail("%s: out of memory", g->set);
			return -1;
		}
		for (int tile = 0; tile < 0x3000; tile++) {
			int t = tile, s = 0;
			static const int sw1[16] = { 0, 4, 2, 1, 3, 6, 5, 7, 8, 9, 10, 11, 12, 13, 14, 15 };   // bit sources (LSB first)
			static const int sw2[16] = { 0, 1, 2, 3, 4, 8, 6, 7, 5, 10, 9, 11, 12, 13, 14, 15 };
			for (int b = 0; b < 16; b++) s |= ((t >> sw1[b]) & 1) << b;
			t = s; s = 0;
			for (int b = 0; b < 16; b++) s |= ((t >> sw2[b]) & 1) << b;
			s ^= 0xd4;
			memcpy(buf + tile * 8, gfx + s * 8, 8);
		}
		memcpy(gfx, buf, 0x18000);
		free(buf);
	}
	s_has_mcu = s_m->kind <= AK_MCU_P3_AY;
	if (s_has_mcu) {
		uint32_t msize;
		uint8_t *mrom = rbx_region("mcu:mcu", &msize);
		memset(&s_mcu, 0, sizeof(s_mcu));
		s_mcu.portb_r_cb = mcu_portb_r;
		if (!mrom || msize < 0x800 ||
		    taito68705_init(&s_mcu, s_m->kind == AK_MCU_P5 ? TAITO68705_ARK_P5 : TAITO68705_ARK_P3, mrom, 0x800)) {
			rbx_fail("%s: MCU ROM missing", g->set);
			return -1;
		}
	}
	s_protdata = NULL;
	if (s_m->kind == AK_BRIXIAN) {
		uint32_t sz;
		s_protdata = rbx_region("protdata", &sz);
		if (!s_protdata || sz < 0x200) {
			rbx_fail("%s: ROM regions missing", g->set);
			return -1;
		}
	}
	// charlayout: 4096 8x8 tiles, 3 bpp, planes { 2*4096*64, 4096*64, 0 } bits, 8 bytes a tile
	s_tiles = (uint8_t *)malloc(4096 * 64);
	if (!s_tiles) {
		rbx_fail("%s: out of memory (graphics)", g->set);
		return -1;
	}
	for (int t = 0; t < 4096; t++)
		for (int y = 0; y < 8; y++) {
			uint8_t p0 = gfx[2 * 4096 * 8 + t * 8 + y], p1 = gfx[4096 * 8 + t * 8 + y], p2 = gfx[t * 8 + y];
			for (int x = 0; x < 8; x++) {
				int b = 7 - x;
				s_tiles[t * 64 + y * 8 + x] = (uint8_t)(((p0 >> b) & 1) << 2 | ((p1 >> b) & 1) << 1 | ((p2 >> b) & 1));
			}
		}
	// RGB_444_PROMS
	s_pal_n = (int)(psize / 3);
	if (s_pal_n > 512)
		s_pal_n = 512;
	for (int i = 0; i < s_pal_n; i++) {
		int r = prom[i] & 15, gg = prom[i + s_pal_n] & 15, b = prom[i + 2 * s_pal_n] & 15;
		s_pal_rgb[i] = (uint32_t)((r * 17) << 16 | (gg * 17) << 8 | (b * 17));
	}
	s_port_system = rbx_port_find("SYSTEM");
	s_port_system2 = rbx_port_find("SYSTEM2");
	s_port_buttons = rbx_port_find("BUTTONS");
	s_port_dsw = rbx_port_find("DSW");
	s_port_unused = rbx_port_find("UNUSED");
	s_port_inputs = rbx_port_find("INPUTS");
	// sound: YM2149 with pin 26 low (clock / 16) on the original boards, AY-3-8910 (clock / 8) on others; MAME
	// route 0.66 (0.50 on hexa / brixian), DC removed
	int ym = s_m->kind == AK_MCU_P5 || s_m->kind == AK_MCU_P3 || s_m->kind == AK_BOOTLEG || s_m->kind == AK_CRUISIN5;
	int gain = s_m->kind >= AK_HEXA ? 128 : 169;
	if (ym)
		ay8910_init(&s_ay, AY8910_TYPE_YM2149, 3000000, AY8910_SINGLE_OUTPUT | YM2149_PIN26_LOW, gain, 4, 1);
	else
		ay8910_init(&s_ay, AY8910_TYPE_AY8910, 3000000, s_m->kind >= AK_HEXA ? AY8910_LEGACY_OUTPUT : AY8910_SINGLE_OUTPUT,
		            gain, 8, 1);
	s_ay.port_a_read = ay_port_a;
	s_ay.port_b_read = ay_port_b;
	s_mix_cap = (int)(ay8910_rate(&s_ay) / 50) + 64;
	s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	if (!s_mix || rbx_audio_setup(ay8910_rate(&s_ay)) || rbx_video_setup(SCREEN_W, VIS_Y1 - VIS_Y0 + 1, s_pal_n)) {
		ak_exit();
		return -1;
	}
	s_dial[0] = s_dial[1] = 0;
	ak_reset();
	return 0;
}

static void ak_frame(void)
{
	s_dial[0] = (s_dial[0] + rbx_analog_dpad(1, RBX_IPT_DIAL, 5)) & 0xff;   // (d-pad: MAME key delta 15 x sensitivity 30%)
	s_dial[1] = (s_dial[1] + rbx_analog_dpad(2, RBX_IPT_DIAL, 5)) & 0xff;
	const int irq_line = s_m->kind == AK_CRUISIN5 ? VIS_Y0 : VIS_Y1 + 1;   // cruisin5: IRQ at vblank end
	// the frame starts at vblank (MAME updates the inputs there)
	for (int i = 0; i < LINES; i++) {
		int line = (VIS_Y1 + 1 + i) % LINES;
		if (line == VIS_Y1 + 1)
			vblank_capture();
		if (line == irq_line)
			s_irq = 1;
		for (int s = 0; s < SLICES; s++) {
			s_slice_start = s_ay_done;
			s_slice_len = AY_LINE / SLICES;
			s_cpu_start = s_z.cyc;
			unsigned long end = s_z.cyc + MAIN_LINE / SLICES;
			z80_run(&s_z, end, &s_irq, 1);
			if (s_has_mcu)
				taito68705_execute(&s_mcu, MCU_LINE / SLICES);   // (the core carries its overshoot)
			ay_run_to(s_slice_start + s_slice_len);
		}
	}
	for (int i = 0; i < s_mix_n; i++)
		rbx_audio_put(s_mix[i]);
	memset(s_mix, 0, sizeof(int32_t) * (size_t)s_mix_n);
	s_mix_n = 0;
	s_ay_done = 0;
}

static int ak_state(void *ctx, int load)
{
	z80 z = s_z;
	int rc = 0;
	rc |= rbx_state_io(ctx, "z80", &z, sizeof(z));
	rc |= rbx_state_io(ctx, "ram", s_ram, sizeof(s_ram));
	rc |= rbx_state_io(ctx, "vram", s_vram, sizeof(s_vram));
	rc |= rbx_state_io(ctx, "spr", s_spr, sizeof(s_spr));
	rc |= rbx_state_io(ctx, "ay", &s_ay, ay8910_state_size());
	if (s_has_mcu) {
		rc |= rbx_state_io(ctx, "mcu", &s_mcu.st, sizeof(s_mcu.st));
		rc |= rbx_state_io(ctx, "mcu6805", &s_mcu.mcu.st, sizeof(s_mcu.mcu.st));
	}
	int32_t misc[12] = { s_irq, s_gfxbank, s_palbank, s_paddle_select, s_flipx, s_flipy, s_hexa_bank, s_bootleg_cmd,
		0, s_dial[0], s_dial[1], 0 };
	rc |= rbx_state_io(ctx, "misc", misc, sizeof(misc));
	if (rc)
		return -1;
	if (load) {
		z.read_byte = main_read;
		z.write_byte = main_write;
		z.port_in = no_in;
		z.port_out = no_out;
		z.userdata = NULL;
		s_z = z;
		s_irq = misc[0]; s_gfxbank = misc[1]; s_palbank = misc[2]; s_paddle_select = misc[3];
		s_flipx = misc[4]; s_flipy = misc[5]; s_hexa_bank = misc[6]; s_bootleg_cmd = (uint8_t)misc[7];
		update_map();
		s_dial[0] = misc[9]; s_dial[1] = misc[10];
	}
	return 0;
}

static int ak_save(void *ctx) { return ak_state(ctx, 0); }
static int ak_load(void *ctx) { return ak_state(ctx, 1); }
static int ak_supported(const char *machine) { return find_machine(machine) != NULL; }

const rbx_driver rbx_drv_arkanoid = { "Arkanoid", ak_init, ak_exit, ak_reset, ak_frame, ak_save, ak_load, ak_supported, ak_render };
