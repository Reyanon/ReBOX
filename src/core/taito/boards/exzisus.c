/*  Exzisus hardware, ported from MAME src/mame/taito/exzisus.cpp (BSD-3-Clause, copyright-holders: Yochizo;
    thanks-to: Richard Bush). Memory maps, machine configuration and the video follow MAME.
    Four Z80s: cpua (banked ROM, object layer 1 shared with cpuc), cpub (banked ROM, object layer 0, inputs,
    PC060HA master side), cpuc (object layer 1 work CPU, reset pulsed by cpua), audiocpu (YM2151 + PC060HA through
    machine/taito_sound, TSND_MAP_9000 with a flat ROM). All three main CPUs take irq0_line_hold at vblank.
    Video: two TC0010VCU object layers, each made of 16x16 sprites and 16x256 tile columns (as Bubble Bobble),
    over pen 1023. Interleave: one scanline (MAME: 600 Hz quantum).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/z80/z80.h"
#include "../machine/taito_sound.h"

#include <stdlib.h>
#include <string.h>

#define LINES 256
#define VIS_Y0 16
#define VIS_Y1 239
#define SCREEN_W 256
#define MAIN_CLOCK 6000000

typedef struct {
	z80 z;
	int irq;
} cpu_t;

static cpu_t s_cpu[3];   // a, b, c
static uint8_t *s_rom[3];
static uint32_t s_rom_size[3];
static int s_bank[2];
static uint8_t s_vram[2][0x2000];   // per layer: objectram 0000-05ff, videoram 0600-1fff
static uint8_t s_ram_ac[0x1000], s_ram_ab[0x800], s_ram_b[0x1000], s_ram_c[0x1000];
static int s_flip;
static uint64_t s_frac;
static int s_cpuc_reset;
// video
static uint8_t *s_tiles[2];   // 16384 8x8 tiles per layer, 1 byte per pixel
static uint8_t *s_tile_used[2];
static uint32_t s_pal[1024];
static uint8_t s_snap[2][0x2000];
static int s_snap_flip;
static int s_port[5];   // P1, P2, SYSTEM, DSWA, DSWB

static uint8_t port8(int i) { return s_port[i] >= 0 ? (uint8_t)rbx_port_read(s_port[i]) : 0xff; }

static void bank_w(int which, uint8_t data)
{
	s_bank[which] = data & 0x0f;
	s_flip = (data & 0x40) ? 1 : 0;
}

static uint8_t rom_r(int c, uint16_t a)
{
	if (a < 0x8000)
		return a < s_rom_size[c] ? s_rom[c][a] : 0;
	uint32_t o = 0x4000u * (uint32_t)s_bank[c] + (a - 0x8000u);   // (bank entries cover the whole region from 0)
	return o < s_rom_size[c] ? s_rom[c][o] : 0;
}

// ---- cpua ----
static uint8_t a_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0xc000) return rom_r(0, a);
	if (a < 0xe000) return s_vram[1][a - 0xc000];
	if (a < 0xf000) return s_ram_ac[a - 0xe000];
	if (a >= 0xf800) return s_ram_ab[a - 0xf800];
	return 0;
}

static void z80_reset_cpu(int i);

static void a_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a < 0xc000) return;
	if (a < 0xe000) { s_vram[1][a - 0xc000] = v; return; }
	if (a < 0xf000) { s_ram_ac[a - 0xe000] = v; return; }
	if (a >= 0xf800) { s_ram_ab[a - 0xf800] = v; return; }
	if (a == 0xf400) bank_w(0, v);
	else if (a == 0xf404) s_cpuc_reset = 1;   // pulse_input_line(INPUT_LINE_RESET): reset cpuc
}

// ---- cpub ----
static uint8_t b_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0xc000) return rom_r(1, a);
	if (a < 0xe000) return s_vram[0][a - 0xc000];
	if (a < 0xf000) return s_ram_b[a - 0xe000];
	if (a >= 0xf800) return s_ram_ab[a - 0xf800];
	switch (a) {
	case 0xf001: return tc0140syt_master_comm_r(tsnd_syt());
	case 0xf400: return port8(0);
	case 0xf401: return port8(1);
	case 0xf402: return port8(2);
	case 0xf404: return port8(3);
	case 0xf405: return port8(4);
	}
	return 0;
}

static void b_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a < 0xc000) return;
	if (a < 0xe000) { s_vram[0][a - 0xc000] = v; return; }
	if (a < 0xf000) { s_ram_b[a - 0xe000] = v; return; }
	if (a >= 0xf800) { s_ram_ab[a - 0xf800] = v; return; }
	switch (a) {
	case 0xf000: tc0140syt_master_port_w(tsnd_syt(), v); return;
	case 0xf001: tc0140syt_master_comm_w(tsnd_syt(), v); return;
	case 0xf400: bank_w(1, v); return;
	}   // (f402: coin lockout / counters)
}

// ---- cpuc ----
static uint8_t c_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x8000) return a < s_rom_size[2] ? s_rom[2][a] : 0;
	if (a < 0xa000) return s_vram[1][a - 0x8000];
	if (a < 0xb000) return s_ram_ac[a - 0xa000];
	if (a < 0xc000) return s_ram_c[a - 0xb000];
	return 0;
}

static void c_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a < 0x8000) return;
	if (a < 0xa000) { s_vram[1][a - 0x8000] = v; return; }
	if (a < 0xb000) { s_ram_ac[a - 0xa000] = v; return; }
	if (a < 0xc000) { s_ram_c[a - 0xb000] = v; return; }
}

static uint8_t no_in(z80 *z, uint8_t p) { (void)z; (void)p; return 0; }
static void no_out(z80 *z, uint8_t p, uint8_t v) { (void)z; (void)p; (void)v; }

static void set_handlers(int i)
{
	static uint8_t (*const rd[3])(void *, uint16_t) = { a_read, b_read, c_read };
	static void (*const wr[3])(void *, uint16_t, uint8_t) = { a_write, b_write, c_write };
	z80 *z = &s_cpu[i].z;
	z->read_byte = rd[i];
	z->write_byte = wr[i];
	z->port_in = no_in;
	z->port_out = no_out;
	z->userdata = NULL;
}

static void z80_reset_cpu(int i)
{
	unsigned long cyc = s_cpu[i].z.cyc;
	z80_init(&s_cpu[i].z);
	s_cpu[i].z.cyc = cyc;
	s_cpu[i].z.int_data = 0xff;
	s_cpu[i].irq = 0;
	set_handlers(i);
}

// ---- video ----
static void ex_render(void)
{
	memcpy(rbx_video.palette, s_pal, sizeof(s_pal));
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch, h = VIS_Y1 - VIS_Y0 + 1;
	for (int y = 0; y < h; y++)
		for (int x = 0; x < SCREEN_W; x++)
			bm[y * pitch + x] = 1023;
	const int flip = s_snap_flip;
	for (int j = 0; j < 2; j++) {
		const uint8_t *obj = s_snap[j], *vram = s_snap[j] + 0x600;
		const uint8_t *tiles = s_tiles[j], *used = s_tile_used[j];
		const int cbase = j * 256;
		int sx = 0;
		for (int offs = 0; offs < 0x600; offs += 4) {
			if (!(obj[offs] | obj[offs + 1] | obj[offs + 2] | obj[offs + 3]))
				continue;
			int gfx_num = obj[offs + 1], gfx_attr = obj[offs + 3], gfx_offs, height;
			if (!(gfx_num & 0x80)) {   // 16x16 sprite
				gfx_offs = (gfx_num & 0x7f) << 3;
				height = 2;
				sx = obj[offs + 2] | (gfx_attr & 0x40) << 2;
			} else {                   // 16x256 tile column
				gfx_offs = ((gfx_num & 0x3f) << 7) + 0x0400;
				height = 32;
				if (gfx_num & 0x40)
					sx += 16;
				else
					sx = obj[offs + 2] | (gfx_attr & 0x40) << 2;
			}
			int sy = 256 - (height << 3) - obj[offs];
			for (int xc = 0; xc < 2; xc++) {
				int goffs = gfx_offs;
				for (int yc = 0; yc < height; yc++) {
					// (videoram is c600-dfff: offsets past it read 0 here)
					int lo = goffs < 0x1a00 ? vram[goffs] : 0, hi = goffs + 1 < 0x1a00 ? vram[goffs + 1] : 0;
					int code = ((hi << 8) | lo) & 0x3fff;
					int color = (hi >> 6) | (gfx_attr & 0x0f);
					int x = (sx + (xc << 3)) & 0xff, y = (sy + (yc << 3)) & 0xff;
					goffs += 2;
					if (!used[code])
						continue;
					if (flip) {
						x = 248 - x;
						y = 248 - y;
					}
					const uint8_t *src = tiles + code * 64;
					const uint16_t base = (uint16_t)(cbase + color * 16);
					for (int py = 0; py < 8; py++) {
						int dy = y + py - VIS_Y0;
						if (dy < 0 || dy >= h)
							continue;
						const uint8_t *r = src + (flip ? 7 - py : py) * 8;
						uint16_t *d = bm + dy * pitch;
						for (int px = 0; px < 8; px++) {
							int dx = x + px;
							uint8_t p = r[flip ? 7 - px : px];
							if (p != 15 && dx >= 0 && dx < SCREEN_W)
								d[dx] = (uint16_t)(base + p);
						}
					}
				}
				gfx_offs += height << 1;
			}
		}
	}
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap, s_vram, sizeof(s_snap));
	s_snap_flip = s_flip;
	rbx_video_frame_done();
}

// charlayout: 8x8, 4 planes { RGN_FRAC(1,2), RGN_FRAC(1,2)+4, 0, 4 }, x bits { 3,2,1,0, 11,10,9,8 }, 16 bits a row
static uint8_t *decode_tiles(const uint8_t *g, uint32_t size, uint8_t **used)
{
	const uint32_t half = size / 2, n = half / 16;
	uint8_t *d = (uint8_t *)malloc((size_t)n * 64);
	*used = (uint8_t *)calloc(n, 1);
	if (!d || !*used)
		return d;
	for (uint32_t t = 0; t < n; t++)
		for (int y = 0; y < 8; y++)
			for (int x = 0; x < 8; x++) {
				uint32_t o = t * 16 + (uint32_t)y * 2 + (x >> 2);
				int b = x & 3;
				uint8_t hi = g[half + o], lo = g[o];
				int p = ((hi >> (4 + b)) & 1) << 3 | ((hi >> b) & 1) << 2 | ((lo >> (4 + b)) & 1) << 1 | ((lo >> b) & 1);
				d[t * 64 + y * 8 + x] = (uint8_t)p;
				if (p != 15)
					(*used)[t] = 1;
			}
	return d;
}

// ---- init / frame ----
static void ex_reset(void)
{
	memset(s_vram, 0, sizeof(s_vram));
	memset(s_ram_ac, 0, sizeof(s_ram_ac));
	memset(s_ram_ab, 0, sizeof(s_ram_ab));
	memset(s_ram_b, 0, sizeof(s_ram_b));
	memset(s_ram_c, 0, sizeof(s_ram_c));
	for (int i = 0; i < 3; i++) {
		memset(&s_cpu[i], 0, sizeof(s_cpu[i]));
		z80_reset_cpu(i);
	}
	s_bank[0] = s_bank[1] = 0;
	s_flip = 0;
	s_frac = 0;
	s_cpuc_reset = 0;
	tsnd_reset();
}

static void ex_exit(void)
{
	for (int j = 0; j < 2; j++) {
		free(s_tiles[j]);
		free(s_tile_used[j]);
		s_tiles[j] = s_tile_used[j] = NULL;
	}
	tsnd_exit();
}

static int ex_init(const rbx_game *g)
{
	static const char *const cpus[3] = { "cpua", "cpub", "cpuc" };
	for (int i = 0; i < 3; i++)
		s_rom[i] = rbx_region(cpus[i], &s_rom_size[i]);
	uint32_t gs[2] = { 0, 0 }, psize = 0;
	uint8_t *gfx[2] = { rbx_region("bg0", &gs[0]), rbx_region("bg1", &gs[1]) };
	uint8_t *prom = rbx_region("proms", &psize);
	if (!s_rom[0] || !s_rom[1] || !s_rom[2] || !gfx[0] || !gfx[1] || !prom || psize < 0xc00) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	for (int j = 0; j < 2; j++) {
		for (uint32_t i = 0; i < gs[j]; i++)   // ROMREGION_INVERT
			gfx[j][i] ^= 0xff;
		s_tiles[j] = decode_tiles(gfx[j], gs[j], &s_tile_used[j]);
		if (!s_tiles[j] || !s_tile_used[j]) {
			rbx_fail("%s: out of memory (graphics)", g->set);
			ex_exit();
			return -1;
		}
	}
	// RGB_444_PROMS, 1024 entries
	for (int i = 0; i < 1024; i++) {
		int r = prom[i] & 15, gg = prom[i + 1024] & 15, b = prom[i + 2048] & 15;
		s_pal[i] = (uint32_t)((r * 17) << 16 | (gg * 17) << 8 | (b * 17));
	}
	static const char *const tags[5] = { "P1", "P2", "SYSTEM", "DSWA", "DSWB" };
	for (int i = 0; i < 5; i++)
		s_port[i] = rbx_port_find(tags[i]);
	// sound: Z80 4 MHz, YM2151 4 MHz routes 0.50 + 0.50, PC060HA
	tsnd_config sc;
	memset(&sc, 0, sizeof(sc));
	sc.map = TSND_MAP_9000;
	sc.z80_clock = 4000000;
	sc.frame_rate_mhz = 60000;
	sc.lines = LINES;
	sc.ym.type = RBX_YM2151;
	sc.ym.clock = 4000000;
	sc.ym.gain[0] = sc.ym.gain[1] = 128;
	sc.rom_flat = 1;
	if (tsnd_init(&sc)) {
		rbx_fail("%s: sound init failed", g->set);
		ex_exit();
		return -1;
	}
	if (rbx_video_setup(SCREEN_W, VIS_Y1 - VIS_Y0 + 1, 1024)) {
		ex_exit();
		return -1;
	}
	ex_reset();
	return 0;
}

static void run_cpu(cpu_t *c, unsigned long end)
{
	while ((long)(c->z.cyc - end) < 0) {
		c->z.int_pending = c->irq ? 1 : 0;
		z80_step(&c->z);
		if (c->irq && !c->z.int_pending)
			c->irq = 0;   // HOLD_LINE: acknowledged
	}
}

static void ex_frame(void)
{
	for (int line = 0; line < LINES; line++) {
		if (line == VIS_Y1 + 1) {
			vblank_capture();
			for (int i = 0; i < 3; i++)
				s_cpu[i].irq = 1;   // irq0_line_hold
		}
		int n = rbx_line_cycles(&s_frac, MAIN_CLOCK, 60000, LINES);
		for (int i = 0; i < 3; i++) {
			unsigned long end = s_cpu[i].z.cyc + (unsigned long)n;
			run_cpu(&s_cpu[i], end);
			if (s_cpuc_reset) {
				s_cpuc_reset = 0;
				z80_reset_cpu(2);
			}
		}
		tsnd_line();
	}
	tsnd_frame_end();
}

static int ex_state(void *ctx, int load)
{
	z80 z[3];
	int irq[3];
	for (int i = 0; i < 3; i++) {
		z[i] = s_cpu[i].z;
		irq[i] = s_cpu[i].irq;
	}
	int rc = 0;
	rc |= rbx_state_io(ctx, "z80", z, sizeof(z));
	rc |= rbx_state_io(ctx, "irq", irq, sizeof(irq));
	rc |= rbx_state_io(ctx, "vram", s_vram, sizeof(s_vram));
	rc |= rbx_state_io(ctx, "ramac", s_ram_ac, sizeof(s_ram_ac));
	rc |= rbx_state_io(ctx, "ramab", s_ram_ab, sizeof(s_ram_ab));
	rc |= rbx_state_io(ctx, "ramb", s_ram_b, sizeof(s_ram_b));
	rc |= rbx_state_io(ctx, "ramc", s_ram_c, sizeof(s_ram_c));
	int32_t misc[4] = { s_bank[0], s_bank[1], s_flip, 0 };
	rc |= rbx_state_io(ctx, "misc", misc, sizeof(misc));
	rc |= rbx_state_io(ctx, "frac", &s_frac, sizeof(s_frac));
	rc |= tsnd_state(ctx, load);
	if (rc)
		return -1;
	if (load) {
		for (int i = 0; i < 3; i++) {
			s_cpu[i].z = z[i];
			s_cpu[i].irq = irq[i];
			set_handlers(i);
		}
		s_bank[0] = misc[0]; s_bank[1] = misc[1]; s_flip = misc[2];
	}
	return 0;
}

static int ex_save(void *ctx) { return ex_state(ctx, 0); }
static int ex_load(void *ctx) { return ex_state(ctx, 1); }
static int ex_supported(const char *machine) { return !strcmp(machine, "exzisus"); }

const rbx_driver rbx_drv_exzisus = { "Exzisus", ex_init, ex_exit, ex_reset, ex_frame, ex_save, ex_load, ex_supported, ex_render };
