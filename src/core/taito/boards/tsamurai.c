/*  Samurai Nihon-Ichi hardware, ported from MAME src/mame/taito/tsamurai.cpp, tsamurai.h and tsamurai_v.cpp
    (BSD-3-Clause, copyright-holders: Phil Stroffolino, Paul Swan).
    Three machine configurations as in MAME:
      tsamurai  main Z80 (NMI at vblank, AY-3-8910 on its I/O ports) + two sample player Z80s with an 8-bit DAC each
                (IRQ HOLD_LINE on a sound command)
      vsgongf   main Z80 + one sound Z80 (AY on I/O, DAC, NMI on a sound command and a 180 Hz NMI when enabled)
      m660      main Z80 + two sample player Z80s + an AY driver Z80 (NMI at vblank)
    The protection reads return MAME's constants. Sound: AY8910 (legacy output, route 0.1) plus the unsigned 8-bit
    R2R DACs (route 0.1 each, DC level removed), mixed in mono at the AY rate; DAC and AY writes are synchronised to
    the writing CPU's position in the scanline.
    Video: background tilemap (scroll), foreground tilemap with column scroll (colour RAM), 32x32 sprites,
    RGB_444_PROMS palette; flip screen mirrors the whole 256x256 picture.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/z80/z80.h"
#include "../../sound/ay8910.h"

#include <stdlib.h>
#include <string.h>

enum { TS_TSAMURAI, TS_VSGONGF, TS_M660 };

typedef struct {
	const char *name;
	int kind;
} machine_t;

static const machine_t s_machines[] = {
	{ "tsamurai", TS_TSAMURAI },
	{ "vsgongf", TS_VSGONGF },
	{ "m660", TS_M660 },
};

// MAME: 60 Hz, 256x256 raster, visible 0-255 x 16-239; every CPU and the AY at 24 MHz / 8
#define CLOCK 3000000
#define LINES 256
#define VIS_Y0 16
#define VIS_Y1 239
#define SCREEN_W 256

typedef struct {
	z80 z;
	int irq;            // IRQ held until acknowledged (HOLD_LINE)
	unsigned long start;
	int len;
} zcpu;

static const machine_t *s_m;
static const char *s_set;
static uint8_t *s_rom[4];
static uint32_t s_rom_size[4];
static uint8_t (*s_ram_hp)[0x4000];               // main CPU 0xc000-0xffff
#define s_ram (*s_ram_hp)
static uint8_t s_sram[3][0x800];            // sound CPU work RAM
static uint8_t s_sram3_hi[4];               // m660 audio3 0xfffc-0xffff
static zcpu s_cpu[4];
static int s_ncpu;
static uint8_t s_cmd[3];
static int s_nmi_enabled, s_flip, s_textbank1, s_textbank2, s_bgcolor, s_scrollx, s_scrolly, s_gong_color;
static int s_snd_nmi_enabled;
static uint8_t s_dac[2];
static int s_line_pos;                       // CPU clocks of the current line start (audio clock)
// sound
static ay8910_t s_ay;
static int32_t *s_mix;
static int s_mix_cap, s_mix_n;
static uint32_t s_ay_done;
// video
static uint8_t *s_bgt, *s_fgt, *s_spr;      // decoded 1 byte per pixel
static uint8_t *s_bgo, *s_fgo;               // tile has a non-zero pixel
static int s_bg_n, s_fg_n;
static uint16_t (*s_frame_hp)[256 * 256];
#define s_frame (*s_frame_hp)
static uint8_t (*s_snap_ram_hp)[0x4000];
#define s_snap_ram (*s_snap_ram_hp)
static int s_snap_flip, s_snap_tb1, s_snap_tb2, s_snap_bgcolor, s_snap_sx, s_snap_sy, s_snap_gong;
static uint32_t s_pal[256];
// ports
static int s_p1, s_p2, s_system, s_dsw1, s_dsw2;
static uint64_t s_frac;

static uint8_t port8(int p) { return (uint8_t)rbx_port_read(p); }

// ---- sound ----
static void ay_run_to(uint32_t pos)
{
	if (pos <= s_ay_done)
		return;
	int n = ay8910_render(&s_ay, pos - s_ay_done, s_mix + s_mix_n, s_mix_cap - s_mix_n);
	// DAC_8BIT_R2R: unsigned, 0..255 -> 0..1, route 0.1 (the DC level is dropped)
	int32_t dac = 0;
	int ndac = s_m->kind == TS_VSGONGF ? 1 : 2;
	for (int i = 0; i < ndac; i++)
		dac += ((int)s_dac[i] - 128) * 3277 / 255;
	for (int i = 0; i < n; i++)
		s_mix[s_mix_n + i] += dac;
	s_mix_n += n;
	s_ay_done = pos;
}

static void sync_cpu(zcpu *c)
{
	int done = (int)(c->z.cyc - c->start);
	if (done > c->len)
		done = c->len;
	if (done < 0)
		done = 0;
	ay_run_to((uint32_t)(s_line_pos + done));
}

static void dac_w(zcpu *c, int n, uint8_t v)
{
	sync_cpu(c);
	s_dac[n] = v;
}

static void ay_w(zcpu *c, int offset, uint8_t v)
{
	sync_cpu(c);
	ay8910_write(&s_ay, offset, v);
}

// ---- main CPU ----
static void mainlatch_w(int bit, int state)
{
	if (s_m->kind == TS_VSGONGF) {
		if (bit == 1) s_nmi_enabled = state;
		else if (bit == 4) s_textbank1 = state;
		return;
	}
	switch (bit) {
	case 0: s_flip = state; break;
	case 1: s_nmi_enabled = state; break;
	case 2: s_textbank1 = state; break;
	case 7: if (s_m->kind == TS_M660) s_textbank2 = state; break;
	}
}

static uint8_t main_read(void *u, uint16_t a)
{
	(void)u;
	if (s_m->kind == TS_VSGONGF) {
		if (a < 0x8000) return s_rom[0][a];
		if (a == 0xa006) {   // sound CPU busy?
			if (!strcmp(s_set, "ringfgta")) return 0xc0;
			return 0x80;
		}
		if (a == 0xa100) {   // protection
			if (!strcmp(s_set, "ringfgt")) return 0x63;
			if (!strcmp(s_set, "ringfgta")) return 0x6a;
			return 0xaa;
		}
		if ((a >= 0xc000 && a < 0xc800) || (a >= 0xe000 && a < 0xe47c)) return s_ram[a - 0xc000];
	} else {
		if (a < 0xc000) return s_rom[0][a];
		if (a < 0xd000) return s_ram[a - 0xc000];
		switch (a) {
		case 0xd803: return (s_m->kind == TS_M660 && strcmp(s_set, "the26thz")) ? 0x53 : 0x6b;
		case 0xd806: return 0x40;
		case 0xd900: return 0x6a;
		case 0xd938: return 0xfb;
		}
		if (a >= 0xe000 && a < 0xf400) return s_ram[a - 0xc000];
	}
	switch (a) {
	case 0xf800: return port8(s_p1);
	case 0xf801: return port8(s_p2);
	case 0xf802: return port8(s_system);
	case 0xf804: return port8(s_dsw1);
	case 0xf805: return port8(s_dsw2);
	}
	return 0;
}

static void snd_cmd(int n, uint8_t v)
{
	s_cmd[n] = v;
	s_cpu[1 + n].irq = 1;
}

static void main_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0xfc00 && a <= 0xfc07) { mainlatch_w(a & 7, v & 1); return; }
	if (s_m->kind == TS_VSGONGF) {
		if ((a >= 0xc000 && a < 0xc800) || (a >= 0xe000 && a < 0xe47c)) { s_ram[a - 0xc000] = v; return; }
		if (a == 0xe800) { s_cmd[0] = v; s_cpu[1].z.nmi_pending = 1; return; }
		if (a == 0xf000) { s_gong_color = v; return; }
		return;
	}
	if (a >= 0xc000 && a < 0xd000) { s_ram[a - 0xc000] = v; return; }
	if (a >= 0xe000 && a < 0xf400) { s_ram[a - 0xc000] = v; return; }
	switch (a) {
	case 0xf401:
		if (s_m->kind == TS_M660) snd_cmd(2, v); else snd_cmd(0, v);
		return;
	case 0xf402: snd_cmd(1, v); return;
	case 0xf403: if (s_m->kind == TS_M660) snd_cmd(0, v); return;
	case 0xf801: s_bgcolor = v; return;
	case 0xf802: s_scrolly = v; return;
	case 0xf803: s_scrollx = v; return;
	}
}

static uint8_t no_in(z80 *z, uint8_t p) { (void)z; (void)p; return 0xff; }
static void no_out(z80 *z, uint8_t p, uint8_t v) { (void)z; (void)p; (void)v; }
static void main_out(z80 *z, uint8_t p, uint8_t v)
{
	(void)z;
	if (s_m->kind == TS_TSAMURAI && p < 2)
		ay_w(&s_cpu[0], p & 1, v);
}
static void ay_cpu_out(z80 *z, uint8_t p, uint8_t v)
{
	if (p < 2)
		ay_w((zcpu *)z, p & 1, v);
}

// ---- sound CPUs ----
// tsamurai sample players (cpu 1, 2): ROM 0000-3fff, command 6000, DAC 6002, RAM 7f00-7fff
// m660 sample players: command c000, DAC c002, RAM 8000-87ff; m660 audio3: ROM 0000-7fff, command c000, RAM 8000-87ff
// vsgongf sound: ROM 0000-3fff, RAM 6000-63ff, latch / NMI enable 8000, DAC a000
static uint8_t snd_read(int n, uint16_t a)
{
	const uint8_t *rom = s_rom[n + 1];
	if (s_m->kind == TS_VSGONGF) {
		if (a < 0x4000) return rom[a];
		if (a >= 0x6000 && a < 0x6400) return s_sram[0][a - 0x6000];
		if (a == 0x8000) return s_cmd[0];
		return 0;
	}
	if (s_m->kind == TS_TSAMURAI) {
		if (a < 0x4000) return rom[a];
		if (a == 0x6000) return s_cmd[n];
		if (a >= 0x7f00) return s_sram[n][a - 0x7f00];
		return 0;
	}
	if (a < (n == 2 ? 0x8000 : 0x4000)) return rom[a];
	if (a >= 0x8000 && a < 0x8800) return s_sram[n][a - 0x8000];
	if (a == 0xc000) return s_cmd[n];
	if (n == 2 && a >= 0xfffc) return s_sram3_hi[a & 3];
	return 0;
}

static void snd_write(int n, uint16_t a, uint8_t v)
{
	zcpu *c = &s_cpu[n + 1];
	if (s_m->kind == TS_VSGONGF) {
		if (a >= 0x6000 && a < 0x6400) s_sram[0][a - 0x6000] = v;
		else if (a == 0x8000) s_snd_nmi_enabled = v;
		else if (a == 0xa000) dac_w(c, 0, v);
		return;
	}
	if (s_m->kind == TS_TSAMURAI) {
		if (a == 0x6002) dac_w(c, n, v);
		else if (a >= 0x7f00) s_sram[n][a - 0x7f00] = v;
		return;
	}
	if (a >= 0x8000 && a < 0x8800) s_sram[n][a - 0x8000] = v;
	else if (a == 0xc002 && n < 2) dac_w(c, n, v);
	else if (n == 2 && a >= 0xfffc) s_sram3_hi[a & 3] = v;
}

static uint8_t snd0_read(void *u, uint16_t a) { (void)u; return snd_read(0, a); }
static uint8_t snd1_read(void *u, uint16_t a) { (void)u; return snd_read(1, a); }
static uint8_t snd2_read(void *u, uint16_t a) { (void)u; return snd_read(2, a); }
static void snd0_write(void *u, uint16_t a, uint8_t v) { (void)u; snd_write(0, a, v); }
static void snd1_write(void *u, uint16_t a, uint8_t v) { (void)u; snd_write(1, a, v); }
static void snd2_write(void *u, uint16_t a, uint8_t v) { (void)u; snd_write(2, a, v); }

static void zcpu_init(zcpu *c, uint8_t (*rd)(void *, uint16_t), void (*wr)(void *, uint16_t, uint8_t),
                      void (*out)(z80 *, uint8_t, uint8_t))
{
	z80_init(&c->z);
	c->z.read_byte = rd;
	c->z.write_byte = wr;
	c->z.port_in = no_in;
	c->z.port_out = out ? out : no_out;
	c->z.int_data = 0xff;
	c->irq = 0;
}

static void zcpu_run(zcpu *c, int cycles)
{
	c->start = c->z.cyc;
	c->len = cycles;
	unsigned long end = c->z.cyc + (unsigned long)cycles;
	while ((long)(c->z.cyc - end) < 0) {
		c->z.int_pending = c->irq ? 1 : 0;
		z80_step(&c->z);
		if (c->irq && !c->z.int_pending)
			c->irq = 0;   // acknowledged
	}
}

// ---- video ----
static void decode_planar8(const uint8_t *src, uint32_t size, uint8_t **out, uint8_t **opaque, int *count)
{
	// gfx_8x8x3_planar: planes RGN_FRAC(2,3), RGN_FRAC(1,3), 0; 8 bytes a tile
	uint32_t third = size / 3;
	int n = (int)(third / 8);
	*count = n;
	*out = (uint8_t *)malloc((size_t)n * 64);
	*opaque = (uint8_t *)calloc((size_t)n, 1);
	if (!*out || !*opaque)
		return;
	for (int t = 0; t < n; t++)
		for (int y = 0; y < 8; y++) {
			uint32_t o = (uint32_t)t * 8 + (uint32_t)y;
			uint8_t b2 = src[2 * third + o], b1 = src[third + o], b0 = src[o];
			for (int x = 0; x < 8; x++) {
				int m = 0x80 >> x;
				uint8_t v = (uint8_t)(((b2 & m) ? 4 : 0) | ((b1 & m) ? 2 : 0) | ((b0 & m) ? 1 : 0));
				(*out)[t * 64 + y * 8 + x] = v;
				if (v)
					(*opaque)[t] = 1;
			}
		}
}

static void decode_sprites(const uint8_t *src, uint32_t size)
{
	// sprite_layout: 32x32, 3 planes RGN_FRAC(2,3) (1,3) (0,3), 8x8 blocks of 64 bits, 4*256 bits a sprite
	uint32_t third = size / 3;
	int n = (int)(third / 128);
	if (n > 128)
		n = 128;
	s_spr = (uint8_t *)calloc(128, 32 * 32);
	if (!s_spr)
		return;
	for (int t = 0; t < n; t++)
		for (int y = 0; y < 32; y++)
			for (int x = 0; x < 32; x++) {
				uint32_t bit = (uint32_t)t * 1024 + (uint32_t)(x / 8) * 64 + (uint32_t)(x % 8) +
				               (uint32_t)(y / 8) * 256 + (uint32_t)(y % 8) * 8;
				int m = 0x80 >> (bit & 7);
				uint32_t o = bit >> 3;
				uint8_t v = (uint8_t)(((src[2 * third + o] & m) ? 4 : 0) | ((src[third + o] & m) ? 2 : 0) |
				                      ((src[o] & m) ? 1 : 0));
				s_spr[t * 1024 + y * 32 + x] = v;
			}
}

static void draw_sprites(const uint8_t *src, int num)
{
	for (int i = num - 1; i >= 0; i--) {
		const uint8_t *s = src + i * 4;
		int attributes = s[2];
		int sx = s[3] - 16;
		int sy = 240 - s[0];
		int code = s[1] & 0x7f;
		int flipy = s[1] & 0x80;
		int color = attributes & 0x1f;
		if (sy < -16)
			sy += 256;
		const uint8_t *g = s_spr + code * 1024;
		uint16_t base = (uint16_t)(color * 8);
		for (int y = 0; y < 32; y++) {
			int dy = sy + y;
			if (dy < VIS_Y0 || dy > VIS_Y1)
				continue;
			const uint8_t *row = g + (flipy ? 31 - y : y) * 32;
			uint16_t *d = s_frame + dy * 256;
			for (int x = 0; x < 32; x++) {
				int dx = sx + x;
				if (dx < 0 || dx >= 256)
					continue;
				uint8_t c = row[x];
				if (c)
					d[dx] = (uint16_t)(base + c);
			}
		}
	}
}

// foreground tilemap: column scroll from colour RAM (tsamurai / m660); vsgongf: one colour, opaque, no scroll
static void draw_fg(const uint8_t *ram)
{
	const uint8_t *vram = ram + 0x2000, *cram = ram + 0x2400;
	const int gong = s_m->kind == TS_VSGONGF;
	for (int col = 0; col < 32; col++) {
		int scroll = gong ? 0 : cram[col * 2];
		int color = gong ? (s_snap_gong & 0x1f) : (cram[col * 2 + 1] & 0x1f);
		uint16_t base = (uint16_t)(color * 8);
		for (int y = VIS_Y0; y <= VIS_Y1; y++) {
			int ty = (y + scroll) & 255;
			int code = vram[(ty >> 3) * 32 + col];
			if (s_snap_tb1) code += 256;
			if (s_snap_tb2) code += 512;
			code %= s_fg_n;
			uint16_t *d = s_frame + y * 256 + col * 8;
			const uint8_t *row = s_fgt + code * 64 + (ty & 7) * 8;
			if (gong) {
				for (int x = 0; x < 8; x++)
					d[x] = (uint16_t)(base + row[x]);
			} else if (s_fgo[code]) {
				for (int x = 0; x < 8; x++)
					if (row[x])
						d[x] = (uint16_t)(base + row[x]);
			}
		}
	}
}

static void draw_bg(const uint8_t *ram)
{
	const uint8_t *bram = ram + 0x2800;
	const int m660 = s_m->kind == TS_M660;
	for (int y = VIS_Y0; y <= VIS_Y1; y++) {
		int ty = (y + s_snap_sy) & 255;
		uint16_t *d = s_frame + y * 256;
		for (int x = 0; x < 256;) {
			int tx = (x + s_snap_sx) & 255;
			int idx = (ty >> 3) * 32 + (tx >> 3);
			int attr = bram[idx * 2 + 1];
			int code = bram[idx * 2] + ((attr & 0xc0) >> 6) * 256;
			if (m660)
				code += ((attr & 0x20) >> 5) * 1024;
			code %= s_bg_n;
			int run = 8 - (tx & 7);
			if (run > 256 - x)
				run = 256 - x;
			if (s_bgo[code]) {
				const uint8_t *row = s_bgt + code * 64 + (ty & 7) * 8 + (tx & 7);
				uint16_t base = (uint16_t)((attr & 0x1f) * 8);
				for (int i = 0; i < run; i++)
					if (row[i])
						d[x + i] = (uint16_t)(base + row[i]);
			}
			x += run;
		}
	}
}

static void ts_render(void)
{
	memcpy(rbx_video.palette, s_pal, sizeof(s_pal));
	const uint8_t *ram = s_snap_ram;
	if (s_m->kind == TS_VSGONGF) {
		draw_fg(ram);
		draw_sprites(ram + 0x2400, 16);
	} else {
		for (int y = VIS_Y0; y <= VIS_Y1; y++)
			for (int x = 0; x < 256; x++)
				s_frame[y * 256 + x] = (uint16_t)s_snap_bgcolor;
		draw_bg(ram);
		draw_sprites(ram + 0x3000, 32);
		draw_fg(ram);
	}
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	for (int y = VIS_Y0; y <= VIS_Y1; y++) {
		uint16_t *d = bm + (y - VIS_Y0) * pitch;
		if (s_snap_flip) {
			const uint16_t *s = s_frame + (255 - y) * 256;
			for (int x = 0; x < 256; x++)
				d[x] = s[255 - x];
		} else
			memcpy(d, s_frame + y * 256, 256 * sizeof(uint16_t));
	}
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap_ram, s_ram, sizeof(s_ram));
	s_snap_flip = s_flip;
	s_snap_tb1 = s_textbank1;
	s_snap_tb2 = s_textbank2;
	s_snap_bgcolor = s_bgcolor;
	s_snap_sx = s_scrollx;
	s_snap_sy = s_scrolly;
	s_snap_gong = s_gong_color;
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

static void ts_reset(void)
{
	memset(s_ram, 0, sizeof(s_ram));
	memset(s_sram, 0, sizeof(s_sram));
	memset(s_sram3_hi, 0, sizeof(s_sram3_hi));
	memset(s_cpu, 0, sizeof(s_cpu));
	static uint8_t (*const rd[3])(void *, uint16_t) = { snd0_read, snd1_read, snd2_read };
	static void (*const wr[3])(void *, uint16_t, uint8_t) = { snd0_write, snd1_write, snd2_write };
	zcpu_init(&s_cpu[0], main_read, main_write, main_out);
	for (int i = 1; i < s_ncpu; i++) {
		int ay_here = (s_m->kind == TS_VSGONGF && i == 1) || (s_m->kind == TS_M660 && i == 3);
		zcpu_init(&s_cpu[i], rd[i - 1], wr[i - 1], ay_here ? ay_cpu_out : NULL);
	}
	memset(s_cmd, 0, sizeof(s_cmd));
	s_nmi_enabled = s_flip = s_textbank1 = s_textbank2 = s_bgcolor = s_scrollx = s_scrolly = s_gong_color = 0;
	s_snd_nmi_enabled = 0;
	s_dac[0] = s_dac[1] = 0x80;
	ay8910_reset(&s_ay);
	s_ay_done = 0;
	s_mix_n = 0;
	s_frac = 0;
}

static void ts_exit(void)
{
	free(s_bgt); free(s_fgt); free(s_spr); free(s_bgo); free(s_fgo);
	s_bgt = s_fgt = s_spr = s_bgo = s_fgo = NULL;
	free(s_mix);
	s_mix = NULL;
	free(s_frame_hp);
	s_frame_hp = NULL;
	free(s_ram_hp);
	s_ram_hp = NULL;
	free(s_snap_ram_hp);
	s_snap_ram_hp = NULL;
}

static int ts_init(const rbx_game *g)
{
	if (!s_frame_hp && !(s_frame_hp = calloc(1, sizeof(*s_frame_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	if (!s_ram_hp && !(s_ram_hp = calloc(1, sizeof(*s_ram_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	if (!s_snap_ram_hp && !(s_snap_ram_hp = calloc(1, sizeof(*s_snap_ram_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	s_m = find_machine(g->machine);
	if (!s_m) {
		rbx_fail("%s: machine %s is not emulated", g->set, g->machine);
		return -1;
	}
	s_set = g->set;
	static const char *const cpus[4] = { "maincpu", "audiocpu", "audio2", "audio3" };
	s_ncpu = s_m->kind == TS_VSGONGF ? 2 : s_m->kind == TS_M660 ? 4 : 3;
	for (int i = 0; i < s_ncpu; i++) {
		s_rom[i] = rbx_region(cpus[i], &s_rom_size[i]);
		if (!s_rom[i] || s_rom_size[i] < 0x10000) {
			rbx_fail("%s: ROM region %s missing", g->set, cpus[i]);
			return -1;
		}
	}
	uint32_t s1 = 0, s2 = 0, s3 = 0, ps = 0;
	uint8_t *g1 = rbx_region("gfx1", &s1), *g2 = rbx_region("gfx2", &s2), *g3 = rbx_region("gfx3", &s3);
	uint8_t *prom = rbx_region("proms", &ps);
	if (!g1 || !g2 || !g3 || !prom || ps < 0x300 || s1 < 24 || s2 < 24 || s3 < 384) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	decode_planar8(g1, s1, &s_bgt, &s_bgo, &s_bg_n);
	decode_planar8(g2, s2, &s_fgt, &s_fgo, &s_fg_n);
	decode_sprites(g3, s3);
	if (!s_bgt || !s_bgo || !s_fgt || !s_fgo || !s_spr) {
		rbx_fail("%s: out of memory (graphics)", g->set);
		ts_exit();
		return -1;
	}
	for (int i = 0; i < 256; i++) {   // RGB_444_PROMS
		int r = prom[i] & 15, gg = prom[i + 256] & 15, b = prom[i + 512] & 15;
		s_pal[i] = (uint32_t)((r * 17) << 16 | (gg * 17) << 8 | (b * 17));
	}
	s_p1 = rbx_port_find("P1");
	s_p2 = rbx_port_find("P2");
	s_system = rbx_port_find("SYSTEM");
	s_dsw1 = rbx_port_find("DSW1");
	s_dsw2 = rbx_port_find("DSW2");

	ay8910_init(&s_ay, AY8910_TYPE_AY8910, CLOCK, AY8910_LEGACY_OUTPUT, 26, 8, 0);   // route 0.1
	s_mix_cap = (int)(ay8910_rate(&s_ay) / 50) + 64;
	s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	if (!s_mix || rbx_audio_setup(ay8910_rate(&s_ay)) || rbx_video_setup(SCREEN_W, VIS_Y1 - VIS_Y0 + 1, 256)) {
		ts_exit();
		return -1;
	}
	ts_reset();
	return 0;
}

static void ts_frame(void)
{
	// the frame starts at vblank (line 240)
	s_line_pos = 0;
	for (int i = 0; i < LINES; i++) {
		int line = (VIS_Y1 + 1 + i) % LINES;
		if (line == VIS_Y1 + 1) {
			vblank_capture();
			if (s_nmi_enabled)
				s_cpu[0].z.nmi_pending = 1;
			if (s_m->kind == TS_M660)
				s_cpu[3].z.nmi_pending = 1;
		}
		// vsgongf: periodic sound NMI at 3 x 60 Hz
		if (s_m->kind == TS_VSGONGF && (i == 0 || i == LINES / 3 || i == 2 * LINES / 3) && s_snd_nmi_enabled)
			s_cpu[1].z.nmi_pending = 1;
		int cyc = rbx_line_cycles(&s_frac, CLOCK, 60000, LINES);
		for (int c = 0; c < s_ncpu; c++)
			zcpu_run(&s_cpu[c], cyc);
		s_line_pos += cyc;
		ay_run_to((uint32_t)s_line_pos);
	}
	for (int i = 0; i < s_mix_n; i++)
		rbx_audio_put(s_mix[i]);
	memset(s_mix, 0, sizeof(int32_t) * (size_t)s_mix_cap);
	s_mix_n = 0;
	s_ay_done = 0;
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

static int ts_state(void *ctx, int load)
{
	static const char *const tags[4] = { "main", "snd1", "snd2", "snd3" };
	int rc = 0;
	for (int i = 0; i < s_ncpu; i++)
		rc |= zcpu_state(ctx, tags[i], &s_cpu[i], load);
	rc |= rbx_state_io(ctx, "ram", s_ram, sizeof(s_ram));
	rc |= rbx_state_io(ctx, "sram", s_sram, sizeof(s_sram));
	rc |= rbx_state_io(ctx, "sram3hi", s_sram3_hi, sizeof(s_sram3_hi));
	rc |= rbx_state_io(ctx, "ay", &s_ay, ay8910_state_size());
	int32_t misc[16] = { s_cmd[0], s_cmd[1], s_cmd[2], s_nmi_enabled, s_flip, s_textbank1, s_textbank2, s_bgcolor,
		s_scrollx, s_scrolly, s_gong_color, s_snd_nmi_enabled, s_dac[0], s_dac[1], 0, 0 };
	rc |= rbx_state_io(ctx, "misc", misc, sizeof(misc));
	if (rc)
		return -1;
	if (load) {
		s_cmd[0] = (uint8_t)misc[0]; s_cmd[1] = (uint8_t)misc[1]; s_cmd[2] = (uint8_t)misc[2];
		s_nmi_enabled = misc[3]; s_flip = misc[4]; s_textbank1 = misc[5]; s_textbank2 = misc[6];
		s_bgcolor = misc[7]; s_scrollx = misc[8]; s_scrolly = misc[9]; s_gong_color = misc[10];
		s_snd_nmi_enabled = misc[11]; s_dac[0] = (uint8_t)misc[12]; s_dac[1] = (uint8_t)misc[13];
	}
	return 0;
}

static int ts_save(void *ctx) { return ts_state(ctx, 0); }
static int ts_load(void *ctx) { return ts_state(ctx, 1); }
static int ts_supported(const char *machine) { return find_machine(machine) != NULL; }

const rbx_driver rbx_drv_tsamurai = { "Samurai Nihon-Ichi", ts_init, ts_exit, ts_reset, ts_frame, ts_save, ts_load, ts_supported, ts_render };
