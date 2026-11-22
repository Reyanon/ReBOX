/*  Chack'n Pop, ported from MAME src/mame/taito/chaknpop.cpp (BSD-3-Clause, copyright-holders: BUT).
    Memory map, the Taito 68705 MCU interface (machine/taito68705), the PROM palette, the text tilemap, the sprites,
    the banked 4-plane bitmap ORed over the picture and the sound (2 x AY-3-8910, DIP switches on AY #1 ports) follow
    MAME.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/z80/z80.h"
#include "../../machine/taito68705.h"
#include "../../sound/ay8910.h"

#include <stdlib.h>
#include <string.h>

// MAME: 59.1828 Hz, 256 x 256, visible 0-255 x 16-239, vblank time 0 (264 lines assumed for the CPU timing)
#define LINES 264
#define VIS_Y0 16
#define VIS_Y1 239
#define SCREEN_W 256
#define RATE_MHZ 59183
#define MAIN_CLOCK 3000000    // 18 MHz / 6
#define MCU_CLOCK 750000      // 18 MHz / 6 / 4
#define AY_CLOCK 1500000      // 18 MHz / 12
#define SLICES 2              // MAME quantum 6000 Hz

enum { GFX_FLIP_X = 0x01, GFX_FLIP_Y = 0x02, GFX_VRAM_BANK = 0x04, GFX_TX_BANK1 = 0x20, GFX_TX_BANK2 = 0x80 };
enum { TX_COLOR1 = 0x0b, TX_COLOR2 = 0x01 };

typedef struct {
	z80 z;
	int irq;
} zcpu;

static uint8_t *s_main_rom;
static uint32_t s_main_size;
static zcpu s_main;
static uint8_t s_ram[0x800], s_tx[0x400], s_attr[0x40], s_spr[0xc0], s_vram[0x8000];
static uint8_t s_gfxmode, s_flip_x, s_flip_y;
static taito68705_t s_mcu;
// sound
static ay8910_t s_ay[2];
static int32_t *s_mix;
static int s_mix_cap, s_mix_n;
static uint32_t s_snd_done;
static unsigned long s_frame_cyc;
static uint64_t s_frac_main, s_frac_mcu;
// video
static uint8_t s_chars[1024 * 64], s_sprites[256 * 256];
static uint32_t s_pal[1024];
static uint8_t s_snap_tx[0x400], s_snap_attr[0x40], s_snap_spr[0xc0], s_snap_vram[0x8000];
static uint8_t s_snap_gfxmode, s_snap_flip_x, s_snap_flip_y;
// ports
static int s_p_system, s_p_p1, s_p_p2, s_p_dsw[3];

static uint8_t port8(int p) { return p >= 0 ? (uint8_t)rbx_port_read(p) : 0xff; }

// ---- sound ----
static void snd_run_to(uint32_t pos)
{
	if (pos <= s_snd_done)
		return;
	uint32_t clk = pos - s_snd_done;
	int room = s_mix_cap - s_mix_n;
	if (room > 0) {
		int n = ay8910_render(&s_ay[0], clk, s_mix + s_mix_n, room);
		ay8910_render(&s_ay[1], clk, s_mix + s_mix_n, room);
		s_mix_n += n;
	}
	s_snd_done = pos;
}

// AY clock = main CPU clock / 2
static void snd_sync(void) { snd_run_to((uint32_t)((s_main.z.cyc - s_frame_cyc) / 2)); }

static uint8_t ay1_pa_r(void *u) { (void)u; return port8(s_p_dsw[0]); }   // DSWA
static uint8_t ay1_pb_r(void *u) { (void)u; return port8(s_p_dsw[1]); }   // DSWB

// ---- main CPU ----
static uint8_t main_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x8000 || (a >= 0xa000 && a < 0xc000)) return a < s_main_size ? s_main_rom[a] : 0xff;
	if (a >= 0xc000) return s_vram[((s_gfxmode & GFX_VRAM_BANK) ? 0x4000 : 0) + (a - 0xc000)];
	if (a < 0x8800) return s_ram[a & 0x7ff];
	if (a >= 0x9000 && a < 0x9400) return s_tx[a & 0x3ff];
	if (a >= 0x9800 && a < 0x9840) return s_attr[a & 0x3f];
	if (a >= 0x9840 && a < 0x9900) return s_spr[a - 0x9840];
	switch (a) {
	case 0x8800: return taito68705_data_r(&s_mcu);
	case 0x8801:   // mcu_status_r
		return (uint8_t)((!taito68705_host_semaphore_r(&s_mcu) ? 1 : 0) | (taito68705_mcu_semaphore_r(&s_mcu) ? 2 : 0));
	case 0x8804: case 0x8805: snd_sync(); return ay8910_data_r(&s_ay[0]);
	case 0x8806: case 0x8807: snd_sync(); return ay8910_data_r(&s_ay[1]);
	case 0x8808: return port8(s_p_dsw[2]);
	case 0x8809: return port8(s_p_p1);
	case 0x880a: return port8(s_p_system);
	case 0x880b: return port8(s_p_p2);
	case 0x880c: return s_gfxmode;
	}
	return 0;
}

static void main_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a < 0x8000 || (a >= 0xa000 && a < 0xc000)) return;
	if (a >= 0xc000) { s_vram[((s_gfxmode & GFX_VRAM_BANK) ? 0x4000 : 0) + (a - 0xc000)] = v; return; }
	if (a < 0x8800) { s_ram[a & 0x7ff] = v; return; }
	if (a >= 0x9000 && a < 0x9400) { s_tx[a & 0x3ff] = v; return; }
	if (a >= 0x9800 && a < 0x9840) { s_attr[a & 0x3f] = v; return; }
	if (a >= 0x9840 && a < 0x9900) { s_spr[a - 0x9840] = v; return; }
	switch (a) {
	case 0x8800: taito68705_data_w(&s_mcu, v); return;
	case 0x8804: case 0x8805: snd_sync(); ay8910_write(&s_ay[0], a & 1, v); return;
	case 0x8806: case 0x8807: snd_sync(); ay8910_write(&s_ay[1], a & 1, v); return;
	case 0x880c:   // gfxmode_w
		s_gfxmode = v;
		s_flip_x = v & GFX_FLIP_X;
		s_flip_y = v & GFX_FLIP_Y;
		return;
	}
}

static uint8_t no_in(z80 *z, uint8_t p) { (void)z; (void)p; return 0xff; }
static void no_out(z80 *z, uint8_t p, uint8_t v) { (void)z; (void)p; (void)v; }

static void zcpu_init(zcpu *c)
{
	unsigned long cyc = c->z.cyc;
	z80_init(&c->z);
	c->z.read_byte = main_read;
	c->z.write_byte = main_write;
	c->z.port_in = no_in;
	c->z.port_out = no_out;
	c->z.int_data = 0xff;
	c->z.cyc = cyc;
	c->irq = 0;
}

static void zcpu_run(zcpu *c, int cycles)
{
	unsigned long end = c->z.cyc + (unsigned long)cycles;
	while ((long)(c->z.cyc - end) < 0) {
		c->z.int_pending = c->irq ? 1 : 0;
		z80_step(&c->z);
		if (c->irq && !c->z.int_pending)
			c->irq = 0;   // HOLD_LINE acknowledged
	}
}

// ---- graphics ----
static void decode_gfx(const uint8_t *spr, const uint8_t *chr)
{
	// planes { 0, 0x2000*8 } (first plane = high bit), bit offsets from the MSB of each byte
	for (int t = 0; t < 256; t++)
		for (int y = 0; y < 16; y++)
			for (int x = 0; x < 16; x++) {
				uint32_t b = (uint32_t)t * 256 + (uint32_t)((x & 7) + (x >> 3) * 64) + (uint32_t)((y & 7) * 8 + (y >> 3) * 128);
				int hi = (spr[b >> 3] >> (7 - (b & 7))) & 1;
				int lo = (spr[0x2000 + (b >> 3)] >> (7 - (b & 7))) & 1;
				s_sprites[t * 256 + y * 16 + x] = (uint8_t)(hi << 1 | lo);
			}
	for (int t = 0; t < 1024; t++)
		for (int y = 0; y < 8; y++)
			for (int x = 0; x < 8; x++) {
				uint32_t o = (uint32_t)t * 8 + (uint32_t)y;
				int hi = (chr[o] >> (7 - x)) & 1;
				int lo = (chr[0x2000 + o] >> (7 - x)) & 1;
				s_chars[t * 64 + y * 8 + x] = (uint8_t)(hi << 1 | lo);
			}
}

static void init_palette(const uint8_t *prom)
{
	for (int i = 0; i < 1024; i++) {
		int col = (prom[i] & 0x0f) | ((prom[i + 1024] & 0x0f) << 4);
		int r = 0x21 * (col & 1) + 0x47 * ((col >> 1) & 1) + 0x97 * ((col >> 2) & 1);
		int g = 0x21 * ((col >> 3) & 1) + 0x47 * ((col >> 4) & 1) + 0x97 * ((col >> 5) & 1);
		int b = 0x52 * ((col >> 6) & 1) + 0xad * ((col >> 7) & 1);
		s_pal[i] = (uint32_t)(r << 16 | g << 8 | b);
	}
}

// ---- video ----
static void draw_tx(void)
{
	const int fx = s_snap_flip_x, fy = s_snap_flip_y;
	const int bank2 = (s_snap_gfxmode & GFX_TX_BANK2) << 2, bank1 = s_snap_gfxmode & GFX_TX_BANK1;
	for (int y = VIS_Y0; y <= VIS_Y1; y++) {
		uint16_t *d = rbx_video.bitmap + (y - VIS_Y0) * rbx_video.pitch;
		int ty = fy ? 255 - y : y;
		for (int col = 0; col < 32; col++) {
			int tcol = fx ? 31 - col : col;
			int tile = s_snap_tx[(ty >> 3) * 32 + tcol];
			int color = tile == 0x74 ? s_snap_attr[TX_COLOR1] : s_snap_attr[TX_COLOR2];
			if (bank1 && tile >= 0xc0)
				tile += 0xc0;
			tile |= bank2;
			const uint8_t *r = s_chars + (tile & 1023) * 64 + (ty & 7) * 8;
			int base = 32 + color * 4;
			uint16_t *dd = d + col * 8;
			for (int x = 0; x < 8; x++)
				dd[x] = (uint16_t)((base + r[fx ? 7 - x : x]) & 1023);
		}
	}
}

static void draw_sprites(void)
{
	for (int offs = 0; offs < 0xc0; offs += 4) {
		const uint8_t *s = s_snap_spr + offs;
		int sx = s[3];
		int sy = 256 - 15 - s[0];
		int flipx = (s[1] >> 6) & 1, flipy = (s[1] >> 7) & 1;
		int color = s[2] & 7;
		int tile = (s[1] & 0x3f) | ((s[2] & 0x38) << 3);
		if (s_snap_flip_x) {
			sx = 240 - sx;
			flipx = !flipx;
		}
		if (s_snap_flip_y) {
			sy = 242 - sy;
			flipy = !flipy;
		}
		const uint8_t *g = s_sprites + (tile & 255) * 256;
		for (int ty = 0; ty < 16; ty++) {
			int y = sy + ty;
			if (y < VIS_Y0 || y > VIS_Y1) continue;
			const uint8_t *r = g + (flipy ? 15 - ty : ty) * 16;
			uint16_t *d = rbx_video.bitmap + (y - VIS_Y0) * rbx_video.pitch;
			for (int tx = 0; tx < 16; tx++) {
				int x = sx + tx;
				if ((unsigned)x >= SCREEN_W) continue;
				uint8_t p = r[flipx ? 15 - tx : tx];
				if (p)
					d[x] = (uint16_t)(color * 4 + p);
			}
		}
	}
}

static void draw_bitmap(void)
{
	const int fx = s_snap_flip_x, fy = s_snap_flip_y;
	const int dx = fx ? -1 : 1;
	for (int offs = 0; offs < 0x2000; offs++) {
		uint8_t p0 = s_snap_vram[offs], p1 = s_snap_vram[0x2000 + offs], p2 = s_snap_vram[0x4000 + offs],
		        p3 = s_snap_vram[0x6000 + offs];
		if (!(p0 | p1 | p2 | p3))
			continue;
		int x = ((offs & 0x1f) << 3) + 7;
		int y = offs >> 5;
		if (!fx) x = 255 - x;
		if (!fy) y = 255 - y;
		if (y < VIS_Y0 || y > VIS_Y1)
			continue;
		uint16_t *d = rbx_video.bitmap + (y - VIS_Y0) * rbx_video.pitch;
		for (int i = 0x80; i > 0; i >>= 1, x += dx) {
			int color = 0;
			if (p0 & i) color |= 0x200;   // green lower cage
			if (p1 & i) color |= 0x080;
			if (p2 & i) color |= 0x100;   // green upper cage
			if (p3 & i) color |= 0x040;   // tx mask
			if (color)
				d[x] |= (uint16_t)color;
		}
	}
}

static void cp_render(void)
{
	memcpy(rbx_video.palette, s_pal, sizeof(s_pal));
	draw_tx();
	draw_sprites();
	draw_bitmap();
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap_tx, s_tx, sizeof(s_tx));
	memcpy(s_snap_attr, s_attr, sizeof(s_attr));
	memcpy(s_snap_spr, s_spr, sizeof(s_spr));
	memcpy(s_snap_vram, s_vram, sizeof(s_vram));
	s_snap_gfxmode = s_gfxmode;
	s_snap_flip_x = s_flip_x;
	s_snap_flip_y = s_flip_y;
	rbx_video_frame_done();
}

// ---- init / frame ----
static void cp_reset(void)
{
	memset(s_ram, 0, sizeof(s_ram));
	memset(s_tx, 0, sizeof(s_tx));
	memset(s_attr, 0, sizeof(s_attr));
	memset(s_spr, 0, sizeof(s_spr));
	memset(s_vram, 0, sizeof(s_vram));
	memset(&s_main, 0, sizeof(s_main));
	zcpu_init(&s_main);
	s_gfxmode = s_flip_x = s_flip_y = 0;
	taito68705_reset(&s_mcu);
	ay8910_reset(&s_ay[0]);
	ay8910_reset(&s_ay[1]);
	s_frac_main = s_frac_mcu = 0;
	s_snd_done = 0;
	s_frame_cyc = s_main.z.cyc;
	s_mix_n = 0;
}

static void cp_exit(void)
{
	free(s_mix);
	s_mix = NULL;
}

static int cp_supported(const char *machine) { return !strcmp(machine, "chaknpop"); }

static int cp_init(const rbx_game *g)
{
	if (!cp_supported(g->machine)) {
		rbx_fail("%s: machine %s is not emulated", g->set, g->machine);
		return -1;
	}
	uint32_t ssize, csize, psize, msize;
	s_main_rom = rbx_region("maincpu", &s_main_size);
	uint8_t *spr = rbx_region("sprites", &ssize);
	uint8_t *chr = rbx_region("chars", &csize);
	uint8_t *prom = rbx_region("proms", &psize);
	uint8_t *mrom = rbx_region("bmcu:mcu", &msize);
	if (!s_main_rom || !spr || ssize < 0x4000 || !chr || csize < 0x4000 || !prom || psize < 0x800) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	memset(&s_mcu, 0, sizeof(s_mcu));
	if (!mrom || msize < 0x800 || taito68705_init(&s_mcu, TAITO68705_STD, mrom, 0x800)) {
		rbx_fail("%s: MCU ROM missing", g->set);
		return -1;
	}
	decode_gfx(spr, chr);
	init_palette(prom);
	s_p_system = rbx_port_find("SYSTEM");
	s_p_p1 = rbx_port_find("P1");
	s_p_p2 = rbx_port_find("P2");
	s_p_dsw[0] = rbx_port_find("DSWA");
	s_p_dsw[1] = rbx_port_find("DSWB");
	s_p_dsw[2] = rbx_port_find("DSWC");
	ay8910_init(&s_ay[0], AY8910_TYPE_AY8910, AY_CLOCK, AY8910_LEGACY_OUTPUT, 38, 4, 1);   // route 0.15
	s_ay[0].port_a_read = ay1_pa_r;
	s_ay[0].port_b_read = ay1_pb_r;
	ay8910_init(&s_ay[1], AY8910_TYPE_AY8910, AY_CLOCK, AY8910_LEGACY_OUTPUT, 26, 4, 1);   // route 0.10
	uint32_t rate = ay8910_rate(&s_ay[0]);
	s_mix_cap = (int)(rate / 50) + 64;
	s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	if (!s_mix || rbx_audio_setup(rate) || rbx_video_setup(SCREEN_W, VIS_Y1 - VIS_Y0 + 1, 1024)) {
		cp_exit();
		return -1;
	}
	cp_reset();
	return 0;
}

static void cp_frame(void)
{
	rbx_audio.frame_mhz = RATE_MHZ;   // (the board's frame rate: a frame's sound is played in a 60 Hz frame, as its picture)
	s_frame_cyc = s_main.z.cyc;
	s_snd_done = 0;
	for (int i = 0; i < LINES; i++) {
		int line = (VIS_Y1 + 1 + i) % LINES;
		if (line == VIS_Y1 + 1) {
			vblank_capture();
			s_main.irq = 1;   // irq0_line_hold
		}
		int mc = rbx_line_cycles(&s_frac_main, MAIN_CLOCK, RATE_MHZ, LINES * SLICES);
		int uc = rbx_line_cycles(&s_frac_mcu, MCU_CLOCK, RATE_MHZ, LINES * SLICES);
		for (int s = 0; s < SLICES; s++) {
			zcpu_run(&s_main, mc);
			taito68705_execute(&s_mcu, uc);
		}
	}
	snd_sync();
	for (int i = 0; i < s_mix_n; i++)
		rbx_audio_put(s_mix[i]);
	memset(s_mix, 0, sizeof(int32_t) * (size_t)s_mix_n);
	s_mix_n = 0;
}

static int cp_state(void *ctx, int load)
{
	zcpu t = s_main;
	int rc = rbx_state_io(ctx, "main", &t, sizeof(t));
	rc |= rbx_state_io(ctx, "ram", s_ram, sizeof(s_ram));
	rc |= rbx_state_io(ctx, "tx", s_tx, sizeof(s_tx));
	rc |= rbx_state_io(ctx, "attr", s_attr, sizeof(s_attr));
	rc |= rbx_state_io(ctx, "spr", s_spr, sizeof(s_spr));
	rc |= rbx_state_io(ctx, "vram", s_vram, sizeof(s_vram));
	rc |= rbx_state_io(ctx, "tmcu", &s_mcu.st, sizeof(s_mcu.st));
	rc |= rbx_state_io(ctx, "tmcu6805", &s_mcu.mcu.st, sizeof(s_mcu.mcu.st));
	rc |= rbx_state_io(ctx, "ay0", &s_ay[0], ay8910_state_size());
	rc |= rbx_state_io(ctx, "ay1", &s_ay[1], ay8910_state_size());
	uint8_t v[4] = { s_gfxmode, s_flip_x, s_flip_y, 0 };
	rc |= rbx_state_io(ctx, "regs", v, sizeof(v));
	if (rc)
		return -1;
	if (load) {
		t.z.read_byte = s_main.z.read_byte;
		t.z.write_byte = s_main.z.write_byte;
		t.z.port_in = s_main.z.port_in;
		t.z.port_out = s_main.z.port_out;
		t.z.userdata = s_main.z.userdata;
		s_main = t;
		s_gfxmode = v[0]; s_flip_x = v[1]; s_flip_y = v[2];
		s_frame_cyc = s_main.z.cyc;
		s_snd_done = 0;
	}
	return 0;
}

static int cp_save(void *ctx) { return cp_state(ctx, 0); }
static int cp_load(void *ctx) { return cp_state(ctx, 1); }

const rbx_driver rbx_drv_chaknpop = { "Chack'n Pop", cp_init, cp_exit, cp_reset, cp_frame, cp_save, cp_load, cp_supported, cp_render };
