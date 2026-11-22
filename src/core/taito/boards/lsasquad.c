/*  Land Sea Air Squad / Storming Party / Daikaiju no Gyakushu, ported from MAME src/mame/taito/lsasquad.cpp
    (BSD-3-Clause, copyright-holders: Nicola Salmoria). Memory maps, the machine configurations (Z80 main, Z80 sound
    with YM2203 IRQ, latches and the INPUT_MERGER_ALL_HIGH sound NMI, Taito 68705 MCU through machine/taito68705),
    the column-scrolled layer drawing (lsasquad draw_layer, daikaiju drawbg), the sprites and the RGB_444 PROM palette
    follow MAME. Sound: YM2203 (rbx_ym) + YM2149 (AY-3-8910A on the bootleg).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/z80/z80.h"
#include "../../machine/taito68705.h"
#include "../../sound/ay8910.h"
#include "../../sound/rbx_ym.h"

#include <stdlib.h>
#include <string.h>

enum { LS_LSASQUAD, LS_STORMING, LS_DAIKAIJU };

// MAME: 60 Hz, 256 x 256, visible 0-255 x 16-239, vblank time 0 (264 lines assumed for the CPU timing)
#define LINES 264
#define VIS_Y0 16
#define VIS_Y1 239
#define SCREEN_W 256
#define RATE_MHZ 60000
#define MAIN_CLOCK 6000000    // 24 MHz / 4
#define AUDIO_CLOCK 3000000   // 24 MHz / 8 (YM2203, YM2149 and the MCU too)
#define MCU_CLOCK 750000      // 3 MHz / 4
#define SLICES 2              // MAME quantum 30000 Hz = 500 slices per frame

typedef struct {
	z80 z;
	int irq;
} zcpu;

static int s_kind, s_has_mcu;
static uint8_t *s_main_rom, *s_audio_rom, *s_gfx1, *s_gfx2, *s_prom;
static uint32_t s_main_size, s_audio_size, s_gfx1_size, s_gfx2_size, s_prom_size;
static zcpu s_main, s_audio;
static uint8_t s_sram[0x2000], s_vram[0x2000], s_scroll[0x400], s_spr[0x400], s_audio_ram[0x800];
static uint32_t s_spr_size;
static uint8_t s_bank, s_flip;
static uint8_t s_latch, s_latch2;
static int s_latch_pending, s_latch2_pending, s_nmi_en, s_nmi_line;
static int s_ym_irq;
static taito68705_t s_mcu;
// sound
static ay8910_t s_ay;
static int32_t *s_mix, *s_mix2;
static int s_mix_cap, s_mix_n, s_mix2_n;
static uint32_t s_snd_done;
static unsigned long s_frame_cyc;
static uint64_t s_frac_main, s_frac_audio, s_frac_mcu;
// video
static uint8_t *s_tiles, *s_sprites;
static uint32_t s_ntiles, s_nsprites;
static uint32_t s_pal[512];
static uint8_t s_snap_vram[0x2000], s_snap_scroll[0x400], s_snap_spr[0x400], s_snap_flip;
// ports
static int s_p_dsw[3], s_p_mcu, s_p_coins, s_p_p1, s_p_p2, s_p_start, s_p_service;

static uint8_t port8(int p) { return p >= 0 ? (uint8_t)rbx_port_read(p) : 0xff; }

// ---- sound ----
static void snd_run_to(uint32_t pos)
{
	if (pos <= s_snd_done)
		return;
	uint32_t clk = pos - s_snd_done;
	s_mix_n += rbx_ym_advance(0, clk, s_mix + s_mix_n, s_mix_cap - s_mix_n);
	s_mix2_n += ay8910_render(&s_ay, clk, s_mix2 + s_mix2_n, s_mix_cap - s_mix2_n);
	s_snd_done = pos;
}

// the sound chips run on the sound CPU clock: the position is the sound CPU time in the frame
static void snd_sync(void) { snd_run_to((uint32_t)(s_audio.z.cyc - s_frame_cyc)); }

static void ym_irq(int state) { s_ym_irq = state; }

static void update_nmi(void)
{
	int line = s_latch_pending && s_nmi_en;   // INPUT_MERGER_ALL_HIGH
	if (line && !s_nmi_line)
		s_audio.z.nmi_pending = 1;
	s_nmi_line = line;
}

static uint8_t sound_status_r(void)
{
	if (s_kind == LS_DAIKAIJU)
		return s_latch_pending ? 0x02 : 0x01;
	return (uint8_t)((s_latch_pending ? 1 : 0) | (s_latch2_pending ? 2 : 0));
}

static uint8_t mcu_status_r(void)
{
	int res = port8(s_p_mcu) & ~3;
	if (!taito68705_host_semaphore_r(&s_mcu)) res |= 0x01;
	if (!taito68705_mcu_semaphore_r(&s_mcu)) res |= 0x02;
	if (s_kind == LS_DAIKAIJU)
		res |= ((s_latch_pending & 0x02) ^ 2) << 3;   // as in MAME (pending_r is 0 / 1: always 0x10)
	return (uint8_t)res;
}

// ---- main CPU ----
static uint8_t main_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x8000) return s_main_rom[a];
	if (a < 0xa000) {
		uint32_t o = 0x8000 + (uint32_t)s_bank * 0x2000 + (a - 0x8000);
		return o < s_main_size ? s_main_rom[o] : 0xff;
	}
	if (a < 0xc000) return s_sram[a - 0xa000];
	if (a < 0xe000) return s_vram[a - 0xc000];
	if (a < 0xe400) return s_scroll[a - 0xe000];
	if (a < 0xe800) return (a - 0xe400u) < s_spr_size ? s_spr[a - 0xe400] : 0xff;
	switch (a) {
	case 0xe800: return port8(s_p_dsw[0]);
	case 0xe801: return port8(s_p_dsw[1]);
	case 0xe802: return s_kind == LS_DAIKAIJU ? 0xff : port8(s_p_dsw[2]);
	case 0xe803: return s_kind == LS_STORMING ? port8(s_p_coins) : mcu_status_r();
	case 0xe804: return port8(s_p_p1);
	case 0xe805: return port8(s_p_p2);
	case 0xe806: return port8(s_p_start);
	case 0xe807: return port8(s_p_service);
	case 0xec00:
		if (s_kind == LS_DAIKAIJU) return 0xff;
		s_latch2_pending = 0;
		return s_latch2;
	case 0xec01: return s_kind == LS_DAIKAIJU ? 0xff : sound_status_r();
	case 0xee00: return s_has_mcu ? taito68705_data_r(&s_mcu) : 0xff;
	}
	return 0xff;
}

static void main_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a < 0xa000) return;
	if (a < 0xc000) { s_sram[a - 0xa000] = v; return; }
	if (a < 0xe000) { s_vram[a - 0xc000] = v; return; }
	if (a < 0xe400) { s_scroll[a - 0xe000] = v; return; }
	if (a < 0xe800) { if ((a - 0xe400u) < s_spr_size) s_spr[a - 0xe400] = v; return; }
	switch (a) {
	case 0xea00:   // bankswitch_w
		s_bank = v & 7;
		s_flip = (v >> 4) & 1;
		return;
	case 0xec00: s_latch = v; s_latch_pending = 1; update_nmi(); return;
	case 0xee00: if (s_has_mcu) taito68705_data_w(&s_mcu, v); return;
	}
}

// ---- sound CPU ----
static uint8_t audio_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x8000 || (a >= 0xe000 && a < 0xf000)) return a < s_audio_size ? s_audio_rom[a] : 0xff;
	if (a < 0x8800) return s_audio_ram[a & 0x7ff];
	if (a == 0xa000 || a == 0xa001) { snd_sync(); return rbx_ym_read(0, a & 1); }
	if (a == 0xd000) { s_latch_pending = 0; update_nmi(); return s_latch; }
	if (a == 0xd800) return sound_status_r();
	return 0xff;
}

static void audio_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a < 0x8000) return;
	if (a < 0x8800) { s_audio_ram[a & 0x7ff] = v; return; }
	switch (a) {
	case 0xa000: case 0xa001: snd_sync(); rbx_ym_write(0, a & 1, v); return;
	case 0xc000: case 0xc001: snd_sync(); ay8910_write(&s_ay, a & 1, v); return;
	case 0xd000: if (s_kind != LS_DAIKAIJU) { s_latch2 = v; s_latch2_pending = 1; } return;
	case 0xd400: s_nmi_en = 0; update_nmi(); return;
	case 0xd800: s_nmi_en = 1; update_nmi(); return;
	}
}

static uint8_t no_in(z80 *z, uint8_t p) { (void)z; (void)p; return 0xff; }
static void no_out(z80 *z, uint8_t p, uint8_t v) { (void)z; (void)p; (void)v; }

static void zcpu_init(zcpu *c, uint8_t (*rd)(void *, uint16_t), void (*wr)(void *, uint16_t, uint8_t))
{
	unsigned long cyc = c->z.cyc;
	z80_init(&c->z);
	c->z.read_byte = rd;
	c->z.write_byte = wr;
	c->z.port_in = no_in;
	c->z.port_out = no_out;
	c->z.int_data = 0xff;
	c->z.cyc = cyc;
	c->irq = 0;
}

static void main_run(int cycles)
{
	zcpu *c = &s_main;
	unsigned long end = c->z.cyc + (unsigned long)cycles;
	while ((long)(c->z.cyc - end) < 0) {
		c->z.int_pending = c->irq ? 1 : 0;
		z80_step(&c->z);
		if (c->irq && !c->z.int_pending)
			c->irq = 0;   // HOLD_LINE acknowledged
	}
}

static void audio_run(int cycles)
{
	zcpu *c = &s_audio;
	unsigned long end = c->z.cyc + (unsigned long)cycles;
	while ((long)(c->z.cyc - end) < 0) {
		c->z.int_pending = s_ym_irq ? 1 : 0;   // level IRQ from the YM2203
		z80_step(&c->z);
	}
}

// ---- video ----
static void decode_gfx(void)
{
	// ROMREGION_INVERT (not applied by the loader)
	for (uint32_t i = 0; i < s_gfx1_size; i++) s_gfx1[i] ^= 0xff;
	for (uint32_t i = 0; i < s_gfx2_size; i++) s_gfx2[i] ^= 0xff;
	static const int xo[16] = { 3, 2, 1, 0, 11, 10, 9, 8, 131, 130, 129, 128, 139, 138, 137, 136 };
	uint32_t half = s_gfx1_size / 2 * 8;
	uint32_t planes[4] = { half, half + 4, 0, 4 };
	for (uint32_t t = 0; t < s_ntiles; t++)
		for (int y = 0; y < 8; y++)
			for (int x = 0; x < 8; x++) {
				int v = 0;
				for (int p = 0; p < 4; p++) {
					uint32_t b = t * 128 + planes[p] + (uint32_t)xo[x] + (uint32_t)y * 16;
					v = v << 1 | ((s_gfx1[b >> 3] >> (7 - (b & 7))) & 1);
				}
				s_tiles[t * 64 + y * 8 + x] = (uint8_t)v;
			}
	half = s_gfx2_size / 2 * 8;
	planes[0] = half; planes[1] = half + 4; planes[2] = 0; planes[3] = 4;
	for (uint32_t t = 0; t < s_nsprites; t++)
		for (int y = 0; y < 16; y++)
			for (int x = 0; x < 16; x++) {
				int v = 0;
				uint32_t yo = (uint32_t)(y < 8 ? y * 16 : 256 + (y - 8) * 16);
				for (int p = 0; p < 4; p++) {
					uint32_t b = t * 512 + planes[p] + (uint32_t)xo[x] + yo;
					v = v << 1 | ((s_gfx2[b >> 3] >> (7 - (b & 7))) & 1);
				}
				s_sprites[t * 256 + y * 16 + x] = (uint8_t)v;
			}
}

// 8x8 tile, transpen 15, both axes flipped when the screen is flipped
static void draw_tile(uint32_t code, int color, int flip, int sx, int sy)
{
	if (sx <= -8 || sx >= SCREEN_W || sy <= VIS_Y0 - 8 || sy > VIS_Y1)
		return;
	const uint8_t *g = s_tiles + (code % s_ntiles) * 64;
	const uint16_t base = (uint16_t)(color * 16);
	for (int ty = 0; ty < 8; ty++) {
		int y = sy + ty;
		if (y < VIS_Y0 || y > VIS_Y1) continue;
		const uint8_t *r = g + (flip ? 7 - ty : ty) * 8;
		uint16_t *d = rbx_video.bitmap + (y - VIS_Y0) * rbx_video.pitch;
		for (int tx = 0; tx < 8; tx++) {
			int x = sx + tx;
			if ((unsigned)x >= SCREEN_W) continue;
			uint8_t p = r[flip ? 7 - tx : tx];
			if (p != 15)
				d[x] = (uint16_t)(base + p);
		}
	}
}

static void draw_column(int base, int sx, int scrolly, int type)
{
	const int fl = s_snap_flip;
	for (int y = 0; y < 32; y++) {
		int sy = 8 * y + scrolly;
		if (fl) sy = 248 - sy;
		sy &= 0xff;
		int attr = s_snap_vram[(base + 2 * y + 1) & 0x1fff];
		uint32_t code = (uint32_t)(s_snap_vram[(base + 2 * y) & 0x1fff] + ((attr & 0x0f) << 8));
		int color = attr >> 4;
		if (type == 1 && color == 0x0d) continue;   // daikaiju: bottom pass
		if (type == 2 && color != 0x0d) continue;   // daikaiju: top pass
		draw_tile(code, color, fl, sx, sy);
		if (sx > 248)   // wraparound
			draw_tile(code, color, fl, sx - 256, sy);
	}
}

static void draw_layer(const uint8_t *scrollram)
{
	int scrollx = scrollram[3];
	int scrolly = -scrollram[0];
	for (int offs = 0; offs < 0x80; offs += 4) {
		int base = 64 * scrollram[offs + 1];
		int sx = 8 * (offs / 4) + scrollx;
		if (s_snap_flip) sx = 248 - sx;
		sx &= 0xff;
		draw_column(base, sx, scrolly, 0);
	}
}

// daikaiju_state::draw_layer_daikaiju (type 0 = bottom, 1 = top)
static int draw_layer_daikaiju(int offs, int *previd, int type)
{
	const uint8_t *sr = s_snap_scroll;
	int stepx = 0, initoffs = offs, globalscrollx = 0;
	int id = sr[offs + 2];
	for (; offs < 0x400; offs += 4) {
		if (id != sr[offs + 2]) {
			*previd = id;
			return offs;
		}
		id = sr[offs + 2];
		if ((sr[offs + 0] | sr[offs + 1] | sr[offs + 2] | sr[offs + 3]) == 0)
			continue;
		int scrolly = -sr[offs + 0];
		int scrollx = sr[offs + 3];
		if (*previd != 1) {
			if (offs != initoffs)
				scrollx += globalscrollx;
			else
				globalscrollx = scrollx;
		}
		int base = 64 * sr[offs + 1];
		int sx = scrollx + stepx;
		if (s_snap_flip) sx = 248 - sx;
		sx &= 0xff;
		draw_column(base, sx, scrolly, type ? 2 : 1);
	}
	return offs;
}

static void drawbg(int type)
{
	int i = 0, id = -1;
	while (i < 0x400) {
		if (!(s_snap_scroll[i + 2] & 1))
			i = draw_layer_daikaiju(i, &id, type);
		else {
			id = s_snap_scroll[i + 2];
			i += 4;
		}
	}
}

static void draw_sprite16(uint32_t code, int color, int flipx, int flipy, int sx, int sy)
{
	if (sx <= -16 || sx >= SCREEN_W || sy <= VIS_Y0 - 16 || sy > VIS_Y1)
		return;
	const uint8_t *g = s_sprites + (code % s_nsprites) * 256;
	const uint16_t base = (uint16_t)(256 + color * 16);
	for (int ty = 0; ty < 16; ty++) {
		int y = sy + ty;
		if (y < VIS_Y0 || y > VIS_Y1) continue;
		const uint8_t *r = g + (flipy ? 15 - ty : ty) * 16;
		uint16_t *d = rbx_video.bitmap + (y - VIS_Y0) * rbx_video.pitch;
		for (int tx = 0; tx < 16; tx++) {
			int x = sx + tx;
			if ((unsigned)x >= SCREEN_W) continue;
			uint8_t p = r[flipx ? 15 - tx : tx];
			if (p != 15)
				d[x] = (uint16_t)(base + p);
		}
	}
}

static void draw_sprites(void)
{
	for (int offs = (int)s_spr_size - 4; offs >= 0; offs -= 4) {
		const uint8_t *s = s_snap_spr + offs;
		int attr = s[1];
		uint32_t code = (uint32_t)(s[2] + ((attr & 0x30) << 4));
		int sx = s[3], sy = 240 - s[0];
		int color = attr & 0x0f;
		int flipx = (attr >> 6) & 1, flipy = (attr >> 7) & 1;
		if (s_snap_flip) {
			sx = 240 - sx;
			sy = 240 - sy;
			flipx = !flipx;
			flipy = !flipy;
		}
		draw_sprite16(code, color, flipx, flipy, sx, sy);
		draw_sprite16(code, color, flipx, flipy, sx - 256, sy);
	}
}

static void ls_render(void)
{
	memcpy(rbx_video.palette, s_pal, sizeof(s_pal));
	for (int y = 0; y < rbx_video.height; y++) {
		uint16_t *d = rbx_video.bitmap + y * rbx_video.pitch;
		for (int x = 0; x < SCREEN_W; x++)
			d[x] = 511;
	}
	if (s_kind == LS_DAIKAIJU) {
		drawbg(0);
		draw_sprites();
		drawbg(1);
	} else {
		draw_layer(s_snap_scroll + 0x000);
		draw_layer(s_snap_scroll + 0x080);
		draw_sprites();
		draw_layer(s_snap_scroll + 0x100);
	}
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap_vram, s_vram, sizeof(s_vram));
	memcpy(s_snap_scroll, s_scroll, sizeof(s_scroll));
	memcpy(s_snap_spr, s_spr, sizeof(s_spr));
	s_snap_flip = s_flip;
	rbx_video_frame_done();
}

// ---- init / frame ----
static int kind_of(const char *m)
{
	if (!strcmp(m, "lsasquad")) return LS_LSASQUAD;
	if (!strcmp(m, "storming")) return LS_STORMING;
	if (!strcmp(m, "daikaiju")) return LS_DAIKAIJU;
	return -1;
}

static void ls_reset(void)
{
	memset(s_sram, 0, sizeof(s_sram));
	memset(s_vram, 0, sizeof(s_vram));
	memset(s_scroll, 0, sizeof(s_scroll));
	memset(s_spr, 0, sizeof(s_spr));
	memset(s_audio_ram, 0, sizeof(s_audio_ram));
	memset(&s_main, 0, sizeof(s_main));
	memset(&s_audio, 0, sizeof(s_audio));
	zcpu_init(&s_main, main_read, main_write);
	zcpu_init(&s_audio, audio_read, audio_write);
	s_bank = 0;
	s_flip = 0;
	s_latch = s_latch2 = 0;
	s_latch_pending = s_latch2_pending = s_nmi_en = s_nmi_line = 0;
	s_ym_irq = 0;
	if (s_has_mcu)
		taito68705_reset(&s_mcu);
	rbx_ym_reset(0);
	ay8910_reset(&s_ay);
	s_frac_main = s_frac_audio = s_frac_mcu = 0;
	s_snd_done = 0;
	s_frame_cyc = s_audio.z.cyc;
	s_mix_n = s_mix2_n = 0;
}

static void ls_exit(void)
{
	free(s_tiles);
	free(s_sprites);
	s_tiles = s_sprites = NULL;
	free(s_mix);
	free(s_mix2);
	s_mix = s_mix2 = NULL;
	rbx_ym_exit(0);
}

static int ls_init(const rbx_game *g)
{
	s_kind = kind_of(g->machine);
	if (s_kind < 0) {
		rbx_fail("%s: machine %s is not emulated", g->set, g->machine);
		return -1;
	}
	s_has_mcu = s_kind != LS_STORMING;
	s_spr_size = s_kind == LS_DAIKAIJU ? 0x400 : 0x200;
	s_main_rom = rbx_region("maincpu", &s_main_size);
	s_audio_rom = rbx_region("audiocpu", &s_audio_size);
	s_gfx1 = rbx_region("gfx1", &s_gfx1_size);
	s_gfx2 = rbx_region("gfx2", &s_gfx2_size);
	s_prom = rbx_region("proms", &s_prom_size);
	if (!s_main_rom || s_main_size < 0x8000 || !s_audio_rom || !s_gfx1 || !s_gfx2 || !s_prom || s_prom_size < 0x600) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	if (s_has_mcu) {
		uint32_t msize;
		uint8_t *mrom = rbx_region("bmcu:mcu", &msize);
		memset(&s_mcu, 0, sizeof(s_mcu));
		if (!mrom || msize < 0x800 || taito68705_init(&s_mcu, TAITO68705_STD, mrom, 0x800)) {
			rbx_fail("%s: MCU ROM missing", g->set);
			return -1;
		}
	}
	s_p_dsw[0] = rbx_port_find("DSWA");
	s_p_dsw[1] = rbx_port_find("DSWB");
	s_p_dsw[2] = rbx_port_find("DSWC");
	s_p_mcu = rbx_port_find("MCU");
	s_p_coins = rbx_port_find("COINS");
	s_p_p1 = rbx_port_find("P1");
	s_p_p2 = rbx_port_find("P2");
	s_p_start = rbx_port_find("START");
	s_p_service = rbx_port_find("SERVICE");
	// RGB_444_PROMS
	for (int i = 0; i < 512; i++)
		s_pal[i] = (uint32_t)((s_prom[i] & 15) * 17 << 16 | (s_prom[i + 0x200] & 15) * 17 << 8 | (s_prom[i + 0x400] & 15) * 17);
	s_ntiles = s_gfx1_size / 2 / 16;
	s_nsprites = s_gfx2_size / 2 / 64;
	s_tiles = (uint8_t *)malloc((size_t)s_ntiles * 64);
	s_sprites = (uint8_t *)malloc((size_t)s_nsprites * 256);
	if (!s_tiles || !s_sprites) {
		ls_exit();
		return -1;
	}
	decode_gfx();
	rbx_ym_config yc;
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM2203;
	yc.clock = AUDIO_CLOCK;
	yc.irq = ym_irq;
	yc.gain[0] = yc.gain[1] = yc.gain[2] = 31;   // 0.12
	yc.gain[3] = 161;                            // 0.63
	if (rbx_ym_init(0, &yc)) {
		rbx_fail("%s: sound init failed", g->set);
		ls_exit();
		return -1;
	}
	uint32_t rate = rbx_ym_rate(0);
	// AY at about the YM rate (native rate 3 MHz / 8), resampled onto the YM stream at the frame end
	int div = (int)((AUDIO_CLOCK / 8 + rate / 2) / rate);
	if (div < 1) div = 1;
	ay8910_init(&s_ay, s_kind == LS_STORMING ? AY8910_TYPE_AY8910 : AY8910_TYPE_YM2149, AUDIO_CLOCK,
		AY8910_LEGACY_OUTPUT, 31, div, 1);   // 0.12
	uint32_t arate = ay8910_rate(&s_ay);
	s_mix_cap = (int)((rate > arate ? rate : arate) / 50) + 64;
	s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	s_mix2 = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	if (!s_mix || !s_mix2 || rbx_audio_setup(rate) || rbx_video_setup(SCREEN_W, VIS_Y1 - VIS_Y0 + 1, 512)) {
		ls_exit();
		return -1;
	}
	ls_reset();
	return 0;
}

static void ls_frame(void)
{
	s_frame_cyc = s_audio.z.cyc;
	s_snd_done = 0;
	for (int i = 0; i < LINES; i++) {
		int line = (VIS_Y1 + 1 + i) % LINES;
		if (line == VIS_Y1 + 1) {
			vblank_capture();
			s_main.irq = 1;   // irq0_line_hold
		}
		int mc = rbx_line_cycles(&s_frac_main, MAIN_CLOCK, RATE_MHZ, LINES * SLICES);
		int ac = rbx_line_cycles(&s_frac_audio, AUDIO_CLOCK, RATE_MHZ, LINES * SLICES);
		int uc = rbx_line_cycles(&s_frac_mcu, MCU_CLOCK, RATE_MHZ, LINES * SLICES);
		for (int s = 0; s < SLICES; s++) {
			main_run(mc);
			audio_run(ac);
			if (s_has_mcu)
				taito68705_execute(&s_mcu, uc);
			snd_sync();
		}
	}
	snd_sync();
	if (s_mix2_n && s_mix_n)   // AY output resampled onto the YM2203 stream (nearest sample)
		for (int i = 0; i < s_mix_n; i++)
			s_mix[i] += s_mix2[(int)((int64_t)i * s_mix2_n / s_mix_n)];
	for (int i = 0; i < s_mix_n; i++)
		rbx_audio_put(s_mix[i]);
	memset(s_mix, 0, sizeof(int32_t) * (size_t)s_mix_n);
	memset(s_mix2, 0, sizeof(int32_t) * (size_t)s_mix2_n);
	s_mix_n = s_mix2_n = 0;
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

static int ls_state(void *ctx, int load)
{
	int rc = 0;
	rc |= zcpu_state(ctx, "main", &s_main, load);
	rc |= zcpu_state(ctx, "audio", &s_audio, load);
	rc |= rbx_state_io(ctx, "sram", s_sram, sizeof(s_sram));
	rc |= rbx_state_io(ctx, "vram", s_vram, sizeof(s_vram));
	rc |= rbx_state_io(ctx, "scrl", s_scroll, sizeof(s_scroll));
	rc |= rbx_state_io(ctx, "spr", s_spr, sizeof(s_spr));
	rc |= rbx_state_io(ctx, "aram", s_audio_ram, sizeof(s_audio_ram));
	if (s_has_mcu) {
		rc |= rbx_state_io(ctx, "tmcu", &s_mcu.st, sizeof(s_mcu.st));
		rc |= rbx_state_io(ctx, "tmcu6805", &s_mcu.mcu.st, sizeof(s_mcu.mcu.st));
	}
	rc |= rbx_ym_state(0, ctx, load);
	rc |= rbx_state_io(ctx, "ay", &s_ay, ay8910_state_size());
	int32_t misc[10] = { s_bank, s_flip, s_latch, s_latch2, s_latch_pending, s_latch2_pending, s_nmi_en, s_nmi_line,
		s_ym_irq, 0 };
	rc |= rbx_state_io(ctx, "misc", misc, sizeof(misc));
	if (rc)
		return -1;
	if (load) {
		s_bank = (uint8_t)misc[0]; s_flip = (uint8_t)misc[1]; s_latch = (uint8_t)misc[2]; s_latch2 = (uint8_t)misc[3];
		s_latch_pending = misc[4]; s_latch2_pending = misc[5]; s_nmi_en = misc[6]; s_nmi_line = misc[7];
		s_ym_irq = misc[8];
		s_frame_cyc = s_audio.z.cyc;
		s_snd_done = 0;
	}
	return 0;
}

static int ls_save(void *ctx) { return ls_state(ctx, 0); }
static int ls_load(void *ctx) { return ls_state(ctx, 1); }
static int ls_supported(const char *machine) { return kind_of(machine) >= 0; }

const rbx_driver rbx_drv_lsasquad = { "Land Sea Air Squad", ls_init, ls_exit, ls_reset, ls_frame, ls_save, ls_load, ls_supported, ls_render };
