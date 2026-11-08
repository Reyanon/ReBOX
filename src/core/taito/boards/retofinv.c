/*  Return of the Invaders, ported from MAME src/mame/taito/retofinv.cpp (BSD-3-Clause, copyright-holders:
    Jarek Parchanski, Andrea Mazzoleni). Three Z80s (main, sub, sound), the LS259 control latch, the 36x28 tilemaps
    with the PROM colour lookup (original and bootleg scrambling), the sprites, the Taito 68705 MCU interface
    (machine/taito68705) and 2 x SN76489A (sound/sn76496) follow MAME.
    The original CLUT PROMs are nibble wide: tools/gen_games.py loads them into "clut:nib_high" / "clut:nib_low".
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/z80/z80.h"
#include "../machine/taito68705.h"
#include "../../sound/sn76496.h"

#include <stdlib.h>
#include <string.h>

enum { RI_MCU, RI_MCU_BL, RI_NOMCU, RI_NOMCU_BL };

#define SCREEN_W 288
#define SCREEN_H 224
#define LINES 256              // (MAME: vblank time 0, the interrupts come at the end of the frame)
#define RATE_MHZ 60580
#define CPU_CLOCK 3072000      // 18.432 MHz / 6
#define MCU_CLOCK 768000
#define NMI_PERIOD (CPU_CLOCK / 120)

typedef struct {
	z80 z;
	int irq;                   // line level (ASSERT until acknowledged through the latch)
	int reset;
} zcpu;

static int s_kind, s_has_mcu;
static uint8_t *s_main_rom, *s_sub_rom, *s_audio_rom;
static uint32_t s_main_size, s_sub_size, s_audio_size;
static zcpu s_main, s_sub, s_audio;
static uint8_t s_fg_vram[0x800], s_bg_vram[0x800], s_share[0x1800], s_audio_ram[0x800];
static uint8_t s_latch[8];   // LS259 outputs
static uint8_t s_cpu2_m6000, s_fg_bank, s_bg_bank, s_flip, s_soundlatch;
static int s_latch_pending;
static uint32_t s_nmi_count;
static taito68705_t s_mcu;
static sn76496_t s_sn[2];
// video
static uint8_t *s_fg_tiles, *s_bg_tiles, *s_sprites;
static uint32_t s_pal_rgb[256];
static uint8_t s_clut[0x800];        // pen -> indirect colour (sprites 0x000-0x3ff, bg 0x400-0x7ff)
static uint8_t s_snap_fg[0x800], s_snap_bg[0x800], s_snap_spr[0x1800], s_snap_fg_bank, s_snap_bg_bank, s_snap_flip;
// sound
static int32_t *s_mix;
static int s_mix_cap, s_mix_n;
static uint32_t s_snd_done, s_slice_start;
static unsigned long s_cpu_start;
static uint64_t s_frac_cpu, s_frac_mcu;
static int s_slice_cycles;
// ports
static int s_p_p1, s_p_p2, s_p_system, s_p_dsw[3];

static uint8_t port8(int p) { return (uint8_t)rbx_port_read(p); }

// ---- sound ----
static void snd_run_to(uint32_t pos)
{
	if (pos <= s_snd_done)
		return;
	int room = s_mix_cap - s_mix_n;
	if (room > 0) {
		int n = sn76496_render(&s_sn[0], pos - s_snd_done, s_mix + s_mix_n, room);
		sn76496_render(&s_sn[1], pos - s_snd_done, s_mix + s_mix_n, room);
		s_mix_n += n;
	}
	s_snd_done = pos;
}
static void snd_sync(void)
{
	uint32_t done = (uint32_t)(s_audio.z.cyc - s_cpu_start);
	if ((int)done > s_slice_cycles)
		done = (uint32_t)s_slice_cycles;
	snd_run_to(s_slice_start + done);   // (the PSGs run on the CPU clock)
}

// ---- LS259 ----
static void zcpu_reset(zcpu *c);

static void mainlatch_w(int offset, int bit)
{
	int old = s_latch[offset];
	s_latch[offset] = (uint8_t)bit;
	switch (offset) {
	case 0: if (!bit) s_main.irq = 0; break;   // irq0_ack_w
	case 2:   // audio CPU reset (inverted)
		if (!bit && old) zcpu_reset(&s_audio);
		s_audio.reset = !bit;
		break;
	case 3:
		if (s_has_mcu) taito68705_reset_w(&s_mcu, !bit);
		break;
	case 4: if (!bit) s_sub.irq = 0; break;    // irq1_ack_w
	case 5:   // sub CPU reset (inverted)
		if (!bit && old) zcpu_reset(&s_sub);
		s_sub.reset = !bit;
		break;
	}
}

// ---- main / sub CPUs ----
static uint8_t shared_read(uint16_t a, int *hit)
{
	*hit = 1;
	if (a >= 0x8000 && a < 0x8800) return s_fg_vram[a & 0x7ff];
	if (a >= 0x8800 && a < 0xa000) return s_share[a - 0x8800];
	if (a >= 0xa000 && a < 0xa800) return s_bg_vram[a & 0x7ff];
	*hit = 0;
	return 0;
}
static int shared_write(uint16_t a, uint8_t v)
{
	if (a >= 0x8000 && a < 0x8800) { s_fg_vram[a & 0x7ff] = v; return 1; }
	if (a >= 0x8800 && a < 0xa000) { s_share[a - 0x8800] = v; return 1; }
	if (a >= 0xa000 && a < 0xa800) { s_bg_vram[a & 0x7ff] = v; return 1; }
	if (a >= 0xc800 && a < 0xc808) { mainlatch_w(a & 7, v & 1); return 1; }
	return 0;
}

static uint8_t mcu_status_r(void)
{
	return (uint8_t)((taito68705_host_semaphore_r(&s_mcu) ? 0 : 0x10) | (taito68705_mcu_semaphore_r(&s_mcu) ? 0x20 : 0));
}

static uint8_t main_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x8000) return a < s_main_size ? s_main_rom[a] : 0xff;
	int hit;
	uint8_t v = shared_read(a, &hit);
	if (hit) return v;
	switch (a) {
	case 0xc000: return port8(s_p_p1);
	case 0xc001: return port8(s_p_p2);
	case 0xc003: return s_has_mcu ? mcu_status_r() : 0;
	case 0xc004: return port8(s_p_system);
	case 0xc005: return port8(s_p_dsw[0]);
	case 0xc006: return port8(s_p_dsw[1]);
	case 0xc007: return port8(s_p_dsw[2]);
	case 0xe000: return s_has_mcu ? taito68705_data_r(&s_mcu) : 0;
	case 0xf800: return s_cpu2_m6000;
	}
	return 0;
}

static void main_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (shared_write(a, v)) return;
	if (a >= 0xb800 && a <= 0xb802) {   // gfx_ctrl_w
		switch (a & 3) {
		case 0: s_flip = v & 1; break;
		case 1: s_fg_bank = v & 1; break;
		case 2: s_bg_bank = v & 1; break;
		}
		return;
	}
	if (a == 0xd800) { s_soundlatch = v; s_latch_pending = 1; return; }
	if (a == 0xe800 && s_has_mcu) { taito68705_data_w(&s_mcu, v); return; }
}

static uint8_t sub_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x8000) return a < s_sub_size ? s_sub_rom[a] : 0xff;
	int hit;
	uint8_t v = shared_read(a, &hit);
	return hit ? v : 0;
}
static void sub_write(void *u, uint16_t a, uint8_t v) { (void)u; shared_write(a, v); }

static uint8_t audio_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x2000 || a >= 0xe000) return a < s_audio_size ? s_audio_rom[a] : 0xff;
	if (a < 0x2800 && a >= 0x2000) return s_audio_ram[a & 0x7ff];
	if (a == 0x4000) { s_latch_pending = 0; return s_soundlatch; }
	return 0;
}
static void audio_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0x2000 && a < 0x2800) { s_audio_ram[a & 0x7ff] = v; return; }
	if (a == 0x6000) { s_cpu2_m6000 = v; return; }
	if (a == 0x8000) { snd_sync(); sn76496_write(&s_sn[0], v); return; }
	if (a == 0xa000) { snd_sync(); sn76496_write(&s_sn[1], v); return; }
}

static uint8_t no_in(z80 *z, uint8_t p) { (void)z; (void)p; return 0xff; }
static void no_out(z80 *z, uint8_t p, uint8_t v) { (void)z; (void)p; (void)v; }

static void zcpu_setup(zcpu *c, uint8_t (*rd)(void *, uint16_t), void (*wr)(void *, uint16_t, uint8_t))
{
	unsigned long cyc = c->z.cyc;
	z80_init(&c->z);
	c->z.read_byte = rd;
	c->z.write_byte = wr;
	c->z.port_in = no_in;
	c->z.port_out = no_out;
	c->z.int_data = 0xff;
	c->z.cyc = cyc;
}
static void zcpu_reset(zcpu *c)
{
	if (c == &s_sub) zcpu_setup(c, sub_read, sub_write);
	else if (c == &s_audio) zcpu_setup(c, audio_read, audio_write);
	else zcpu_setup(c, main_read, main_write);
}

static void zcpu_run(zcpu *c, int cycles, int level)
{
	unsigned long end = c->z.cyc + (unsigned long)cycles;
	if (c->reset) {
		c->z.cyc = end;
		return;
	}
	while ((long)(c->z.cyc - end) < 0) {
		c->z.int_pending = level ? 1 : 0;
		z80_step(&c->z);
	}
}

// ---- video ----
static int tile_index(int col, int row)   // MAME tilemap_scan (36x28 -> 32x32)
{
	row += 2;
	col -= 2;
	return (col & 0x20) ? ((col & 0x1f) << 5) + row : (row << 5) + col;
}

static uint8_t bitswap8(uint8_t v, int b7, int b6, int b5, int b4, int b3, int b2, int b1, int b0)
{
	return (uint8_t)(((v >> b7) & 1) << 7 | ((v >> b6) & 1) << 6 | ((v >> b5) & 1) << 5 | ((v >> b4) & 1) << 4 |
	                 ((v >> b3) & 1) << 3 | ((v >> b2) & 1) << 2 | ((v >> b1) & 1) << 1 | ((v >> b0) & 1));
}

static int decode_gfx(const char *set)
{
	uint32_t fsz, ssz, bsz, psz, csz;
	uint8_t *fg = rbx_region("fgtiles", &fsz), *sp = rbx_region("sprites", &ssz), *bg = rbx_region("bgtiles", &bsz);
	uint8_t *pal = rbx_region("palette", &psz), *clut = rbx_region("clut", &csz);
	if (!fg || !sp || !bg || !pal || psz < 0x300 || fsz < 0x2000 || ssz < 0x8000 || bsz < 0x4000) {
		rbx_fail("%s: ROM regions missing", set);
		return -1;
	}
	uint8_t raw[0x800];
	uint32_t hs, ls;
	uint8_t *hi = rbx_region("clut:nib_high", &hs), *lo = rbx_region("clut:nib_low", &ls);
	const int bootleg = s_kind == RI_MCU_BL || s_kind == RI_NOMCU_BL;
	if (hi && lo && hs >= 0x800 && ls >= 0x800) {
		for (int i = 0; i < 0x800; i++)
			raw[i] = (uint8_t)((hi[i] & 0x0f) << 4 | (lo[i] & 0x0f));
	} else if (clut && csz >= 0x800)
		memcpy(raw, clut, 0x800);
	else {
		rbx_fail("%s: colour lookup PROMs missing", set);
		return -1;
	}
	for (int i = 0; i < 256; i++)
		s_pal_rgb[i] = (uint32_t)((pal[i] & 15) * 17 << 16 | (pal[i | 0x100] & 15) * 17 << 8 | (pal[i | 0x200] & 15) * 17);
	for (int i = 0; i < 0x800; i++) {
		if (bootleg)
			s_clut[i] = bitswap8(raw[i], 4, 5, 6, 7, 3, 2, 1, 0);
		else {
			int j = (i & ~7) | ((i & 1) << 2) | (i & 2) | ((i >> 2) & 1);   // bitswap(.., 0, 1, 2) of the low bits
			s_clut[i] = raw[j];
		}
	}
	s_fg_tiles = (uint8_t *)malloc(512 * 64);
	s_bg_tiles = (uint8_t *)malloc(512 * 64);
	s_sprites = (uint8_t *)malloc(256 * 256);
	if (!s_fg_tiles || !s_bg_tiles || !s_sprites)
		return -1;
	for (int t = 0; t < 512; t++)   // 1bpp, x = bit 0 first
		for (int y = 0; y < 8; y++)
			for (int x = 0; x < 8; x++)
				s_fg_tiles[t * 64 + y * 8 + x] = (uint8_t)((fg[t * 8 + y] >> x) & 1);
	// planes (MSB first): 0, half + 4, half, 4 (bit offsets, bit 0 = 0x80)
	const uint32_t bh = 0x2000 * 8, sh = 0x4000 * 8;
	const uint32_t bpl[4] = { 0, bh + 4, bh, 4 }, spl[4] = { 0, sh + 4, sh, 4 };
	static const int bxo[8] = { 0, 1, 2, 3, 64, 65, 66, 67 };
	static const int sxo[16] = { 0, 1, 2, 3, 64, 65, 66, 67, 128, 129, 130, 131, 192, 193, 194, 195 };
	for (int t = 0; t < 512; t++)
		for (int y = 0; y < 8; y++)
			for (int x = 0; x < 8; x++) {
				int v = 0;
				for (int p = 0; p < 4; p++) {
					uint32_t b = (uint32_t)t * 128 + bpl[p] + (uint32_t)bxo[x] + (uint32_t)y * 8;
					v = v << 1 | ((bg[b >> 3] >> (7 - (b & 7))) & 1);
				}
				s_bg_tiles[t * 64 + y * 8 + x] = (uint8_t)v;
			}
	for (int t = 0; t < 256; t++)
		for (int y = 0; y < 16; y++)
			for (int x = 0; x < 16; x++) {
				int v = 0;
				uint32_t yo = (uint32_t)(y < 8 ? y * 8 : 256 + (y - 8) * 8);
				for (int p = 0; p < 4; p++) {
					uint32_t b = (uint32_t)t * 512 + spl[p] + (uint32_t)sxo[x] + yo;
					v = v << 1 | ((sp[b >> 3] >> (7 - (b & 7))) & 1);
				}
				s_sprites[t * 256 + y * 16 + x] = (uint8_t)v;
			}
	return 0;
}

static void ri_render(void)
{
	for (int i = 0; i < 256; i++)
		rbx_video.palette[i] = s_pal_rgb[i];
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch, fl = s_snap_flip;
	// background (opaque) then sprites then foreground
	for (int y = 0; y < SCREEN_H; y++) {
		uint16_t *d = bm + y * pitch;
		int ty = fl ? SCREEN_H - 1 - y : y;
		for (int x = 0; x < SCREEN_W; x++) {
			int tx = fl ? SCREEN_W - 1 - x : x;
			int ti = tile_index(tx >> 3, ty >> 3);
			int code = s_snap_bg[ti] + 256 * s_snap_bg_bank;
			int color = s_snap_bg[0x400 + ti] & 0x3f;
			int px = tx & 7, py = ty & 7;   // (a flipped tilemap mirrors the whole layer)
			d[x] = s_clut[0x400 + color * 16 + s_bg_tiles[(code & 511) * 64 + py * 8 + px]];
		}
	}
	const uint8_t *s1 = s_snap_spr + 0x780, *s2 = s_snap_spr + 0xf80, *s3 = s_snap_spr + 0x1780;
	static const int gfx_offs[2][2] = { { 0, 1 }, { 2, 3 } };
	for (int offs = 0; offs < 0x80; offs += 2) {
		int sprite = s1[offs];
		int color = s1[offs + 1] & 0x3f;
		int sx = ((s2[offs + 1] << 1) + ((s3[offs + 1] & 0x80) >> 7)) - 39;
		int sy = 256 - ((s2[offs] << 1) + ((s3[offs] & 0x80) >> 7)) + 1;
		int flipx = s3[offs] & 1, flipy = (s3[offs] & 2) >> 1;
		int sizey = (s3[offs] & 4) >> 2, sizex = (s3[offs] & 8) >> 3;
		sprite &= ~sizex;
		sprite &= ~(sizey << 1);
		if (fl) { flipx ^= 1; flipy ^= 1; }
		sy -= 16 * sizey;
		sy = (sy & 0xff) - 32;
		for (int yy = 0; yy <= sizey; yy++)
			for (int xx = 0; xx <= sizex; xx++) {
				int code = (sprite + gfx_offs[yy ^ (sizey * flipy)][xx ^ (sizex * flipx)]) & 255;
				const uint8_t *g = s_sprites + code * 256;
				int ox = sx + 16 * xx, oy = sy + 16 * yy;
				for (int ty = 0; ty < 16; ty++) {
					int y = oy + ty;
					if (y < 0 || y >= SCREEN_H) continue;
					const uint8_t *r = g + (flipy ? 15 - ty : ty) * 16;
					uint16_t *d = bm + y * pitch;
					for (int tx = 0; tx < 16; tx++) {
						int x = ox + tx;
						if (x < 16 || x > 34 * 8 - 1) continue;
						uint8_t c = s_clut[color * 16 + r[flipx ? 15 - tx : tx]];
						if (c != 0xff)
							d[x] = c;
					}
				}
			}
	}
	for (int y = 0; y < SCREEN_H; y++) {
		uint16_t *d = bm + y * pitch;
		int ty = fl ? SCREEN_H - 1 - y : y;
		for (int x = 0; x < SCREEN_W; x++) {
			int tx = fl ? SCREEN_W - 1 - x : x;
			int ti = tile_index(tx >> 3, ty >> 3);
			int color = s_snap_fg[0x400 + ti];
			if (!color)
				continue;
			int code = s_snap_fg[ti] + 256 * s_snap_fg_bank;
			int px = tx & 7, py = ty & 7;
			if (s_fg_tiles[(code & 511) * 64 + py * 8 + px])
				d[x] = (uint16_t)color;
		}
	}
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap_fg, s_fg_vram, sizeof(s_fg_vram));
	memcpy(s_snap_bg, s_bg_vram, sizeof(s_bg_vram));
	memcpy(s_snap_spr, s_share, sizeof(s_share));
	s_snap_fg_bank = s_fg_bank;
	s_snap_bg_bank = s_bg_bank;
	s_snap_flip = s_flip;
	rbx_video_frame_done();
}

// ---- init / frame ----
static int kind_of(const char *m)
{
	if (!strcmp(m, "retofinv")) return RI_MCU;
	if (!strcmp(m, "retofinvb1")) return RI_MCU_BL;
	if (!strcmp(m, "retofinvb_nomcu")) return RI_NOMCU;
	if (!strcmp(m, "retofinvb1_nomcu")) return RI_NOMCU_BL;
	return -1;
}

static void ri_reset(void)
{
	// (first: the sound syncs read them, from a CPU write or a port callback of a chip reset below, before a frame
	// sets them; the last game's values made a render of billions of clocks, the next game of the board hung)
	s_slice_start = s_cpu_start = 0;
	memset(s_fg_vram, 0, sizeof(s_fg_vram));
	memset(s_bg_vram, 0, sizeof(s_bg_vram));
	memset(s_share, 0, sizeof(s_share));
	memset(s_audio_ram, 0, sizeof(s_audio_ram));
	memset(&s_main, 0, sizeof(s_main));
	memset(&s_sub, 0, sizeof(s_sub));
	memset(&s_audio, 0, sizeof(s_audio));
	zcpu_reset(&s_main);
	zcpu_reset(&s_sub);
	zcpu_reset(&s_audio);
	s_cpu2_m6000 = s_fg_bank = s_bg_bank = s_flip = s_soundlatch = 0;
	s_latch_pending = 0;
	s_nmi_count = 0;
	if (s_has_mcu)
		taito68705_reset(&s_mcu);
	// LS259 cleared: sub, sound CPU and MCU held in reset, interrupts masked
	memset(s_latch, 1, sizeof(s_latch));
	for (int i = 0; i < 8; i++)
		mainlatch_w(i, 0);
	sn76496_reset(&s_sn[0]);
	sn76496_reset(&s_sn[1]);
	s_frac_cpu = s_frac_mcu = 0;
	s_snd_done = 0;
	s_mix_n = 0;
}

static void ri_exit(void)
{
	free(s_fg_tiles);
	free(s_bg_tiles);
	free(s_sprites);
	s_fg_tiles = s_bg_tiles = s_sprites = NULL;
	free(s_mix);
	s_mix = NULL;
}

static int ri_init(const rbx_game *g)
{
	s_kind = kind_of(g->machine);
	if (s_kind < 0) {
		rbx_fail("%s: machine %s is not emulated", g->set, g->machine);
		return -1;
	}
	s_has_mcu = s_kind == RI_MCU || s_kind == RI_MCU_BL;
	s_main_rom = rbx_region("maincpu", &s_main_size);
	s_sub_rom = rbx_region("sub", &s_sub_size);
	s_audio_rom = rbx_region("audiocpu", &s_audio_size);
	if (!s_main_rom || !s_sub_rom || !s_audio_rom) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	if (s_has_mcu) {
		uint32_t msize;
		uint8_t *mrom = rbx_region("68705:mcu", &msize);
		memset(&s_mcu, 0, sizeof(s_mcu));
		if (!mrom || msize < 0x800 || taito68705_init(&s_mcu, TAITO68705_STD, mrom, 0x800)) {
			rbx_fail("%s: MCU ROM missing", g->set);
			return -1;
		}
	}
	if (decode_gfx(g->set)) {
		ri_exit();
		return -1;
	}
	s_p_p1 = rbx_port_find("P1");
	s_p_p2 = rbx_port_find("P2");
	s_p_system = rbx_port_find("SYSTEM");
	s_p_dsw[0] = rbx_port_find("DSW1");
	s_p_dsw[1] = rbx_port_find("DSW2");
	s_p_dsw[2] = rbx_port_find("DSW3");
	// SN76489A x 2, route 0.80; 3.072 MHz / 2 / 32 = 48 kHz
	sn76496_init(&s_sn[0], CPU_CLOCK, SN76489A_CONFIG, 205, 32, 1);
	sn76496_init(&s_sn[1], CPU_CLOCK, SN76489A_CONFIG, 205, 32, 1);
	uint32_t rate = sn76496_rate(&s_sn[0]);
	s_mix_cap = (int)(rate / 50) + 64;
	s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	if (!s_mix || rbx_audio_setup(rate) || rbx_video_setup(SCREEN_W, SCREEN_H, 256)) {
		ri_exit();
		return -1;
	}
	ri_reset();
	return 0;
}

static void ri_frame(void)
{
	rbx_audio.frame_mhz = RATE_MHZ;   // (the board's frame rate: a frame's sound is played in a 60 Hz frame, as its picture)
	for (int i = 0; i < LINES; i++) {
		int cc = rbx_line_cycles(&s_frac_cpu, CPU_CLOCK, RATE_MHZ, LINES);
		int uc = rbx_line_cycles(&s_frac_mcu, MCU_CLOCK, RATE_MHZ, LINES);
		zcpu_run(&s_main, cc, s_main.irq);
		zcpu_run(&s_sub, cc, s_sub.irq);
		s_slice_start = s_snd_done;
		s_slice_cycles = cc;
		s_cpu_start = s_audio.z.cyc;
		if (!s_audio.reset) {
			s_nmi_count += (uint32_t)cc;
			if (s_nmi_count >= NMI_PERIOD) {
				s_nmi_count -= NMI_PERIOD;
				s_audio.z.nmi_pending = 1;   // nmi_line_pulse at 120 Hz
			}
		}
		zcpu_run(&s_audio, cc, s_latch_pending);
		if (s_has_mcu)
			taito68705_execute(&s_mcu, uc);
		snd_run_to(s_slice_start + (uint32_t)cc);
		if (i == LINES - 1) {
			// end of the visible frame: vblank interrupts (asserted while unmasked)
			vblank_capture();
			if (s_latch[0]) s_main.irq = 1;
			if (s_latch[4]) s_sub.irq = 1;
		}
	}
	for (int i = 0; i < s_mix_n; i++)
		rbx_audio_put(s_mix[i]);
	memset(s_mix, 0, sizeof(int32_t) * (size_t)s_mix_n);
	s_mix_n = 0;
	s_snd_done = 0;
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

static int ri_state(void *ctx, int load)
{
	int rc = 0;
	rc |= zcpu_state(ctx, "main", &s_main, load);
	rc |= zcpu_state(ctx, "sub", &s_sub, load);
	rc |= zcpu_state(ctx, "audio", &s_audio, load);
	rc |= rbx_state_io(ctx, "fg", s_fg_vram, sizeof(s_fg_vram));
	rc |= rbx_state_io(ctx, "bg", s_bg_vram, sizeof(s_bg_vram));
	rc |= rbx_state_io(ctx, "share", s_share, sizeof(s_share));
	rc |= rbx_state_io(ctx, "aram", s_audio_ram, sizeof(s_audio_ram));
	rc |= rbx_state_io(ctx, "latch", s_latch, sizeof(s_latch));
	rc |= rbx_state_io(ctx, "sn0", &s_sn[0], sn76496_state_size());
	rc |= rbx_state_io(ctx, "sn1", &s_sn[1], sn76496_state_size());
	if (s_has_mcu) {
		rc |= rbx_state_io(ctx, "tmcu", &s_mcu.st, sizeof(s_mcu.st));
		rc |= rbx_state_io(ctx, "tmcu6805", &s_mcu.mcu.st, sizeof(s_mcu.mcu.st));
	}
	int32_t misc[8] = { s_cpu2_m6000, s_fg_bank, s_bg_bank, s_flip, s_soundlatch, s_latch_pending, (int32_t)s_nmi_count, 0 };
	rc |= rbx_state_io(ctx, "misc", misc, sizeof(misc));
	if (rc)
		return -1;
	if (load) {
		s_cpu2_m6000 = (uint8_t)misc[0]; s_fg_bank = (uint8_t)misc[1]; s_bg_bank = (uint8_t)misc[2]; s_flip = (uint8_t)misc[3];
		s_soundlatch = (uint8_t)misc[4]; s_latch_pending = misc[5]; s_nmi_count = (uint32_t)misc[6];
	}
	return 0;
}

static int ri_save(void *ctx) { return ri_state(ctx, 0); }
static int ri_load(void *ctx) { return ri_state(ctx, 1); }
static int ri_supported(const char *machine) { return kind_of(machine) >= 0; }

const rbx_driver rbx_drv_retofinv = { "Return of the Invaders", ri_init, ri_exit, ri_reset, ri_frame, ri_save, ri_load, ri_supported, ri_render };
