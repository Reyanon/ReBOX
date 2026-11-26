/*  N.Y. Captor / Colt / Cycle Shooting / Bronx, ported from MAME src/mame/taito/nycaptor.cpp
    (BSD-3-Clause, copyright-holders: Tomasz Slanina). Memory maps of the three machine configs (nycaptor, cyclshtg,
    bronx), the main / sub Z80 pair sharing video RAM, the column-scrolled tilemap with its four transparency groups
    and "spot" dependent layer / sprite order, the banked xBGR_444 palette, the light gun reads, the Taito 68705 MCU
    (machine/taito68705, nycaptor only; the others use MAME's MCU stubs) and the sound board (Z80 + 2 x AY-3-8910 +
    MSM5232 + 8-bit DAC) follow MAME.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/z80/z80.h"
#include "../../machine/taito68705.h"
#include "../../sound/ay8910.h"
#include "../../sound/msm5232.h"

#include <stdlib.h>
#include <string.h>

enum { NY_NYCAPTOR, NY_CYCLSHTG, NY_BRONX };

// MAME: 60 Hz, 256 x 256, visible 0-255 x 16-239, vblank_int at line 240
#define LINES 256
#define VIS_Y0 16
#define VIS_Y1 239
#define SCREEN_W 256
#define VIS_H (VIS_Y1 - VIS_Y0 + 1)
#define RATE_MHZ 60000
#define CPU_CLOCK 4000000
#define MCU_CLOCK 500000      // 2 MHz / 4
#define SND_CLOCK 2000000
#define AUDIO_IRQ_PERIOD 32768   // 0x10000 ticks of 8 MHz in 4 MHz CPU cycles (~122 Hz)

typedef struct {
	z80 z;
	int irq;
} zcpu;

static int s_kind, s_gametype, s_has_mcu;
static uint8_t *s_main_rom, *s_sub_rom, *s_audio_rom, *s_user1, *s_gfx;
static uint32_t s_main_size, s_sub_size, s_audio_size, s_user1_size, s_gfx_size;
static uint32_t s_vram_mask, s_sub_rom_end;
static zcpu s_main, s_sub, s_audio;
static int s_audio_reset, s_sub_halt, s_sub_reset;
static uint8_t s_vram[0x1000], s_spr[0xa0], s_scrl[0x20], s_shared[0x2000], s_audio_ram[0x800];
static uint8_t s_pal[0x200], s_pal_ext[0x200];
static uint8_t s_gfxctrl, s_char_bank, s_pal_bank, s_generic, s_bank;
static uint8_t s_latch0, s_latch1;
static int s_latch0_pending, s_latch1_pending, s_nmi_in1, s_nmi_line;
static uint32_t s_rand;
static taito68705_t s_mcu;
// tilemap cache: the group of a tile is computed when MAME realizes a dirty tile (it depends on spot())
static uint8_t s_tile_dirty[0x400], s_tile_group[0x400];
// light gun (LIGHTX / LIGHTY)
static int s_p_gun[2], s_gun_type[2], s_gun[2];
// sound
static ay8910_t s_ay[2];
static msm5232_t s_msm;
static int32_t s_dac_out;
static uint32_t s_audio_irq_count;
static int32_t *s_mix;
static int s_mix_cap, s_mix_n;
static uint32_t s_snd_done, s_slice_start, s_slice_len;
static unsigned long s_cpu_start;
static int s_cpu_len;
static uint64_t s_frac_main, s_frac_sub, s_frac_audio, s_frac_mcu, s_frac_snd;
// video
static uint8_t *s_tiles, *s_sprites;
static uint32_t s_ntiles, s_nsprites;
static uint8_t s_snap_vram[0x800], s_snap_group[0x400], s_snap_spr[0xa0], s_snap_scrl[0x20];
static uint8_t s_snap_pal[0x200], s_snap_pal_ext[0x200];
static uint8_t s_snap_char_bank;
static int s_snap_spot;
static int8_t (*s_key_hp)[SCREEN_W * VIS_H];
#define s_key (*s_key_hp)
// ports
static int s_p_dsw[3], s_p_in0, s_p_in1;

static uint8_t port8(int p) { return (uint8_t)rbx_port_read(p); }

static uint8_t mame_rand(void)
{
	s_rand = s_rand * 1103515245u + 12345u;
	return (uint8_t)(s_rand >> 16);
}

// ---- sound ----
static void snd_run_to(uint32_t pos)
{
	if (pos <= s_snd_done)
		return;
	uint32_t clk = pos - s_snd_done;
	int room = s_mix_cap - s_mix_n;
	if (room <= 0) {
		s_snd_done = pos;
		return;
	}
	int32_t *o = s_mix + s_mix_n;
	int n = ay8910_render(&s_ay[0], clk, o, room);
	ay8910_render(&s_ay[1], clk, o, room);
	for (int k = 0; k < n; k++)
		o[k] += s_dac_out;
	msm5232_render(&s_msm, clk, o, room);
	s_mix_n += n;
	s_snd_done = pos;
}

static void snd_sync(void)
{
	uint32_t done = (uint32_t)(s_audio.z.cyc - s_cpu_start);
	if ((int)done > s_cpu_len)
		done = (uint32_t)s_cpu_len;
	snd_run_to(s_slice_start + (s_cpu_len ? (uint32_t)((uint64_t)s_slice_len * done / (uint32_t)s_cpu_len) : 0));
}

static void update_nmi(void)
{
	int line = s_latch0_pending && s_nmi_in1;   // INPUT_MERGER_ALL_HIGH
	if (line && !s_nmi_line)
		s_audio.z.nmi_pending = 1;
	s_nmi_line = line;
}

static uint8_t sound_status_r(void) { return (uint8_t)((s_latch0_pending ? 1 : 0) | (s_latch1_pending ? 2 : 0)); }

// ---- shared video / palette ----
static int spot(void)
{
	if (s_gametype == 0 || s_gametype == 2)
		return s_shared[0x299] ? s_shared[0x298] : 0;
	return 0;
}

static void videoram_w(uint32_t o, uint8_t v)
{
	s_vram[o] = v;
	if ((o >> 1) < 0x400)
		s_tile_dirty[o >> 1] = 1;
}

static uint8_t palette_r(uint32_t o)
{
	int i = (int)(o & 0xff) + (s_pal_bank << 8);
	return (o & 0x100) ? s_pal_ext[i] : s_pal[i];
}

static void palette_w(uint32_t o, uint8_t v)
{
	int i = (int)(o & 0xff) + (s_pal_bank << 8);
	if (o & 0x100) s_pal_ext[i] = v; else s_pal[i] = v;
}

static void gfxctrl_w(uint8_t v)
{
	s_gfxctrl = v;
	uint8_t cb = (uint8_t)((v & 0x18) >> 3);
	if (cb != s_char_bank) {
		s_char_bank = cb;
		memset(s_tile_dirty, 1, sizeof(s_tile_dirty));
	}
	s_pal_bank = (v >> 5) & 1;
}

static void zcpu_init(zcpu *c, uint8_t (*rd)(void *, uint16_t), void (*wr)(void *, uint16_t, uint8_t));
static uint8_t main_read(void *u, uint16_t a);
static void main_write(void *u, uint16_t a, uint8_t v);
static uint8_t sub_read(void *u, uint16_t a);
static void sub_write(void *u, uint16_t a, uint8_t v);
static uint8_t audio_read(void *u, uint16_t a);
static void audio_write(void *u, uint16_t a, uint8_t v);

static void generic_control_w(uint8_t v)
{
	s_generic = v;
	if (s_kind == NY_NYCAPTOR)
		s_bank = (v >> 3) & 1;
	else {
		s_bank = (v >> 2) & 3;
		if (s_gametype == 2) {   // colt: sub CPU reset on bit 1 low
			int r = !(v & 2);
			if (r && !s_sub_reset)
				zcpu_init(&s_sub, sub_read, sub_write);
			s_sub_reset = r;
		}
	}
}

static uint8_t gun_r(int axis) { return (uint8_t)s_gun[axis]; }
static uint8_t bx_r(void) { return (uint8_t)((gun_r(0) + 0x27) | 1); }
static uint8_t by_r(void)
{
	int port = gun_r(1);
	if (s_gametype == 1)
		port = 255 - port;
	return (uint8_t)(port - 8);
}

// ---- main CPU ----
static uint8_t main_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x8000) return a < s_main_size ? s_main_rom[a] : 0;
	if (a < 0xc000) {
		uint32_t o = 0x10000u + s_bank * 0x4000u + (a - 0x8000u);
		return o < s_main_size ? s_main_rom[o] : 0;
	}
	if (a < 0xd000) return (uint32_t)(a - 0xc000) <= s_vram_mask ? s_vram[a - 0xc000] : 0;
	if (a >= 0xe000) return s_shared[a & 0x1fff];
	if (a >= 0xdc00 && a < 0xdca0) return s_spr[a - 0xdc00];
	if (a >= 0xdca0 && a < 0xdcc0) return s_scrl[a - 0xdca0];
	if (a >= 0xdd00 && a < 0xdf00) return palette_r(a - 0xdd00);
	switch (a) {
	case 0xd000:
		if (s_kind == NY_NYCAPTOR) return taito68705_data_r(&s_mcu);
		return 7;   // cyclshtg_state::mcu_r
	case 0xd002: return s_generic;
	case 0xd400: s_latch1_pending = 0; return s_latch1;
	case 0xd401: return s_kind == NY_BRONX ? mame_rand() : 0;
	case 0xd800: return port8(s_p_dsw[0]);
	case 0xd801: return port8(s_p_dsw[1]);
	case 0xd802: return port8(s_p_dsw[2]);
	case 0xd803: return port8(s_p_in0);
	case 0xd804: return port8(s_p_in1);
	case 0xd805:
		if (s_kind == NY_NYCAPTOR) return taito68705_mcu_semaphore_r(&s_mcu) ? 2 : 0;   // mcu_status_r1
		return 0xff;
	case 0xd806: return sound_status_r();
	case 0xd807:
		if (s_kind == NY_NYCAPTOR) return taito68705_host_semaphore_r(&s_mcu) ? 0 : 1;   // mcu_status_r2
		return 0xff;
	case 0xdf03: return s_gfxctrl;
	}
	return 0;
}

static void main_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a < 0xc000) return;
	if (a < 0xd000) {
		if ((uint32_t)(a - 0xc000) <= s_vram_mask) videoram_w(a - 0xc000u, v);
		return;
	}
	if (a >= 0xe000) { s_shared[a & 0x1fff] = v; return; }
	if (a >= 0xdc00 && a < 0xdca0) { s_spr[a - 0xdc00] = v; return; }
	if (a >= 0xdca0 && a < 0xdcc0) { s_scrl[a - 0xdca0] = v; return; }
	if (a >= 0xdd00 && a < 0xdf00) { palette_w(a - 0xdd00u, v); return; }
	switch (a) {
	case 0xd000: if (s_kind == NY_NYCAPTOR) taito68705_data_w(&s_mcu, v); return;
	case 0xd001: s_sub_halt = v != 0; return;   // sub_cpu_halt_w
	case 0xd002: generic_control_w(v); return;
	case 0xd400: s_latch0 = v; s_latch0_pending = 1; update_nmi(); return;
	case 0xd403: {   // sound_cpu_reset_w
		int r = v & 1;
		if (r && !s_audio_reset)
			zcpu_init(&s_audio, audio_read, audio_write);
		s_audio_reset = r;
		return;
	}
	case 0xdf03: gfxctrl_w(v); return;
	}
}

// ---- sub CPU ----
static uint8_t sub_read(void *u, uint16_t a)
{
	(void)u;
	if (a < s_sub_rom_end) return a < s_sub_size ? s_sub_rom[a] : 0;
	if (a >= 0xc000 && a < 0xd000) return (uint32_t)(a - 0xc000) <= s_vram_mask ? s_vram[a - 0xc000] : 0;
	if (a >= 0xe000) return s_shared[a & 0x1fff];
	if (a >= 0xdc00 && a < 0xdca0) return s_spr[a - 0xdc00];
	if (a >= 0xdd00 && a < 0xdf00) return palette_r(a - 0xdd00);
	switch (a) {
	case 0xd800: return port8(s_p_dsw[0]);
	case 0xd801: return port8(s_p_dsw[1]);
	case 0xd802: return port8(s_p_dsw[2]);
	case 0xd803: return port8(s_p_in0);
	case 0xd804: return port8(s_p_in1);
	case 0xd805: return s_kind == NY_BRONX ? mame_rand() : 0;   // cyclshtg_state::mcu_status_r1
	case 0xd807: return s_kind == NY_BRONX ? 0xff : 0;
	case 0xdf00: return bx_r();
	case 0xdf01: return by_r();
	case 0xdf02: return 1;   // b_r
	case 0xdf03: return s_gfxctrl;
	}
	return 0;
}

static void sub_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0xc000 && a < 0xd000) {
		if ((uint32_t)(a - 0xc000) <= s_vram_mask) videoram_w(a - 0xc000u, v);
		return;
	}
	if (a >= 0xe000) { s_shared[a & 0x1fff] = v; return; }
	if (a >= 0xdc00 && a < 0xdca0) { s_spr[a - 0xdc00] = v; return; }
	if (a >= 0xdca0 && a < 0xdcc0) { s_scrl[a - 0xdca0] = v; return; }
	if (a >= 0xdd00 && a < 0xdf00) { palette_w(a - 0xdd00u, v); return; }
	if (a == 0xdf03 && s_kind == NY_BRONX) gfxctrl_w(v);
}

// bronx_slave_io_map: 0x0000-0x7fff = "user1"
static uint8_t sub_in(z80 *z, uint8_t p)
{
	(void)p;
	uint16_t a = z->mem_ptr;
	if (a < 0x8000 && s_user1 && a < s_user1_size) return s_user1[a];
	return 0;
}

// ---- sound CPU ----
static uint8_t audio_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0xc000) return a < s_audio_size ? s_audio_rom[a] : 0;
	if (a < 0xc800) return s_audio_ram[a & 0x7ff];
	if (a == 0xd000) { s_latch0_pending = 0; update_nmi(); return s_latch0; }
	return 0;
}

static void audio_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a < 0xc000) return;
	if (a < 0xc800) { s_audio_ram[a & 0x7ff] = v; return; }
	if (a >= 0xc800 && a <= 0xc803) { snd_sync(); ay8910_write(&s_ay[(a >> 1) & 1], a & 1, v); return; }
	if (a >= 0xc900 && a <= 0xc90d) { snd_sync(); msm5232_write(&s_msm, a - 0xc900, v); return; }
	switch (a) {
	case 0xd000: s_latch1 = v; s_latch1_pending = 1; return;
	case 0xd200: s_nmi_in1 = 1; update_nmi(); return;
	case 0xd400: s_nmi_in1 = 0; update_nmi(); return;
	case 0xd600:   // DAC_8BIT_R2R, route 0.25
		snd_sync();
		s_dac_out = (int32_t)(32767.0 * 0.25 * (2.0 * v / 255.0 - 1.0));
		return;
	}
}

static uint8_t no_in(z80 *z, uint8_t p) { (void)z; (void)p; return 0xff; }
static void no_out(z80 *z, uint8_t p, uint8_t v) { (void)z; (void)p; (void)v; }

static void zcpu_init(zcpu *c, uint8_t (*rd)(void *, uint16_t), void (*wr)(void *, uint16_t, uint8_t))
{
	unsigned long cyc = c->z.cyc;
	uint8_t (*pin)(z80 *, uint8_t) = c->z.port_in;
	z80_init(&c->z);
	c->z.read_byte = rd;
	c->z.write_byte = wr;
	c->z.port_in = pin ? pin : no_in;
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

// ---- video ----
static void decode_gfx(void)
{
	// ROMREGION_INVERT (not applied by the loader)
	for (uint32_t i = 0; i < s_gfx_size; i++)
		s_gfx[i] ^= 0xff;
	const uint32_t half = s_gfx_size / 2 * 8;   // in bits
	const uint32_t planes[4] = { half, half + 4, 0, 4 };
	static const int xo[16] = { 3, 2, 1, 0, 11, 10, 9, 8, 131, 130, 129, 128, 139, 138, 137, 136 };
	for (uint32_t t = 0; t < s_ntiles; t++)
		for (int y = 0; y < 8; y++)
			for (int x = 0; x < 8; x++) {
				int v = 0;
				for (int p = 0; p < 4; p++) {
					uint32_t b = t * 128 + planes[p] + (uint32_t)xo[x] + (uint32_t)y * 16;
					v = v << 1 | ((s_gfx[b >> 3] >> (7 - (b & 7))) & 1);
				}
				s_tiles[t * 64 + y * 8 + x] = (uint8_t)v;
			}
	for (uint32_t t = 0; t < s_nsprites; t++)
		for (int y = 0; y < 16; y++)
			for (int x = 0; x < 16; x++) {
				int v = 0;
				uint32_t yo = (uint32_t)(y < 8 ? y * 16 : 256 + (y - 8) * 16);
				for (int p = 0; p < 4; p++) {
					uint32_t b = t * 512 + planes[p] + (uint32_t)xo[x] + yo;
					v = v << 1 | ((s_gfx[b >> 3] >> (7 - (b & 7))) & 1);
				}
				s_sprites[t * 256 + y * 16 + x] = (uint8_t)v;
			}
}

// draw order of screen_update per spot(): tilemap passes (layer << 2 | category) and sprite priorities (0x10 | pri)
#define T(layer, cat) ((layer) << 2 | (cat))
#define S(pri) (0x10 | (pri))
static const uint8_t s_order[4][16] = {
	{ T(1, 3), S(6), T(0, 3), T(1, 2), T(0, 2), T(1, 1), S(3), T(0, 1), S(0), S(2), T(1, 0), S(1), T(0, 0), 0xff },
	{ T(1, 3), T(0, 3), S(3), S(2), T(1, 2), T(1, 1), S(1), T(0, 1), T(0, 2), S(0), T(1, 0), T(0, 0), 0xff },
	{ T(1, 3), T(0, 3), T(1, 1), S(1), T(0, 1), T(1, 2), T(0, 2), S(0), T(1, 0), T(0, 0), 0xff },
	{ T(1, 1), S(1), T(0, 1), S(0), T(1, 0), T(0, 0), 0xff },
};

static void draw_sprites_pri(int pri, int key)
{
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	const uint8_t *spr = s_snap_spr;
	for (int i = 0; i < 0x20; i++) {
		int pr = spr[0x9f - i];
		if (((pr & 0xe0) >> 5) != pri)
			continue;
		int offs = (pr & 0x1f) * 4;
		uint32_t code = (uint32_t)(spr[offs + 2] + ((spr[offs + 1] & 0x10) << 4));
		int pal = spr[offs + 1] & 0x0f;
		int sy = 240 - spr[offs + 0];
		int flipx = (spr[offs + 1] >> 6) & 1, flipy = (spr[offs + 1] >> 7) & 1;
		const uint8_t *g = s_sprites + (rbx_wrap(code, s_nsprites)) * 256;
		const uint16_t base = (uint16_t)(256 + pal * 16);
		int sx0 = spr[offs + 3];
		for (int rep = 0; rep < (sx0 > 240 ? 2 : 1); rep++) {
			int ox = rep ? sx0 - 256 : sx0;
			for (int ty = 0; ty < 16; ty++) {
				int y = sy + ty;
				if (y < VIS_Y0 || y > VIS_Y1) continue;
				const uint8_t *r = g + (flipy ? 15 - ty : ty) * 16;
				uint16_t *d = bm + (y - VIS_Y0) * pitch;
				int8_t *k = s_key + (y - VIS_Y0) * SCREEN_W;
				for (int tx = 0; tx < 16; tx++) {
					int x = ox + tx;
					if (x < 0 || x >= SCREEN_W) continue;
					uint8_t p = r[flipx ? 15 - tx : tx];
					if (p == 15 || key <= k[x]) continue;
					d[x] = (uint16_t)(base + p);
				}
			}
		}
	}
}

static void ny_render(void)
{
	for (int i = 0; i < 512; i++) {   // xBGR_444: base = low byte, ext = high byte
		int c = s_snap_pal[i] | s_snap_pal_ext[i] << 8;
		rbx_video.palette[i] = (uint32_t)((c & 15) * 17 << 16 | ((c >> 4) & 15) * 17 << 8 | ((c >> 8) & 15) * 17);
	}
	const uint8_t *ord = s_order[s_snap_spot & 3];
	int8_t tkey[8], skey[8];
	memset(tkey, -1, sizeof(tkey));
	memset(skey, -1, sizeof(skey));
	for (int n = 0; ord[n] != 0xff; n++) {
		if (ord[n] & 0x10) skey[ord[n] & 7] = (int8_t)n;
		else tkey[ord[n]] = (int8_t)n;
	}
	// transmask per group: layer 0 (front), layer 1 (back); a set bit = pen transparent in that layer
	static const uint16_t tm0[4] = { 0xf800, 0xfe00, 0xfffc, 0xfff0 };
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	// every tile pixel belongs to exactly one (layer, category) pass and is opaque in it: draw it once, keyed by
	// its position in the draw order; sprites then go over the pixels drawn before them
	for (int y = VIS_Y0; y <= VIS_Y1; y++) {
		uint16_t *d = bm + (y - VIS_Y0) * pitch;
		int8_t *k = s_key + (y - VIS_Y0) * SCREEN_W;
		for (int col = 0; col < 32; col++) {
			int ly = (y + s_snap_scrl[col]) & 0xff;
			int ti = (ly >> 3) * 32 + col;
			uint8_t code = s_snap_vram[ti * 2], attr = s_snap_vram[ti * 2 + 1];
			uint32_t num = (uint32_t)(code + ((attr & 0xc0) << 2) + 0x400 * s_snap_char_bank);
			int pal = attr & 0x0f, cat = (attr & 0x30) >> 4;
			uint16_t m0 = tm0[s_snap_group[ti]];
			const uint8_t *row = s_tiles + (rbx_wrap(num, s_ntiles)) * 64 + (ly & 7) * 8;
			int8_t k0 = tkey[T(0, cat)], k1 = tkey[T(1, cat)];
			for (int px = 0; px < 8; px++) {
				int x = col * 8 + px;
				uint8_t p = row[px];
				int8_t kk = (m0 & (1 << p)) ? k1 : k0;
				k[x] = kk;
				if (kk >= 0)
					d[x] = (uint16_t)(pal * 16 + p);
			}
		}
	}
	for (int n = 0; ord[n] != 0xff; n++)
		if (ord[n] & 0x10)
			draw_sprites_pri(ord[n] & 7, n);
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	// realize the dirty tiles (MAME tilemap: get_tile_info runs at draw time with the current spot())
	int sp = spot();
	for (int t = 0; t < 0x400; t++) {
		if (!s_tile_dirty[t])
			continue;
		s_tile_dirty[t] = 0;
		int pal = s_vram[t * 2 + 1] & 0x0f, cat = (s_vram[t * 2 + 1] & 0x30) >> 4, grp = 0;
		if (!sp && pal == 6) grp = 1;
		if ((sp == 3 && pal == 8) || (sp == 1 && pal == 0xc)) grp = 2;
		if (sp == 1 && cat == 2) grp = 3;
		s_tile_group[t] = (uint8_t)grp;
	}
	memcpy(s_snap_vram, s_vram, sizeof(s_snap_vram));
	memcpy(s_snap_group, s_tile_group, sizeof(s_snap_group));
	memcpy(s_snap_spr, s_spr, sizeof(s_spr));
	memcpy(s_snap_scrl, s_scrl, sizeof(s_scrl));
	memcpy(s_snap_pal, s_pal, sizeof(s_pal));
	memcpy(s_snap_pal_ext, s_pal_ext, sizeof(s_pal_ext));
	s_snap_char_bank = s_char_bank;
	s_snap_spot = sp;
	rbx_video_frame_done();
}

// ---- init / frame ----
static int kind_of(const char *m)
{
	if (!strcmp(m, "nycaptor")) return NY_NYCAPTOR;
	if (!strcmp(m, "bronx")) return NY_BRONX;
	// cyclshtg: MACHINE_NOT_WORKING in MAME (MCU not dumped, two main ROMs are bad dumps); emulated as MAME does
	// (MCU stubs) when forced, but not listed
	if (!strcmp(m, "cyclshtg")) return NY_CYCLSHTG;
	return -1;
}

static void ny_reset(void)
{
	// (first: the sound syncs read them, from a CPU write or a port callback of a chip reset below, before a frame
	// sets them; the last game's values made a render of billions of clocks, the next game of the board hung)
	s_slice_start = s_slice_len = s_cpu_start = s_cpu_len = 0;
	memset(s_vram, 0, sizeof(s_vram));
	memset(s_spr, 0, sizeof(s_spr));
	memset(s_scrl, 0, sizeof(s_scrl));
	memset(s_shared, 0, sizeof(s_shared));
	memset(s_audio_ram, 0, sizeof(s_audio_ram));
	memset(s_pal, 0, sizeof(s_pal));
	memset(s_pal_ext, 0, sizeof(s_pal_ext));
	memset(s_tile_dirty, 1, sizeof(s_tile_dirty));
	memset(s_tile_group, 0, sizeof(s_tile_group));
	memset(&s_main, 0, sizeof(s_main));
	memset(&s_sub, 0, sizeof(s_sub));
	memset(&s_audio, 0, sizeof(s_audio));
	zcpu_init(&s_main, main_read, main_write);
	zcpu_init(&s_sub, sub_read, sub_write);
	if (s_kind != NY_NYCAPTOR)
		s_sub.z.port_in = sub_in;
	zcpu_init(&s_audio, audio_read, audio_write);
	s_audio_reset = s_sub_halt = s_sub_reset = 0;
	// machine_reset
	s_generic = 0;
	s_bank = 0;
	s_char_bank = 0;
	s_pal_bank = 0;
	s_gfxctrl = 0;
	s_latch0 = s_latch1 = 0;
	s_latch0_pending = s_latch1_pending = s_nmi_in1 = s_nmi_line = 0;
	s_rand = 0x12345678;
	if (s_has_mcu)
		taito68705_reset(&s_mcu);
	for (int i = 0; i < 2; i++)
		ay8910_reset(&s_ay[i]);
	msm5232_reset(&s_msm);
	for (int i = 0; i < 8; i++)
		s_msm.gain[i] = 256;   // routes 0-7 at 1.0
	s_dac_out = (int32_t)(32767.0 * 0.25 * -1.0);
	s_audio_irq_count = 0;
	s_gun[0] = s_gun[1] = 0x80;
	s_frac_main = s_frac_sub = s_frac_audio = s_frac_mcu = s_frac_snd = 0;
	s_snd_done = 0;
	s_mix_n = 0;
}

static void ny_exit(void)
{
	free(s_tiles);
	free(s_sprites);
	s_tiles = s_sprites = NULL;
	free(s_mix);
	s_mix = NULL;
	free(s_key_hp);
	s_key_hp = NULL;
}

static int ny_init(const rbx_game *g)
{
	if (!s_key_hp && !(s_key_hp = calloc(1, sizeof(*s_key_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	s_kind = kind_of(g->machine);
	if (s_kind < 0) {
		rbx_fail("%s: machine %s is not emulated", g->set, g->machine);
		return -1;
	}
	s_gametype = s_kind == NY_NYCAPTOR ? 0 : !strcmp(g->set, "colt") ? 2 : 1;   // init_nycaptor / init_colt / others
	s_has_mcu = s_kind == NY_NYCAPTOR;
	s_vram_mask = s_kind == NY_NYCAPTOR ? 0x7ff : 0xfff;
	s_sub_rom_end = s_kind == NY_CYCLSHTG ? 0xc000 : 0x8000;
	s_main_rom = rbx_region("maincpu", &s_main_size);
	s_sub_rom = rbx_region("sub", &s_sub_size);
	s_audio_rom = rbx_region("audiocpu", &s_audio_size);
	s_gfx = rbx_region("gfx", &s_gfx_size);
	s_user1 = rbx_region("user1", &s_user1_size);
	if (!s_main_rom || !s_sub_rom || !s_audio_rom || !s_gfx || s_gfx_size < 0x8000) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	if (s_kind == NY_BRONX)   // init_bronx / init_colt
		for (uint32_t i = 0; i < s_main_size; i++) {
			uint8_t v = s_main_rom[i], r = 0;
			for (int b = 0; b < 8; b++)
				r |= (uint8_t)(((v >> b) & 1) << (7 - b));
			s_main_rom[i] = r;
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
	s_p_in0 = rbx_port_find("IN0");
	s_p_in1 = rbx_port_find("IN1");
	s_p_gun[0] = rbx_port_find("LIGHTX");
	s_p_gun[1] = rbx_port_find("LIGHTY");
	for (int a = 0; a < 2; a++) {   // the analog type of LIGHTX / LIGHTY (swapped on the ROT90 games)
		s_gun_type[a] = a ? RBX_IPT_LIGHTGUN_Y : RBX_IPT_LIGHTGUN_X;
		for (const rbx_ipt_port *p = g->ports; p && p->tag; p++)
			if (!strcmp(p->tag, a ? "LIGHTY" : "LIGHTX") && p->count)
				s_gun_type[a] = g->fields[p->first].type;
	}
	s_ntiles = s_gfx_size / 2 / 16;
	s_nsprites = s_gfx_size / 2 / 64;
	s_tiles = (uint8_t *)malloc((size_t)s_ntiles * 64);
	s_sprites = (uint8_t *)malloc((size_t)s_nsprites * 256);
	if (!s_tiles || !s_sprites) {
		ny_exit();
		return -1;
	}
	decode_gfx();
	for (int i = 0; i < 2; i++)   // AY-3-8910 at 2 MHz, route 0.15
		ay8910_init(&s_ay[i], AY8910_TYPE_AY8910, SND_CLOCK, AY8910_LEGACY_OUTPUT, 38, 4, 1);
	static const double caps[8] = { 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6 };
	msm5232_init(&s_msm, SND_CLOCK, caps, 2);
	uint32_t rate = ay8910_rate(&s_ay[0]);
	s_mix_cap = (int)(rate / 50) + 64;
	s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	if (!s_mix || rbx_audio_setup(rate) || rbx_video_setup(SCREEN_W, VIS_H, 512)) {
		ny_exit();
		return -1;
	}
	ny_reset();
	return 0;
}

static void update_gun(void)
{
	for (int a = 0; a < 2; a++) {
		int v = s_gun[a] + rbx_analog(1, s_gun_type[a]);
		s_gun[a] = v < 0 ? 0 : v > 255 ? 255 : v;
	}
}

static void ny_frame(void)
{
	update_gun();
	for (int i = 0; i < LINES; i++) {
		int line = (VIS_Y1 + 1 + i) % LINES;
		if (line == VIS_Y1 + 1) {
			vblank_capture();
			s_main.irq = 1;   // irq0_line_hold
			if (!s_sub_halt && !s_sub_reset)
				s_sub.irq = 1;
		}
		int mc = rbx_line_cycles(&s_frac_main, CPU_CLOCK, RATE_MHZ, LINES);
		int bc = rbx_line_cycles(&s_frac_sub, CPU_CLOCK, RATE_MHZ, LINES);
		int ac = rbx_line_cycles(&s_frac_audio, CPU_CLOCK, RATE_MHZ, LINES);
		int uc = rbx_line_cycles(&s_frac_mcu, MCU_CLOCK, RATE_MHZ, LINES);
		int sc = rbx_line_cycles(&s_frac_snd, SND_CLOCK, RATE_MHZ, LINES);
		zcpu_run(&s_main, mc);
		if (s_sub_halt || s_sub_reset)
			s_sub.z.cyc += (unsigned long)bc;
		else
			zcpu_run(&s_sub, bc);
		s_slice_start = s_snd_done;
		s_slice_len = (uint32_t)sc;
		s_cpu_start = s_audio.z.cyc;
		s_cpu_len = ac;
		if (s_audio_reset)
			s_audio.z.cyc += (unsigned long)ac;
		else {
			s_audio_irq_count += (uint32_t)ac;
			if (s_audio_irq_count >= AUDIO_IRQ_PERIOD) {
				s_audio_irq_count -= AUDIO_IRQ_PERIOD;
				s_audio.irq = 1;
			}
			zcpu_run(&s_audio, ac);
		}
		if (s_has_mcu)
			taito68705_execute(&s_mcu, uc);
		snd_run_to(s_slice_start + s_slice_len);
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

static int ny_state(void *ctx, int load)
{
	int rc = 0;
	rc |= zcpu_state(ctx, "main", &s_main, load);
	rc |= zcpu_state(ctx, "sub", &s_sub, load);
	rc |= zcpu_state(ctx, "audio", &s_audio, load);
	rc |= rbx_state_io(ctx, "vram", s_vram, sizeof(s_vram));
	rc |= rbx_state_io(ctx, "spr", s_spr, sizeof(s_spr));
	rc |= rbx_state_io(ctx, "scrl", s_scrl, sizeof(s_scrl));
	rc |= rbx_state_io(ctx, "shared", s_shared, sizeof(s_shared));
	rc |= rbx_state_io(ctx, "aram", s_audio_ram, sizeof(s_audio_ram));
	rc |= rbx_state_io(ctx, "pal", s_pal, sizeof(s_pal));
	rc |= rbx_state_io(ctx, "palx", s_pal_ext, sizeof(s_pal_ext));
	rc |= rbx_state_io(ctx, "tgrp", s_tile_group, sizeof(s_tile_group));
	rc |= rbx_state_io(ctx, "tdirty", s_tile_dirty, sizeof(s_tile_dirty));
	if (s_has_mcu) {
		rc |= rbx_state_io(ctx, "tmcu", &s_mcu.st, sizeof(s_mcu.st));
		rc |= rbx_state_io(ctx, "tmcu6805", &s_mcu.mcu.st, sizeof(s_mcu.mcu.st));
	}
	rc |= rbx_state_io(ctx, "ay0", &s_ay[0], ay8910_state_size());
	rc |= rbx_state_io(ctx, "ay1", &s_ay[1], ay8910_state_size());
	rc |= rbx_state_io(ctx, "msm", &s_msm, msm5232_state_size());
	int32_t misc[20] = { s_audio_reset, s_sub_halt, s_sub_reset, s_gfxctrl, s_char_bank, s_pal_bank, s_generic, s_bank,
		s_latch0, s_latch1, s_latch0_pending, s_latch1_pending, s_nmi_in1, s_nmi_line, (int32_t)s_rand, s_dac_out,
		(int32_t)s_audio_irq_count, s_gun[0], s_gun[1], 0 };
	rc |= rbx_state_io(ctx, "misc", misc, sizeof(misc));
	if (rc)
		return -1;
	if (load) {
		s_audio_reset = misc[0]; s_sub_halt = misc[1]; s_sub_reset = misc[2]; s_gfxctrl = (uint8_t)misc[3];
		s_char_bank = (uint8_t)misc[4]; s_pal_bank = (uint8_t)misc[5]; s_generic = (uint8_t)misc[6];
		s_bank = (uint8_t)misc[7]; s_latch0 = (uint8_t)misc[8]; s_latch1 = (uint8_t)misc[9];
		s_latch0_pending = misc[10]; s_latch1_pending = misc[11]; s_nmi_in1 = misc[12]; s_nmi_line = misc[13];
		s_rand = (uint32_t)misc[14]; s_dac_out = misc[15]; s_audio_irq_count = (uint32_t)misc[16];
		s_gun[0] = misc[17]; s_gun[1] = misc[18];
	}
	return 0;
}

static int ny_save(void *ctx) { return ny_state(ctx, 0); }
static int ny_load(void *ctx) { return ny_state(ctx, 1); }
// cyclshtg is MACHINE_NOT_WORKING in MAME (no MCU dump, bad main ROM dumps): not listed
static int ny_supported(const char *machine) { int k = kind_of(machine); return k == NY_NYCAPTOR || k == NY_BRONX; }

const rbx_driver rbx_drv_nycaptor = { "N.Y. Captor", ny_init, ny_exit, ny_reset, ny_frame, ny_save, ny_load, ny_supported, ny_render };
