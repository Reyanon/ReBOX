/*  SNK / Alpha Denshi Alpha 68K II, III and V: Time Soldiers, Battle Field, Sky Soldiers, Gold Medalist, Sky Adventure,
    Gang Wars and Super Champion Baseball. Ported from MAME src/mame/alpha/alpha68k.cpp, alpha68k_v.cpp,
    src/mame/shared/alpha68k_spr.cpp and alpha68k_palette.cpp (BSD-3-Clause, copyright-holders: Pierpaolo Prazzoli,
    Bryan McPhail, Stephane Humbert, Angelo Salese).
    68000 at 8 MHz (II) or 10 MHz (III, V); the Alpha MCU simulated as MAME does (coins, DIP switches, the ID);
    16x16 sprites in columns of 32 (the SNK 68000 chip), an 8x8 text layer, the Alpha palette (5 bits and the dark bit).
    Sound: Z80 at 6 MHz (32 banks of 16 KB, a periodic NMI the YM2203's port A masks), YM2203, YM2413 and an 8-bit DAC.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m68k_bus.h"
#include "../../cpu/m68k/m68k.h"
#include "../../cpu/z80/z80.h"
#include "../../sound/rbx_ym.h"
#include "../../video/decoband.h"
#include "../../video/gfxlayout.h"

#include <stdlib.h>
#include <string.h>

#define Z80_CLOCK 6000000
#define YM1_CLOCK 3000000          // ALPHA68K_PIXEL_CLOCK / 2
#define YM2_CLOCK 3579545
#define FRAME_MHZ 59186            // set_raw(24 MHz / 4, 384, 0, 256, 264, 16, 240)
#define LINES 264
#define VIS_Y0 16
#define VIS_W 256
#define VIS_H 224
#define PAL_MAX 4096

enum { B_II, B_III, B_V };
enum { T_DEFAULT, T_NOFLIPX, T_NOFLIPY };

static int s_board, s_tilecb, s_npal, s_cmask;
static int s_vbl_level, s_irq2_hz, s_btlfieldb, s_invert;
static uint32_t s_cpu_clock, s_mcu_id, s_coin_id;
static uint8_t *s_rom, *s_zrom;
static uint32_t s_rom_size, s_zrom_size;
static uint16_t s_shared[0x2000];            // 0x040000 .. 0x043fff (II: .. 0x040fff)
static uint16_t s_vram[0x800];               // 0x100000 .. 0x100fff (8-bit)
static uint16_t (*s_spriteram_hp)[0x4000];   // 0x200000 .. 0x207fff
#define s_spriteram (*s_spriteram_hp)
static uint16_t s_pal[PAL_MAX];              // 0x400000 ..
static uint8_t s_palsh[PAL_MAX];             // the shift each entry was set with (the reference color)
static rbx_gfx s_gfxtx, s_gfxspr;
static int s_port[7];
static uint64_t s_cpu_frac, s_z80_frac, s_ym1_frac, s_ym2_frac, s_irq2_acc;

// machine state
static uint8_t s_bank_base, s_outlatch;
static int s_deposits1, s_deposits2, s_credits, s_coinvalue, s_mcu_data, s_mcu_latch;

static struct {
	uint16_t vram[0x800], spr[0x4000], pal[PAL_MAX];
	uint8_t palsh[PAL_MAX];
	uint8_t bank;
} *s_snap;

// ---- sound ----
static z80 s_z80;
static uint8_t s_zram[0x800];
static uint8_t s_latch, s_zbank, s_nmi_mask, s_pa_latch, s_dac;
static unsigned long s_zt, s_nmi_t;
static uint32_t s_nmi_f, s_nmi_period;   // the NMI's period in Z80 cycles, 16.16
static int32_t *s_mix, *s_ym2buf;
static int s_mix_n, s_mix_cap, s_ym2_n, s_ym2_cap;
static const uint8_t *s_zmap[256];

static void zbank_map(void)
{
	const uint32_t base = (uint32_t)s_zbank * 0x4000;
	for (int p = 0; p < 0x40; p++)
		s_zmap[0xc0 + p] = base + (uint32_t)p * 256 + 256 <= s_zrom_size ? s_zrom + base + p * 256 : NULL;
}

static uint8_t z80_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x8000) return a < s_zrom_size ? s_zrom[a] : 0;
	if (a < 0x8800) return s_zram[a - 0x8000];
	if (a >= 0xc000) {
		const uint32_t o = (uint32_t)s_zbank * 0x4000 + (a - 0xc000u);
		return o < s_zrom_size ? s_zrom[o] : 0;
	}
	return 0;
}
static void z80_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0x8000 && a < 0x8800) s_zram[a - 0x8000] = v;
}
static uint8_t z80_in(z80 *z, uint8_t port)
{
	(void)z;
	(void)port;
	return s_latch;   // (the latch at every port)
}
static void z80_out(z80 *z, uint8_t port, uint8_t v)
{
	(void)z;
	switch (port & 0x0f) {
	case 0x00: case 0x01: s_latch = 0; break;   // (clear_w)
	case 0x08: case 0x09: s_dac = v; break;
	case 0x0a: case 0x0b: rbx_ym_write(1, port & 1, v); break;
	case 0x0c: case 0x0d: rbx_ym_write(0, port & 1, v); break;
	case 0x0e: case 0x0f:
		s_zbank = v & 0x1f;
		zbank_map();
		break;
	}
}

static void porta_w(uint8_t data)
{
	if (data == 0xff)
		return;
	if (data == 0 && s_pa_latch)   // 1 -> 0: the NMI on
		s_nmi_mask = 1;
	if (data && s_pa_latch == 0)   // 0 -> 1: off
		s_nmi_mask = 0;
	s_pa_latch = data & 1;
}

static void z80_start(void)
{
	z80_init(&s_z80);
	s_z80.read_byte = z80_read;
	s_z80.write_byte = z80_write;
	s_z80.port_in = z80_in;
	s_z80.port_out = z80_out;
	s_z80.read_map = s_zmap;
}

static void sound_line(void)
{
	static int no_line;
	const int cycles = rbx_line_cycles(&s_z80_frac, Z80_CLOCK, FRAME_MHZ, LINES);
	const unsigned long end = s_zt + (unsigned long)cycles;
	while ((long)(s_nmi_t - end) <= 0) {   // the periodic NMI
		z80_run(&s_z80, s_nmi_t, &no_line, 0);
		if (s_nmi_mask)
			z80_gen_nmi(&s_z80);
		s_nmi_f += s_nmi_period;
		s_nmi_t += s_nmi_f >> 16;
		s_nmi_f &= 0xffff;
	}
	z80_run(&s_z80, end, &no_line, 0);
	s_zt = end;
	const int n = rbx_ym_advance(0, (uint32_t)rbx_line_cycles(&s_ym1_frac, YM1_CLOCK, FRAME_MHZ, LINES), s_mix + s_mix_n,
	                             s_mix_cap - s_mix_n);
	s_ym2_n += rbx_ym_advance(1, (uint32_t)rbx_line_cycles(&s_ym2_frac, YM2_CLOCK, FRAME_MHZ, LINES), s_ym2buf + s_ym2_n,
	                          s_ym2_cap - s_ym2_n);
	const int32_t dac = ((int32_t)s_dac - 0x80) * 96;   // (0.75)
	for (int i = 0; i < n; i++)
		s_mix[s_mix_n + i] += dac;
	s_mix_n += n;
}

static void sound_frame_end(void)
{
	rbx_ym_async_take(0, s_mix, s_mix_n);
	rbx_ym_async_take(1, s_ym2buf, s_ym2_n);
	if (s_ym2_n > 0)
		for (int i = 0; i < s_mix_n; i++)
			s_mix[i] += s_ym2buf[(int)((int64_t)i * s_ym2_n / s_mix_n)];
	for (int i = 0; i < s_mix_n; i++)
		rbx_audio_put(s_mix[i]);
	memset(s_mix, 0, sizeof(int32_t) * (size_t)s_mix_n);
	s_mix_n = s_ym2_n = 0;
}

// ---- the MCU (alpha_II_trigger_r / alpha_V_trigger_r) ----
static uint8_t in_port(int i) { return (uint8_t)rbx_port_read(s_port[i]); }

static void coin_insert(int which, int ram29, int ram22, const uint8_t (*c1)[2], const uint8_t (*c2)[2], int v_board)
{
	const uint16_t source = s_shared[ram29];
	const int id = which ? (int)(s_coin_id >> 8) : (int)(s_coin_id & 0xff);
	s_shared[ram29] = (uint16_t)((source & 0xff00) | id);
	s_shared[ram22] = (uint16_t)(source & 0xff00);
	s_mcu_latch = 1;
	if (id != 0x22)
		return;
	if (v_board)
		s_coinvalue = (~in_port(4) >> 1) & 7;
	else if (s_btlfieldb)
		s_coinvalue = in_port(4) & 7;
	else
		s_coinvalue = ~in_port(4) & 7;
	int *dep = which ? &s_deposits2 : &s_deposits1;
	const uint8_t (*c)[2] = which ? c2 : c1;
	if (++*dep == c[s_coinvalue][0]) {
		s_credits = c[s_coinvalue][1];
		*dep = 0;
	} else
		s_credits = 0;
}

static void coin_query(int ram29, int ram22, const uint8_t (*c1)[2], const uint8_t (*c2)[2], int v_board)
{
	const uint8_t in2 = in_port(2);
	if ((in2 & 3) == 3)
		s_mcu_latch = 0;
	if ((in2 & 1) == 0 && !s_mcu_latch)
		coin_insert(0, ram29, ram22, c1, c2, v_board);
	else if ((in2 & 2) == 0 && !s_mcu_latch)
		coin_insert(1, ram29, ram22, c1, c2, v_board);
	else {
		s_mcu_data = !v_board && s_mcu_id == 0x8803 ? 0x21 : 0x00;
		s_shared[ram29] = (uint16_t)((s_shared[ram29] & 0xff00) | s_mcu_data);
	}
}

static void trigger_II(uint32_t offset)
{
	static const uint8_t c1[8][2] = { {1,1}, {1,2}, {1,3}, {1,4}, {1,5}, {1,6}, {2,3}, {3,2} };
	static const uint8_t c2[8][2] = { {1,1}, {2,1}, {3,1}, {4,1}, {5,1}, {6,1}, {7,1}, {8,1} };
	const uint16_t source = s_shared[offset];
	switch (offset) {
	case 0: s_shared[0] = (uint16_t)((source & 0xff00) | in_port(4)); break;   // DIP switch 2
	case 0x22: s_shared[0x22] = (uint16_t)((source & 0xff00) | (s_credits & 0xff)); break;
	case 0x29: coin_query(0x29, 0x22, c1, c2, 0); break;
	case 0xfe: s_shared[0xfe] = (uint16_t)((source & 0xff00) | 0x87); break;
	case 0xff: s_shared[0xff] = (uint16_t)((source & 0xff00) | 0x13); break;
	}
}

static void trigger_V(uint32_t offset)
{
	static const uint8_t c1[8][2] = { {1,1}, {1,5}, {1,3}, {2,3}, {1,2}, {1,6}, {1,4}, {3,2} };
	static const uint8_t c2[8][2] = { {1,1}, {5,1}, {3,1}, {7,1}, {2,1}, {6,1}, {4,1}, {8,1} };
	const uint16_t source = s_shared[offset];
	switch (offset) {
	case 0: s_shared[0] = (uint16_t)((source & 0xff00) | in_port(4)); break;   // DIP switch 1
	case 0x22: s_shared[0x22] = (uint16_t)((source & 0xff00) | (s_credits & 0xff)); break;
	case 0x29: coin_query(0x29, 0x22, c1, c2, 1); break;
	case 0xfe: s_shared[0xfe] = (uint16_t)((source & 0xff00) | (s_mcu_id >> 8)); break;
	case 0xff: s_shared[0xff] = (uint16_t)((source & 0xff00) | (s_mcu_id & 0xff)); break;
	case 0x1f00: s_shared[0x1f00] = (uint16_t)((source & 0xff00) | in_port(4)); break;
	case 0x1f29:
		coin_query(0x1f29, 0x1f22, c1, c2, 1);
		// (Gang Wars expects the first DIP switch in RAM at 0x02c6)
		s_shared[0x0163] = (uint16_t)((s_shared[0x0163] & 0x00ff) | in_port(4) << 8);
		break;
	case 0x1ffe: s_shared[0x1ffe] = (uint16_t)((source & 0xff00) | (s_mcu_id >> 8)); break;
	case 0x1fff: s_shared[0x1fff] = (uint16_t)((source & 0xff00) | (s_mcu_id & 0xff)); break;
	}
}

// ---- bus ----
// the 12-way rotary joysticks (timesold, btlfield: MAME's IN5 / IN6 positional ports, 0 .. 11) on the right stick
static int s_rotgame, s_rotary[2];
static rbx_rotary s_rot[2];
// (the game reads the switch as it is, 12 directions: the player's facing is the value; AK_ROT_UP the value aiming up
// on the screen, AK_ROT_CW 1 when a growing value turns it clockwise. Its copy in RAM 0x040016 / 0x040017 is 11 - value)
#define AK_ROT_UP 11
#define AK_ROT_CW -1
static int ak_facing(int p) { return ((s_rotary[p] - AK_ROT_UP) * AK_ROT_CW % 12 + 12) % 12; }
static uint32_t rot(int i)
{
	const uint32_t v = s_rotgame ? (uint32_t)s_rotary[i - 5] : rbx_port_read(s_port[i]);
	return ~(1u << (v & 31));
}

static uint16_t control_1(void)
{
	const uint16_t v = (uint16_t)(in_port(0) + (in_port(1) << 8));
	return s_invert ? (uint16_t)~v : v;
}

static uint16_t main_read16(uint32_t a)
{
	a &= 0xffffff;
	if (a >= 0x100000 && a < 0x101000)
		return s_vram[(a - 0x100000) >> 1];
	if (a >= 0x200000 && a < 0x208000) {   // spriteram_r: the MSB of every 32-bit word reads 0xff
		const uint32_t o = (a - 0x200000) >> 1;
		return (uint16_t)(s_spriteram[o] | (o & 1 ? 0 : 0xff00));
	}
	if (a >= 0x400000 && a < 0x400000 + (uint32_t)s_npal * 2)
		return s_pal[(a - 0x400000) >> 1];
	if (a >= 0x300000 && a < 0x304000) {   // the MCU's triggers (values read do not matter)
		const uint32_t o = (a - 0x300000) >> 1;
		if (s_board == B_V)
			trigger_V(o);
		else if (o < 0x100)
			trigger_II(o);
		return 0;
	}
	if (a == 0x080000)
		return control_1();
	if (s_board == B_II) {
		uint32_t v;
		switch (a) {
		case 0x0c0000: v = in_port(3) + ((rot(5) << 8) & 0xff00); break;   // CN1 and DIP 1
		case 0x0c8000: v = (rot(6) << 8) & 0xff00; break;
		case 0x0d0000: v = ((rot(6) << 4) & 0xf000) + (rot(5) & 0x0f00); break;
		default: return 0;
		}
		return (uint16_t)(s_invert ? ~v : v);
	}
	if (a == 0x0c0000)
		return (uint16_t)rbx_port_read(s_port[3]);
	return 0;
}
static uint8_t main_read8(uint32_t a)
{
	const uint16_t v = main_read16(a & ~1u);
	return (uint8_t)(a & 1 ? v : v >> 8);
}

static void video_bank_w(uint8_t data) { s_bank_base = data & 0xf; }

static void pal_w(uint32_t o, uint16_t v, uint16_t mask)
{
	s_pal[o] = (uint16_t)((s_pal[o] & ~mask) | (v & mask));
	s_palsh[o] = 0;
	if (o == 0) {   // the reference color: every entry again (2 dimmer if it is not black)
		const uint8_t sh = (s_pal[0] & 0x7fff) == 0 ? 0 : 2;
		memset(s_palsh, sh, (size_t)s_npal);
	}
}

static void main_write16m(uint32_t a, uint16_t v, uint16_t mask)
{
	a &= 0xffffff;
	if (a >= 0x100000 && a < 0x101000) {   // 8-bit: either byte lands in the same place
		s_vram[(a - 0x100000) >> 1] = mask == 0xff00 ? (uint16_t)(v >> 8) : (uint16_t)(v & 0xff);
	} else if (a >= 0x200000 && a < 0x208000) {   // spriteram_w
		const uint32_t o = (a - 0x200000) >> 1;
		if (!(o & 1))
			v |= 0xff00;
		s_spriteram[o] = (uint16_t)((s_spriteram[o] & ~mask) | (v & mask));
	} else if (a >= 0x400000 && a < 0x400000 + (uint32_t)s_npal * 2) {
		pal_w((a - 0x400000) >> 1, v, mask);
	} else if (a == 0x080000) {
		if (mask & 0x00ff)
			s_latch = (uint8_t)v;   // the sound latch
		if ((mask & 0xff00) && s_board != B_II)
			video_bank_w((uint8_t)(v >> 8));
	} else if ((a & ~0x7fu) == 0x0c0000 && (mask & 0x00ff)) {   // the LS259 (select 0x78)
		const int bit = (a >> 3) & 7, val = (a >> 6) & 1;
		s_outlatch = (uint8_t)((s_outlatch & ~(1u << bit)) | (unsigned)val << bit);
		if (s_board == B_II && (s_bank_base ^ ((s_outlatch >> 4) & 7)) & 0xf)
			video_bank_w((s_outlatch >> 4) & 7);
	}
}
static void main_write16(uint32_t a, uint16_t v) { main_write16m(a, v, 0xffff); }
static void main_write8(uint32_t a, uint8_t v)
{
	const int sh = a & 1 ? 0 : 8;
	main_write16m(a & ~1u, (uint16_t)(v << 8 | v), (uint16_t)(0xff << sh));
}

// ---- video ----
static uint16_t (*s_frame_hp)[VIS_H][VIS_W];
#define s_frame (*s_frame_hp)

static void draw_sprites(int group, int start, int end)   // alpha68k_sprite_device::draw_sprites (xpos_shift 15)
{
	const uint16_t *sp = s_snap->spr;
	const uint16_t *tiledata = &sp[0x800 * group + start];
	for (int offs = start; offs < end; offs += 0x40) {
		int mx = (sp[offs + 2 * group] & 0xff) << 1;
		int my = sp[offs + 2 * group + 1];
		mx = mx | (my >> 15);
		mx = ((mx + 16) & 0x1ff) - 16;
		my = -my;
		if (group == 1 && start == 0x7c0)
			my++;
		for (int i = 0; i < 0x20; i++, tiledata += 2, my += 16) {
			my &= 0x1ff;
			if (!(my <= VIS_Y0 + VIS_H - 1 && my + 15 >= VIS_Y0))
				continue;
			uint32_t tile = tiledata[1];
			int fx, fy;
			if (s_tilecb == T_NOFLIPX) {
				fx = 0;
				fy = (tile & 0x8000) != 0;
				tile &= 0x7fff;
			} else if (s_tilecb == T_NOFLIPY) {
				fx = (tile & 0x8000) != 0;
				fy = 0;
				tile &= 0x7fff;
			} else {
				fx = (tile & 0x4000) != 0;
				fy = (tile & 0x8000) != 0;
				tile &= 0x3fff;
			}
			tile %= s_gfxspr.count;
			if (!s_gfxspr.used[tile])
				continue;
			const uint16_t c0 = (uint16_t)(((tiledata[0] & 0xff) & s_cmask) * 16);
			const uint8_t *g = s_gfxspr.pix + tile * 256;
			const int sy = my - VIS_Y0;
			for (int j = 0; j < 16; j++) {
				const int yy = sy + j;
				if ((unsigned)yy >= VIS_H)
					continue;
				const uint8_t *src = g + (fy ? 15 - j : j) * 16;
				uint16_t *d = s_frame[yy];
				for (int k = 0; k < 16; k++) {
					const int xx = mx + k;
					if ((unsigned)xx >= VIS_W)
						continue;
					const uint8_t p = src[fx ? 15 - k : k];
					if (p)
						d[xx] = (uint16_t)(c0 + p);
				}
			}
		}
	}
}

static void fix_line(int by, uint16_t *d)   // 32x32 of 8x8 (scan columns); pen 0 transparent unless the tile is opaque
{
	const uint32_t ty = (uint32_t)by & 255;
	const uint32_t bank = (uint32_t)s_snap->bank << 8;
	for (int x = 0; x < VIS_W; x += 8) {
		const uint32_t i = ((uint32_t)x >> 3) * 32 + (ty >> 3);
		const uint32_t attr = s_snap->vram[2 * i + 1] & 0xff;
		const uint32_t code = rbx_wrap(((s_snap->vram[2 * i] & 0xff) | bank), s_gfxtx.count);
		const int opaque = (attr >> 4) & 1;
		if (!opaque && !s_gfxtx.used[code])
			continue;
		const uint16_t c0 = (uint16_t)((attr & 0xf) * 16);
		const uint8_t *src = s_gfxtx.pix + code * 64 + (ty & 7) * 8;
		for (int k = 0; k < 8; k++) {
			const uint8_t p = src[k];
			if (p || opaque)
				d[x + k] = (uint16_t)(c0 + p);
		}
	}
}

#define BANDS 8
static int s_phase;
static void ak_band(int band)
{
	for (int y = VIS_H * band / BANDS; y < VIS_H * (band + 1) / BANDS; y++) {
		uint16_t *d = s_frame[y];
		if (s_phase == 0)   // the backdrop
			for (int x = 0; x < VIS_W; x++)
				d[x] = (uint16_t)(s_npal - 1);
		else {              // (the sprites), the text
			fix_line(y + VIS_Y0, d);
			memcpy(rbx_video.bitmap + y * rbx_video.pitch, d, VIS_W * 2);
		}
	}
}
static decoband_t s_bands = { 0, 0, 0, 0, BANDS, ak_band };
static int ak_help(int helper) { return decoband_help(&s_bands, helper); }

static uint8_t s_lookup[32][2];   // the palette's resistor network (normal, dark)

static void ak_render(void)
{
	for (int i = 0; i < s_npal; i++) {   // set_color_entry
		const uint16_t p = s_snap->pal[i];
		const int dark = p >> 15, sh = s_snap->palsh[i];
		const int r = (((p >> 14) & 1) | ((p >> 7) & 0x1e)) >> sh;
		const int g = (((p >> 13) & 1) | ((p >> 3) & 0x1e)) >> sh;
		const int b = (((p >> 12) & 1) | ((p << 1) & 0x1e)) >> sh;
		rbx_video.palette[i] = (uint32_t)s_lookup[r][dark] << 16 | (uint32_t)s_lookup[g][dark] << 8 | s_lookup[b][dark];
	}
	s_phase = 0;
	decoband_run(&s_bands);
	draw_sprites(1, 0x7c0, 0x800);   // draw_sprites_alt
	draw_sprites(2, 0x000, 0x800);
	draw_sprites(3, 0x000, 0x800);
	draw_sprites(1, 0x000, 0x7c0);
	s_phase = 1;
	decoband_run(&s_bands);
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap->vram, s_vram, sizeof(s_vram));
	memcpy(s_snap->spr, s_spriteram, sizeof(s_spriteram));
	memcpy(s_snap->pal, s_pal, sizeof(uint16_t) * (size_t)s_npal);
	memcpy(s_snap->palsh, s_palsh, (size_t)s_npal);
	s_snap->bank = s_bank_base;
	rbx_video_frame_done();
}

static void rgb_lookups(void)   // alpha68k_palette create_rgb_lookups (compute_resistor_weights, normal and dark)
{
	static const double res[5] = { 3900, 2200, 1000, 470, 220 };
	static const double pd[2] = { 0, 8200 };
	double w[2][5];
	double scale = 0;
	for (int net = 0; net < 2; net++) {
		double sum = 0;
		for (int i = 0; i < 5; i++) {
			double r0 = pd[net] == 0 ? 1.0 / 1e12 : 1.0 / pd[net], r1 = 1.0 / 1e12;
			for (int j = 0; j < 5; j++) {
				if (j == i)
					r1 += 1.0 / res[j];
				else
					r0 += 1.0 / res[j];
			}
			r0 = 1.0 / r0;
			r1 = 1.0 / r1;
			double v = 255.0 * r0 / (r1 + r0);
			if (v < 0) v = 0;
			if (v > 255) v = 255;
			w[net][i] = v;
			sum += v;
		}
		if (net == 0)
			scale = 255.0 / sum;
	}
	for (int i = 0; i < 32; i++)
		for (int net = 0; net < 2; net++) {
			double s = 0;
			for (int b = 0; b < 5; b++)
				if ((i >> b) & 1)
					s += w[net][b] * scale;
			s_lookup[i][net] = (uint8_t)(int)(s + 0.5);
		}
}

// ---- init / frame ----
static void ak_reset(void)
{
	memset(s_shared, 0, sizeof(s_shared));
	memset(s_vram, 0, sizeof(s_vram));
	memset(s_spriteram, 0, sizeof(s_spriteram));
	memset(s_pal, 0, sizeof(s_pal));
	memset(s_palsh, 0, sizeof(s_palsh));
	s_rotary[0] = s_rotary[1] = 0;
	memset(s_rot, 0, sizeof(s_rot));
	s_bank_base = s_outlatch = 0;
	s_deposits1 = s_deposits2 = s_credits = s_coinvalue = s_mcu_data = s_mcu_latch = 0;
	s_latch = s_zbank = s_nmi_mask = s_pa_latch = 0;
	s_dac = 0x80;
	memset(s_zram, 0, sizeof(s_zram));
	z80_start();
	zbank_map();
	s_zt = s_z80.cyc;
	s_nmi_f = s_nmi_period & 0xffff;
	s_nmi_t = s_zt + (s_nmi_period >> 16);
	rbx_ym_reset(0);
	rbx_ym_reset(1);
	s_mix_n = s_ym2_n = 0;
	m68k_bus_irq_reset();
	m68k_pulse_reset();
	s_cpu_frac = s_z80_frac = s_ym1_frac = s_ym2_frac = s_irq2_acc = 0;
}

static void ak_exit(void);

static int ak_init(const rbx_game *g)
{
	if (!s_frame_hp && !(s_frame_hp = calloc(1, sizeof(*s_frame_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	if (!s_spriteram_hp && !(s_spriteram_hp = calloc(1, sizeof(*s_spriteram_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	const char *m = g->machine, *set = g->set;
	const int goldmedal = !strcmp(m, "goldmedal");
	s_board = goldmedal ? (strcmp(set, "goldmedl") ? B_III : B_II)
	        : (!strcmp(m, "alpha68k_II") || !strcmp(m, "btlfieldb")) ? B_II : B_V;
	s_tilecb = !strcmp(m, "skyadventure") ? T_NOFLIPX : !strcmp(m, "gangwars") ? T_NOFLIPY : T_DEFAULT;
	s_btlfieldb = !strcmp(m, "btlfieldb");
	s_vbl_level = s_btlfieldb || goldmedal ? 1 : s_board == B_III ? 1 : 3;
	s_irq2_hz = s_btlfieldb ? 240 : goldmedal ? 100 : 0;
	s_cpu_clock = s_board == B_II ? 8000000 : 10000000;
	s_npal = s_board == B_II ? 2048 : 4096;
	s_cmask = s_npal / 16 - 1;
	// (7614 Hz; III / V: the pixel clock / 384 / 2)
	s_nmi_period = s_board == B_II ? (uint32_t)((double)Z80_CLOCK * 65536.0 / 7614.0 + 0.5) : (uint32_t)(768u << 16);
	// init_*: the controls inverted, the MCU's ID and coin IDs
	s_invert = !strcmp(set, "timesold1") || !strcmp(set, "btlfield") || s_btlfieldb;
	if (!strncmp(set, "goldmedl", 8)) {
		s_mcu_id = 0x8803;
		s_coin_id = 0x23 | 0x24 << 8;
	} else if (!strncmp(set, "skyadvnt", 8)) {
		s_mcu_id = 0x8814;
		s_coin_id = !strcmp(set, "skyadvntu") ? 0x23 | 0x24 << 8 : 0x22 | 0x22 << 8;
	} else if (!strncmp(set, "gangwars", 8) || !strncmp(set, "sbasebal", 8)) {
		s_mcu_id = 0x8512;
		s_coin_id = 0x23 | 0x24 << 8;
	} else {
		s_mcu_id = 0;
		s_coin_id = 0x22 | 0x22 << 8;
	}
	uint32_t ts, ss;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_zrom = rbx_region("audiocpu", &s_zrom_size);
	uint8_t *tx = rbx_region("tiles", &ts), *spr = rbx_region("sprites", &ss);
	if (!s_rom || s_rom_size < 0x80000 || !s_zrom || !tx || !spr) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	m68k_bus_swap16(s_rom, s_rom_size);
	if (!strcmp(set, "sbasebal")) {   // init_sbasebal: the MCU's pitch speed check and the ROM checksums patched out
		uint16_t *w = (uint16_t *)s_rom;
		w[0xb672 / 2] = w[0x44e / 2] = w[0x450 / 2] = w[0x458 / 2] = w[0x45a / 2] = 0x4e71;
	}
	{
		static uint32_t xc[8], yc[8], xs[16], ys[16];
		static const uint32_t pc_ii[4] = { 0, 4, 8, 12 }, pc_v[4] = { 0, 1, 2, 3 }, ps[4] = { 0, 8, 16, 24 };
		static const uint32_t xc_v[8] = { 16 * 8 + 4, 16 * 8 + 0, 24 * 8 + 4, 24 * 8 + 0, 4, 0, 8 * 8 + 4, 8 * 8 + 0 };
		for (int i = 0; i < 4; i++) {   // charlayout: STEP4(8*4*4+3,-1), STEP4(3,-1); STEP8(0,4*4)
			xc[i] = (uint32_t)(8 * 4 * 4 + 3 - i);
			xc[4 + i] = (uint32_t)(3 - i);
		}
		for (int i = 0; i < 8; i++) {
			yc[i] = (uint32_t)(i * (s_board == B_II ? 16 : 8));   // (charlayout_V: STEP8(0,8))
			xs[i] = (uint32_t)(16 * 8 * 4 + 7 - i);
			xs[8 + i] = (uint32_t)(7 - i);
		}
		for (int i = 0; i < 16; i++)
			ys[i] = (uint32_t)(i * 32);
		const rbx_gfx_layout lc = { 8, 8, 4, s_board == B_II ? pc_ii : pc_v, s_board == B_II ? xc : xc_v, yc, 32 * 8 };
		const rbx_gfx_layout ls = { 16, 16, 4, ps, xs, ys, 16 * 16 * 4 };
		if (rbx_gfx_decode(&s_gfxtx, &lc, tx, ts) || rbx_gfx_decode(&s_gfxspr, &ls, spr, ss)) {
			rbx_fail("%s: out of memory (graphics)", g->set);
			ak_exit();
			return -1;
		}
	}
	rgb_lookups();
	s_snap = calloc(1, sizeof(*s_snap));
	rbx_ym_config yc;   // YM2203 0.65 (port A: the NMI mask), YM2413 1.0
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM2203;
	yc.clock = YM1_CLOCK;
	yc.gain[0] = yc.gain[1] = yc.gain[2] = yc.gain[3] = 166;
	yc.port_a_write = porta_w;
	int rc = !s_snap || rbx_ym_init(0, &yc) || rbx_ym_set_async(0, 1);
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM2413;
	yc.clock = YM2_CLOCK;
	yc.gain[0] = yc.gain[1] = 256;
	rc = rc || rbx_ym_init(1, &yc) || rbx_ym_set_async(1, 1);
	if (!rc) {
		s_mix_cap = (int)(rbx_ym_rate(0) / 50) + 256;
		s_ym2_cap = (int)(rbx_ym_rate(1) / 50) + 256;
		s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
		s_ym2buf = (int32_t *)calloc((size_t)s_ym2_cap, sizeof(int32_t));
		rc = !s_mix || !s_ym2buf || rbx_audio_setup(rbx_ym_rate(0)) || rbx_video_setup(VIS_W, VIS_H, s_npal);
	}
	if (rc) {
		rbx_fail("%s: out of memory", g->set);
		ak_exit();
		return -1;
	}
	memset(s_zmap, 0, sizeof(s_zmap));
	for (uint32_t p = 0; p < 0x80 && p * 256 + 256 <= s_zrom_size; p++)
		s_zmap[p] = s_zrom + p * 256;
	for (uint32_t p = 0; p < 8; p++)
		s_zmap[0x80 + p] = s_zram + p * 256;
	static const char *const names[7] = { "IN0", "IN1", "IN2", "IN3", "IN4", "IN5", "IN6" };
	for (int i = 0; i < 7; i++)
		s_port[i] = rbx_port_find(names[i]);
	s_rotgame = s_board == B_II && rbx_port_has(s_port[5], RBX_IPT_POSITIONAL);
	rbx_set_rstick_dial(s_rotgame);
	m68k_init();
	m68k_set_cpu_type(M68K_CPU_TYPE_68000);
	m68k_set_mame_timing(1);
	m68k_bus_reset_map();
	m68k_bus_map(0, 0x3ffff, s_rom, 0x40000, 1, 0);
	m68k_bus_map(0x040000, s_board == B_II ? 0x040fff : 0x043fff, s_shared, s_board == B_II ? 0x1000 : 0x4000, 1, 1);
	m68k_bus_map(0x800000, 0x83ffff, s_rom + 0x40000, 0x40000, 1, 0);
	m68k_bus.read8 = main_read8;
	m68k_bus.read16 = main_read16;
	m68k_bus.write8 = main_write8;
	m68k_bus.write16 = main_write16;
	ak_reset();
	return 0;
}

static void ak_exit(void)
{
	m68k_set_mame_timing(0);
	rbx_ym_exit(0);
	rbx_ym_exit(1);
	free(s_mix);
	free(s_ym2buf);
	s_mix = s_ym2buf = NULL;
	rbx_gfx_free(&s_gfxtx);
	rbx_gfx_free(&s_gfxspr);
	free(s_snap);
	s_snap = NULL;
	free(s_frame_hp);
	s_frame_hp = NULL;
	free(s_spriteram_hp);
	s_spriteram_hp = NULL;
}

static void ak_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	if (s_rotgame)
		for (int p = 0; p < 2; p++)
			s_rotary[p] = rbx_rotary_turn(&s_rot[p], p + 1, 12, s_rotary[p], AK_ROT_CW, 12, ak_facing(p));
	const uint64_t irq2_step = (uint64_t)FRAME_MHZ * LINES;
	for (int line = 0; line < LINES; line++) {
		if (line == VIS_Y0 + VIS_H) {   // the vblank: the frame drawn, the IRQ held
			vblank_capture();
			m68k_bus_hold_irq(s_vbl_level);
		}
		if (s_irq2_hz) {   // the MCU's periodic IRQ 2 (held)
			s_irq2_acc += (uint64_t)s_irq2_hz * 1000;
			if (s_irq2_acc >= irq2_step) {
				s_irq2_acc -= irq2_step;
				m68k_bus_hold_irq(2);
			}
		}
		m68k_execute(rbx_line_cycles(&s_cpu_frac, s_cpu_clock, FRAME_MHZ, LINES));
		sound_line();
	}
	sound_frame_end();
}

static int ak_state(void *ctx, int load)
{
	const uint32_t csize = m68k_context_size();
	uint8_t *cpu = (uint8_t *)malloc(csize);
	if (!cpu)
		return -1;
	if (!load)
		m68k_get_context(cpu);
	uint32_t irq[2] = { m68k_bus_irq_pending(), m68k_bus_cur->irq_asserted };
	uint64_t fr[5] = { s_cpu_frac, s_z80_frac, s_ym1_frac, s_ym2_frac, s_irq2_acc };
	int32_t regs[16] = { s_bank_base, s_outlatch, s_deposits1, s_deposits2, s_credits, s_coinvalue, s_mcu_data,
	                     s_mcu_latch, s_latch, s_zbank, s_nmi_mask, s_pa_latch, s_dac, (int32_t)s_nmi_f,
	                     (int32_t)(s_nmi_t - s_zt), (int32_t)(s_z80.cyc - s_zt) };
	z80 z = s_z80;
	int rc = 0;
	rc |= rbx_state_io(ctx, "m68k", cpu, csize);
	rc |= rbx_state_io(ctx, "irq", irq, sizeof(irq));
	rc |= rbx_state_io(ctx, "ram", s_shared, sizeof(s_shared));
	rc |= rbx_state_io(ctx, "vram", s_vram, sizeof(s_vram));
	rc |= rbx_state_io(ctx, "spr", s_spriteram, sizeof(s_spriteram));
	rc |= rbx_state_io(ctx, "pal", s_pal, sizeof(s_pal));
	rc |= rbx_state_io(ctx, "palsh", s_palsh, sizeof(s_palsh));
	rc |= rbx_state_io(ctx, "regs", regs, sizeof(regs));
	rc |= rbx_state_io(ctx, "z80", &z, sizeof(z));
	rc |= rbx_state_io(ctx, "zram", s_zram, sizeof(s_zram));
	rc |= rbx_ym_state(0, ctx, load);
	rc |= rbx_ym_state(1, ctx, load);
	rc |= rbx_state_io(ctx, "frac", fr, sizeof(fr));
	if (s_rotgame)
		rc |= rbx_state_io(ctx, "rot", s_rotary, sizeof(s_rotary));
	if (!rc && load) {
		m68k_set_context(cpu);
		m68k_bus_set_irq_pending(irq[0]);
		m68k_bus_cur->irq_asserted = irq[1];
		m68k_bus_update_irq();
		s_bank_base = (uint8_t)regs[0];
		s_outlatch = (uint8_t)regs[1];
		s_deposits1 = regs[2];
		s_deposits2 = regs[3];
		s_credits = regs[4];
		s_coinvalue = regs[5];
		s_mcu_data = regs[6];
		s_mcu_latch = regs[7];
		s_latch = (uint8_t)regs[8];
		s_zbank = (uint8_t)regs[9];
		s_nmi_mask = (uint8_t)regs[10];
		s_pa_latch = (uint8_t)regs[11];
		s_dac = (uint8_t)regs[12];
		s_nmi_f = (uint32_t)regs[13];
		z.read_byte = z80_read;
		z.write_byte = z80_write;
		z.port_in = z80_in;
		z.port_out = z80_out;
		z.userdata = NULL;
		z.read_map = s_zmap;
		s_z80 = z;
		s_zt = s_z80.cyc - (unsigned long)regs[15];
		s_nmi_t = s_zt + (unsigned long)regs[14];
		zbank_map();
		s_cpu_frac = fr[0];
		s_z80_frac = fr[1];
		s_ym1_frac = fr[2];
		s_ym2_frac = fr[3];
		s_irq2_acc = fr[4];
	}
	free(cpu);
	return rc ? -1 : 0;
}

static int ak_save(void *ctx) { return ak_state(ctx, 0); }
static int ak_load(void *ctx) { return ak_state(ctx, 1); }
static int ak_supported(const char *machine)
{
	static const char *const ok[] = { "alpha68k_II", "btlfieldb", "goldmedal", "alpha68k_V", "skyadventure", "gangwars" };
	for (unsigned i = 0; i < sizeof(ok) / sizeof(ok[0]); i++)
		if (!strcmp(machine, ok[i]))
			return 1;
	return 0;
}

const rbx_driver rbx_drv_alpha68k = { "SNK Alpha 68K", ak_init, ak_exit, ak_reset, ak_frame, ak_save, ak_load,
                                      ak_supported, ak_render, ak_help };
