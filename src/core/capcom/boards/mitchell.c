/*  Mitchell boards (Capcom CPS-style hardware): Pang / Buster Bros, Super Pang, Block Block, Capcom World, Hatena
    Hatena no Daibouken, Quiz games, Mahjong Gakuen / Poker Ladies / Marukin ... Ported from MAME
    src/mame/capcom/mitchell.cpp and kabuki.cpp (BSD-3-Clause, copyright-holders: Paul Leaman, Nicola Salmoria).
    Z80 at 8 MHz with Kabuki-encrypted opcodes and data (the bootlegs: decoded copies in the ROM), IRQ at lines 0 and
    240; a YM2413 at 4 MHz and an OKI6295 at 1 MHz (two sample banks), a 93C46 EEPROM. Video: a 512x256 tilemap of 8x8
    (4 bpp, 128 colours, pen 15 clear), 16x16 sprites; xRGB 444 palette RAM in two banks.
    Not here: the bootlegs with their own sound CPU (spangbl, pangba, pangbb, pangbc, mstworld, mstworld2, pkladiesbl).
    The screen is MAME's (refresh 57.42 Hz or 60 Hz, 2.5 ms of vblank, 256 lines): vblank also covers the frame's first
    lines, as in MAME.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/z80/z80.h"
#include "../../machine/eepromser.h"
#include "../../sound/okim6295.h"
#include "../../sound/rbx_ym.h"
#include "../../video/gfxlayout.h"
#include "../machine/kabuki.h"
#include "cap8.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define LINES 256
#define VIS_X0 64
#define VIS_W 384
#define VIS_Y0 8
#define VIS_H 240
#define CPU_CLOCK 8000000
#define YM_CLOCK 4000000
#define OKI_CLOCK 1000000

enum { IN_NORMAL, IN_MAHJONG, IN_BLOCK };

static int s_mgakuen, s_input_type, s_npal, s_vbl_lines;
static uint32_t s_frame_mhz;
static uint8_t *s_rom;
static uint32_t s_rom_size;
static uint8_t *s_dec;                 // the decoded opcodes (0x0000-0x7fff fixed, from 0x10000 the banks)
static const uint8_t *s_opfix, *s_opbanks;
static uint8_t *s_opmem;               // 64 KB (calloc in init: ADR-040)       // the opcode view (op_rom): ROM, the bank, the work RAM at 0xe000
static rbx_gfx s_gchar, s_gspr;
static int s_port_sys, s_port_in[3], s_port_dsw[2], s_port_key[10], s_port_dial[2];
static z80 s_main;
static okim6295_t s_oki;
static eepser_t s_eeprom;
static const uint8_t *s_mmap[256];
static int32_t *s_mix;
static int s_mix_n, s_mix_cap;
static int s_line;

#define RAM (s_opmem + 0xe000)         // the work RAM (also the opcodes there)

static struct {
	uint8_t pal[0x1000], color[0x800], vram[0x1000], obj[0x1000];
	uint8_t flip, video_bank, pal_bank, bank, irq_source, keymatrix, dial_selected, dial[2], dir[2];
	int irq;
	uint64_t frac, ym_frac;
	uint32_t oki_acc;
	int32_t oki_hold;
	uint8_t state_end;
} t;

static struct {
	uint8_t pal[0x1000], color[0x800], vram[0x1000], obj[0x1000];
	uint8_t flip;
} *s_snap;

static uint8_t z_in(z80 *z, uint8_t port);
static void z_out(z80 *z, uint8_t port, uint8_t v);

// ---- main CPU ----
static void map_bank(void)
{
	for (int p = 0; p < 0x40; p++)
		s_mmap[0x80 + p] = s_rom + 0x10000 + ((uint32_t)t.bank * 0x4000 + p * 256) % (s_rom_size - 0x10000);
	if (s_opbanks)
		memcpy(s_opmem + 0x8000, s_opbanks + (uint32_t)t.bank * 0x4000, 0x4000);
}

static uint8_t main_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x8000) return s_rom[a];
	if (a < 0xc000) return s_rom[0x10000 + ((uint32_t)t.bank * 0x4000 + (a - 0x8000)) % (s_rom_size - 0x10000)];
	if (a < 0xc800) return s_mgakuen ? t.pal[a - 0xc000] : t.pal[(a - 0xc000) + (t.pal_bank ? 0x800 : 0)];
	if (a < 0xd000) return t.color[a - 0xc800];
	if (a < 0xe000) return (!s_mgakuen && t.video_bank) ? t.obj[a - 0xd000] : t.vram[a - 0xd000];
	if (s_mgakuen && a >= 0xf000) return t.obj[a - 0xf000];
	return RAM[a - 0xe000];
}
static void main_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a < 0xc000) return;
	if (a < 0xc800) {
		if (s_mgakuen) t.pal[a - 0xc000] = v;
		else t.pal[(a - 0xc000) + (t.pal_bank ? 0x800 : 0)] = v;
	} else if (a < 0xd000) t.color[a - 0xc800] = v;
	else if (a < 0xe000) {
		if (!s_mgakuen && t.video_bank) t.obj[a - 0xd000] = v;
		else t.vram[a - 0xd000] = v;
	} else if (s_mgakuen && a >= 0xf000) t.obj[a - 0xf000] = v;
	else RAM[a - 0xe000] = v;
}

static int64_t now(void) { return (int64_t)s_main.cyc; }

static uint8_t block_input_r(int o)
{
	if (t.dial_selected) {
		int delta = ((int)rbx_port_read(s_port_dial[o]) - t.dial[o]) & 0xff;
		if (delta & 0x80) {
			delta = (-delta) & 0xff;
			if (t.dir[o]) {
				t.dir[o] = 0;
				delta = 0;
			}
		} else if (delta > 0) {
			if (!t.dir[o]) {
				t.dir[o] = 1;
				delta = 0;
			}
		}
		if (delta > 0x3f)
			delta = 0x3f;
		return (uint8_t)(delta << 2);
	}
	int res = (int)rbx_port_read(s_port_in[o + 1]) & 0xf7;
	if (t.dir[o])
		res |= 0x08;
	return (uint8_t)res;
}

static uint8_t z_in(z80 *z, uint8_t port)
{
	(void)z;
	if (port <= 2) {   // input_r
		if (port == 0 || s_input_type == IN_NORMAL)
			return (uint8_t)rbx_port_read(s_port_in[port]);
		if (s_input_type == IN_MAHJONG) {
			for (int i = 0; i < 5; i++)
				if (t.keymatrix & (0x80 >> i))
					return (uint8_t)rbx_port_read(s_port_key[(port - 1) * 5 + i]);
			return 0xff;
		}
		return block_input_r(port - 1);
	}
	if (s_mgakuen && (port == 3 || port == 4))
		return (uint8_t)rbx_port_read(s_port_dsw[port - 3]);
	if (port == 5) {   // port5_r: SYS0 (bit 3 vblank, bit 7 the EEPROM's DO) with the IRQ source in bit 0
		const int vbl = s_line >= VIS_Y0 + VIS_H || s_line < s_vbl_lines;
		rbx_port_custom(s_port_sys, 0x88, (vbl ? 0x08u : 0) | (eepser_do_read_93cxx(&s_eeprom, now()) ? 0x80u : 0));
		return (uint8_t)((rbx_port_read(s_port_sys) & 0xfe) | (t.irq_source & 1));
	}
	return 0;
}
static void z_out(z80 *z, uint8_t port, uint8_t v)
{
	(void)z;
	switch (port) {
	case 0x00:   // gfxctrl_w
		t.flip = (v & 4) != 0;
		s_oki.bank = (uint32_t)((v >> 4) & 1) * 0x40000;
		t.pal_bank = v & 0x20;
		break;
	case 0x01:   // input_w
		if (s_input_type == IN_MAHJONG)
			t.keymatrix = v;
		else if (s_input_type == IN_BLOCK) {
			if (v == 0x08) {
				t.dial[0] = (uint8_t)rbx_port_read(s_port_dial[0]);
				t.dial[1] = (uint8_t)rbx_port_read(s_port_dial[1]);
			} else if (v == 0x80)
				t.dial_selected = 0;
			else
				t.dial_selected = 1;
		}
		break;
	case 0x02: t.bank = v & 0x0f; map_bank(); break;
	case 0x03: rbx_ym_write(0, 1, v); break;   // (data_w)
	case 0x04: rbx_ym_write(0, 0, v); break;   // (address_w)
	case 0x05: oki_write(&s_oki, v); break;
	case 0x07: if (!s_mgakuen) t.video_bank = v; break;
	case 0x08: eepser_cs_write(&s_eeprom, v ? 1 : 0, now()); break;
	case 0x10: eepser_clk_write(&s_eeprom, v ? 1 : 0, now()); break;
	case 0x18: eepser_di_write(&s_eeprom, v & 1); break;
	}
}

// ---- video ----
static void mt_render(void)
{
	for (int i = 0; i < s_npal; i++) {   // xRGB_444, little-endian words
		const uint32_t w = (uint32_t)(s_snap->pal[2 * i] | s_snap->pal[2 * i + 1] << 8);
		rbx_video.palette[i] = ((w >> 8) & 15) * 17 << 16 | ((w >> 4) & 15) * 17 << 8 | (w & 15) * 17;
	}
	const int flip = s_snap->flip;
	for (int y = VIS_Y0; y < VIS_Y0 + VIS_H; y++) {
		uint16_t *d = rbx_video.bitmap + (y - VIS_Y0) * rbx_video.pitch;
		const uint32_t ty = (uint32_t)(flip ? 255 - y : y) & 255;
		for (int x = VIS_X0; x < VIS_X0 + VIS_W; x++) {   // (bitmap.fill(0), then the tilemap with pen 15 clear)
			const uint32_t tx = (uint32_t)(flip ? 511 - x : x) & 511;
			const uint32_t idx = (ty >> 3) * 64 + (tx >> 3);
			const uint8_t attr = s_snap->color[idx];
			const uint32_t code = rbx_wrap((uint32_t)(s_snap->vram[2 * idx] | s_snap->vram[2 * idx + 1] << 8), s_gchar.count);
			const uint32_t px = (attr & 0x80) ? 7 - (tx & 7) : tx & 7;
			const uint8_t p = s_gchar.pix[code * 64 + (ty & 7) * 8 + px];
			d[x - VIS_X0] = p == 15 ? 0 : (uint16_t)((attr & 0x7f) * 16 + p);
		}
	}
	for (int o = 0x1000 - 0x40; o >= 0; o -= 0x20) {
		const uint8_t *s = &s_snap->obj[o];
		const int attr = s[1];
		int sx = s[3] + ((attr & 0x10) << 4), sy = ((s[2] + 8) & 0xff) - 8;
		const uint32_t code = (uint32_t)(s[0] + ((attr & 0xe0) << 3));
		if (flip) {
			sx = 496 - sx;
			sy = 240 - sy;
		}
		cap8_transpen(&s_gspr, code, (uint16_t)((attr & 0x0f) * 16), flip, flip, sx, sy, 15, VIS_Y0, VIS_Y0 + VIS_H, VIS_Y0,
		              VIS_X0, VIS_W);
	}
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap->pal, t.pal, sizeof(t.pal));
	memcpy(s_snap->color, t.color, sizeof(t.color));
	memcpy(s_snap->vram, t.vram, sizeof(t.vram));
	memcpy(s_snap->obj, t.obj, sizeof(t.obj));
	s_snap->flip = t.flip;
	rbx_video_frame_done();
}

// ---- init / frame ----
static void mt_reset(void)
{
	memset(&t, 0, offsetof(__typeof__(t), state_end));
	memset(RAM, 0, 0x2000);
	if (s_opfix)
		memcpy(s_opmem, s_opfix, 0x8000);
	map_bank();
	z80_init(&s_main);
	s_main.read_byte = main_read;
	s_main.write_byte = main_write;
	s_main.port_in = z_in;
	s_main.port_out = z_out;
	s_main.read_map = s_mmap;
	s_main.op_rom = s_opfix ? s_opmem : NULL;
	rbx_ym_reset(0);
	oki_reset(&s_oki);
	s_oki.bank = 0;
	s_mix_n = 0;
	s_line = 0;
}

static void mt_exit(void);

static int mt_init(const rbx_game *g)
{
	free(s_opmem);
	s_opmem = (uint8_t *)calloc(1, 0x10000);
	if (!s_opmem) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	const char *m = g->machine, *init = strchr(m, ':');
	init = init ? init + 1 : "";
	s_mgakuen = !strncmp(m, "mgakuen", 7);
	const int marukin_gfx = s_mgakuen || !strncmp(m, "marukin", 7);
	s_npal = s_mgakuen ? 1024 : 2048;
	s_frame_mhz = marukin_gfx ? 60000 : 57420;
	s_vbl_lines = (int)(2500 * (int)s_frame_mhz / 1000 * LINES / 1000000) - (LINES - VIS_Y0 - VIS_H);   // (2.5 ms past line 248)
	s_input_type = (!strcmp(init, "mgakuen") || !strcmp(init, "mgakuen2") || !strcmp(init, "pkladies") ||
	                !strcmp(init, "marukin")) ? IN_MAHJONG : (!strcmp(init, "block") || !strcmp(init, "blockbl")) ? IN_BLOCK : IN_NORMAL;
	uint32_t cs, ss, os;
	s_rom = rbx_region("maincpu", &s_rom_size);
	uint8_t *chars = rbx_region("chars", &cs), *spr = rbx_region("sprites", &ss), *oki = rbx_region("oki", &os);
	if (!s_rom || s_rom_size < 0x50000 || !chars || !spr || ss < 0x40000 || !oki) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	// the opcodes: configure_banks (Kabuki keys by init) / bootleg_decode (decoded copies at 0x50000 / 0x60000)
	s_opfix = s_opbanks = NULL;
	free(s_dec);
	s_dec = NULL;
	static const struct { const char *init; uint32_t k1, k2; int ak, xk; } keys[] = {
		{ "dokaben", 0x76543210, 0x01234567, 0xaa55, 0xa5 }, { "mgakuen2", 0x76543210, 0x01234567, 0xaa55, 0xa5 },
		{ "pkladies", 0x76543210, 0x01234567, 0xaa55, 0xa5 }, { "pang", 0x01234567, 0x76543210, 0x6548, 0x24 },
		{ "cworld", 0x04152637, 0x40516273, 0x5751, 0x43 }, { "hatena", 0x45670123, 0x45670123, 0x5751, 0x43 },
		{ "spang", 0x45670123, 0x45670123, 0x5852, 0x43 }, { "spangj", 0x45123670, 0x67012345, 0x55aa, 0x5a },
		{ "sbbros", 0x45670123, 0x45670123, 0x2130, 0x12 }, { "marukin", 0x54321076, 0x54321076, 0x4854, 0x4f },
		{ "qtono1", 0x12345670, 0x12345670, 0x1111, 0x11 }, { "qsangoku", 0x23456701, 0x23456701, 0x1828, 0x18 },
		{ "block", 0x02461357, 0x64207531, 0x0002, 0x01 },
	};
	if (!strcmp(init, "pangb") || !strcmp(init, "blockbl")) {
		if (s_rom_size < 0xa0000) {
			rbx_fail("%s: ROM regions missing", g->set);
			return -1;
		}
		s_opfix = s_rom + 0x50000;
		s_opbanks = s_rom + 0x60000;
	} else
		for (size_t i = 0; i < sizeof(keys) / sizeof(keys[0]); i++)
			if (!strcmp(init, keys[i].init)) {
				s_dec = (uint8_t *)calloc(1, s_rom_size);
				if (!s_dec) {
					rbx_fail("%s: out of memory", g->set);
					return -1;
				}
				kabuki_mitchell_decode(s_rom, s_dec, s_rom_size, keys[i].k1, keys[i].k2, keys[i].ak, keys[i].xk);
				s_opfix = s_dec;
				s_opbanks = s_dec + 0x10000;
				break;
			}
	memset(s_opmem, 0, 0x10000);
	{	// charlayout (32768, planes apart) / marukin_charlayout (65536), spritelayout (2048)
		static const uint32_t xc[8] = { 0, 1, 2, 3, 8, 9, 10, 11 }, yc[8] = { 0, 16, 32, 48, 64, 80, 96, 112 };
		static const uint32_t pc[4] = { 32768 * 16 * 8 + 4, 32768 * 16 * 8 + 0, 4, 0 };
		static const uint32_t xm[8] = { 0, 1, 2, 3, 16, 17, 18, 19 }, ym[8] = { 0, 32, 64, 96, 128, 160, 192, 224 };
		static const uint32_t pm[4] = { 12, 8, 4, 0 };
		static const uint32_t xs[16] = { 0, 1, 2, 3, 8, 9, 10, 11, 256, 257, 258, 259, 264, 265, 266, 267 };
		static uint32_t ys[16];
		for (int i = 0; i < 16; i++)
			ys[i] = (uint32_t)i * 16;
		static const uint32_t ps[4] = { 2048 * 64 * 8 + 4, 2048 * 64 * 8 + 0, 4, 0 };
		const rbx_gfx_layout lc = { 8, 8, 4, pc, xc, yc, 128 }, lm = { 8, 8, 4, pm, xm, ym, 256 };
		const rbx_gfx_layout ls = { 16, 16, 4, ps, xs, ys, 512 };
		const int rc = marukin_gfx ? (cs < 65536u * 32 || rbx_gfx_decode(&s_gchar, &lm, chars, 65536u * 32))
		                           : (cs < 32768u * 32 || rbx_gfx_decode(&s_gchar, &lc, chars, 32768u * 16));
		if (rc || rbx_gfx_decode(&s_gspr, &ls, spr, 2048 * 64)) {
			rbx_fail("%s: out of memory (graphics)", g->set);
			mt_exit();
			return -1;
		}
	}
	s_snap = calloc(1, sizeof(*s_snap));
	rbx_ym_config yc;
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM2413;
	yc.clock = YM_CLOCK;
	yc.gain[0] = yc.gain[1] = 256;   // (1.0)
	if (!s_snap || rbx_ym_init(0, &yc) || rbx_ym_set_async(0, 1)) {
		rbx_fail("%s: out of memory", g->set);
		mt_exit();
		return -1;
	}
	s_mix_cap = (int)(rbx_ym_rate(0) / 50) + 256;
	s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	if (!s_mix || rbx_audio_setup(rbx_ym_rate(0)) || rbx_video_setup(VIS_W, VIS_H, s_npal)) {
		rbx_fail("%s: out of memory", g->set);
		mt_exit();
		return -1;
	}
	oki_init(&s_oki, OKI_CLOCK, 1, oki, os, s_mgakuen ? 128 : 77);   // (0.50 / 0.30)
	eepser_init(&s_eeprom, EEPSER_93CXX, 64, 16, CPU_CLOCK);
	{
		uint32_t esz;
		const uint8_t *e = rbx_region_opt("eeprom", &esz);
		if (e)
			eepser_default(&s_eeprom, e, esz);
		eepser_nv_load(&s_eeprom, g->set);
	}
	memset(s_mmap, 0, sizeof(s_mmap));
	for (int p = 0; p < 0x80; p++)
		s_mmap[p] = s_rom + p * 256;
	for (int p = 0xe0; p < (s_mgakuen ? 0xf0 : 0x100); p++)
		s_mmap[p] = RAM + (p - 0xe0) * 256;
	s_port_sys = rbx_port_find("SYS0");
	for (int i = 0; i < 3; i++) {
		static const char *const n[3] = { "IN0", "IN1", "IN2" };
		s_port_in[i] = rbx_port_find(n[i]);
	}
	s_port_dsw[0] = rbx_port_find("DSW0");
	s_port_dsw[1] = rbx_port_find("DSW1");
	for (int i = 0; i < 10; i++) {
		char n[8] = "KEY0";
		n[3] = (char)('0' + i);
		s_port_key[i] = rbx_port_find(n);
	}
	s_port_dial[0] = rbx_port_find("DIAL1");
	s_port_dial[1] = rbx_port_find("DIAL2");
	mt_reset();
	return 0;
}

static void mt_exit(void)
{
	rbx_video_wait_idle();
	eepser_nv_save(&s_eeprom);
	rbx_ym_exit(0);
	free(s_mix);
	s_mix = NULL;
	rbx_gfx_free(&s_gchar);
	rbx_gfx_free(&s_gspr);
	free(s_snap);
	s_snap = NULL;
	free(s_dec);
	s_dec = NULL;
	free(s_opmem);
	s_opmem = NULL;
}

static void mt_frame(void)
{
	rbx_audio.frame_mhz = s_frame_mhz;
	for (int line = 0; line < LINES; line++) {
		s_line = line;
		if (line == VIS_Y0 + VIS_H)
			vblank_capture();
		if (line == 240 || line == 0) {   // (mitchell_irq)
			s_main.int_data = 0xff;
			t.irq = 1;
			t.irq_source = line == 240;
		}
		z80_run(&s_main, s_main.cyc + (unsigned long)rbx_line_cycles(&t.frac, CPU_CLOCK, s_frame_mhz, LINES), &t.irq, 1);
		const uint32_t clocks = (uint32_t)rbx_line_cycles(&t.ym_frac, YM_CLOCK, s_frame_mhz, LINES);
		const int n = rbx_ym_advance(0, clocks, s_mix + s_mix_n, s_mix_cap - s_mix_n);
		const uint32_t yr = rbx_ym_rate(0);
		for (int i = 0; i < n; i++) {
			t.oki_acc += s_oki.rate;
			while (t.oki_acc >= yr) {
				t.oki_acc -= yr;
				t.oki_hold = oki_sample(&s_oki);
			}
			s_mix[s_mix_n + i] += t.oki_hold;
		}
		s_mix_n += n;
	}
	rbx_ym_async_take(0, s_mix, s_mix_n);
	for (int i = 0; i < s_mix_n; i++)
		rbx_audio_put(s_mix[i]);
	memset(s_mix, 0, sizeof(int32_t) * (size_t)s_mix_n);
	s_mix_n = 0;
}

static int mt_state(void *ctx, int load)
{
	int rc = 0;
	z80 c = s_main;
	rc |= rbx_state_io(ctx, "mt", &t, (uint32_t)offsetof(__typeof__(t), state_end));
	rc |= rbx_state_io(ctx, "mt_ram", RAM, 0x2000);
	rc |= rbx_state_io(ctx, "mt_main", &c, sizeof(c));
	rc |= rbx_ym_state(0, ctx, load);
	rc |= rbx_state_io(ctx, "mt_oki", s_oki.v, sizeof(s_oki.v) + sizeof(int));
	rc |= eepser_state(&s_eeprom, ctx, load, "mt_eeprom");
	if (!rc && load) {
		c.read_byte = main_read;
		c.write_byte = main_write;
		c.port_in = z_in;
		c.port_out = z_out;
		c.userdata = NULL;
		c.read_map = s_mmap;
		c.op_rom = s_opfix ? s_opmem : NULL;
		s_main = c;
		map_bank();
	}
	return rc ? -1 : 0;
}

static int mt_save(void *ctx) { return mt_state(ctx, 0); }
static int mt_load(void *ctx) { return mt_state(ctx, 1); }
static int mt_supported(const char *machine)
{
	return !strncmp(machine, "pang:", 5) || !strncmp(machine, "pangnv:", 7) || !strncmp(machine, "marukin:", 8) ||
	       !strncmp(machine, "mgakuen:", 8);
}

const rbx_driver rbx_drv_mitchell = { "Mitchell", mt_init, mt_exit, mt_reset, mt_frame, mt_save, mt_load,
                                      mt_supported, mt_render, NULL };
