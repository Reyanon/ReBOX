/*  Tumble Pop bootleg hardware and the games on it: Tumble Pop bootlegs, Jump Kids, Pang Pang, Super Trio, Fancy
    World, Magic Purple and the SemiCom games (Hatch Catch, Cookie & Bibi, Choky! Choky!, B.C. Story ...). Ported from
    MAME src/mame/dataeast/tumbleb.cpp (BSD-3-Clause, copyright-holders: Bryan McPhail, David Haywood).
    68000 (IRQ 6 held at vblank), generic tilemaps in place of the chip 55 (an 8x8 or a 16x16 front layer, a 16x16
    back layer), a bootleg copy of the sprite chip 52 (video/decospr.c), and one of:
    - an OKI on the 68000 bus (tumbleb), or a PIC driving it (tumbleb2, pangpang: MAME's simulation of the PIC);
    - Z80 + OKI (jumpkids, suprtrio);
    - YM2151 + OKI on the 68000 bus (Unico: fncywld, magipur);
    - Z80 + YM2151 + OKI (SemiCom), with an 87C52 / AT89C52 MCU writing the main RAM through its ports, or the RAM
      contents MAME has from it ("user1").
    Not supported (MAME: not working): tumblepba, funkyjetb.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m68k_bus.h"
#include "../../cpu/m68k/m68k.h"
#include "../../cpu/mcs51/mcs51.h"
#include "../../cpu/z80/z80.h"
#include "../../sound/okim6295.h"
#include "../../sound/rbx_ym.h"
#include "../../video/gfxlayout.h"
#include "../video/decospr.h"

#include <stdlib.h>
#include <string.h>

#define VIS_W 320
#define VIS_H 240
#define VIS_Y0 8                 // set_visarea(0, 319, 8, 247)
#define SCREEN_H 256             // (set_size(40*8, 32*8); suprtrio: the raw screen's lines)

// the board's kind (memory map and sound)
enum { K_TUMBLEB, K_TUMBLEB2, K_PANGPANG, K_JUMPKIDS, K_SUPRTRIO, K_FNCYWLD, K_MAGIPUR, K_SEMICOM };
// tile formats
enum { T_NORMAL, T_FNCYWLD, T_PANGPANG };
// screen_update offsets
enum { O_TUMBLEPB, O_ALT, O_BCSTORY, O_SEMIBASE, O_SDFIGHT };
// palette formats
enum { P_BGR444, P_BGR555, P_RGB444 };
// 0x100002 tile bank writes (SemiCom)
enum { B_BCSTORY, B_CHOKCHOK, B_WLSTAR };

static int s_kind, s_tiles, s_offs, s_palfmt, s_bankw, s_mcu_on, s_bcstory_io, s_sdfight, s_palext;
static uint32_t s_cpu_clock, s_frame_mhz;
static int s_lines, s_vbl_lines;

static uint8_t *s_rom;
static uint32_t s_rom_size;
static uint16_t s_ram[0x8000];           // main RAM (0x120000 16 KB; fncywld / magipur 64 KB; suprtrio 32 KB)
static uint16_t s_ram2[0x800];           // 0x1a0000 (SemiCom 4 KB)
static uint16_t s_ram3[0x1000];          // 0x341000 .. 0x342fff (SemiCom "extra RAM")
static uint16_t s_pal[0x800];
static uint16_t s_spriteram[0x800];
static uint16_t s_pf1[0x1000], s_pf2[0x1000];
static uint16_t s_control0[8];           // tumblepb_control_0_w (suprtrio: the "control" RAM)
static uint16_t s_tilebank;
static const uint8_t *s_protdata;        // "user1" (big-endian words)
static uint32_t s_protdata_size;
static int s_port_players, s_port_dsw, s_port_sys;
static int s_line;
static uint32_t s_frame, s_rand;
static uint64_t s_cpu_frac;
static uint8_t s_suprtrio_latch;

static rbx_gfx s_gfx8, s_gfx16, s_gfxspr;
static decospr_t s_spr;
typedef struct {
	uint16_t pal[0x800], spriteram[0x800], pf1[0x1000], pf2[0x1000], control0[8], tilebank;
	uint32_t frame;
} snap_t;
static snap_t *s_snap;

// ---- sound ----
static okim6295_t s_oki;
static uint8_t *s_okibuf;                // the OKI's ROM as the board changes it (bank copies), from the region
static const uint8_t *s_okirom, *s_oki2rom;
static uint32_t s_oki_size, s_oki2_size;
static int s_okibank;                    // the last bank copy (-1: none)
static int s_has_ym, s_has_z80;
static uint32_t s_ym_clock, s_z80_clock;
static uint64_t s_z80_frac, s_ym_frac, s_snd_frac;
static uint32_t s_oki_acc;
static int32_t s_oki_hold;
static int32_t *s_mix;
static int s_mix_n, s_mix_cap;
static z80 s_z80;
static uint8_t s_zram[0x800];
static const uint8_t *s_zrom;
static uint32_t s_zrom_size, s_zrom_mapped;
static const uint8_t *s_zmap[256];
static uint8_t s_latch;
static int s_z80_irq, s_ym_irq;
static int s_music_command, s_music_bank, s_music_is_playing;   // (tumbleb2 PIC simulation)

// ---- MCU (SemiCom protection) ----
static mcs51_t s_mcu;
static uint64_t s_mcu_frac;
static uint8_t s_prot_offset;
static uint16_t s_protbase;

static void prot_port_w(void *u, int port, uint8_t v)
{
	(void)u;
	uint16_t *w = &s_ram[((s_protbase / 2) + s_prot_offset) & 0x7fff];
	if (port == 0)       // prot_p0_w
		*w = (uint16_t)((*w & 0xff00) | v);
	else if (port == 1)  // prot_p1_w
		*w = (uint16_t)((*w & 0x00ff) | v << 8);
	else if (port == 2)  // prot_p2_w
		s_prot_offset = v;
}

// ---- OKI ROM bank copies ----
static void oki_copy(uint32_t dst, const uint8_t *src, uint32_t src_size, uint32_t from, uint32_t len)
{
	if (dst + len <= s_oki_size && from + len <= src_size)
		memcpy(s_okibuf + dst, src + from, len);
}
static void oki_bank_apply(int bank)
{
	s_okibank = bank;
	if (bank < 0)
		return;
	switch (s_kind) {
	case K_TUMBLEB2: case K_PANGPANG:   // tumbleb2_set_music_bank
		oki_copy(0x38000, s_okibuf, s_oki_size, 0x80000 + 0x38000 + 0x8000u * (uint32_t)bank, 0x8000);
		break;
	case K_JUMPKIDS:                    // jumpkids_oki_bank_w
		if (s_oki2rom)
			oki_copy(0x20000, s_oki2rom, s_oki2_size, (uint32_t)(bank & 3) * 0x20000u, 0x20000);
		break;
	default:                            // oki_sound_bank_w
		oki_copy(0x30000, s_okibuf, s_oki_size, (uint32_t)bank * 0x10000u + 0x40000, 0x10000);
		break;
	}
}

// ---- tumbleb2 / pangpang: MAME's simulation of the protected sound PIC ----
static const int tumbleb_sound_lookup[256] = {
	/*0     1     2     3     4     5     6     7     8     9     a     b     c     d     e    f*/
	0x00,  -2,  0x00, 0x00,   -2,   -2,   -2,   -2,   -2,   -2,   -2,   -2,   -2,   -2, 0x00,   -2, /* 0 */
	  -2, 0x00,   -2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 1 */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, /* 2 */
	0x19, 0x00, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, /* 3 */
	0x28, 0x29, 0x2a, 0x2b, 0x2c, 0x2d, 0x2e, 0x2f, 0x30, 0x31, 0x32, 0x00, 0x00, 0x00, 0x00, 0x00, /* 4 */
	0
};

static void tumbleb2_playmusic(void)
{
	const int status = oki_read(&s_oki);
	if (s_music_is_playing && !((status >> 3) & 1)) {
		oki_write(&s_oki, (uint8_t)(0x80 | s_music_command));
		oki_write(&s_oki, 0x00 | 0x82);
	}
}

static void tumbleb2_play_sound(int data)
{
	const int status = oki_read(&s_oki);
	if (!(status & 1)) {
		oki_write(&s_oki, (uint8_t)(0x80 | data));
		oki_write(&s_oki, 0x00 | 0x12);
	} else if (!(status & 2)) {
		oki_write(&s_oki, (uint8_t)(0x80 | data));
		oki_write(&s_oki, 0x00 | 0x22);
	} else if (!(status & 4)) {
		oki_write(&s_oki, (uint8_t)(0x80 | data));
		oki_write(&s_oki, 0x00 | 0x42);
	}
}

static void process_tumbleb2_music_command(int data)
{
	const int status = oki_read(&s_oki);
	if (data == 1) {   // stop?
		if ((status >> 3) & 1) {
			oki_write(&s_oki, 0x40);   // stop playing music
			s_music_is_playing = 0;
		}
		return;
	}
	if (s_music_is_playing == data)
		return;
	s_music_is_playing = data;
	oki_write(&s_oki, 0x40);   // stop the current music
	switch (data) {
	case 0x04: s_music_bank = 1; s_music_command = 0x38; break;   // map screen
	case 0x05: s_music_bank = 6; s_music_command = 0x38; break;   // america
	case 0x06: s_music_bank = 2; s_music_command = 0x38; break;   // asia
	case 0x07: s_music_bank = 4; s_music_command = 0x38; break;   // africa / egypt
	case 0x08: s_music_bank = 3; s_music_command = 0x38; break;   // antartica
	case 0x09: s_music_bank = 4; s_music_command = 0x38; break;   // brazil / south america
	case 0x0a: s_music_bank = 2; s_music_command = 0x38; break;   // japan
	case 0x0b: s_music_bank = 5; s_music_command = 0x38; break;   // australia
	case 0x0c: s_music_bank = 6; s_music_command = 0x38; break;   // france / europe
	case 0x0d: s_music_bank = 7; s_music_command = 0x38; break;   // how to play
	case 0x0f: s_music_bank = 0; s_music_command = 0x33; break;   // stage clear
	case 0x10: s_music_bank = 8; s_music_command = 0x38; break;   // boss stage
	case 0x12: s_music_bank = 0; s_music_command = 0x34; break;   // world clear
	default:   s_music_bank = 8; s_music_command = 0x38; break;
	}
	oki_bank_apply(s_music_bank);
	tumbleb2_playmusic();
}

static void tumbleb2_soundmcu_w(uint16_t data)
{
	const int sound = tumbleb_sound_lookup[data & 0xff];
	if (sound == -2)
		process_tumbleb2_music_command(data);
	else if (sound != 0)
		tumbleb2_play_sound(sound);
}

// ---- sound CPU ----
static uint8_t z80_read(void *u, uint16_t a)
{
	(void)u;
	if (s_kind == K_JUMPKIDS) {   // jumpkids_sound_map
		if (a < 0x1000) return a < s_zrom_size ? s_zrom[a] : 0;
		if (a >= 0x8000 && a < 0x8800) return s_zram[a - 0x8000];
		if (a == 0x9800) return oki_read(&s_oki);
		if (a == 0xa000) return s_latch;
		return 0;
	}
	// semicom_sound_map / suprtrio_sound_map
	if (a < 0xd000) return a < s_zrom_size ? s_zrom[a] : 0;
	if (a < 0xd800) return s_zram[a - 0xd000];
	if (a == 0xf000 || a == 0xf001) return s_has_ym ? rbx_ym_read(0, a & 1) : 0;
	if (a == 0xf002) return oki_read(&s_oki);
	if (a == 0xf008) return s_latch;
	return 0;
}
static void z80_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (s_kind == K_JUMPKIDS) {
		if (a >= 0x8000 && a < 0x8800) s_zram[a - 0x8000] = v;
		else if (a == 0x9000) oki_bank_apply(v & 3);
		else if (a == 0x9800) oki_write(&s_oki, v);
		return;
	}
	if (a >= 0xd000 && a < 0xd800) s_zram[a - 0xd000] = v;
	else if ((a == 0xf000 || a == 0xf001) && s_has_ym) rbx_ym_write(0, a & 1, v);
	else if (a == 0xf002) oki_write(&s_oki, v);
	else if (a == 0xf00e) oki_bank_apply(v);
}
static uint8_t z80_in(z80 *z, uint8_t port) { (void)z; (void)port; return 0; }
static void z80_out(z80 *z, uint8_t port, uint8_t v) { (void)z; (void)port; (void)v; }

static void z80_hook(z80 *z)
{
	z->read_byte = z80_read;
	z->write_byte = z80_write;
	z->port_in = z80_in;
	z->port_out = z80_out;
	z->userdata = NULL;
	z->read_map = s_zmap;
}

static void ym_irq(int state) { s_ym_irq = state; }

// one line of sound: the Z80, the MCU, the chips
static void sound_line(void)
{
	if (s_has_z80) {
		const int cycles = rbx_line_cycles(&s_z80_frac, s_z80_clock, s_frame_mhz, s_lines);
		if (s_kind == K_JUMPKIDS)   // (INT held from the latch write until taken)
			z80_run(&s_z80, s_z80.cyc + (unsigned long)cycles, &s_z80_irq, 1);
		else
			z80_run(&s_z80, s_z80.cyc + (unsigned long)cycles, &s_ym_irq, 0);   // (YM IRQ: level; suprtrio none)
	}
	if (s_has_ym) {   // the YM's rate out, the OKI's samples held
		const uint32_t clocks = (uint32_t)rbx_line_cycles(&s_ym_frac, s_ym_clock, s_frame_mhz, s_lines);
		const int n = rbx_ym_advance(0, clocks, s_mix + s_mix_n, s_mix_cap - s_mix_n);
		const uint32_t yr = rbx_ym_rate(0);
		for (int i = 0; i < n; i++) {
			s_oki_acc += s_oki.rate;
			while (s_oki_acc >= yr) {
				s_oki_acc -= yr;
				s_oki_hold = oki_sample(&s_oki);
			}
			s_mix[s_mix_n + i] += s_oki_hold;
		}
		s_mix_n += n;
	} else {          // the OKI's rate out
		s_snd_frac += (uint64_t)s_oki.rate * 1000;
		const uint64_t den = (uint64_t)s_frame_mhz * (uint64_t)s_lines;
		while (s_snd_frac >= den) {
			s_snd_frac -= den;
			rbx_audio_put(oki_sample(&s_oki));
		}
	}
}

static void sound_frame_end(void)
{
	if (!s_has_ym)
		return;
	rbx_ym_async_take(0, s_mix, s_mix_n);
	for (int i = 0; i < s_mix_n; i++)
		rbx_audio_put(s_mix[i]);
	memset(s_mix, 0, sizeof(int32_t) * (size_t)s_mix_n);
	s_mix_n = 0;
}

// ---- main bus ----
static int vblank(void) { return ((s_line - (VIS_Y0 + VIS_H) + s_lines) % s_lines) < s_vbl_lines; }

static uint16_t system_r(void)
{
	if (s_kind == K_SUPRTRIO)   // bits 7-4: the protection latch (suprtrio_prot_latch_r)
		rbx_port_custom(s_port_sys, 0x00f0, (uint32_t)s_suprtrio_latch << 4);
	else
		rbx_port_custom(s_port_sys, 0x0008, vblank() ? 0x0008 : 0);
	return (uint16_t)rbx_port_read(s_port_sys);
}

static uint16_t controls_r(uint32_t a)   // tumblepopb_controls_r
{
	switch (a & 0xe) {
	case 0: return (uint16_t)rbx_port_read(s_port_players);
	case 2: return (uint16_t)rbx_port_read(s_port_dsw);
	case 8:
		if (s_bcstory_io && (m68k_get_reg(NULL, M68K_REG_PC) & 0xffffff) == 0x0560)   // bcstory_1a0_read (MAME pc(): past the operands)
			return 0x1a0;
		return system_r();
	}
	return 0;
}

static uint16_t main_read16(uint32_t a)
{
	a &= 0xffffff;
	switch (s_kind) {
	case K_SUPRTRIO:
		if (a == 0xe00000) return (uint16_t)rbx_port_read(s_port_players);
		if (a == 0xe40000) return system_r();
		if (a == 0xe80002) return (uint16_t)rbx_port_read(s_port_dsw);
		return 0;
	case K_FNCYWLD: case K_MAGIPUR:   // unico_base_map
		if (a == 0x100000 || a == 0x100002) return (uint16_t)(0xff00 | rbx_ym_read(0, (a >> 1) & 1));
		if (a == 0x100004) return (uint16_t)(0xff00 | oki_read(&s_oki));
		break;
	case K_SEMICOM:
		if (a >= 0x100000 && a < 0x100010) {   // semibase_unknown_r (machine().rand())
			s_rand = s_rand * 1103515245u + 12345u;
			return (uint16_t)(s_rand >> 16);
		}
		break;
	case K_TUMBLEB: case K_TUMBLEB2:
		if (a == 0x100000) return 0xffff;   // tumblepb_prot_r
		break;
	}
	if (a >= 0x180000 && a < 0x180010) return controls_r(a);
	return 0;
}
static uint8_t main_read8(uint32_t a)
{
	const uint16_t v = main_read16(a & ~1u);
	return (uint8_t)(a & 1 ? v : v >> 8);
}

static void tilebank_w(uint16_t v, uint16_t mask)
{
	if (s_kind == K_SUPRTRIO)
		s_tilebank = (uint16_t)((v & mask) << 14);   // suprtrio_tilebank_w
	else if (s_bankw == B_CHOKCHOK)
		s_tilebank = (uint16_t)((v & mask) << 1);
	else if (s_bankw == B_WLSTAR)
		s_tilebank = (uint16_t)(v & 0x4000);         // (write16smo: the whole word)
	else
		s_tilebank = (uint16_t)(v & mask);
}

static void main_write16m(uint32_t a, uint16_t v, uint16_t mask)
{
	a &= 0xffffff;
	if (s_kind == K_SUPRTRIO) {
		if (a == 0xe00000) tilebank_w(v, mask);
		else if (a == 0xec0000) { if (mask & 0xff) s_latch = (uint8_t)v; }   // semicom_soundcmd_w
		else if (a == 0xee0000) { if (mask & 0xff) s_suprtrio_latch = v & 0xf; }
		return;
	}
	if (a >= 0x300000 && a < 0x300010) {   // tumblepb_control_0_w
		uint16_t *w = &s_control0[(a >> 1) & 7];
		*w = (uint16_t)((*w & ~mask) | (v & mask));
		return;
	}
	switch (s_kind) {
	case K_TUMBLEB:
		if (a == 0x100000)   // tumblepb_oki_w
			oki_write(&s_oki, (uint8_t)(mask == 0xffff ? v : v >> 8));
		break;
	case K_TUMBLEB2: case K_PANGPANG:
		if (a == 0x100000)
			tumbleb2_soundmcu_w(v);
		break;
	case K_JUMPKIDS:
		if (a == 0x100000) {   // jumpkids_sound_w
			s_latch = (uint8_t)v;
			s_z80_irq = 1;
		}
		break;
	case K_FNCYWLD: case K_MAGIPUR:
		if ((a == 0x100000 || a == 0x100002) && (mask & 0xff)) rbx_ym_write(0, (a >> 1) & 1, (uint8_t)v);
		else if (a == 0x100004 && (mask & 0xff)) oki_write(&s_oki, (uint8_t)v);
		break;
	case K_SEMICOM:
		if (a == 0x100000) { if (mask & 0xff) s_latch = (uint8_t)v; }   // semicom_soundcmd_w
		else if (a == 0x100002) tilebank_w(v, mask);
		else if (a >= 0x140800 && a < 0x141000 && s_palext) {
			// init_chokchok: the palette's write handler over 0x140000-0x140fff (past the 0x800-byte RAM)
			uint16_t *w = &s_pal[(a - 0x140000) >> 1];
			*w = (uint16_t)((*w & ~mask) | (v & mask));
		}
		break;
	}
}
static void main_write16(uint32_t a, uint16_t v) { main_write16m(a, v, 0xffff); }
static void main_write8(uint32_t a, uint8_t v)
{
	const int sh = a & 1 ? 0 : 8;
	main_write16m(a & ~1u, (uint16_t)(v << sh), (uint16_t)(0xff << sh));
}

// ---- video ----
static int wrap(int v, int m) { v %= m; return v < 0 ? v + m : v; }

// MAME tilemap mapping of a screen coordinate to a tilemap coordinate (flip: the whole tilemap mirrored)
static int map_x(int sx, int scroll, int w, int flip)
{
	return !flip ? wrap(sx + scroll, w) : w - 1 - wrap(sx - (VIS_W - w + scroll), w);
}
static int map_y(int sy, int scroll, int h, int flip)
{
	return !flip ? wrap(sy + scroll, h) : h - 1 - wrap(sy - (SCREEN_H - h + scroll), h);
}

enum { L_FG8, L_BG1, L_BG2 };

// the tile at index idx of a layer (the get_*_tile_info callbacks): code, pen base
static void tile_info(const snap_t *s, int layer, int idx, uint32_t *code, int *base)
{
	const uint16_t *ram = layer == L_BG2 ? s->pf2 : s->pf1;
	int data, color;
	static const int base_n[3] = { 256, 256, 512 }, base_f[3] = { 0x400, 0x200, 0x400 };
	switch (s_tiles) {
	case T_FNCYWLD:
		data = ram[(idx * 2) & 0xfff];
		color = ram[(idx * 2 + 1) & 0xfff] & 0x1f;
		*code = (uint32_t)(data & 0x1fff);
		*base = base_f[layer] + color * 16;
		return;
	case T_PANGPANG: {
		const int attr = ram[(idx * 2) & 0xfff];
		data = ram[(idx * 2 + 1) & 0xfff];
		if (layer == L_FG8) { *code = (uint32_t)(data & 0x1fff); color = (attr >> 12) & 0x1f; }
		else if (layer == L_BG1) { *code = (uint32_t)(data & 0x1fff); color = (attr >> 12) & 0xf; }
		else { *code = (uint32_t)((data & 0xfff) + 0x1000); color = (attr >> 12) & 0xf; }
		*base = base_n[layer] + color * 16;
		return;
	}
	default:
		data = ram[idx & 0xfff];
		*code = (uint32_t)((data & 0x0fff) | (layer == L_FG8 ? s->tilebank : s->tilebank >> 2));
		*base = base_n[layer] + (data >> 12) * 16;
		return;
	}
}

// one tilemap over the visible area: pens equal to tp are transparent (opaque: every pen drawn)
static void draw_layer(uint16_t *bm, int pitch, const snap_t *s, int layer, int scrollx, int scrolly, int flip,
                       int opaque, int tp)
{
	const rbx_gfx *g = layer == L_FG8 ? &s_gfx8 : &s_gfx16;
	const int ts = layer == L_FG8 ? 8 : 16, sh = layer == L_FG8 ? 3 : 4;
	const int w = layer == L_FG8 ? 512 : 1024, h = layer == L_FG8 ? (s_sdfight ? 512 : 256) : 512;
	for (int y = 0; y < VIS_H; y++) {
		uint16_t *d = bm + y * pitch;
		const int ty = map_y(y + VIS_Y0, scrolly, h, flip);
		const int row = ty >> sh, py = ty & (ts - 1);
		int x = 0;
		while (x < VIS_W) {
			const int tx = map_x(x, scrollx, w, flip);
			const int col = tx >> sh, px = tx & (ts - 1);
			const int idx = layer == L_FG8 ? row * 64 + col                                       // TILEMAP_SCAN_ROWS
			                               : (col & 0x1f) + ((row & 0x1f) << 5) + ((col & 0x60) << 5);   // tumblep_scan
			uint32_t code;
			int base;
			tile_info(s, layer, idx, &code, &base);
			const uint8_t *src = g->pix + (size_t)(code % g->count) * (size_t)(ts * ts) + py * ts;
			int n = flip ? px + 1 : ts - px;
			if (n > VIS_W - x)
				n = VIS_W - x;
			for (int k = 0; k < n; k++) {
				const uint8_t pen = src[flip ? px - k : px + k];
				if (opaque || pen != tp)
					d[x + k] = (uint16_t)(base + pen);
			}
			x += n;
		}
	}
}

static uint32_t pal4(int v) { return (uint32_t)(v * 17); }
static uint32_t pal5(int v) { return (uint32_t)(v << 3 | v >> 2); }

static void tb_render(void)
{
	const snap_t *s = s_snap;
	const int npal = s_palfmt == P_RGB444 ? 0x800 : 0x400;
	for (int i = 0; i < npal; i++) {
		const uint16_t p = s->pal[i];
		uint32_t r, g, b;
		if (s_palfmt == P_BGR444) { r = pal4(p & 15); g = pal4((p >> 4) & 15); b = pal4((p >> 8) & 15); }
		else if (s_palfmt == P_RGB444) { r = pal4((p >> 8) & 15); g = pal4((p >> 4) & 15); b = pal4(p & 15); }
		else { r = pal5(p & 31); g = pal5((p >> 5) & 31); b = pal5((p >> 10) & 31); }
		rbx_video.palette[i] = r << 16 | g << 8 | b;
	}
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	const uint16_t *c = s->control0;
	decospr_t spr = s_spr;
	if (s_kind == K_SUPRTRIO) {   // screen_update_suprtrio
		draw_layer(bm, pitch, s, L_BG2, -(int)c[3] - 2, -(int)c[4], 0, 1, 0);
		draw_layer(bm, pitch, s, L_BG1, -(int)c[1] - 6, -(int)c[2], 0, 0, 0);
		decospr_draw(&spr, s->spriteram, 0x800, bm, NULL, pitch, VIS_W, VIS_H, VIS_Y0, 1, s->frame);
		return;
	}
	const int flip = (c[0] >> 7) & 1;
	int pf1x, pf1y = 0, pf2x, pf2y = 0;
	switch (s_offs) {
	case O_ALT: pf1x = -5; pf2x = -1; pf2y = 2; break;
	case O_BCSTORY: pf2x = flip ? 1 : 8; pf1x = flip ? -3 : 8; break;
	case O_SEMIBASE: pf2x = -1; pf1x = -2; break;
	case O_SDFIGHT: pf2x = -1; pf1x = -5; pf1y = -16; break;
	default: pf2x = flip ? 1 : -1; pf1x = flip ? -3 : -5; break;
	}
	// tumbleb_draw_common: pf2 opaque, then the 8x8 or the 16x16 front layer, then the sprites
	const int tp = s_tiles == T_FNCYWLD ? 15 : 0;
	draw_layer(bm, pitch, s, L_BG2, c[3] + pf2x, c[4] + pf2y, flip, 1, 0);
	draw_layer(bm, pitch, s, (c[6] & 0x80) ? L_FG8 : L_BG1, c[1] + pf1x, c[2] + pf1y, flip, 0, tp);
	spr.flip_screen = flip;
	decospr_draw(&spr, s->spriteram, s_kind == K_SEMICOM ? 0x800 : 0x400, bm, NULL, pitch, VIS_W, VIS_H, VIS_Y0, 1,
	             s->frame);
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap->pal, s_pal, sizeof(s_pal));
	memcpy(s_snap->spriteram, s_spriteram, sizeof(s_spriteram));
	memcpy(s_snap->pf1, s_pf1, sizeof(s_pf1));
	memcpy(s_snap->pf2, s_pf2, sizeof(s_pf2));
	memcpy(s_snap->control0, s_control0, sizeof(s_control0));
	s_snap->tilebank = s_tilebank;
	s_snap->frame = s_frame;
	rbx_video_frame_done();
}

// ---- graphics decoding ----
static void gfx_rearrange(uint8_t *rom, uint32_t len)   // tumblepb_gfx_rearrange
{
	for (uint32_t i = 0; i < len; i++)   // (the gfx data is in the wrong order)
		if ((i & 0x20) == 0 && i + 0x20 < len) {
			const uint8_t t = rom[i];
			rom[i] = rom[i + 0x20];
			rom[i + 0x20] = t;
		}
	for (uint32_t i = 0; i < len / 2; i++) {   // (low / high halves swapped)
		const uint8_t t = rom[i];
		rom[i] = rom[i + len / 2];
		rom[i + len / 2] = t;
	}
}

// a permutation of the 16-bit words of a region (suprtrio_decrypt_code / _gfx)
static int permute_words(uint8_t *rom, uint32_t words, int gfx)
{
	uint8_t *buf = (uint8_t *)malloc((size_t)words * 2);
	if (!buf)
		return -1;
	memcpy(buf, rom, (size_t)words * 2);
	for (uint32_t i = 0; i < words; i++) {
		uint32_t j;
		if (gfx) {
			j = i ^ 0x02;
			if (i & 1) j ^= 0x04;
		} else {
			j = i ^ 0x06;
			if ((i & 1) == 0) j ^= 0x02;
			if ((i & 3) == 0) j ^= 0x08;
		}
		rom[i * 2] = buf[j * 2];
		rom[i * 2 + 1] = buf[j * 2 + 1];
	}
	free(buf);
	return 0;
}

static int decode_gfx(uint8_t *tiles, uint32_t tsize, uint8_t *spr, uint32_t ssize)
{
	// tcharlayout / tlayout (RGN_FRAC(1,2): planes half + 8, half, 8, 0; rows of 16 bits)
	static uint32_t x8[8], y8[8], x16[16], y16[16], sx[16], sy[16];
	for (int i = 0; i < 8; i++) {
		x8[i] = (uint32_t)i;
		y8[i] = (uint32_t)(i * 16);
		x16[i] = (uint32_t)(256 + i);
		x16[8 + i] = (uint32_t)i;
		sx[i] = (uint32_t)i;
		sx[8 + i] = (uint32_t)(16 * 8 + i);
	}
	static const uint32_t sy8[8] = { 1 * 8, 0 * 8, 2 * 8, 3 * 8, 5 * 8, 4 * 8, 6 * 8, 7 * 8 };
	for (int i = 0; i < 16; i++) {
		y16[i] = (uint32_t)(i * 16);
		sy[i] = i < 8 ? sy8[i] : (uint32_t)(i * 8);
	}
	const uint32_t th = tsize * 4, sh = ssize * 4;
	const uint32_t pt[4] = { th + 8, th, 8, 0 }, ps[4] = { sh + 8, sh, 8, 0 };
	const rbx_gfx_layout l8 = { 8, 8, 4, pt, x8, y8, 16 * 8 };
	const rbx_gfx_layout l16 = { 16, 16, 4, pt, x16, y16, 64 * 8 };
	const rbx_gfx_layout ls = { 16, 16, 4, ps, x16, y16, 64 * 8 };
	if (s_kind == K_SUPRTRIO) {   // suprtrio_tlayout (RGN_FRAC(1,4): planes 2/4, 0/4, 3/4, 1/4)
		const uint32_t q = tsize * 2;
		const uint32_t pq[4] = { 2 * q, 0, 3 * q, q };
		const rbx_gfx_layout lt = { 16, 16, 4, pq, sx, sy, 32 * 8 };
		if (rbx_gfx_decode(&s_gfx16, &lt, tiles, tsize / 4))
			return -1;
	} else if (rbx_gfx_decode(&s_gfx16, &l16, tiles, tsize / 2))
		return -1;
	return rbx_gfx_decode(&s_gfx8, &l8, tiles, tsize / 2) || rbx_gfx_decode(&s_gfxspr, &ls, spr, ssize / 2);
}

// ---- init / frame ----
static void copy_protdata(void)   // MACHINE_RESET htchctch / init_htchctch: "user1" into the main RAM
{
	for (uint32_t i = 0; i + 1 < s_protdata_size && i / 2 < 0x8000; i += 2)
		s_ram[i / 2] = (uint16_t)(s_protdata[i] << 8 | s_protdata[i + 1]);
}

static void tb_reset(void)
{
	memset(s_ram, 0, sizeof(s_ram));
	memset(s_ram2, 0, sizeof(s_ram2));
	memset(s_ram3, 0, sizeof(s_ram3));
	memset(s_pal, 0, sizeof(s_pal));
	memset(s_spriteram, 0, sizeof(s_spriteram));
	memset(s_pf1, 0, sizeof(s_pf1));
	memset(s_pf2, 0, sizeof(s_pf2));
	memset(s_control0, 0, sizeof(s_control0));
	s_tilebank = 0;
	s_suprtrio_latch = 0;
	s_music_command = s_music_bank = s_music_is_playing = 0;
	s_prot_offset = 0;
	s_rand = 0;
	if (s_kind == K_MAGIPUR)   // init_magipur: the vectors copied into the RAM at 0
		memcpy(s_ram, s_rom, 0x80);   // (both in the bus's word order)
	if (s_kind == K_SEMICOM && s_protdata)
		copy_protdata();
	memcpy(s_okibuf, s_okirom, s_oki_size);
	s_okibank = -1;
	oki_reset(&s_oki);
	s_oki_acc = 0;
	s_oki_hold = 0;
	s_mix_n = 0;
	s_latch = 0;
	s_z80_irq = s_ym_irq = 0;
	memset(s_zram, 0, sizeof(s_zram));
	if (s_has_z80) {
		z80_init(&s_z80);
		z80_hook(&s_z80);
		s_z80.int_data = 0xff;
	}
	if (s_has_ym)
		rbx_ym_reset(0);
	if (s_mcu_on)
		mcs51_reset(&s_mcu);
	m68k_bus_irq_reset();
	m68k_pulse_reset();
	s_line = 0;
	s_frame = 0;
	s_cpu_frac = s_z80_frac = s_ym_frac = s_snd_frac = s_mcu_frac = 0;
}

static void tb_exit(void);

static int is(const char *set, const char *const *list)
{
	for (; *list; list++)
		if (!strcmp(set, *list))
			return 1;
	return 0;
}

static int tb_init(const rbx_game *g)
{
	const char *m = g->machine, *set = g->set;
	uint32_t tsize, ssize, zsize;
	// the machine configuration
	s_mcu_on = s_bcstory_io = s_sdfight = s_palext = 0;
	s_bankw = B_BCSTORY;
	s_tiles = T_NORMAL;
	s_offs = O_TUMBLEPB;
	s_palfmt = P_BGR444;
	s_has_ym = s_has_z80 = 0;
	s_protbase = 0;
	s_lines = SCREEN_H;
	if (!strcmp(m, "tumblepb")) { s_kind = K_TUMBLEB; s_cpu_clock = 14000000; s_frame_mhz = 58000; s_vbl_lines = 8; }
	else if (!strcmp(m, "tumbleb2")) { s_kind = K_TUMBLEB2; s_cpu_clock = 14000000; s_frame_mhz = 58000; s_vbl_lines = 8; }
	else if (!strcmp(m, "pangpang")) {
		s_kind = K_PANGPANG; s_tiles = T_PANGPANG; s_cpu_clock = 14000000; s_frame_mhz = 58000; s_vbl_lines = 23;
	} else if (!strcmp(m, "jumpkids")) {
		s_kind = K_JUMPKIDS; s_cpu_clock = 12000000; s_frame_mhz = 60000; s_vbl_lines = 9;
		s_has_z80 = 1; s_z80_clock = 4000000;
	} else if (!strcmp(m, "suprtrio")) {   // set_raw(14 MHz / 2, 442, 0, 320, 274, 8, 248)
		s_kind = K_SUPRTRIO; s_palfmt = P_BGR555; s_cpu_clock = 14000000; s_frame_mhz = 57798; s_lines = 274;
		s_vbl_lines = 274 - VIS_H; s_has_z80 = 1; s_z80_clock = 4000000;
	} else if (!strcmp(m, "fncywld") || !strcmp(m, "magipur")) {
		s_kind = !strcmp(m, "magipur") ? K_MAGIPUR : K_FNCYWLD; s_tiles = T_FNCYWLD; s_palfmt = P_RGB444;
		s_cpu_clock = 12000000; s_frame_mhz = 60000; s_vbl_lines = 9; s_has_ym = 1; s_ym_clock = 4000000;
	} else {   // htchctch and its derivatives (vblank 2400 us)
		s_kind = K_SEMICOM; s_palfmt = P_BGR555; s_cpu_clock = 15000000; s_frame_mhz = 60000; s_vbl_lines = 37;
		s_has_z80 = s_has_ym = 1; s_z80_clock = 3750000; s_ym_clock = 3750000;
		if (!strcmp(m, "htchctch_mcu") || !strcmp(m, "chokchok") || !strcmp(m, "cookbib_mcu")) s_mcu_on = 1;
		if (!strcmp(m, "cookbib") || !strcmp(m, "cookbib_mcu") || !strcmp(m, "metlsavr")) s_offs = O_ALT;
		else if (!strcmp(m, "bcstory")) s_offs = O_BCSTORY;
		else if (!strcmp(m, "semibase")) s_offs = O_SEMIBASE;
		else if (!strcmp(m, "sdfight")) { s_offs = O_SDFIGHT; s_sdfight = 1; }
		if (!strcmp(m, "chokchok") || !strcmp(m, "metlsavr")) s_palfmt = P_BGR444;
		if (!strcmp(m, "metlsavr")) s_ym_clock = 3579545;
		// the init functions
		static const char *const chok[] = { "metlsavr", "magicbal", "chokchok", "carket", NULL };
		static const char *const wl[] = { "wlstar", "wondl96", NULL };
		static const char *const bc[] = { "sdfight", "bcstry", "bcstrya", "semibase", NULL };
		if (is(set, chok)) s_bankw = B_CHOKCHOK;
		else if (is(set, wl)) s_bankw = B_WLSTAR;
		s_palext = is(set, chok) && strcmp(set, "carket");   // (init_chokchok, not init_carket)
		if (is(set, bc)) s_bcstory_io = 1;
		if (!strcmp(set, "wondl96") || !strcmp(set, "dquizgo")) s_protbase = 0x0200;
	}
	s_rom = rbx_region("maincpu", &s_rom_size);
	uint8_t *tiles = rbx_region("tilegfx", &tsize), *spr = rbx_region("sprgfx", &ssize);
	s_okirom = rbx_region("oki", &s_oki_size);
	s_oki2rom = rbx_region("oki2", &s_oki2_size);
	s_zrom = s_has_z80 ? rbx_region("audiocpu", &zsize) : NULL;
	s_zrom_size = s_zrom ? zsize : 0;
	s_protdata = rbx_region("user1", &s_protdata_size);
	if (!s_rom || s_rom_size < 0x80000 || !tiles || !spr || !s_okirom || (s_has_z80 && !s_zrom)) {
		rbx_fail("%s: ROM regions missing", set);
		return -1;
	}
	if (s_kind == K_SUPRTRIO) {   // init_suprtrio
		if (permute_words(s_rom, 0x40000, 0) || permute_words(tiles, tsize < 0x100000 ? tsize / 2 : 0x80000, 1)) {
			rbx_fail("%s: out of memory", set);
			return -1;
		}
	} else
		gfx_rearrange(tiles, tsize);
	if (decode_gfx(tiles, tsize, spr, ssize)) {
		rbx_fail("%s: out of memory (graphics)", set);
		tb_exit();
		return -1;
	}
	memset(&s_spr, 0, sizeof(s_spr));
	s_spr.gfx = &s_gfxspr;
	s_spr.bootleg = 1;
	s_spr.transpen = s_tiles == T_FNCYWLD ? 15 : 0;
	s_snap = (snap_t *)calloc(1, sizeof(*s_snap));
	s_okibuf = (uint8_t *)malloc(s_oki_size);
	if (!s_snap || !s_okibuf) {
		rbx_fail("%s: out of memory", set);
		tb_exit();
		return -1;
	}
	memcpy(s_okibuf, s_okirom, s_oki_size);
	// sound
	uint32_t oki_clock = 1000000;
	int oki_gain = 179;   // (0.70)
	if (s_kind == K_TUMBLEB || s_kind == K_TUMBLEB2) oki_clock = 800000;
	else if (s_kind == K_SUPRTRIO) oki_gain = 128;
	else if (s_kind == K_FNCYWLD || s_kind == K_MAGIPUR) oki_gain = 256;
	else if (s_kind == K_SEMICOM) { oki_clock = 4096000 / 4; oki_gain = 256; }
	oki_init(&s_oki, oki_clock, 1, s_okibuf, s_oki_size, oki_gain);
	int rc = 0;
	if (s_has_ym) {   // YM2151 0.10 on each output, one speaker
		rbx_ym_config yc;
		memset(&yc, 0, sizeof(yc));
		yc.type = RBX_YM2151;
		yc.clock = s_ym_clock;
		yc.gain[0] = yc.gain[1] = 26;
		yc.irq = s_kind == K_SEMICOM ? ym_irq : NULL;
		rc = rbx_ym_init(0, &yc) || rbx_ym_set_async(0, 1);
		if (!rc) {
			s_mix_cap = (int)(rbx_ym_rate(0) / 50) + 256;
			s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
			rc = !s_mix || rbx_audio_setup(rbx_ym_rate(0));
		}
	} else
		rc = rbx_audio_setup(s_oki.rate);
	if (rc || rbx_video_setup(VIS_W, VIS_H, s_palfmt == P_RGB444 ? 0x800 : 0x400)) {
		rbx_fail("%s: out of memory", set);
		tb_exit();
		return -1;
	}
	memset(s_zmap, 0, sizeof(s_zmap));
	if (s_has_z80) {
		s_zrom_mapped = s_kind == K_JUMPKIDS ? 0x1000 : 0xd000;
		for (uint32_t p = 0; p * 256 < s_zrom_mapped && p * 256 + 256 <= s_zrom_size; p++)
			s_zmap[p] = s_zrom + p * 256;
	}
	if (s_mcu_on) {
		uint32_t psize;
		const uint8_t *prog = rbx_region("protection", &psize);
		if (!prog) {
			rbx_fail("%s: ROM regions missing", set);
			tb_exit();
			return -1;
		}
		mcs51_init(&s_mcu, prog, psize < 0x2000 ? psize : 0x2000);   // (program width 13)
		s_mcu.is_8052 = 1;
		s_mcu.port_out = prot_port_w;
	}
	s_port_players = rbx_port_find("PLAYERS");
	s_port_dsw = rbx_port_find("DSW");
	s_port_sys = rbx_port_find("SYSTEM");
	// main CPU
	m68k_bus_swap16(s_rom, s_rom_size);
	m68k_init();
	m68k_set_cpu_type(M68K_CPU_TYPE_68000);
	m68k_bus_reset_map();
	switch (s_kind) {
	case K_SUPRTRIO:   // suprtrio_main_map
		m68k_bus_map(0, 0x7ffff, s_rom, 0x80000, 1, 0);
		m68k_bus_map(0x700000, 0x700fff, s_spriteram, 0x1000, 1, 1);
		m68k_bus_map(0xa00000, 0xa0000f, s_control0, sizeof(s_control0), 1, 1);
		m68k_bus_map(0xa20000, 0xa20fff, s_pf1, 0x1000, 1, 1);
		m68k_bus_map(0xa22000, 0xa22fff, s_pf2, 0x1000, 1, 1);
		m68k_bus_map(0xcf0000, 0xcf05ff, s_pal, 0x600, 1, 1);
		m68k_bus_map(0xf00000, 0xf07fff, s_ram, 0x8000, 1, 1);
		break;
	case K_FNCYWLD: case K_MAGIPUR:   // fncywld_main_map / magipur_main_map + unico_base_map
		if (s_kind == K_FNCYWLD) {
			m68k_bus_map(0, 0xfffff, s_rom, s_rom_size, 1, 0);
			m68k_bus_map(0xff0000, 0xffffff, s_ram, 0x10000, 1, 1);
		} else {
			m68k_bus_map(0, 0xffff, s_ram, 0x10000, 1, 1);
			m68k_bus_map(0xf00000, 0xffffff, s_rom, s_rom_size, 1, 0);
		}
		m68k_bus_map(0x140000, 0x140fff, s_pal, 0x1000, 1, 1);
		m68k_bus_map(0x160000, 0x1607ff, s_spriteram, 0x800, 1, 1);
		m68k_bus_map(0x1a0000, 0x1a07ff, s_ram2, 0x800, 1, 1);
		m68k_bus_map(0x320000, 0x321fff, s_pf1, 0x2000, 1, 1);
		m68k_bus_map(0x322000, 0x323fff, s_pf2, 0x2000, 1, 1);
		break;
	case K_SEMICOM:   // htchctch_main_map
		m68k_bus_map(0, 0xfffff, s_rom, s_rom_size, 1, 0);
		m68k_bus_map(0x120000, 0x123fff, s_ram, 0x4000, 1, 1);
		m68k_bus_map(0x140000, 0x1407ff, s_pal, 0x800, 1, 1);
		m68k_bus_map(0x160000, 0x160fff, s_spriteram, 0x1000, 1, 1);
		m68k_bus_map(0x1a0000, 0x1a0fff, s_ram2, 0x1000, 1, 1);
		m68k_bus_map(0x320000, 0x321fff, s_pf1, 0x2000, 0, 1);
		m68k_bus_map(0x322000, 0x322fff, s_pf2, 0x1000, 0, 1);
		m68k_bus_map(0x341000, 0x342fff, s_ram3, 0x2000, 1, 1);
		break;
	default:   // tumblepopb_main_map / jumpkids_main_map / pangpang_main_map
		m68k_bus_map(0, 0x7ffff, s_rom, 0x80000, 1, 0);
		m68k_bus_map(0x120000, 0x123fff, s_ram, 0x4000, 1, 1);
		m68k_bus_map(0x140000, 0x1407ff, s_pal, 0x800, 1, 1);
		m68k_bus_map(0x160000, 0x1607ff, s_spriteram, 0x800, 1, 1);
		m68k_bus_map(0x1a0000, 0x1a07ff, s_ram2, 0x800, 1, 1);
		if (s_kind == K_PANGPANG) {
			m68k_bus_map(0x320000, 0x321fff, s_pf1, 0x2000, 1, 1);
			m68k_bus_map(0x340000, 0x341fff, s_pf2, 0x2000, 1, 1);
		} else {
			m68k_bus_map(0x320000, 0x320fff, s_pf1, 0x1000, 0, 1);
			m68k_bus_map(0x322000, 0x322fff, s_pf2, 0x1000, 0, 1);
		}
		break;
	}
	m68k_bus.read8 = main_read8;
	m68k_bus.read16 = main_read16;
	m68k_bus.write8 = main_write8;
	m68k_bus.write16 = main_write16;
	tb_reset();
	return 0;
}

static void tb_exit(void)
{
	if (s_has_ym)
		rbx_ym_exit(0);
	free(s_mix);
	s_mix = NULL;
	rbx_gfx_free(&s_gfx8);
	rbx_gfx_free(&s_gfx16);
	rbx_gfx_free(&s_gfxspr);
	free(s_snap);
	s_snap = NULL;
	free(s_okibuf);
	s_okibuf = NULL;
}

static void tb_frame(void)
{
	rbx_audio.frame_mhz = s_frame_mhz;
	for (int line = 0; line < s_lines; line++) {
		s_line = line;
		if (line == VIS_Y0 + VIS_H) {   // irq6_line_hold at vblank (tumbleb2_interrupt: and the music)
			vblank_capture();
			m68k_bus_hold_irq(6);
			if (s_kind == K_TUMBLEB2 || s_kind == K_PANGPANG)
				tumbleb2_playmusic();
		}
		m68k_execute(rbx_line_cycles(&s_cpu_frac, s_cpu_clock, s_frame_mhz, s_lines));
		if (s_mcu_on)   // (15 MHz: machine cycles of 12 clocks)
			mcs51_execute(&s_mcu, rbx_line_cycles(&s_mcu_frac, 15000000 / 12, s_frame_mhz, s_lines));
		sound_line();
	}
	sound_frame_end();
	s_frame++;
}

static int tb_state(void *ctx, int load)
{
	const uint32_t csize = m68k_context_size();
	uint8_t *cpu = (uint8_t *)malloc(csize);
	if (!cpu)
		return -1;
	if (!load)
		m68k_get_context(cpu);
	uint32_t irq[2] = { m68k_bus_irq_pending(), m68k_bus_cur->irq_asserted };
	uint64_t fr[5] = { s_cpu_frac, s_z80_frac, s_ym_frac, s_snd_frac, s_mcu_frac };
	int32_t misc[16] = { (int32_t)s_frame, s_tilebank, s_suprtrio_latch, s_music_command, s_music_bank,
	                     s_music_is_playing, s_prot_offset, (int32_t)s_rand, s_okibank, s_latch, s_z80_irq, s_ym_irq,
	                     (int32_t)s_oki_acc, s_oki_hold, 0, 0 };
	z80 z = s_z80;
	int rc = 0;
	rc |= rbx_state_io(ctx, "m68k", cpu, csize);
	rc |= rbx_state_io(ctx, "irq", irq, sizeof(irq));
	rc |= rbx_state_io(ctx, "ram", s_ram, sizeof(s_ram));
	rc |= rbx_state_io(ctx, "ram2", s_ram2, sizeof(s_ram2));
	rc |= rbx_state_io(ctx, "ram3", s_ram3, sizeof(s_ram3));
	rc |= rbx_state_io(ctx, "pal", s_pal, sizeof(s_pal));
	rc |= rbx_state_io(ctx, "spr", s_spriteram, sizeof(s_spriteram));
	rc |= rbx_state_io(ctx, "pf1", s_pf1, sizeof(s_pf1));
	rc |= rbx_state_io(ctx, "pf2", s_pf2, sizeof(s_pf2));
	rc |= rbx_state_io(ctx, "ctl", s_control0, sizeof(s_control0));
	rc |= rbx_state_io(ctx, "oki", s_oki.v, sizeof(s_oki.v) + sizeof(int));
	if (s_has_z80) {
		rc |= rbx_state_io(ctx, "z80", &z, sizeof(z));
		rc |= rbx_state_io(ctx, "zram", s_zram, sizeof(s_zram));
	}
	if (s_has_ym)
		rc |= rbx_ym_state(0, ctx, load);
	if (s_mcu_on) {
		rc |= rbx_state_io(ctx, "mcu", &s_mcu.st, sizeof(s_mcu.st));
		rc |= rbx_state_io(ctx, "mcu2", &s_mcu.st2, sizeof(s_mcu.st2));
	}
	rc |= rbx_state_io(ctx, "frac", fr, sizeof(fr));
	rc |= rbx_state_io(ctx, "misc", misc, sizeof(misc));
	if (!rc && load) {
		m68k_set_context(cpu);
		m68k_bus_set_irq_pending(irq[0]);
		m68k_bus_cur->irq_asserted = irq[1];
		m68k_bus_update_irq();
		if (s_has_z80) {
			z80_hook(&z);
			s_z80 = z;
		}
		s_cpu_frac = fr[0];
		s_z80_frac = fr[1];
		s_ym_frac = fr[2];
		s_snd_frac = fr[3];
		s_mcu_frac = fr[4];
		s_frame = (uint32_t)misc[0];
		s_tilebank = (uint16_t)misc[1];
		s_suprtrio_latch = (uint8_t)misc[2];
		s_music_command = misc[3];
		s_music_bank = misc[4];
		s_music_is_playing = misc[5];
		s_prot_offset = (uint8_t)misc[6];
		s_rand = (uint32_t)misc[7];
		memcpy(s_okibuf, s_okirom, s_oki_size);
		oki_bank_apply(misc[8]);
		s_latch = (uint8_t)misc[9];
		s_z80_irq = misc[10];
		s_ym_irq = misc[11];
		s_oki_acc = (uint32_t)misc[12];
		s_oki_hold = misc[13];
	}
	free(cpu);
	return rc ? -1 : 0;
}

static int tb_save(void *ctx) { return tb_state(ctx, 0); }
static int tb_load(void *ctx) { return tb_state(ctx, 1); }
static int tb_supported(const char *machine)
{
	static const char *const ok[] = { "tumblepb", "tumbleb2", "pangpang", "jumpkids", "suprtrio", "fncywld", "magipur",
	                                  "metlsavr", "chokchok", "cookbib_mcu", "htchctch_mcu", "htchctch", "cookbib",
	                                  "sdfight", "bcstory", "semibase", NULL };
	return is(machine, ok);
}

const rbx_driver rbx_drv_tumbleb = { "DECO Tumble Pop bootlegs", tb_init, tb_exit, tb_reset, tb_frame, tb_save, tb_load,
                                     tb_supported, tb_render, NULL };
