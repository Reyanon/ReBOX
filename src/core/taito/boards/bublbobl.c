/*  Bubble Bobble / Tokio hardware, ported from MAME src/mame/taito/bublbobl.cpp, bublbobl_m.cpp and bublbobl_v.cpp
    (BSD-3-Clause, copyright-holders: Chris Moore, Nicola Salmoria). Memory maps, machine configurations, the MC6801U4
    and 68705 MCU interfaces, the bootleg IC43 PAL and the sprite-only video follow MAME. MCUs: cpu/m6800 (MC6801U4),
    cpu/m6805 + machine/taito68705 (Tokio, bub68705). Sound: YM2203 + YM3526 (rbx_ym).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/z80/z80.h"
#include "../../cpu/m6800/m6801.h"
#include "../../cpu/m6805/m68705.h"
#include "../machine/taito68705.h"
#include "../../sound/rbx_ym.h"

#include <stdlib.h>
#include <string.h>

enum { BB_6801, BB_BOOT, BB_68705, TK_MCU, TK_BOOT, TK_PROTO };

typedef struct {
	const char *name;
	int kind;
} machine_t;

static const machine_t s_machines[] = {
	{ "bublbobl", BB_6801 },
	{ "boblbobl", BB_BOOT },
	{ "bub68705", BB_68705 },
	{ "tokio", TK_MCU },
	{ "tokiob", TK_BOOT },
	{ "bublboblp", TK_PROTO },
};

// MAME set_raw(24 MHz / 4, 384, 0, 256, 264, 16, 240): the CPUs run at the pixel clock (6 MHz = 384 clocks a line)
#define LINES 264
#define VIS_Y0 16
#define VIS_Y1 239
#define SCREEN_W 256
#define MAIN_LINE 384
#define AUDIO_LINE 192      // 3 MHz
#define YM_LINE 192         // 3 MHz
#define M6801_LINE 64       // 4 MHz / 4
#define M68705P3_LINE 64    // 4 MHz / 4
#define TOKIO_MCU_LINE 48   // 3 MHz / 4
#define SLICES 2

typedef struct {
	z80 z;
	int irq;              // IRQ line held until acknowledged (HOLD_LINE / MAME acknowledge callback)
	int reset;
	unsigned long start;
	int len;
} zcpu;

static const machine_t *s_m;
static int s_tokio;
static uint8_t *s_main_rom, *s_sub_rom, *s_audio_rom;
static uint32_t s_main_size, s_sub_size, s_audio_size;
static uint8_t s_vram[0x1d00], s_objram[0x300], s_share[0x1800], s_pal[0x200], s_mcuram[0x400], s_audio_ram[0x1000];
static zcpu s_main, s_sub, s_audio;
static int s_bank, s_video_enable, s_flip;
static uint8_t s_main_to_sound, s_sound_to_main;
static int s_m2s_pending, s_s2m_pending, s_nmi_enable, s_nmi_out, s_sreset_old;
static int s_ym_irq[2], s_ym_any;   // s_ym_any = either YM IRQ (the sound CPU's INT level)
static uint8_t s_ic43_a, s_ic43_b;
static uint32_t s_rand = 1;
// MC6801U4
static m6801 s_m6801;
static uint8_t s_port1_out, s_port2_out, s_port3_out, s_port4_out, s_port3_in;
// 68705 (bub68705) / Taito 68705 (tokio)
static m68705_t s_m68705;
static uint8_t s_pa_out, s_pb_out, s_latch;
static uint16_t s_address;
static taito68705_t s_tmcu;
// video
static uint8_t *s_tiles, *s_opaque;   // 16384 8x8 tiles, one byte per pixel
static uint8_t s_snap_vram[0x1d00], s_snap_obj[0x300], s_snap_pal[0x200];
static int s_snap_enable, s_snap_flip;
static const uint8_t *s_prom;
// ports
static int s_port_in0, s_port_in1, s_port_in2, s_port_dsw0, s_port_dsw1;
// sound
static int32_t *s_mix, *s_mix2;
static int s_mix_cap, s_mix_n, s_mix2_n;
static uint32_t s_ym_done, s_ym_slice_start, s_ym_slice_len;
static int s_has_opl;

static uint8_t port8(int p) { return (uint8_t)rbx_port_read(p); }
static uint32_t rnd(void) { s_rand = s_rand * 1103515245u + 12345u; return s_rand >> 16; }

// ---- sound ----
static void ym_run_to(uint32_t pos)
{
	if (pos <= s_ym_done)
		return;
	s_mix_n += rbx_ym_advance(0, pos - s_ym_done, s_mix + s_mix_n, s_mix_cap - s_mix_n);
	if (s_has_opl)
		s_mix2_n += rbx_ym_advance(1, pos - s_ym_done, s_mix2 + s_mix2_n, s_mix_cap - s_mix2_n);
	s_ym_done = pos;
}

static void ym_sync(void)
{
	zcpu *c = &s_audio;
	uint32_t done = (uint32_t)(c->z.cyc - c->start);
	if ((int)done > c->len)
		done = (uint32_t)c->len;
	ym_run_to(s_ym_slice_start + (c->len ? rbx_muldiv(s_ym_slice_len, done, (uint32_t)c->len) : 0));
}

static void ym_irq0(int state) { s_ym_irq[0] = state; s_ym_any = s_ym_irq[0] || s_ym_irq[1]; }
static void ym_irq1(int state) { s_ym_irq[1] = state; s_ym_any = s_ym_irq[0] || s_ym_irq[1]; }

// sound NMI: INPUT_MERGER_ALL_HIGH of (NMI enable, main-to-sound latch pending)
static void update_sound_nmi(void)
{
	int out = s_nmi_enable && s_m2s_pending;
	if (out && !s_nmi_out)
		s_audio.z.nmi_pending = 1;
	s_nmi_out = out;
}

static void audio_reset_cpu(void);

static void common_sreset(int state)
{
	if (state && !s_sreset_old) {
		ym_sync();
		rbx_ym_reset(0);
		if (s_has_opl)
			rbx_ym_reset(1);
		s_ym_irq[0] = s_ym_irq[1] = s_ym_any = 0;
		s_s2m_pending = 0;
		s_nmi_enable = 0;
		update_sound_nmi();
		audio_reset_cpu();
	}
	s_audio.reset = state;
	s_sreset_old = state;
}

static uint8_t sound_semaphores_r(void) { return (uint8_t)(0xfc | (s_m2s_pending ? 2 : 0) | (s_s2m_pending ? 1 : 0)); }

// ---- MC6801U4 (bublbobl) ----
static uint8_t m6801_read(void *u, uint16_t a) { (void)u; (void)a; return 0xff; }
static void m6801_write(void *u, uint16_t a, uint8_t v) { (void)u; (void)a; (void)v; }
static uint8_t m6801_port_r(void *u, int port)
{
	(void)u;
	if (port == 1) return port8(s_port_in0);
	if (port == 3) return s_port3_in;
	return 0xff;
}
static void m6801_port_w(void *u, int port, uint8_t data, uint8_t ddr)
{
	(void)u; (void)ddr;
	switch (port) {
	case 1:
		// bit 6: IRQ on the main CPU on a high -> low transition
		if ((s_port1_out & 0x40) && (~data & 0x40))
			s_main.irq = 1;
		s_port1_out = data;
		break;
	case 2:
		// bit 4: clock, latch on low -> high
		if ((~s_port2_out & 0x10) && (data & 0x10)) {
			int address = s_port4_out | ((data & 0x0f) << 8);
			if (s_port1_out & 0x80) {
				if ((address & 0x0800) == 0) {
					static const int *ports[4] = { &s_port_dsw0, &s_port_dsw1, &s_port_in1, &s_port_in2 };
					s_port3_in = port8(*ports[address & 3]);
				} else if ((address & 0x0c00) == 0x0c00)
					s_port3_in = s_mcuram[address & 0x3ff];
				m6801_set_irq_line(&s_m6801, M6801_IS3_LINE, 1);
				m6801_set_irq_line(&s_m6801, M6801_IS3_LINE, 0);
			} else if ((address & 0x0c00) == 0x0c00)
				s_mcuram[address & 0x3ff] = s_port3_out;
		}
		s_port2_out = data;
		break;
	case 3:
		s_port3_out = data;
		break;
	case 4:
		s_port4_out = data;
		break;
	}
}

// ---- 68705 (bub68705, MAME's "guesswork" interface) ----
static uint8_t bub68705_port_r(void *u, int n, uint8_t mask)
{
	(void)u; (void)mask;
	return n == 2 ? port8(s_port_in0) : 0xff;
}
static void bub68705_port_w(void *u, int n, uint8_t data, uint8_t mask)
{
	(void)u;
	if (n == 0) {
		s_pa_out = data;
		return;
	}
	if (n != 1)
		return;
	if ((mask & 1) && !(data & 1) && (s_pb_out & 1))
		m68705_port_input(&s_m68705, 0, s_latch);
	if ((mask & 2) && (data & 2) && !(s_pb_out & 2))
		s_address = (uint16_t)((s_address & 0xff00) | s_pa_out);
	if ((mask & 4) && (data & 4) && !(s_pb_out & 4))
		s_address = (uint16_t)((s_address & 0x00ff) | ((s_pa_out & 0x0f) << 8));
	if ((mask & 0x10) && !(data & 0x10) && (s_pb_out & 0x10)) {
		if (data & 8) {
			if ((s_address & 0x0800) == 0) {
				static const int *ports[4] = { &s_port_dsw0, &s_port_dsw1, &s_port_in1, &s_port_in2 };
				s_latch = port8(*ports[s_address & 3]);
			} else if ((s_address & 0x0c00) == 0x0c00)
				s_latch = s_mcuram[s_address & 0x3ff];
		} else if ((s_address & 0x0c00) == 0x0c00)
			s_mcuram[s_address & 0x3ff] = s_pa_out;
	}
	if ((mask & 0x20) && !(data & 0x20) && (s_pb_out & 0x20)) {
		s_mcuram[0x7c] = (uint8_t)(rnd() % 6);   // MAME hack: random EXTEND letters
		s_main.irq = 1;
	}
	s_pb_out = data;
}

// ---- bootleg IC43 (boblbobl) ----
static uint8_t ic43_a_r(int offset) { return offset == 0 ? (uint8_t)(s_ic43_a << 4) : (uint8_t)rnd(); }
static void ic43_a_w(int offset)
{
	int res = 0, a = s_ic43_a;
	switch (offset) {
	case 0:
		if (~a & 8) res ^= 1;
		if (~a & 1) res ^= 2;
		if (~a & 1) res ^= 4;
		if (~a & 2) res ^= 4;
		if (~a & 4) res ^= 8;
		break;
	case 1:
		if (~a & 8) res ^= 1;
		if (~a & 2) res ^= 1;
		if (~a & 8) res ^= 2;
		if (~a & 1) res ^= 4;
		if (~a & 4) res ^= 8;
		break;
	case 2:
		if (~a & 4) res ^= 1;
		if (~a & 8) res ^= 2;
		if (~a & 2) res ^= 4;
		if (~a & 1) res ^= 8;
		if (~a & 4) res ^= 8;
		break;
	default:
		if (~a & 2) res ^= 1;
		if (~a & 4) res ^= 2;
		if (~a & 8) res ^= 2;
		if (~a & 8) res ^= 4;
		if (~a & 1) res ^= 8;
		break;
	}
	s_ic43_a = (uint8_t)res;
}
static void ic43_b_w(int offset, uint8_t data)
{
	static const int xorval[4] = { 4, 1, 8, 2 };
	s_ic43_b = (uint8_t)((data >> 4) ^ xorval[offset]);
}

// ---- Z80 read maps: pages read straight from memory (the same bytes the read handlers return) ----
static const uint8_t *s_main_map[256], *s_sub_map[256], *s_audio_map[256];

static void map_range(const uint8_t **map, uint32_t start, uint32_t end, const uint8_t *mem)
{
	for (uint32_t a = start; a < end; a += 0x100)
		map[a >> 8] = mem ? mem + (a - start) : NULL;
}

static int s_map_bank = -1;   // bank in the main map's 8000-bfff window

static void map_bank(void)
{
	if (s_bank == s_map_bank)
		return;
	uint32_t o = 0x10000u + 0x4000u * (uint32_t)s_bank;
	map_range(s_main_map, 0x8000, 0xc000, o + 0x4000u <= s_main_size ? s_main_rom + o : NULL);
	s_map_bank = s_bank;
}

static void update_maps(void)
{
	memset(s_main_map, 0, sizeof(s_main_map));
	memset(s_sub_map, 0, sizeof(s_sub_map));
	memset(s_audio_map, 0, sizeof(s_audio_map));
	map_range(s_main_map, 0x0000, 0x8000, s_main_rom);
	s_map_bank = -1;
	map_bank();
	map_range(s_main_map, 0xc000, 0xdd00, s_vram);
	map_range(s_main_map, 0xdd00, 0xe000, s_objram);
	map_range(s_main_map, 0xe000, 0xf800, s_share);
	map_range(s_main_map, 0xf800, 0xfa00, s_pal);
	if (s_sub_rom && s_sub_size >= 0x8000)
		map_range(s_sub_map, 0x0000, 0x8000, s_sub_rom);
	if (s_tokio)
		map_range(s_sub_map, 0x8000, 0x9800, s_share);
	else
		map_range(s_sub_map, 0xe000, 0xf800, s_share);
	if (s_audio_rom && s_audio_size >= 0x8000)
		map_range(s_audio_map, 0x0000, 0x8000, s_audio_rom);
	map_range(s_audio_map, 0x8000, 0x9000, s_audio_ram);
	if (s_audio_rom && s_audio_size >= 0x10000)
		map_range(s_audio_map, 0xe000, 0x10000, s_audio_rom + 0xe000);
	s_main.z.read_map = s_main_map;
	s_sub.z.read_map = s_sub_map;
	s_audio.z.read_map = s_audio_map;
}

// ---- main CPU ----
static void sub_reset_cpu(void);

static void bublbobl_bankswitch_w(uint8_t data)
{
	s_bank = (data ^ 4) & 7;
	map_bank();
	int sub_reset = !(data & 0x10);
	if (sub_reset && !s_sub.reset)
		sub_reset_cpu();
	s_sub.reset = sub_reset;
	if (s_m->kind == BB_6801)
		m6801_set_reset(&s_m6801, !(data & 0x20));
	else if (s_m->kind == BB_68705)
		m68705_set_reset_line(&s_m68705, !(data & 0x20));
	s_video_enable = data & 0x40;
	s_flip = (data >> 7) & 1;
}

static void tokio_bankswitch_w(uint8_t data)
{
	s_bank = data & 7;
	map_bank();
	if (s_m->kind == TK_MCU)
		taito68705_reset_w(&s_tmcu, !(data & 0x08));
	s_video_enable = data & 0x40;
}

static uint8_t main_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x8000) return s_main_rom[a];
	if (a < 0xc000) {
		uint32_t o = 0x10000u + 0x4000u * (uint32_t)s_bank + (a - 0x8000u);
		return o < s_main_size ? s_main_rom[o] : 0xff;
	}
	if (a < 0xdd00) return s_vram[a - 0xc000];
	if (a < 0xe000) return s_objram[a - 0xdd00];
	if (a < 0xf800) return s_share[a - 0xe000];
	if (a < 0xfa00) return s_pal[a - 0xf800];
	if (s_tokio) {
		switch (a) {
		case 0xfa03: return port8(s_port_dsw0);
		case 0xfa04: return port8(s_port_dsw1);
		case 0xfa05:
			if (s_m->kind == TK_MCU) {
				int p = s_port_in0;
				rbx_port_custom(p, 0x30, (taito68705_host_semaphore_r(&s_tmcu) ? 0x10 : 0) |
				                         (taito68705_mcu_semaphore_r(&s_tmcu) ? 0x20 : 0));
			}
			return port8(s_port_in0);
		case 0xfa06: return port8(s_port_in1);
		case 0xfa07: return port8(s_port_in2);
		case 0xfc00: s_s2m_pending = 0; return s_sound_to_main;
		case 0xfe00:
			if (s_m->kind == TK_MCU) return taito68705_data_r(&s_tmcu);
			if (s_m->kind == TK_BOOT) return 0xbf;
			return 0;
		}
		return 0;
	}
	if (a >= 0xfa00 && a < 0xfa80) {
		switch (a & 3) {
		case 0: s_s2m_pending = 0; return s_sound_to_main;
		case 1: return sound_semaphores_r();
		}
		return 0;
	}
	if (a < 0xfc00) return 0;
	if (s_m->kind != BB_BOOT) return s_mcuram[a - 0xfc00];
	if (a < 0xfe00) return s_mcuram[a - 0xfc00];
	if (a >= 0xfe00 && a <= 0xfe03) return ic43_a_r(a & 3);
	if (a >= 0xfe80 && a <= 0xfe83) return (a & 3) ? 0xff : (uint8_t)(s_ic43_b << 4);
	if (a == 0xff00) return port8(s_port_dsw0);
	if (a == 0xff01) return port8(s_port_dsw1);
	if (a == 0xff02) return port8(s_port_in0);
	if (a == 0xff03) return port8(s_port_in1);
	return 0;
}

static void main_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a < 0xc000) return;
	if (a < 0xdd00) { s_vram[a - 0xc000] = v; return; }
	if (a < 0xe000) { s_objram[a - 0xdd00] = v; return; }
	if (a < 0xf800) { s_share[a - 0xe000] = v; return; }
	if (a < 0xfa00) { s_pal[a - 0xf800] = v; return; }
	if (s_tokio) {
		switch (a) {
		case 0xfa80: tokio_bankswitch_w(v); return;
		case 0xfb00: s_flip = (v >> 7) & 1; return;   // tokio_videoctrl_w (bit 4: coin lockout)
		case 0xfb80: s_sub.z.nmi_pending = 1; return;
		case 0xfc00: s_main_to_sound = v; s_m2s_pending = 1; update_sound_nmi(); return;
		case 0xfe00: if (s_m->kind == TK_MCU) taito68705_data_w(&s_tmcu, v); return;
		}
		return;
	}
	if (a >= 0xfa00 && a < 0xfa80) {
		if ((a & 3) == 0) { s_main_to_sound = v; s_m2s_pending = 1; update_sound_nmi(); }
		else if ((a & 3) == 3) common_sreset(v ? 1 : 0);
		return;
	}
	if (a < 0xfb00) return;                                        // watchdog
	if (a < 0xfb40) { s_sub.z.nmi_pending = 1; return; }           // NMI on the sub CPU
	if (a < 0xfb80) { bublbobl_bankswitch_w(v); return; }
	if (a < 0xfc00) return;
	if (s_m->kind != BB_BOOT || a < 0xfe00) { s_mcuram[a - 0xfc00] = v; return; }
	if (a >= 0xfe00 && a <= 0xfe03) { ic43_a_w(a & 3); return; }
	if (a >= 0xfe80 && a <= 0xfe83) { ic43_b_w(a & 3, v); return; }
}

// ---- sub CPU ----
static uint8_t sub_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x8000) return a < s_sub_size ? s_sub_rom[a] : 0xff;
	if (s_tokio) return a < 0x9800 ? s_share[a - 0x8000] : 0;
	return (a >= 0xe000 && a < 0xf800) ? s_share[a - 0xe000] : 0;
}
static void sub_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (s_tokio) {
		if (a >= 0x8000 && a < 0x9800) s_share[a - 0x8000] = v;
	} else if (a >= 0xe000 && a < 0xf800)
		s_share[a - 0xe000] = v;
}

// ---- sound CPU ----
static uint8_t audio_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x8000) return a < s_audio_size ? s_audio_rom[a] : 0xff;
	if (a < 0x9000) return s_audio_ram[a - 0x8000];
	if (s_tokio) {
		if (a == 0x9000) { s_m2s_pending = 0; update_sound_nmi(); return s_main_to_sound; }
		if (a == 0x9800) return sound_semaphores_r();
		if (a == 0xb000 || a == 0xb001) { ym_sync(); return rbx_ym_read(0, a & 1); }
		if (a >= 0xe000) return a < s_audio_size ? s_audio_rom[a] : 0xff;
		return 0xff;
	}
	if (a < 0xa000) { ym_sync(); return rbx_ym_read(0, a & 1); }
	if (a < 0xb000) { ym_sync(); return rbx_ym_read(1, a & 1); }
	if (a < 0xc000) {
		switch (a & 3) {
		case 0: s_m2s_pending = 0; update_sound_nmi(); return s_main_to_sound;
		case 1: return sound_semaphores_r();
		}
		return 0xff;
	}
	if (a >= 0xe000) return a < s_audio_size ? s_audio_rom[a] : 0xff;
	return 0xff;
}

static void audio_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a < 0x8000) return;
	if (a < 0x9000) { s_audio_ram[a - 0x8000] = v; return; }
	if (s_tokio) {
		if (a == 0x9000) { s_sound_to_main = v; s_s2m_pending = 1; return; }
		if (a == 0xa000) { s_nmi_enable = 0; update_sound_nmi(); return; }
		if (a == 0xa800) { s_nmi_enable = 1; update_sound_nmi(); return; }
		if (a == 0xb000 || a == 0xb001) { ym_sync(); rbx_ym_write(0, a & 1, v); }
		return;
	}
	if (a < 0xa000) { ym_sync(); rbx_ym_write(0, a & 1, v); return; }
	if (a < 0xb000) { ym_sync(); rbx_ym_write(1, a & 1, v); return; }
	if (a < 0xc000) {
		switch (a & 3) {
		case 0: s_sound_to_main = v; s_s2m_pending = 1; break;
		case 1: s_nmi_enable = 1; update_sound_nmi(); break;
		case 2: s_nmi_enable = 0; update_sound_nmi(); break;
		}
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
	update_maps();
}
static void sub_reset_cpu(void) { zcpu_init(&s_sub, sub_read, sub_write); }
static void audio_reset_cpu(void) { zcpu_init(&s_audio, audio_read, audio_write); }

static void zcpu_run(zcpu *c, int cycles, int ym_irq)
{
	c->start = c->z.cyc;
	c->len = cycles;
	unsigned long end = c->z.cyc + (unsigned long)cycles;
	if (c->reset) {
		c->z.cyc = end;
		return;
	}
	if (ym_irq) {
		z80_run(&c->z, end, &s_ym_any, 0);   // level IRQ from the YMs
		return;
	}
	if (!(c == &s_main && (s_m->kind == BB_6801 || s_m->kind == BB_68705))) {
		z80_run(&c->z, end, &c->irq, 1);   // (irq cleared when acknowledged)
		return;
	}
	while ((long)(c->z.cyc - end) < 0) {
		if (ym_irq) {
			c->z.int_pending = (s_ym_irq[0] || s_ym_irq[1]) ? 1 : 0;   // level IRQ from the YMs
		} else {
			c->z.int_pending = c->irq ? 1 : 0;
			if (c == &s_main && (s_m->kind == BB_6801 || s_m->kind == BB_68705))
				c->z.int_data = s_mcuram[0];   // MAME mcram_vect_r
		}
		z80_step(&c->z);
		if (!ym_irq && c->irq && !c->z.int_pending)
			c->irq = 0;   // acknowledged
	}
}

// ---- video ----
static void bb_render(void)
{
	uint32_t *pal = rbx_video.palette;
	for (int i = 0; i < 256; i++) {   // RGBx_444, big endian
		int c = s_snap_pal[i * 2] << 8 | s_snap_pal[i * 2 + 1];
		int r = (c >> 12) & 15, g = (c >> 8) & 15, b = (c >> 4) & 15;
		pal[i] = (uint32_t)((r * 17) << 16 | (g * 17) << 8 | (b * 17));
	}
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch, h = rbx_video.height;
	for (int y = 0; y < h; y++)
		for (int x = 0; x < SCREEN_W; x++)
			bm[y * pitch + x] = 255;
	if (!s_snap_enable)
		return;
	const uint8_t *vram = s_snap_vram, *obj = s_snap_obj;
	int sx = 0;
	for (int offs = 0; offs < 0x300; offs += 4) {
		if (!(obj[offs] | obj[offs + 1] | obj[offs + 2] | obj[offs + 3]))
			continue;
		int gfx_num = obj[offs + 1], gfx_attr = obj[offs + 3];
		const uint8_t *prom_line = s_prom + 0x80 + ((gfx_num & 0xe0) >> 1);
		int gfx_offs = (gfx_num & 0x1f) * 0x80;
		if ((gfx_num & 0xa0) == 0xa0)
			gfx_offs |= 0x1000;
		int sy = -obj[offs];
		for (int yc = 0; yc < 32; yc++) {
			int pl = prom_line[yc / 2];
			if (pl & 0x08)
				continue;
			if (!(pl & 0x04)) {
				sx = obj[offs + 2];
				if (gfx_attr & 0x40)
					sx -= 256;
			}
			for (int xc = 0; xc < 2; xc++) {
				int goffs = gfx_offs + xc * 0x40 + (yc & 7) * 0x02 + (pl & 0x03) * 0x10;
				if (goffs + 1 >= (int)sizeof(s_snap_vram))
					continue;
				int code = vram[goffs] + 256 * (vram[goffs + 1] & 0x03) + 1024 * (gfx_attr & 0x0f);
				if (!s_opaque[code])
					continue;
				int color = (vram[goffs + 1] & 0x3c) >> 2;
				int flipx = vram[goffs + 1] & 0x40, flipy = vram[goffs + 1] & 0x80;
				int x = sx + xc * 8, y = (sy + yc * 8) & 0xff;
				if (s_snap_flip) {
					x = 248 - x;
					y = 248 - y;
					flipx = !flipx;
					flipy = !flipy;
				}
				const uint8_t *src = s_tiles + code * 64;
				uint16_t base = (uint16_t)(color * 16);
				for (int ty = 0; ty < 8; ty++) {
					int dy = y + ty - VIS_Y0;
					if (dy < 0 || dy >= h)
						continue;
					const uint8_t *row = src + (flipy ? 7 - ty : ty) * 8;
					uint16_t *d = bm + dy * pitch;
					for (int tx = 0; tx < 8; tx++) {
						int dx = x + tx;
						if (dx < 0 || dx >= SCREEN_W)
							continue;
						uint8_t c = row[flipx ? 7 - tx : tx];
						if (c != 15)
							d[dx] = (uint16_t)(base + c);
					}
				}
			}
		}
		sx += 16;
	}
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap_vram, s_vram, sizeof(s_vram));
	memcpy(s_snap_obj, s_objram, sizeof(s_objram));
	memcpy(s_snap_pal, s_pal, sizeof(s_pal));
	s_snap_enable = s_video_enable;
	s_snap_flip = s_flip;
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

static void bb_reset(void)
{
	// (first: the /SRESET pulse below syncs the YM, which reads them: zero as at start, not the last game's)
	s_ym_slice_start = s_ym_slice_len = s_ym_done = 0;
	memset(s_vram, 0, sizeof(s_vram));
	memset(s_objram, 0, sizeof(s_objram));
	memset(s_share, 0, sizeof(s_share));
	memset(s_pal, 0, sizeof(s_pal));
	memset(s_mcuram, 0, sizeof(s_mcuram));
	memset(s_audio_ram, 0, sizeof(s_audio_ram));
	memset(&s_main, 0, sizeof(s_main));
	memset(&s_sub, 0, sizeof(s_sub));
	memset(&s_audio, 0, sizeof(s_audio));
	zcpu_init(&s_main, main_read, main_write);
	zcpu_init(&s_sub, sub_read, sub_write);
	zcpu_init(&s_audio, audio_read, audio_write);
	s_main_to_sound = s_sound_to_main = 0;
	s_m2s_pending = s_s2m_pending = 0;
	s_nmi_enable = s_nmi_out = 0;
	s_ym_irq[0] = s_ym_irq[1] = s_ym_any = 0;
	rbx_ym_reset(0);
	if (s_has_opl)
		rbx_ym_reset(1);
	s_ic43_a = s_ic43_b = 0;
	s_port1_out = s_port2_out = s_port3_out = s_port4_out = s_port3_in = 0;
	s_pa_out = s_pb_out = 0xff;
	s_latch = 0;
	s_address = 0;
	s_video_enable = s_flip = 0;
	if (s_m->kind == BB_6801) {
		m6801_init(&s_m6801);
		m6801_reset(&s_m6801);
	} else if (s_m->kind == BB_68705) {
		m68705_reset(&s_m68705);
	} else if (s_m->kind == TK_MCU) {
		taito68705_reset(&s_tmcu);
	}
	// MACHINE_RESET common: /SRESET pulsed; then the bank latch is cleared
	s_sreset_old = 0;
	if (!s_tokio) {
		common_sreset(1);
		common_sreset(0);
		bublbobl_bankswitch_w(0);
	} else {
		tokio_bankswitch_w(0);
		s_flip = 0;
	}
	s_ym_done = 0;
	s_mix_n = s_mix2_n = 0;
}

static void bb_exit(void)
{
	free(s_tiles);
	free(s_opaque);
	s_tiles = s_opaque = NULL;
	free(s_mix);
	free(s_mix2);
	s_mix = s_mix2 = NULL;
	rbx_ym_exit(0);
	rbx_ym_exit(1);
}

static uint8_t bitswap8(uint8_t v, int b7, int b6, int b5, int b4, int b3, int b2, int b1, int b0)
{
	return (uint8_t)(((v >> b7) & 1) << 7 | ((v >> b6) & 1) << 6 | ((v >> b5) & 1) << 5 | ((v >> b4) & 1) << 4 |
	                 ((v >> b3) & 1) << 3 | ((v >> b2) & 1) << 2 | ((v >> b1) & 1) << 1 | ((v >> b0) & 1));
}

static int decode_gfx(const char *set, uint8_t *gfx, uint32_t size)
{
	// ROMREGION_INVERT (not applied by the loader)
	for (uint32_t i = 0; i < size; i++)
		gfx[i] ^= 0xff;
	if (!strcmp(set, "dland")) {   // MAME init_dland
		for (uint32_t i = 0; i < 0x40000 && i < size; i++)
			gfx[i] = bitswap8(gfx[i], 7, 6, 5, 4, 0, 1, 2, 3);
		for (uint32_t i = 0x40000; i < 0x80000 && i < size; i++)
			gfx[i] = bitswap8(gfx[i], 7, 4, 5, 6, 3, 0, 1, 2);
	}
	const uint32_t n = 16384;
	s_tiles = (uint8_t *)malloc((size_t)n * 64);
	s_opaque = (uint8_t *)calloc(n, 1);
	if (!s_tiles || !s_opaque)
		return -1;
	// charlayout: 8x8, planes { 0, 4, RGN_FRAC(1,2), RGN_FRAC(1,2)+4 }, x { 3,2,1,0, 11,10,9,8 }, 16 bits a row
	static const int xo[8] = { 3, 2, 1, 0, 11, 10, 9, 8 };
	const uint32_t half = size / 2 * 8;
	const uint32_t planes[4] = { 0, 4, half, half + 4 };
	for (uint32_t t = 0; t < n; t++) {
		int trans = 0;
		for (int y = 0; y < 8; y++)
			for (int x = 0; x < 8; x++) {
				int v = 0;
				for (int p = 0; p < 4; p++) {
					uint32_t b = t * 128 + planes[p] + (uint32_t)xo[x] + (uint32_t)y * 16;
					v = v << 1 | ((b >> 3) < size && (gfx[b >> 3] & (0x80 >> (b & 7))) ? 1 : 0);
				}
				s_tiles[t * 64 + y * 8 + x] = (uint8_t)v;
				trans += v == 15;
			}
		s_opaque[t] = trans < 64;
	}
	return 0;
}

static int bb_init(const rbx_game *g)
{
	s_m = find_machine(g->machine);
	if (!s_m) {
		rbx_fail("%s: machine %s is not emulated", g->set, g->machine);
		return -1;
	}
	s_tokio = s_m->kind >= TK_MCU;
	uint32_t gsize = 0, psize = 0;
	s_main_rom = rbx_region("maincpu", &s_main_size);
	s_sub_rom = rbx_region("subcpu", &s_sub_size);
	s_audio_rom = rbx_region("audiocpu", &s_audio_size);
	uint8_t *gfx = rbx_region("gfx1", &gsize);
	s_prom = rbx_region("proms", &psize);
	if (!s_main_rom || !s_sub_rom || !s_audio_rom || !gfx || !s_prom || psize < 0x100) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	if (s_m->kind == BB_6801) {
		uint32_t msize;
		uint8_t *mrom = rbx_region("mcu", &msize);
		if (!mrom || msize < 0x1000) {
			rbx_fail("%s: MCU ROM missing", g->set);
			return -1;
		}
		memset(&s_m6801, 0, sizeof(s_m6801));
		s_m6801.read = m6801_read;
		s_m6801.write = m6801_write;
		s_m6801.port_r = m6801_port_r;
		s_m6801.port_w = m6801_port_w;
		s_m6801.rom = mrom;
		s_m6801.rom_base = 0xf000;
		s_m6801.rom_size = 0x1000;
		s_m6801.type = M6801_TYPE_6801U4;
	} else if (s_m->kind == BB_68705) {
		uint32_t msize;
		uint8_t *mrom = rbx_region("mcu", &msize);
		if (!mrom || msize < 0x800 || m68705_init(&s_m68705, M68705_P3, mrom, 0x800)) {
			rbx_fail("%s: MCU ROM missing", g->set);
			return -1;
		}
		s_m68705.port_r[2] = bub68705_port_r;   // port C = IN0; port A input comes from the latch (pa_w)
		s_m68705.port_w[0] = bub68705_port_w;
		s_m68705.port_w[1] = bub68705_port_w;
	} else if (s_m->kind == TK_MCU) {
		uint32_t msize;
		uint8_t *mrom = rbx_region("bmcu:mcu", &msize);
		memset(&s_tmcu, 0, sizeof(s_tmcu));
		if (!mrom || msize < 0x800 || taito68705_init(&s_tmcu, TAITO68705_STD, mrom, 0x800)) {
			rbx_fail("%s: MCU ROM missing", g->set);
			return -1;
		}
	}
	if (decode_gfx(g->set, gfx, gsize)) {
		rbx_fail("%s: out of memory (graphics)", g->set);
		bb_exit();
		return -1;
	}
	s_port_in0 = rbx_port_find("IN0");
	s_port_in1 = rbx_port_find("IN1");
	s_port_in2 = rbx_port_find("IN2");
	s_port_dsw0 = rbx_port_find("DSW0");
	s_port_dsw1 = rbx_port_find("DSW1");

	rbx_ym_config yc;
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM2203;
	yc.clock = 3000000;
	yc.irq = ym_irq0;
	if (s_tokio) {
		yc.gain[0] = yc.gain[1] = yc.gain[2] = 20;   // 0.08
		yc.gain[3] = 256;
	} else
		yc.gain[0] = yc.gain[1] = yc.gain[2] = yc.gain[3] = 64;   // 0.25
	if (rbx_ym_init(0, &yc)) {
		rbx_fail("%s: sound init failed", g->set);
		bb_exit();
		return -1;
	}
	s_has_opl = !s_tokio;
	if (s_has_opl) {
		memset(&yc, 0, sizeof(yc));
		yc.type = RBX_YM3526;
		yc.clock = 3000000;
		yc.irq = ym_irq1;
		yc.gain[0] = 128;   // 0.5
		if (rbx_ym_init(1, &yc)) {
			rbx_fail("%s: sound init failed", g->set);
			bb_exit();
			return -1;
		}
	}
	// (the YM chips' samples made on the worker: rbx_ym_async_take at the frame's end)
	if (rbx_ym_set_async(0, 1) || (s_has_opl && rbx_ym_set_async(1, 1))) {
		bb_exit();
		return -1;
	}
	uint32_t rate = rbx_ym_rate(0);
	if (s_has_opl && rbx_ym_rate(1) > rate)
		rate = rbx_ym_rate(1);
	s_mix_cap = (int)(rate / 50) + 64;
	s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	s_mix2 = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	if (!s_mix || !s_mix2 || rbx_audio_setup(rbx_ym_rate(0)) || rbx_video_setup(SCREEN_W, VIS_Y1 - VIS_Y0 + 1, 256)) {
		bb_exit();
		return -1;
	}
	bb_reset();
	return 0;
}

static void mcu_run(int cycles)
{
	// (the MCU cores carry their overshoot into the next call)
	switch (s_m->kind) {
	case BB_6801: m6801_execute(&s_m6801, cycles); break;
	case BB_68705: m68705_execute(&s_m68705, cycles); break;
	case TK_MCU: taito68705_execute(&s_tmcu, cycles); break;
	}
}

static void bb_frame(void)
{
	const int hold_main = s_m->kind == BB_BOOT || s_tokio;
	const int mcu_line = s_m->kind == BB_6801 ? M6801_LINE : s_m->kind == BB_68705 ? M68705P3_LINE :
	                     s_m->kind == TK_MCU ? TOKIO_MCU_LINE : 0;
	// the frame starts at vblank (MAME updates the inputs there)
	for (int i = 0; i < LINES; i++) {
		int line = (VIS_Y1 + 1 + i) % LINES;
		if (line == VIS_Y1 + 1) {
			vblank_capture();
			if (hold_main)
				s_main.irq = 1;
			s_sub.irq = 1;
			if (s_m->kind == BB_6801)
				m6801_set_irq_line(&s_m6801, M6801_IRQ1_LINE, 1);
			else if (s_m->kind == BB_68705)
				m68705_set_irq_line(&s_m68705, 1);
		} else if (line == VIS_Y0) {
			if (s_m->kind == BB_6801)
				m6801_set_irq_line(&s_m6801, M6801_IRQ1_LINE, 0);
		} else if (line == VIS_Y1 && s_m->kind == BB_68705)
			m68705_set_irq_line(&s_m68705, 0);   // MAME: acknowledged by a timer 1/60 s later
		for (int s = 0; s < SLICES; s++) {
			s_ym_slice_start = s_ym_done;
			s_ym_slice_len = YM_LINE / SLICES;
			zcpu_run(&s_main, MAIN_LINE / SLICES, 0);
			zcpu_run(&s_sub, MAIN_LINE / SLICES, 0);
			zcpu_run(&s_audio, AUDIO_LINE / SLICES, 1);
			if (mcu_line)
				mcu_run(mcu_line / SLICES);
			ym_run_to(s_ym_slice_start + s_ym_slice_len);
		}
	}
	rbx_ym_async_take(0, s_mix, s_mix_n);
	if (s_has_opl)
		rbx_ym_async_take(1, s_mix2, s_mix2_n);
	if (s_has_opl && s_mix2_n && s_mix_n) {
		// YM3526 output resampled onto the YM2203 stream (nearest sample)
		// index i * s_mix2_n / s_mix_n stepped without a division per sample
		int q = 0, r = 0;
		for (int i = 0; i < s_mix_n; i++) {
			s_mix[i] += s_mix2[q];
			r += s_mix2_n;
			while (r >= s_mix_n) {
				r -= s_mix_n;
				q++;
			}
		}
	}
	for (int i = 0; i < s_mix_n; i++)
		rbx_audio_put(s_mix[i]);
	memset(s_mix, 0, sizeof(int32_t) * (size_t)s_mix_n);
	memset(s_mix2, 0, sizeof(int32_t) * (size_t)s_mix2_n);
	s_mix_n = s_mix2_n = 0;
	s_ym_done = 0;
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
		t.z.read_map = c->z.read_map;
		*c = t;
	}
	return 0;
}

static int bb_state(void *ctx, int load)
{
	int rc = 0;
	rc |= zcpu_state(ctx, "main", &s_main, load);
	rc |= zcpu_state(ctx, "sub", &s_sub, load);
	rc |= zcpu_state(ctx, "audio", &s_audio, load);
	rc |= rbx_state_io(ctx, "vram", s_vram, sizeof(s_vram));
	rc |= rbx_state_io(ctx, "objram", s_objram, sizeof(s_objram));
	rc |= rbx_state_io(ctx, "share", s_share, sizeof(s_share));
	rc |= rbx_state_io(ctx, "pal", s_pal, sizeof(s_pal));
	rc |= rbx_state_io(ctx, "mcuram", s_mcuram, sizeof(s_mcuram));
	rc |= rbx_state_io(ctx, "audram", s_audio_ram, sizeof(s_audio_ram));
	if (s_m->kind == BB_6801)
		rc |= rbx_state_io(ctx, "m6801", m6801_state_data(&s_m6801), m6801_state_size());
	else if (s_m->kind == BB_68705)
		rc |= rbx_state_io(ctx, "m68705", &s_m68705.st, sizeof(s_m68705.st));
	else if (s_m->kind == TK_MCU) {
		rc |= rbx_state_io(ctx, "tmcu", &s_tmcu.st, sizeof(s_tmcu.st));
		rc |= rbx_state_io(ctx, "tmcu6805", &s_tmcu.mcu.st, sizeof(s_tmcu.mcu.st));
	}
	int32_t misc[24] = { s_bank, s_video_enable, s_flip, s_main_to_sound, s_sound_to_main, s_m2s_pending, s_s2m_pending,
		s_nmi_enable, s_nmi_out, s_sreset_old, s_ym_irq[0], s_ym_irq[1], s_ic43_a, s_ic43_b, (int32_t)s_rand,
		s_port1_out, s_port2_out, s_port3_out, s_port4_out, s_port3_in, s_pa_out, s_pb_out, s_latch, s_address };
	int32_t misc2[2] = { 0, 0 };
	rc |= rbx_state_io(ctx, "misc", misc, sizeof(misc));
	rc |= rbx_state_io(ctx, "misc2", misc2, sizeof(misc2));
	rc |= rbx_ym_state(0, ctx, load);
	if (s_has_opl)
		rc |= rbx_ym_state(1, ctx, load);
	if (rc)
		return -1;
	if (load) {
		s_bank = misc[0]; s_video_enable = misc[1]; s_flip = misc[2];
		s_main_to_sound = (uint8_t)misc[3]; s_sound_to_main = (uint8_t)misc[4];
		s_m2s_pending = misc[5]; s_s2m_pending = misc[6]; s_nmi_enable = misc[7]; s_nmi_out = misc[8];
		s_sreset_old = misc[9]; s_ym_irq[0] = misc[10]; s_ym_irq[1] = misc[11];
		s_ym_any = s_ym_irq[0] || s_ym_irq[1];
		s_ic43_a = (uint8_t)misc[12]; s_ic43_b = (uint8_t)misc[13]; s_rand = (uint32_t)misc[14];
		s_port1_out = (uint8_t)misc[15]; s_port2_out = (uint8_t)misc[16]; s_port3_out = (uint8_t)misc[17];
		s_port4_out = (uint8_t)misc[18]; s_port3_in = (uint8_t)misc[19];
		s_pa_out = (uint8_t)misc[20]; s_pb_out = (uint8_t)misc[21]; s_latch = (uint8_t)misc[22];
		s_address = (uint16_t)misc[23];
		update_maps();
	}
	return 0;
}

static int bb_save(void *ctx) { return bb_state(ctx, 0); }
static int bb_load(void *ctx) { return bb_state(ctx, 1); }
static int bb_supported(const char *machine) { return find_machine(machine) != NULL; }

const rbx_driver rbx_drv_bublbobl = { "Bubble Bobble", bb_init, bb_exit, bb_reset, bb_frame, bb_save, bb_load, bb_supported, bb_render };
