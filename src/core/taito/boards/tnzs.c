/*  Taito / Seta "tnzs" family (The NewZealand Story, Arkanoid II, Extermination, Dr. Toppel, Insector X, Kageki,
    Chuka Taisen, Plump Pop, Jumping Pop, Kabuki-Z), ported from MAME src/mame/taito/tnzs.cpp and tnzs_video.cpp
    (BSD-3-Clause, copyright-holders: Luca Elia, Mirko Buffoni, Takahiro Nogi). Memory maps, machine configurations,
    the Arkanoid II MCU simulation and the uPD4701 spinner interface follow MAME (upd4701.cpp, BSD-3-Clause,
    copyright-holders: smf, AJR). Sprites / tiles: Seta X1-001 (video/x1_001.c); M-Chip: I8742 (cpu/mcs48).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/z80/z80.h"
#include "../../cpu/mcs48/mcs48.h"
#include "../../sound/rbx_ym.h"
#include "../../video/x1_001.h"

#include <stdlib.h>
#include <string.h>

enum { TZ_TNZSB, TZ_KABUKIZ, TZ_INSECTX, TZ_KAGEKI, TZ_JPOPNICS, TZ_TNZS, TZ_EXTRMATN, TZ_PLUMPPOP, TZ_ARKNOID2 };
enum { PAL_MAIN_RAM, PAL_PROM, PAL_SUB_F000, PAL_SUB_F800, PAL_JPOPNICS };

typedef struct {
	const char *name;
	int kind;
	int pal;
	uint32_t rate_mhz;
	int mcu;        // I8742 present
	int spinner;    // uPD4701
	int insectx_gfx;
} machine_t;

static const machine_t s_machines[] = {
	{ "tnzsb",    TZ_TNZSB,    PAL_SUB_F000, 59150, 0, 0, 0 },
	{ "kabukiz",  TZ_KABUKIZ,  PAL_SUB_F800, 59150, 0, 0, 0 },
	{ "insectx",  TZ_INSECTX,  PAL_MAIN_RAM, 60000, 0, 0, 1 },
	{ "kageki",   TZ_KAGEKI,   PAL_MAIN_RAM, 60000, 0, 0, 0 },
	{ "jpopnics", TZ_JPOPNICS, PAL_JPOPNICS, 60000, 0, 1, 0 },
	{ "tnzs",     TZ_TNZS,     PAL_MAIN_RAM, 59150, 1, 0, 0 },
	{ "extrmatn", TZ_EXTRMATN, PAL_PROM,     60000, 1, 0, 0 },
	{ "plumppop", TZ_PLUMPPOP, PAL_PROM,     60000, 1, 1, 0 },
	{ "arknoid2", TZ_ARKNOID2, PAL_PROM,     60000, 0, 1, 0 },
};

#define LINES 256
#define VIS_Y0 16
#define VIS_Y1 239
#define SCREEN_W 256
#define SLICES 8          // CPU interleave per scanline (MAME: perfect quantum on the main CPU)
#define Z80_CLOCK 6000000
#define YM_CLOCK 3000000
#define MCU_CLOCK 6000000

typedef struct {
	z80 z;
	int hold;             // HOLD_LINE IRQ
	int reset;            // RESET line asserted
	unsigned long start;  // cycle count at the start of the current slice
	int len;              // cycles of the current slice
	uint64_t frac;
	int line_cycles;
} zcpu;

static const machine_t *s_m;
static uint8_t *s_main_rom, *s_sub_rom, *s_audio_rom, *s_scpu_rom, *s_mcu_rom;
static uint32_t s_main_size, s_sub_size, s_audio_size, s_scpu_size;
static uint8_t s_ram_bank[2][0x4000];
static uint8_t s_share[0x1000], s_sub_ram[0x1000], s_audio_ram[0x2000];
static uint8_t s_pal[0x800];
static uint32_t s_prom_rgb[512];
static zcpu s_main, s_sub, s_audio, s_scpu;
static int s_has_audio, s_has_scpu;
static int s_ramrom, s_subbank, s_audiobank;
static uint8_t s_soundlatch;
static mcs48 s_mcu;
static int s_mcu_debt;
static uint64_t s_mcu_frac;
static uint8_t s_input_select;
static int s_csport_sel;
static x1_001_t *s_x1, *s_snap;
static uint8_t s_snap_pal[0x800];
static int s_port_in[3], s_port_dswa, s_port_dswb, s_port_coin1, s_port_coin2;
// uPD4701
static uint16_t s_spin_x, s_spin_y, s_spin_startx, s_spin_starty;
static int s_spin_resetx, s_spin_resety;
// arknoid2 MCU simulation
static int s_mcu_initializing, s_mcu_coinage_init, s_mcu_command, s_mcu_readcredits, s_mcu_reportcoin, s_insertcoin;
static uint8_t s_mcu_coinage[4], s_mcu_coins_a, s_mcu_coins_b, s_mcu_credits;
// sound
static int32_t *s_mix;
static int s_mix_cap, s_mix_n;
static uint32_t s_ym_done;
static uint64_t s_ym_frac;
static int s_ym_irq;
static int s_dac, s_dac_gain;

static uint8_t port8(int p) { return (uint8_t)rbx_port_read(p); }

// ---- sound timing: the YM runs to the position of the CPU that touches it ----
static zcpu *s_sound_owner;   // the CPU whose slice position is the time base for ym_sync()

// YM clocks are tracked per slice: s_ym_slice_start .. + s_ym_slice_len
static uint32_t s_ym_slice_start, s_ym_slice_len;

static void ym_run_to(uint32_t pos)
{
	if (pos <= s_ym_done)
		return;
	int n = rbx_ym_advance(0, pos - s_ym_done, s_mix + s_mix_n, s_mix_cap - s_mix_n);
	if (s_dac_gain) {
		int32_t d = (s_dac * s_dac_gain) >> 8;
		for (int i = 0; i < n; i++)
			s_mix[s_mix_n + i] += d;
	}
	s_mix_n += n;
	s_ym_done = pos;
}

static void ym_sync(void)
{
	zcpu *c = s_sound_owner;
	uint32_t done = (uint32_t)(c->z.cyc - c->start);
	if ((int)done > c->len)
		done = (uint32_t)c->len;
	uint32_t pos = s_ym_slice_start + (c->len ? rbx_muldiv(s_ym_slice_len, done, (uint32_t)c->len) : 0);
	ym_run_to(pos);
}

static uint8_t ym_read(zcpu *c, int offset) { s_sound_owner = c; ym_sync(); return rbx_ym_read(0, (uint32_t)offset); }
static void ym_write(zcpu *c, int offset, uint8_t v) { s_sound_owner = c; ym_sync(); rbx_ym_write(0, (uint32_t)offset, v); }

static void ym_irq_cb(int state)
{
	// tnzsb: YM2203 IRQ -> audio CPU NMI
	if (state && !s_ym_irq && s_has_audio)
		s_audio.z.nmi_pending = 1;
	s_ym_irq = state;
}
static uint8_t ym_dswa(void) { return port8(s_port_dswa); }
static uint8_t ym_dswb(void) { return port8(s_port_dswb); }

static void map_banks(void);

static void kabukiz_bank_w(uint8_t data)
{
	if (data != 0xff) {
		s_audiobank = data & 7;
		map_banks();
	}
}
static void dac_w(uint8_t data) { s_dac = (int)data - 0x80; }

static uint8_t kageki_csport_r(void)
{
	int dsw1 = port8(s_port_dswa), dsw2 = port8(s_port_dswb), dsw;
	switch (s_csport_sel & 3) {
	case 0: dsw = ((dsw2 & 0x10) >> 1) | ((dsw2 & 0x01) << 2) | ((dsw1 & 0x10) >> 3) | ((dsw1 & 0x01) >> 0); break;
	case 1: dsw = ((dsw2 & 0x40) >> 3) | ((dsw2 & 0x04) >> 0) | ((dsw1 & 0x40) >> 5) | ((dsw1 & 0x04) >> 2); break;
	case 2: dsw = ((dsw2 & 0x20) >> 2) | ((dsw2 & 0x02) << 1) | ((dsw1 & 0x20) >> 4) | ((dsw1 & 0x02) >> 1); break;
	default: dsw = ((dsw2 & 0x80) >> 4) | ((dsw2 & 0x08) >> 1) | ((dsw1 & 0x80) >> 6) | ((dsw1 & 0x08) >> 3); break;
	}
	return (uint8_t)dsw;
}
static void kageki_csport_w(uint8_t data)
{
	s_csport_sel = data;
	if (!(data & 0x40))
		s_scpu.z.nmi_pending = 1;
}

// ---- uPD4701 (read_xy) ----
static uint8_t spinner_r(int offset)
{
	uint16_t x = (uint16_t)((s_spin_x - s_spin_startx) & 0xfff), y = (uint16_t)((s_spin_y - s_spin_starty) & 0xfff);
	// switches: left = COIN1, middle = COIN2 (active high, see MAME's arknoid2 kludge), right = SERVICE1 (active low)
	int sw = 0;
	if (s_m->kind == TZ_ARKNOID2) {   // (plumppop: switches not connected)
		if (!(port8(s_port_coin1) & 1)) sw |= 4;
		if (!(port8(s_port_coin2) & 1)) sw |= 1;
		if (!(port8(s_port_in[2]) & 1)) sw |= 2;
	}
	if (sw)
		sw |= 8;
	uint16_t d = (uint16_t)(((offset & 2) ? y : x) | sw << 12);
	return (offset & 1) ? (uint8_t)(d >> 8) : (uint8_t)d;
}

static void spinner_reset_w(int state)
{
	if (s_spin_resetx != state) { s_spin_resetx = state; if (state) s_spin_startx = s_spin_x; }
	if (s_spin_resety != state) { s_spin_resety = state; if (state) s_spin_starty = s_spin_y; }
}

static void spinner_frame(void)
{
	int dx = rbx_analog_dpad(1, RBX_IPT_DIAL, 10), dy = rbx_analog_dpad(2, RBX_IPT_DIAL, 10);   // (d-pad: 15 x 70%)
	if (!s_spin_resetx) s_spin_x = (uint16_t)(s_spin_x + dx);
	if (!s_spin_resety) s_spin_y = (uint16_t)(s_spin_y + dy);
}

// ---- arknoid2 MCU simulation (MAME arknoid2_state) ----
static void a2_mcu_reset(void)
{
	s_mcu_initializing = 3;
	s_mcu_coinage_init = 0;
	s_mcu_coinage[0] = s_mcu_coinage[1] = s_mcu_coinage[2] = s_mcu_coinage[3] = 1;
	s_mcu_coins_a = s_mcu_coins_b = 0;
	s_mcu_credits = 0;
	s_mcu_reportcoin = 0;
	s_mcu_command = 0;
}

static void a2_handle_coins(int coin)
{
	if (coin & 0x08)
		s_mcu_reportcoin = coin;
	else if (coin && coin != s_insertcoin) {
		if (coin & 0x01) {
			s_mcu_coins_a++;
			if (s_mcu_coins_a >= s_mcu_coinage[0]) {
				s_mcu_coins_a -= s_mcu_coinage[0];
				s_mcu_credits += s_mcu_coinage[1];
				if (s_mcu_credits >= 9)
					s_mcu_credits = 9;
			}
		}
		if (coin & 0x02) {
			s_mcu_coins_b++;
			if (s_mcu_coins_b >= s_mcu_coinage[2]) {
				s_mcu_coins_b -= s_mcu_coinage[2];
				s_mcu_credits += s_mcu_coinage[3];
				if (s_mcu_credits >= 9)
					s_mcu_credits = 9;
			}
		}
		if (coin & 0x04)
			s_mcu_credits++;
		s_mcu_reportcoin = coin;
	} else
		s_mcu_reportcoin = 0;
	s_insertcoin = coin;
}

static uint8_t a2_mcu_r(int offset)
{
	static const uint8_t startup[3] = { 0x55, 0xaa, 0x5a };
	if (offset == 0) {
		if (s_mcu_initializing) {
			s_mcu_initializing--;
			return startup[2 - s_mcu_initializing];
		}
		switch (s_mcu_command) {
		case 0x41:
			return s_mcu_credits;
		case 0xc1:
			if (s_mcu_readcredits == 0) {
				s_mcu_readcredits = 1;
				if (s_mcu_reportcoin & 0x08) {
					s_mcu_initializing = 3;
					return 0xee;
				}
				return s_mcu_credits;
			}
			return port8(s_port_in[0]);
		default:
			return 0xff;
		}
	}
	if (s_mcu_reportcoin & 0x08) return 0xe1;
	if (s_mcu_reportcoin & 0x01) return 0x11;
	if (s_mcu_reportcoin & 0x02) return 0x21;
	if (s_mcu_reportcoin & 0x04) return 0x31;
	return 0x01;
}

static void a2_mcu_w(int offset, uint8_t data)
{
	if (offset == 0) {
		if (s_mcu_command == 0x41)
			s_mcu_credits = (uint8_t)(s_mcu_credits + data);
		return;
	}
	if (s_mcu_initializing) {
		s_mcu_coinage[s_mcu_coinage_init++] = data;
		if (s_mcu_coinage_init == 4)
			s_mcu_coinage_init = 0;
	}
	if (data == 0xc1)
		s_mcu_readcredits = 0;
	if (data == 0x15) {
		s_mcu_credits = (uint8_t)(s_mcu_credits - 1);
		if (s_mcu_credits == 0xff)
			s_mcu_credits = 0;
	}
	s_mcu_command = data;
}

// ---- I8742 M-Chip ----
static uint8_t mcu_port_r(void *u, int port)
{
	(void)u;
	if (port == 2)
		return port8(s_port_in[2]);
	switch (s_input_select) {
	case 0x0a: return port8(s_port_in[2]);
	case 0x0c: return port8(s_port_in[0]);
	case 0x0d: return port8(s_port_in[1]);
	default: return 0xff;
	}
}
static void mcu_port_w(void *u, int port, uint8_t v)
{
	(void)u;
	if (port == 2)
		s_input_select = v & 0x0f;   // (bits 4-7: coin counters / lockout)
}
static int mcu_test_r(void *u, int t)
{
	(void)u;
	return port8(t ? s_port_coin2 : s_port_coin1) & 1;
}

// ---- main CPU ----
// ---- Z80 read maps: pages read straight from memory (the same bytes the read handlers return) ----
static const uint8_t *s_main_map[256], *s_sub_map[256], *s_audio_map[256];

static void map_range(const uint8_t **map, uint32_t start, uint32_t end, const uint8_t *mem)
{
	for (uint32_t a = start; a < end; a += 0x100)
		map[a >> 8] = mem ? mem + (a - start) : NULL;
}

static int s_map_ramrom = -1, s_map_subbank = -1, s_map_audiobank = -1;   // banks in the maps' windows

// the banked windows (only when a bank changed)
static void map_banks(void)
{
	if (s_ramrom != s_map_ramrom) {
		const uint8_t *m = NULL;
		if (s_ramrom < 2)
			m = s_ram_bank[s_ramrom];
		else if (0x4000u * (uint32_t)s_ramrom + 0x4000u <= s_main_size)
			m = s_main_rom + 0x4000u * (uint32_t)s_ramrom;
		map_range(s_main_map, 0x8000, 0xc000, m);
		s_map_ramrom = s_ramrom;
	}
	if (s_subbank != s_map_subbank) {
		uint32_t o = 0x8000u + 0x2000u * (uint32_t)s_subbank;
		map_range(s_sub_map, 0x8000, 0xa000, o + 0x2000u <= s_sub_size ? s_sub_rom + o : NULL);
		s_map_subbank = s_subbank;
	}
	if (s_m->kind == TZ_KABUKIZ && s_audiobank != s_map_audiobank) {
		uint32_t o = 0x4000u * (uint32_t)s_audiobank;
		map_range(s_audio_map, 0x8000, 0xc000, s_audio_rom && o + 0x4000u <= s_audio_size ? s_audio_rom + o : NULL);
		s_map_audiobank = s_audiobank;
	}
}

static void update_maps(void)
{
	memset(s_main_map, 0, sizeof(s_main_map));
	memset(s_sub_map, 0, sizeof(s_sub_map));
	memset(s_audio_map, 0, sizeof(s_audio_map));
	map_range(s_main_map, 0x0000, 0x8000, s_main_rom);
	map_range(s_main_map, 0xe000, 0xf000, s_share);
	map_range(s_sub_map, 0x0000, 0x8000, s_sub_rom);
	map_range(s_sub_map, 0xd000, 0xe000, s_sub_ram);
	map_range(s_sub_map, 0xe000, 0xf000, s_share);
	if (s_audio_rom && s_audio_size >= 0x8000)
		map_range(s_audio_map, 0x0000, 0x8000, s_audio_rom);
	if (s_m->kind == TZ_KABUKIZ)
		map_range(s_audio_map, 0xe000, 0x10000, s_audio_ram);
	else
		map_range(s_audio_map, 0xc000, 0xe000, s_audio_ram);
	s_map_ramrom = s_map_subbank = s_map_audiobank = -1;
	map_banks();
	s_main.z.read_map = s_main_map;
	s_sub.z.read_map = s_sub_map;
	s_audio.z.read_map = s_audio_map;
}

static uint8_t main_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x8000)
		return s_main_rom[a];
	if (a < 0xc000) {
		if (s_ramrom < 2)
			return s_ram_bank[s_ramrom][a - 0x8000];
		uint32_t o = 0x4000u * (uint32_t)s_ramrom + (a - 0x8000u);
		return o < s_main_size ? s_main_rom[o] : 0xff;
	}
	if (a < 0xd000) return (uint8_t)s_x1->code[a - 0xc000];
	if (a < 0xe000) return (uint8_t)(s_x1->code[a - 0xd000] >> 8);
	if (a < 0xf000) return s_share[a - 0xe000];
	if (a < 0xf300) return s_x1->ylow[a - 0xf000];
	if (a >= 0xf800) {
		if (s_m->pal == PAL_JPOPNICS) return s_pal[a - 0xf800];
		if (s_m->pal == PAL_MAIN_RAM && a < 0xfc00) return s_pal[a - 0xf800];
	}
	return 0;
}

static void sub_set_reset(int state)
{
	if (state && !s_sub.reset) {
		unsigned long cyc = s_sub.z.cyc;
		z80 cfg = s_sub.z;
		z80_init(&s_sub.z);
		s_sub.z.read_byte = cfg.read_byte;
		s_sub.z.read_map = cfg.read_map;
		s_sub.z.write_byte = cfg.write_byte;
		s_sub.z.port_in = cfg.port_in;
		s_sub.z.port_out = cfg.port_out;
		s_sub.z.cyc = cyc;
		s_sub.z.int_data = 0xff;
		s_sub.hold = 0;
	}
	s_sub.reset = state;
}

static void main_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a < 0x8000)
		return;
	if (a < 0xc000) {
		if (s_ramrom < 2)
			s_ram_bank[s_ramrom][a - 0x8000] = v;
		return;
	}
	if (a < 0xd000) { s_x1->code[a - 0xc000] = (uint16_t)((s_x1->code[a - 0xc000] & 0xff00) | v); return; }
	if (a < 0xe000) { s_x1->code[a - 0xd000] = (uint16_t)((s_x1->code[a - 0xd000] & 0x00ff) | v << 8); return; }
	if (a < 0xf000) {   // (the sub CPU catches up to here before the main goes on: run_main_slice)
		s_share[a - 0xe000] = v;
		z80_break = 1;
		return;
	}
	if (a < 0xf300) { s_x1->ylow[a - 0xf000] = v; return; }
	if (a < 0xf400) { s_x1->ctrl[a & 3] = v; return; }
	if (a == 0xf400) { s_x1->bgflag = v; return; }
	if (a == 0xf600) {
		sub_set_reset(!(v & 0x10));
		s_ramrom = v & 7;
		map_banks();
		return;
	}
	if (a >= 0xf800) {
		if (s_m->pal == PAL_JPOPNICS) s_pal[a - 0xf800] = v;
		else if (s_m->pal == PAL_MAIN_RAM && a < 0xfc00) s_pal[a - 0xf800] = v;
	}
}

// ---- sub CPU ----
static uint8_t sub_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x8000)
		return s_sub_rom[a];
	if (a < 0xa000) {
		uint32_t o = 0x8000u + 0x2000u * (uint32_t)s_subbank + (a - 0x8000u);
		return o < s_sub_size ? s_sub_rom[o] : 0xff;
	}
	if (a >= 0xd000 && a < 0xe000) return s_sub_ram[a - 0xd000];
	if (a >= 0xe000 && a < 0xf000) return s_share[a - 0xe000];
	switch (s_m->kind) {
	case TZ_TNZSB:
	case TZ_KABUKIZ:
		if (a == 0xb002) return port8(s_port_dswa);
		if (a == 0xb003) return port8(s_port_dswb);
		if (a >= 0xc000 && a <= 0xc002) return port8(s_port_in[a - 0xc000]);
		if (s_m->pal == PAL_SUB_F000 && a >= 0xf000 && a < 0xf400) return s_pal[a - 0xf000];
		if (s_m->pal == PAL_SUB_F800 && a >= 0xf800 && a < 0xfc00) return s_pal[a - 0xf800];
		return 0;
	case TZ_JPOPNICS:
		if (a == 0xb000 || a == 0xb001) return ym_read(&s_sub, a & 1);
		if (a == 0xc000) return port8(s_port_in[1]);
		if (a == 0xc001) return port8(s_port_in[2]);
		if (a == 0xc600) return port8(s_port_dswa);
		if (a == 0xc601) return port8(s_port_dswb);
		if (a >= 0xf000 && a <= 0xf003) return spinner_r(a & 3);
		return 0;
	default:
		break;
	}
	if (a == 0xb000 || a == 0xb001) return ym_read(&s_sub, a & 1);
	if (a >= 0xc000 && a <= 0xc002) {
		if (s_m->kind == TZ_INSECTX || s_m->kind == TZ_KAGEKI)
			return port8(s_port_in[a - 0xc000]);
		if (a == 0xc002)
			return 0;
		if (s_m->kind == TZ_ARKNOID2)
			return a2_mcu_r(a & 1);
		if (s_m->mcu)
			return upi41_master_r(&s_mcu, a & 1);
		return 0;
	}
	if (a >= 0xf000 && a <= 0xf003 && s_m->spinner)
		return spinner_r(a & 3);
	return 0;
}

static void sub_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0xd000 && a < 0xe000) { s_sub_ram[a - 0xd000] = v; return; }
	if (a >= 0xe000 && a < 0xf000) { s_share[a - 0xe000] = v; return; }
	if (a == 0xa000) {
		s_subbank = v & 3;
		map_banks();
		if (s_m->mcu && (v & 4))
			mcs48_reset(&s_mcu);
		if (s_m->kind == TZ_ARKNOID2 && (v & 4))
			a2_mcu_reset();
		if (s_m->spinner)
			spinner_reset_w((v >> 5) & 1);
		return;
	}
	switch (s_m->kind) {
	case TZ_TNZSB:
	case TZ_KABUKIZ:
		if (a == 0xb004) { s_soundlatch = v; s_audio.hold = 1; return; }
		if (s_m->pal == PAL_SUB_F000 && a >= 0xf000 && a < 0xf400) { s_pal[a - 0xf000] = v; return; }
		if (s_m->pal == PAL_SUB_F800 && a >= 0xf800 && a < 0xfc00) { s_pal[a - 0xf800] = v; return; }
		return;
	default:
		break;
	}
	if (a == 0xb000 || a == 0xb001) { ym_write(&s_sub, a & 1, v); return; }
	if (a == 0xc000 || a == 0xc001) {
		if (s_m->kind == TZ_ARKNOID2)
			a2_mcu_w(a & 1, v);
		else if (s_m->mcu)
			upi41_master_w(&s_mcu, a & 1, v);
	}
}

// ---- audio CPU (tnzsb / kabukiz) ----
static uint8_t audio_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x8000)
		return a < s_audio_size ? s_audio_rom[a] : 0xff;
	if (s_m->kind == TZ_KABUKIZ) {
		if (a < 0xc000) {
			uint32_t o = 0x4000u * (uint32_t)s_audiobank + (a - 0x8000u);
			return o < s_audio_size ? s_audio_rom[o] : 0xff;
		}
		if (a >= 0xe000) return s_audio_ram[a - 0xe000];
		return 0;
	}
	if (a >= 0xc000 && a < 0xe000) return s_audio_ram[a - 0xc000];
	return 0;
}
static void audio_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (s_m->kind == TZ_KABUKIZ) {
		if (a >= 0xe000) s_audio_ram[a - 0xe000] = v;
		return;
	}
	if (a >= 0xc000 && a < 0xe000) s_audio_ram[a - 0xc000] = v;
}
static uint8_t audio_in(z80 *z, uint8_t p)
{
	(void)z;
	if (p < 2) return ym_read(&s_audio, p);
	if (p == 2) return s_soundlatch;
	return 0;
}
static void audio_out(z80 *z, uint8_t p, uint8_t v)
{
	(void)z;
	if (p < 2) ym_write(&s_audio, p, v);
}

// ---- kageki sample CPU ----
static uint8_t scpu_read(void *u, uint16_t a) { (void)u; return a < s_scpu_size ? s_scpu_rom[a] : 0xff; }
static void scpu_write(void *u, uint16_t a, uint8_t v) { (void)u; (void)a; (void)v; }
static uint8_t scpu_in(z80 *z, uint8_t p) { (void)z; (void)p; return (uint8_t)s_csport_sel; }
static void scpu_out(z80 *z, uint8_t p, uint8_t v)
{
	(void)z; (void)p;
	s_sound_owner = &s_scpu;
	ym_sync();
	dac_w(v);
}

static uint8_t main_in(z80 *z, uint8_t p) { (void)z; (void)p; return 0xff; }
static void main_out(z80 *z, uint8_t p, uint8_t v) { (void)z; (void)p; (void)v; }

// ---- CPU running ----
static void zcpu_start(zcpu *c, uint8_t (*rd)(void *, uint16_t), void (*wr)(void *, uint16_t, uint8_t),
	uint8_t (*in)(z80 *, uint8_t), void (*out)(z80 *, uint8_t, uint8_t))
{
	memset(c, 0, sizeof(*c));
	z80_init(&c->z);
	c->z.read_byte = rd;
	c->z.write_byte = wr;
	c->z.port_in = in;
	c->z.port_out = out;
	c->z.int_data = 0xff;
}

static void zcpu_run(zcpu *c, int cycles)
{
	c->start = c->z.cyc;
	c->len = cycles;
	unsigned long end = c->z.cyc + (unsigned long)cycles;
	if (c->reset) {
		c->z.cyc = end;
		return;
	}
	z80_run(&c->z, end, &c->hold, 1);   // (hold cleared when acknowledged)
}

static int slice_of(int total, int s) { return total * (s + 1) / SLICES - total * s / SLICES; }

// the main CPU's slice then the sub's. MAME runs the main with a perfect quantum: the sub is never ahead of it. Here the
// main runs its slice first; a write of the main to the shared RAM ends its run there, the sub runs up to the main's time,
// then the main goes on. Without that the sub saw the main's later writes early: tnzs's command at ef10 taken and
// cleared (ff) before the main read it back, the main then waited for ever (a hit by an enemy froze the game)
static void run_main_sub(int mcyc, int scyc)
{
	zcpu *m = &s_main, *sb = &s_sub;
	m->start = m->z.cyc;
	m->len = mcyc;
	sb->start = sb->z.cyc;
	sb->len = scyc;
	const unsigned long mend = m->z.cyc + (unsigned long)mcyc, send = sb->z.cyc + (unsigned long)scyc;
	if (m->reset)
		m->z.cyc = mend;
	else
		for (;;) {
			z80_break = 0;
			z80_run(&m->z, mend, &m->hold, 1);
			if (!z80_break)
				break;
			z80_break = 0;
			unsigned long st = sb->start + (unsigned long)(m->z.cyc - m->start);   // (the same clock)
			if ((long)(st - send) > 0)
				st = send;
			if (!sb->reset)
				z80_run(&sb->z, st, &sb->hold, 1);
			if ((long)(m->z.cyc - mend) >= 0)
				break;
		}
	if (sb->reset)
		sb->z.cyc = send;
	else
		z80_run(&sb->z, send, &sb->hold, 1);
}

// ---- video ----
static void pal_convert(const uint8_t *pal)
{
	uint32_t *out = rbx_video.palette;
	switch (s_m->pal) {
	case PAL_PROM:
		memcpy(out, s_prom_rgb, sizeof(s_prom_rgb));
		return;
	case PAL_JPOPNICS:
		for (int i = 0; i < 1024; i++) {   // GBRx_444, big endian
			int c = pal[i * 2] << 8 | pal[i * 2 + 1];
			int g = (c >> 12) & 15, b = (c >> 8) & 15, r = (c >> 4) & 15;
			out[i] = (uint32_t)((r * 17) << 16 | (g * 17) << 8 | (b * 17));
		}
		return;
	default:
		for (int i = 0; i < 512; i++) {   // xRGB_555, little endian
			int c = pal[i * 2] | pal[i * 2 + 1] << 8;
			int r = (c >> 10) & 31, g = (c >> 5) & 31, b = c & 31;
			r = (r << 3) | (r >> 2); g = (g << 3) | (g >> 2); b = (b << 3) | (b >> 2);
			out[i] = (uint32_t)(r << 16 | g << 8 | b);
		}
		return;
	}
}

static void tz_render(void)
{
	pal_convert(s_snap_pal);
	uint16_t *bm = rbx_video.bitmap;
	int h = rbx_video.height;
	for (int y = 0; y < h; y++)
		for (int x = 0; x < SCREEN_W; x++)
			bm[y * rbx_video.pitch + x] = 0x1f0;
	x1_draw(s_snap, bm, rbx_video.pitch, SCREEN_W, h, 0, VIS_Y0, LINES, VIS_Y1, 0x800);
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap->ctrl, s_x1->ctrl, sizeof(s_x1->ctrl));
	memcpy(s_snap->ylow, s_x1->ylow, sizeof(s_x1->ylow));
	memcpy(s_snap->code, s_x1->code, 0x1000 * sizeof(uint16_t));
	s_snap->bgflag = s_x1->bgflag;
	memcpy(s_snap_pal, s_pal, sizeof(s_pal));
	rbx_video_frame_done();
	x1_tnzs_eof(s_x1);
}

// ---- init / frame ----
static const machine_t *find_machine(const char *name)
{
	for (size_t i = 0; i < sizeof(s_machines) / sizeof(s_machines[0]); i++)
		if (!strcmp(s_machines[i].name, name))
			return &s_machines[i];
	return NULL;
}

static void tz_reset(void)
{
	memset(s_ram_bank, 0, sizeof(s_ram_bank));
	memset(s_share, 0, sizeof(s_share));
	memset(s_sub_ram, 0, sizeof(s_sub_ram));
	memset(s_audio_ram, 0, sizeof(s_audio_ram));
	if (s_m->pal != PAL_PROM)
		memset(s_pal, 0, sizeof(s_pal));
	x1_reset(s_x1);
	zcpu_start(&s_main, main_read, main_write, main_in, main_out);
	zcpu_start(&s_sub, sub_read, sub_write, main_in, main_out);
	zcpu_start(&s_audio, audio_read, audio_write, audio_in, audio_out);
	zcpu_start(&s_scpu, scpu_read, scpu_write, scpu_in, scpu_out);
	s_ramrom = 2;
	s_subbank = 0;
	s_audiobank = 0;
	update_maps();
	s_soundlatch = 0;
	s_input_select = 0;
	s_csport_sel = 0;
	if (s_m->mcu)
		mcs48_reset(&s_mcu);
	s_mcu_debt = 0;
	s_mcu_frac = 0;
	s_spin_x = s_spin_y = s_spin_startx = s_spin_starty = 0;
	s_spin_resetx = s_spin_resety = 0;
	a2_mcu_reset();
	s_mcu_readcredits = 0;
	s_insertcoin = 0;
	rbx_ym_reset(0);
	s_ym_irq = 0;
	s_ym_frac = 0;
	s_ym_done = 0;
	s_mix_n = 0;
	s_dac = 0;
}

static void tz_exit(void);

static void layout_tnzs(x1_layout *l, uint32_t size)
{
	static const uint8_t xo[16] = { 0, 1, 2, 3, 4, 5, 6, 7, 64, 65, 66, 67, 68, 69, 70, 71 };
	static const uint8_t yo[16] = { 0, 8, 16, 24, 32, 40, 48, 56, 128, 136, 144, 152, 160, 168, 176, 184 };
	uint32_t q = size / 4 * 8;
	l->total = size / 4 / 32;
	for (int p = 0; p < 4; p++)
		l->planes[p] = q * (uint32_t)(3 - p);
	for (int i = 0; i < 16; i++) { l->xoffs[i] = xo[i]; l->yoffs[i] = yo[i]; }
	l->charincrement = 32 * 8;
}

static void layout_insectx(x1_layout *l)
{
	l->total = 8192;
	l->planes[0] = 8; l->planes[1] = 0; l->planes[2] = 8192 * 64 * 8 + 8; l->planes[3] = 8192 * 64 * 8;
	for (int i = 0; i < 8; i++) {
		l->xoffs[i] = (uint32_t)i; l->xoffs[i + 8] = (uint32_t)(8 * 16 + i);
		l->yoffs[i] = (uint32_t)(i * 16); l->yoffs[i + 8] = (uint32_t)((16 + i) * 16);
	}
	l->charincrement = 64 * 8;
}

static int tz_init(const rbx_game *g)
{
	s_m = find_machine(g->machine);
	if (!s_m) {
		rbx_fail("%s: machine %s is not emulated", g->set, g->machine);
		return -1;
	}
	s_main_rom = rbx_region("maincpu", &s_main_size);
	s_sub_rom = rbx_region("sub", &s_sub_size);
	uint32_t gsize = 0, msize = 0;
	uint8_t *gfx = rbx_region("gfx1", &gsize);
	if (!s_main_rom || !s_sub_rom || !gfx || s_main_size < 0x8000 || s_sub_size < 0x8000) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	s_has_audio = s_m->kind == TZ_TNZSB || s_m->kind == TZ_KABUKIZ;
	s_has_scpu = s_m->kind == TZ_KAGEKI;
	s_audio_rom = s_has_audio ? rbx_region("audiocpu", &s_audio_size) : NULL;
	s_scpu_rom = s_has_scpu ? rbx_region("samples", &s_scpu_size) : NULL;
	if ((s_has_audio && !s_audio_rom) || (s_has_scpu && !s_scpu_rom)) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	if (s_m->mcu) {
		s_mcu_rom = rbx_region("mcu", &msize);
		if (!s_mcu_rom || msize < 0x800) {
			rbx_fail("%s: MCU ROM missing", g->set);
			return -1;
		}
		mcs48_init(&s_mcu, MCS48_I8742);
		s_mcu.rom = s_mcu_rom;
		s_mcu.port_r = mcu_port_r;
		s_mcu.port_w = mcu_port_w;
		s_mcu.test_r = mcu_test_r;
	}
	if (s_m->pal == PAL_PROM) {
		uint32_t psize;
		uint8_t *prom = rbx_region("proms", &psize);
		if (!prom || psize < 0x400) {
			rbx_fail("%s: color PROMs missing", g->set);
			return -1;
		}
		for (int i = 0; i < 512; i++) {
			int c = prom[i] << 8 | prom[i + 512];
			int r = (c >> 10) & 31, gg = (c >> 5) & 31, b = c & 31;
			r = (r << 3) | (r >> 2); gg = (gg << 3) | (gg >> 2); b = (b << 3) | (b >> 2);
			s_prom_rgb[i] = (uint32_t)(r << 16 | gg << 8 | b);
		}
	}
	x1_layout lay;
	memset(&lay, 0, sizeof(lay));
	if (s_m->insectx_gfx)
		layout_insectx(&lay);
	else
		layout_tnzs(&lay, gsize);
	s_x1 = (x1_001_t *)calloc(1, sizeof(x1_001_t));
	s_snap = (x1_001_t *)calloc(1, sizeof(x1_001_t));
	if (!s_x1 || !s_snap || x1_init_layout(s_x1, gfx, gsize, 32, &lay)) {
		rbx_fail("%s: out of memory (graphics)", g->set);
		tz_exit();
		return -1;
	}
	s_x1->fg_flipyoffs = -0x12;
	s_x1->fg_noflipyoffs = 0x0e;
	s_x1->bg_flipyoffs = 0x1;
	s_x1->bg_noflipyoffs = -0x1;
	*s_snap = *s_x1;
	s_port_in[0] = rbx_port_find("IN0");
	s_port_in[1] = rbx_port_find("IN1");
	s_port_in[2] = rbx_port_find("IN2");
	s_port_dswa = rbx_port_find("DSWA");
	s_port_dswb = rbx_port_find("DSWB");
	s_port_coin1 = rbx_port_find("COIN1");
	s_port_coin2 = rbx_port_find("COIN2");

	rbx_ym_config yc;
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM2203;
	yc.clock = YM_CLOCK;
	s_dac_gain = 0;
	switch (s_m->kind) {
	case TZ_TNZSB:
	case TZ_KABUKIZ:
		yc.gain[0] = yc.gain[1] = yc.gain[2] = 256;
		yc.gain[3] = 512;
		yc.irq = ym_irq_cb;
		if (s_m->kind == TZ_KABUKIZ) {
			yc.port_a_write = kabukiz_bank_w;
			yc.port_b_write = dac_w;
			s_dac_gain = 128 * 256 / 128;   // 0.5 of full scale (value -128..127 -> 1/128 per step)
		}
		break;
	case TZ_KAGEKI:
		yc.gain[0] = yc.gain[1] = yc.gain[2] = 38;
		yc.gain[3] = 90;
		yc.port_a_read = kageki_csport_r;
		yc.port_b_write = kageki_csport_w;
		s_dac_gain = 128 * 256 / 128;
		break;
	case TZ_JPOPNICS:
		yc.type = RBX_YM2151;
		yc.gain[0] = yc.gain[1] = 77;
		break;
	default:
		yc.gain[0] = yc.gain[1] = yc.gain[2] = yc.gain[3] = 77;
		yc.port_a_read = ym_dswa;
		yc.port_b_read = ym_dswb;
		break;
	}
	// (the YM's samples made on the worker: rbx_ym_async_take at the frame's end)
	if (rbx_ym_init(0, &yc) || rbx_ym_set_async(0, 1)) {
		rbx_fail("%s: sound init failed", g->set);
		tz_exit();
		return -1;
	}
	s_mix_cap = (int)(rbx_ym_rate(0) / 50) + 64;
	s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	if (!s_mix || rbx_audio_setup(rbx_ym_rate(0)) ||
		rbx_video_setup(SCREEN_W, VIS_Y1 - VIS_Y0 + 1, s_m->pal == PAL_JPOPNICS ? 1024 : 512)) {
		tz_exit();
		return -1;
	}
	tz_reset();
	return 0;
}

static void tz_exit(void)
{
	if (s_x1)
		x1_exit(s_x1);
	free(s_x1);
	free(s_snap);
	s_x1 = s_snap = NULL;
	free(s_mix);
	s_mix = NULL;
	rbx_ym_exit(0);
}

static void tz_frame(void)
{
	rbx_audio.frame_mhz = s_m->rate_mhz;   // (the board's frame rate: a frame's sound is played in a 60 Hz frame, as its picture)
	const uint32_t rate = s_m->rate_mhz;
	if (s_m->spinner)
		spinner_frame();
	// the frame starts at vblank: MAME updates the inputs there, right before the vblank IRQs (kageki's coin edge
	// detection depends on that order)
	for (int line = 0; line < LINES; line++) {
		if (line == 0) {
			vblank_capture();
			if (s_m->kind == TZ_ARKNOID2) {
				int coin = (port8(s_port_coin1) & 1) | (port8(s_port_coin2) & 1) << 1 | (port8(s_port_in[2]) & 3) << 2;
				a2_handle_coins(coin ^ 0x0c);
			}
			s_main.hold = 1;
			s_sub.hold = 1;
		}
		int mc = rbx_line_cycles(&s_main.frac, Z80_CLOCK, rate, LINES);
		int sc = rbx_line_cycles(&s_sub.frac, Z80_CLOCK, rate, LINES);
		int ac = s_has_audio ? rbx_line_cycles(&s_audio.frac, Z80_CLOCK, rate, LINES) : 0;
		int kc = s_has_scpu ? rbx_line_cycles(&s_scpu.frac, Z80_CLOCK / 2, rate, LINES) : 0;
		int uc = s_m->mcu ? rbx_line_cycles(&s_mcu_frac, MCU_CLOCK / 15, rate, LINES) : 0;
		int yl = rbx_line_cycles(&s_ym_frac, YM_CLOCK, rate, LINES);
		uint32_t ym_line_start = s_ym_done;
		for (int s = 0; s < SLICES; s++) {
			s_ym_slice_start = ym_line_start + (uint32_t)(yl * s / SLICES);
			s_ym_slice_len = (uint32_t)slice_of(yl, s);
			run_main_sub(slice_of(mc, s), slice_of(sc, s));
			if (s_has_audio)
				zcpu_run(&s_audio, slice_of(ac, s));
			if (s_has_scpu)
				zcpu_run(&s_scpu, slice_of(kc, s));
			if (s_m->mcu) {
				int n = slice_of(uc, s) - s_mcu_debt;
				s_mcu_debt = n > 0 ? mcs48_execute(&s_mcu, n) - n : -n;
			}
			ym_run_to(s_ym_slice_start + s_ym_slice_len);
		}
	}
	rbx_ym_async_take(0, s_mix, s_mix_n);
	for (int i = 0; i < s_mix_n; i++)
		rbx_audio_put(s_mix[i]);
	memset(s_mix, 0, sizeof(int32_t) * (size_t)s_mix_n);
	s_mix_n = 0;
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

static int tz_state(void *ctx, int load)
{
	int rc = 0;
	rc |= zcpu_state(ctx, "main", &s_main, load);
	rc |= zcpu_state(ctx, "sub", &s_sub, load);
	rc |= zcpu_state(ctx, "audio", &s_audio, load);
	rc |= zcpu_state(ctx, "scpu", &s_scpu, load);
	rc |= rbx_state_io(ctx, "rambank", s_ram_bank, sizeof(s_ram_bank));
	rc |= rbx_state_io(ctx, "share", s_share, sizeof(s_share));
	rc |= rbx_state_io(ctx, "subram", s_sub_ram, sizeof(s_sub_ram));
	rc |= rbx_state_io(ctx, "audram", s_audio_ram, sizeof(s_audio_ram));
	rc |= rbx_state_io(ctx, "pal", s_pal, sizeof(s_pal));
	rc |= rbx_state_io(ctx, "x1ctrl", s_x1->ctrl, sizeof(s_x1->ctrl));
	rc |= rbx_state_io(ctx, "x1ylow", s_x1->ylow, sizeof(s_x1->ylow));
	rc |= rbx_state_io(ctx, "x1code", s_x1->code, sizeof(s_x1->code));
	rc |= rbx_state_io(ctx, "x1bg", &s_x1->bgflag, 1);
	if (s_m->mcu)
		rc |= rbx_state_io(ctx, "mcu", (uint8_t *)&s_mcu + MCS48_STATE_OFFSET, (uint32_t)MCS48_STATE_SIZE);
	int32_t misc[32] = {
		s_ramrom, s_subbank, s_audiobank, s_soundlatch, s_input_select, s_csport_sel, s_mcu_debt, s_ym_irq, s_dac,
		s_spin_x, s_spin_y, s_spin_startx, s_spin_starty, s_spin_resetx, s_spin_resety,
		s_mcu_initializing, s_mcu_coinage_init, s_mcu_command, s_mcu_readcredits, s_mcu_reportcoin, s_insertcoin,
		s_mcu_coinage[0], s_mcu_coinage[1], s_mcu_coinage[2], s_mcu_coinage[3], s_mcu_coins_a, s_mcu_coins_b,
		s_mcu_credits, 0, 0, 0, 0 };
	uint64_t fr[2] = { s_mcu_frac, s_ym_frac };
	rc |= rbx_state_io(ctx, "misc", misc, sizeof(misc));
	rc |= rbx_state_io(ctx, "frac", fr, sizeof(fr));
	rc |= rbx_ym_state(0, ctx, load);
	if (rc)
		return -1;
	if (load) {
		s_ramrom = misc[0]; s_subbank = misc[1]; s_audiobank = misc[2]; s_soundlatch = (uint8_t)misc[3];
		s_input_select = (uint8_t)misc[4]; s_csport_sel = misc[5]; s_mcu_debt = misc[6]; s_ym_irq = misc[7]; s_dac = misc[8];
		s_spin_x = (uint16_t)misc[9]; s_spin_y = (uint16_t)misc[10]; s_spin_startx = (uint16_t)misc[11];
		s_spin_starty = (uint16_t)misc[12]; s_spin_resetx = misc[13]; s_spin_resety = misc[14];
		s_mcu_initializing = misc[15]; s_mcu_coinage_init = misc[16]; s_mcu_command = misc[17];
		s_mcu_readcredits = misc[18]; s_mcu_reportcoin = misc[19]; s_insertcoin = misc[20];
		for (int i = 0; i < 4; i++) s_mcu_coinage[i] = (uint8_t)misc[21 + i];
		s_mcu_coins_a = (uint8_t)misc[25]; s_mcu_coins_b = (uint8_t)misc[26]; s_mcu_credits = (uint8_t)misc[27];
		s_mcu_frac = fr[0]; s_ym_frac = fr[1];
		update_maps();
	}
	return 0;
}

static int tz_save(void *ctx) { return tz_state(ctx, 0); }
static int tz_load(void *ctx) { return tz_state(ctx, 1); }
static int tz_supported(const char *machine) { return find_machine(machine) != NULL; }

const rbx_driver rbx_drv_tnzs = { "Taito TNZS", tz_init, tz_exit, tz_reset, tz_frame, tz_save, tz_load, tz_supported, tz_render };
