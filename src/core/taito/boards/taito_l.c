/*  Taito L System, ported from MAME src/mame/taito/taito_l.cpp (BSD-3-Clause, copyright-holders: Olivier Galibert).
    Memory maps, machine configurations, interrupt timing (scanline 0 / 120 / 240 IRQ levels through the TC0090LVC
    vectors, plgirls IM2 kludge) and sound routes follow MAME. The TC0090LVC (Z80 core with RAM / ROM banking,
    tilemaps, sprites, palette, bitmap) is video/tc0090lvc.c; the Z80s are cpu/z80, the YM chips sound/rbx_ym,
    the Champion Wrestler ADPCM sound/msm5205, the TC0140SYT / PC060HA machine/taitosnd.
    Not emulated: Puzznic sets with the 68705 MCU (puzznic, puzznicu, puzznicj).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/z80/z80.h"
#include "../machine/taitosnd.h"
#include "../../sound/rbx_ym.h"
#include "../../sound/msm5205.h"
#include "../video/tc0090lvc.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

enum { M_FHAWK, M_CHAMPWR, M_RAIMAIS, M_KURIKINT, M_EVILSTON, M_PLOTTING, M_PUZZNICI, M_HORSHOES, M_PALAMED };
enum { IO_NONE, IO_TC0220IOC, IO_TC0040IOC, IO_TC0510NIO };

typedef struct {
	const char *name;
	int kind;
	int ncpu;            // 1: main only, 2: main + audiocpu (dpram), 3: main + slave + sound CPU
	int ym;              // RBX_YM2203 / RBX_YM2610
	uint32_t ym_clock;
	int gain[4];
} machine_t;

static const machine_t s_machines[] = {
	{ "fhawk",    M_FHAWK,    3, RBX_YM2203, 3000000, { 51, 51, 51, 205 } },
	{ "champwr",  M_CHAMPWR,  3, RBX_YM2203, 3000000, { 51, 51, 51, 205 } },
	{ "raimais",  M_RAIMAIS,  3, RBX_YM2610, 8000000, { 192, 256, 256, 0 } },
	{ "kurikint", M_KURIKINT, 2, RBX_YM2203, 3000000, { 51, 51, 51, 205 } },
	{ "evilston", M_EVILSTON, 2, RBX_YM2203, 3000000, { 64, 64, 64, 205 } },
	{ "plotting", M_PLOTTING, 1, RBX_YM2203, 3332640, { 51, 51, 51, 205 } },
	{ "puzznici", M_PUZZNICI, 1, RBX_YM2203, 3332640, { 51, 51, 51, 205 } },
	{ "horshoes", M_HORSHOES, 1, RBX_YM2203, 3332640, { 51, 51, 51, 205 } },
	{ "palamed",  M_PALAMED,  1, RBX_YM2203, 3332640, { 51, 51, 51, 205 } },
	{ "cachat",   M_PALAMED,  1, RBX_YM2203, 3332640, { 51, 51, 51, 205 } },
};

#define LINES 256
#define MAIN_CLOCK 6665280      // 13.33056 MHz / 2
#define SUB_CLOCK  4000000      // 12 MHz / 3
#define MSM_CLOCK  384000
#define SLICES     4            // main / slave / sound interleave per scanline (MAME perfect quantum on the main CPU)

enum { CPU_MAIN, CPU_SLAVE, CPU_AUDIO };

typedef struct {
	z80 z;
	uint32_t clock;
	uint64_t frac;
	int hold;            // HOLD_LINE IRQ (vblank) until acknowledged
	int reset;           // held in reset
	unsigned long start; // cycle count at the start of the line
	int cycles;          // cycles of the current line
} cpu_t;

static const machine_t *s_m;
static tc0090lvc_t *s_lvc;
static tc0090lvc_view *s_view;
static cpu_t s_cpu[3];
static int s_ncpu, s_ym_owner;
static uint8_t *s_slave_rom, *s_audio_rom, *s_adpcm;
static uint32_t s_slave_size, s_audio_size, s_adpcm_size;
static uint8_t s_ram[0x2000], s_share[0x2000], s_dpram[0x800], s_slave_ram[0x2000], s_audio_ram[0x2000];
static void map_banks(void);
static tc0140syt_t s_syt;
static int s_irq_line, s_last_irq_level;
static int s_slave_bank, s_audio_bank, s_ym_irq, s_intl;
static uint8_t s_ioc_regs[8], s_ioc_port;
static int s_mux_sel;
static uint8_t s_ym_addr, s_ym_reg7, s_ym_portb;
// horshoes uPD4701A
static uint16_t s_upd_x, s_upd_y, s_upd_sx, s_upd_sy;
// champwr MSM5205
static msm5205_t s_msm;
static uint32_t s_adpcm_pos;
static int s_adpcm_data;
static int s_msm_gain;    // 1/256, volume from YM2203 port B times the route gain 0.80
static uint32_t s_msm_acc;
// sound mixing
static uint32_t s_ym_line, s_ym_done;
static uint64_t s_ym_frac;
static int32_t *s_mix;
static int s_mix_cap, s_mix_n;
static int s_port_dswa, s_port_dswb, s_port_in[3], s_port_an[2];
static uint32_t s_frame;
static int s_msm_starts;
static int s_dbg;   // RBX_DBG: periodic CPU / video state on stderr

// ---- sound chips: advance to the owner CPU's position in the line ----
static void ym_sync(void)
{
	cpu_t *c = &s_cpu[s_ym_owner];
	uint32_t done = (uint32_t)(c->z.cyc - c->start);
	uint32_t target = c->cycles > 0 ? (uint32_t)((uint64_t)s_ym_line * done / (uint32_t)c->cycles) : s_ym_line;
	if (target > s_ym_line)
		target = s_ym_line;
	if (target <= s_ym_done)
		return;
	int n = rbx_ym_advance(0, target - s_ym_done, s_mix + s_mix_n, s_mix_cap - s_mix_n);
	if (s_m->kind == M_CHAMPWR) {
		uint32_t yr = rbx_ym_rate(0);
		for (int i = 0; i < n; i++) {
			s_msm_acc += MSM_CLOCK;
			uint32_t k = s_msm_acc / yr;
			s_msm_acc -= k * yr;
			msm5205_advance(&s_msm, k);
			s_mix[s_mix_n + i] += (msm5205_output(&s_msm) * s_msm_gain) >> 8;
		}
	}
	s_mix_n += n;
	s_ym_done = target;
}

static void ym_w(int offset, uint8_t v)
{
	ym_sync();
	if (s_m->ym == RBX_YM2203) {
		// track the SSG port B output (champwr MSM5205 volume: MAME port_b_write_callback)
		if (!(offset & 1))
			s_ym_addr = v;
		else if (s_ym_addr == 0x07) {
			if ((v & 0x80) && !(s_ym_reg7 & 0x80))
				s_msm_gain = s_ym_portb * 205 / 255;
			s_ym_reg7 = v;
		} else if (s_ym_addr == 0x0f) {
			s_ym_portb = v;
			if (s_ym_reg7 & 0x80)
				s_msm_gain = v * 205 / 255;
		}
	}
	rbx_ym_write(0, (uint32_t)offset, v);
}

static uint8_t ym_r(int offset)
{
	ym_sync();
	return rbx_ym_read(0, (uint32_t)offset);
}

static void ym_irq_cb(int state) { s_ym_irq = state; }
static void ym_port_a_w(uint8_t data) { s_audio_bank = data & 3; map_banks(); }   // fhawk_state::portA_w
static uint8_t ym_port_a_r(void) { return (uint8_t)rbx_port_read(s_m->kind == M_PALAMED || !s_mux_sel ? s_port_dswa : s_port_dswb); }
static uint8_t ym_port_b_r(void)
{
	if (s_m->kind == M_PALAMED)
		return (uint8_t)rbx_port_read(s_port_dswb);
	return (uint8_t)rbx_port_read(s_mux_sel ? s_port_in[1] : s_port_in[0]);
}

static void msm_vck(void)
{
	// champwr_state::msm5205_vck
	if (s_adpcm_data != -1) {
		msm5205_data_w(&s_msm, (uint8_t)(s_adpcm_data & 0x0f));
		s_adpcm_data = -1;
	} else {
		s_adpcm_data = s_adpcm[s_adpcm_pos % s_adpcm_size];
		s_adpcm_pos = (s_adpcm_pos + 1) & 0x1ffff;
		msm5205_data_w(&s_msm, (uint8_t)(s_adpcm_data >> 4));
	}
}

// ---- I/O chips (TC0220IOC / TC0040IOC / TC0510NIO), ports DSWA DSWB IN0 IN1 - - - IN2 ----
static uint8_t ioc_port(int n)
{
	switch (n) {
	case 0: return (uint8_t)rbx_port_read(s_port_dswa);
	case 1: return (uint8_t)rbx_port_read(s_port_dswb);
	case 2: return (uint8_t)rbx_port_read(s_port_in[0]);
	case 3: return (uint8_t)rbx_port_read(s_port_in[1]);
	case 4: return s_ioc_regs[4];
	case 7: return (uint8_t)rbx_port_read(s_port_in[2]);
	}
	return 0xff;
}

static uint8_t ioc8_r(int offset) { return ioc_port(offset & 7); }
static void ioc8_w(int offset, uint8_t v) { s_ioc_regs[offset & 7] = v; }   // coin counters / lockout, watchdog
static uint8_t ioc40_r(int offset) { return (offset & 1) ? 0 : ioc_port(s_ioc_port); }
static void ioc40_w(int offset, uint8_t v)
{
	if (offset & 1)
		s_ioc_port = v;
	else if (s_ioc_port < 8)
		s_ioc_regs[s_ioc_port] = v;
}

// ---- MB8421 dual-port RAM: right = main CPU, left = second CPU; INTL (evilston) = audiocpu NMI ----
static void set_intl(int state)
{
	if (s_m->kind != M_EVILSTON)
		return;
	if (state && !s_intl) {
		z80_gen_nmi(&s_cpu[CPU_AUDIO].z);
	}
	s_intl = state;
}
static uint8_t dpram_right_r(uint16_t o) { return s_dpram[o & 0x7ff]; }
static void dpram_right_w(uint16_t o, uint8_t v)
{
	o &= 0x7ff;
	s_dpram[o] = v;
	if (o == 0x7fe)
		set_intl(1);
}
static uint8_t dpram_left_r(uint16_t o)
{
	o &= 0x7ff;
	if (o == 0x7fe)
		set_intl(0);
	return s_dpram[o];
}
static void dpram_left_w(uint16_t o, uint8_t v) { s_dpram[o & 0x7ff] = v; }

// ---- main CPU (TC0090LVC) ----
static void irq_enable_w(uint8_t v)
{
	s_lvc->irq_enable = v;
	if ((v & (1 << s_last_irq_level)) == 0) {   // fix Plotting test mode
		s_irq_line = 0;
		s_cpu[CPU_MAIN].z.int_pending = 0;
	}
}

static uint8_t upd_read_xy(int o)
{
	uint16_t d = (o & 2) ? (uint16_t)((s_upd_y - s_upd_sy) & 0xfff) : (uint16_t)((s_upd_x - s_upd_sx) & 0xfff);
	return (o & 1) ? (uint8_t)(d >> 8) : (uint8_t)d;
}

static uint8_t main_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x8000 || a >= 0xc000)
		return tc0090lvc_read(s_lvc, a);
	switch (s_m->kind) {
	case M_FHAWK:
		return a < 0xa000 ? s_share[a & 0x1fff] : s_ram[a & 0x1fff];
	case M_CHAMPWR:
		return a < 0xa000 ? s_ram[a & 0x1fff] : s_share[a & 0x1fff];
	case M_RAIMAIS:
		if (a < 0x8800) return dpram_right_r(a);
		if (a < 0x8802) return ioc40_r(a & 1);
		if (a == 0x8c01) return tc0140syt_master_comm_r(&s_syt);
		if (a >= 0xa000) return s_ram[a & 0x1fff];
		return 0;
	case M_KURIKINT:
	case M_EVILSTON:
		if (a < 0xa000) return s_ram[a & 0x1fff];
		if (a < 0xa800) return dpram_right_r(a);
		if (s_m->kind == M_KURIKINT && a < 0xa802) return ioc40_r(a & 1);
		if (s_m->kind == M_EVILSTON && a < 0xa808) return ioc8_r(a & 7);
		return 0;
	default:
		break;
	}
	// one-CPU boards
	if (a < 0xa000)
		return s_ram[a & 0x1fff];
	if (a < 0xa004) {
		if (s_m->kind == M_PALAMED)
			return ym_r(a & 1);
		s_mux_sel = (a >> 1) & 1;   // extport_select_and_ym2203_r
		return ym_r(a & 1);
	}
	if (s_m->kind == M_PALAMED) {
		if (a >= 0xa800 && a < 0xa803) return (uint8_t)rbx_port_read(s_port_in[a - 0xa800]);   // i8255 ports A..C
		if (a == 0xa803) return 0xff;
		return 0;
	}
	if (s_m->kind == M_HORSHOES) {
		if (a == 0xa802) { s_upd_sx = s_upd_x; return 0; }
		if (a == 0xa803) { s_upd_sy = s_upd_y; return 0; }
		if ((a & ~0x000c) == 0xa800) return upd_read_xy((a >> 2) & 3);
		return 0;
	}
	if (s_m->kind == M_PUZZNICI && a == 0xb801)
		return 0x01;   // mcu_control_r
	return 0;
}

static void main_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0xc000) {
		if (a >= 0xff00 && (a & 0x0f) == 3)
			irq_enable_w(v);
		else {
			tc0090lvc_write(s_lvc, a, v);
			if (a >= 0xff00 && (a & 0x0f) == 8)
				map_banks();   // (the ROM bank)
		}
		return;
	}
	if (a < 0x8000)
		return;
	switch (s_m->kind) {
	case M_FHAWK:
		if (a < 0xa000) s_share[a & 0x1fff] = v; else s_ram[a & 0x1fff] = v;
		return;
	case M_CHAMPWR:
		if (a < 0xa000) s_ram[a & 0x1fff] = v; else s_share[a & 0x1fff] = v;
		return;
	case M_RAIMAIS:
		if (a < 0x8800) dpram_right_w(a, v);
		else if (a < 0x8802) ioc40_w(a & 1, v);
		else if (a == 0x8c00) tc0140syt_master_port_w(&s_syt, v);
		else if (a == 0x8c01) tc0140syt_master_comm_w(&s_syt, v);
		else if (a >= 0xa000) s_ram[a & 0x1fff] = v;
		return;
	case M_KURIKINT:
	case M_EVILSTON:
		if (a < 0xa000) s_ram[a & 0x1fff] = v;
		else if (a < 0xa800) dpram_right_w(a, v);
		else if (s_m->kind == M_KURIKINT && a < 0xa802) ioc40_w(a & 1, v);
		else if (s_m->kind == M_EVILSTON && a < 0xa808) ioc8_w(a & 7, v);
		return;
	default:
		break;
	}
	if (a < 0xa000) { s_ram[a & 0x1fff] = v; return; }
	if (a < 0xa004) { ym_w(a & 1, v); return; }
	if (s_m->kind == M_HORSHOES && a == 0xb802) {   // bankg_w
		if (s_dbg && s_lvc->tile_or != ((uint32_t)v << 12)) fprintf(stderr, "bank %02x f%u cyc %lu\n", v, s_frame, s_cpu[0].z.cyc - s_cpu[0].start);
		s_lvc->tile_or = (uint32_t)v << 12;
		return;
	}
	// a800 watchdog / b800 bc00 control registers, i8255 control: ignored
}

// ---- slave CPU (fhawk, champwr, raimais) ----
static uint8_t slave_read(void *u, uint16_t a)
{
	(void)u;
	if (s_m->kind == M_RAIMAIS) {
		if (a < 0xc000) return a < s_slave_size ? s_slave_rom[a] : 0;
		if (a < 0xe000) return s_slave_ram[a & 0x1fff];
		if (a < 0xe800) return dpram_left_r(a);
		return 0;
	}
	if (a < 0x8000) return s_slave_rom[a % s_slave_size];
	if (a < 0xc000) {
		uint32_t banks = s_slave_size / 0x4000;
		return s_slave_rom[((uint32_t)s_slave_bank % banks) * 0x4000 + (a & 0x3fff)];
	}
	if (s_m->kind == M_FHAWK) {
		if (a == 0xc801) return tc0140syt_master_comm_r(&s_syt);
		if (a >= 0xd000 && a < 0xd008) return ioc8_r(a & 7);
		if (a >= 0xe000) return s_share[a & 0x1fff];
		return 0;
	}
	// champwr
	if (a < 0xe000) return s_share[a & 0x1fff];
	if (a < 0xe008) return ioc8_r(a & 7);
	if (a == 0xe801) return tc0140syt_master_comm_r(&s_syt);
	if (a == 0xf000) return (uint8_t)s_slave_bank;
	return 0;
}

static void slave_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (s_m->kind == M_RAIMAIS) {
		if (a >= 0xc000 && a < 0xe000) s_slave_ram[a & 0x1fff] = v;
		else if (a >= 0xe000 && a < 0xe800) dpram_left_w(a, v);
		return;
	}
	if (s_m->kind == M_FHAWK) {
		if (a == 0xc000) { s_slave_bank = v & 0xf; map_banks(); }
		else if (a == 0xc800) tc0140syt_master_port_w(&s_syt, v);
		else if (a == 0xc801) tc0140syt_master_comm_w(&s_syt, v);
		else if (a >= 0xd000 && a < 0xd008) ioc8_w(a & 7, v);
		else if (a >= 0xe000) s_share[a & 0x1fff] = v;
		return;
	}
	if (a >= 0xc000 && a < 0xe000) s_share[a & 0x1fff] = v;
	else if (a >= 0xe000 && a < 0xe008) ioc8_w(a & 7, v);
	else if (a == 0xe800) tc0140syt_master_port_w(&s_syt, v);
	else if (a == 0xe801) tc0140syt_master_comm_w(&s_syt, v);
	else if (a == 0xf000) { s_slave_bank = v & 0xf; map_banks(); }
}

// ---- sound CPU / second CPU ----
static uint8_t audio_read(void *u, uint16_t a)
{
	(void)u;
	switch (s_m->kind) {
	case M_KURIKINT:
	case M_EVILSTON:
		if (a < (s_m->kind == M_KURIKINT ? 0x8000 : 0xc000)) return a < s_audio_size ? s_audio_rom[a] : 0;
		if (a >= 0xc000 && a < 0xe000) return s_audio_ram[a & 0x1fff];
		if (a >= 0xe000 && a < 0xe800) return dpram_left_r(a);
		if (a >= 0xe800 && a < 0xe802) return ym_r(a & 1);
		if (s_m->kind == M_EVILSTON && a >= 0xf000 && a < 0xf800) return s_audio_rom[a & 0x7ff];   // audiobank entry 0
		return 0;
	default:
		break;
	}
	if (a < 0x4000) return s_audio_rom[a];
	if (a < 0x8000) {
		uint32_t banks = s_audio_size / 0x4000;
		return s_audio_rom[((uint32_t)s_audio_bank % banks) * 0x4000 + (a & 0x3fff)];
	}
	switch (s_m->kind) {
	case M_FHAWK:
		if (a < 0xa000) return s_audio_ram[a & 0x1fff];
		if (a == 0xe001) return tc0140syt_slave_comm_r(&s_syt);
		if (a == 0xf000 || a == 0xf001) return ym_r(a & 1);
		return 0;
	case M_CHAMPWR:
		if (a < 0x9000) return s_audio_ram[a & 0xfff];
		if (a < 0x9002) return ym_r(a & 1);
		if (a == 0xa001) return tc0140syt_slave_comm_r(&s_syt);
		return 0;
	case M_RAIMAIS:
		if (a >= 0xc000 && a < 0xe000) return s_audio_ram[a & 0x1fff];
		if (a >= 0xe000 && a < 0xe004) return ym_r(a & 3);
		if (a == 0xe201) return tc0140syt_slave_comm_r(&s_syt);
		return 0;
	}
	return 0;
}

static void audio_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	switch (s_m->kind) {
	case M_KURIKINT:
	case M_EVILSTON:
		if (a >= 0xc000 && a < 0xe000) s_audio_ram[a & 0x1fff] = v;
		else if (a >= 0xe000 && a < 0xe800) dpram_left_w(a, v);
		else if (a >= 0xe800 && a < 0xe802) ym_w(a & 1, v);
		return;
	case M_FHAWK:
		if (a >= 0x8000 && a < 0xa000) s_audio_ram[a & 0x1fff] = v;
		else if (a == 0xe000) tc0140syt_slave_port_w(&s_syt, v);
		else if (a == 0xe001) tc0140syt_slave_comm_w(&s_syt, v);
		else if (a == 0xf000 || a == 0xf001) ym_w(a & 1, v);
		return;
	case M_CHAMPWR:
		if (a >= 0x8000 && a < 0x9000) s_audio_ram[a & 0xfff] = v;
		else if (a >= 0x9000 && a < 0x9002) ym_w(a & 1, v);
		else if (a == 0xa000) tc0140syt_slave_port_w(&s_syt, v);
		else if (a == 0xa001) tc0140syt_slave_comm_w(&s_syt, v);
		else if (a == 0xb000) { ym_sync(); s_adpcm_pos = ((s_adpcm_pos & 0x00ffff) | ((uint32_t)v << 16)) & 0x1ffff; }
		else if (a == 0xc000) { ym_sync(); s_adpcm_pos = (s_adpcm_pos & 0xff00ff) | ((uint32_t)v << 8); }
		else if (a == 0xd000) { ym_sync(); msm5205_reset_w(&s_msm, 0); s_msm_starts++; }
		else if (a == 0xe000) { ym_sync(); msm5205_reset_w(&s_msm, 1); s_adpcm_pos &= 0x1ff00; }
		return;
	case M_RAIMAIS:
		if (a >= 0xc000 && a < 0xe000) s_audio_ram[a & 0x1fff] = v;
		else if (a >= 0xe000 && a < 0xe004) ym_w(a & 3, v);
		else if (a == 0xe200) tc0140syt_slave_port_w(&s_syt, v);
		else if (a == 0xe201) tc0140syt_slave_comm_w(&s_syt, v);
		else if (a == 0xf200) { s_audio_bank = v & 3; map_banks(); }   // sound_bankswitch_w
		return;
	}
}

// ---- Z80 read maps: pages read straight from memory (the same bytes the read handlers above return) ----
static const uint8_t *s_map[3][256];

static void map_range(const uint8_t **map, uint32_t start, uint32_t end, const uint8_t *mem)
{
	for (uint32_t a = start; a < end; a += 0x100)
		map[a >> 8] = mem ? mem + (a - start) : NULL;
}

// ROM from 0 up to end, the pages the ROM holds (the handlers read 0 past it)
static void map_rom(const uint8_t **map, uint32_t end, const uint8_t *rom, uint32_t size)
{
	for (uint32_t a = 0; a < end && a + 0x100 <= size; a += 0x100)
		map[a >> 8] = rom + a;
}

// the banked windows: main 6000-7fff (TC0090LVC ROM bank), slave 8000-bfff, sound 4000-7fff
static void map_banks(void)
{
	const tc0090lvc_t *t = s_lvc;
	if (t->rom_mask >= 0x1fff)
		map_range(s_map[CPU_MAIN], 0x6000, 0x8000, t->rom + ((((uint32_t)t->rom_bank << 13)) & t->rom_mask));
	if (s_slave_rom && s_m->kind != M_RAIMAIS && s_slave_size >= 0x8000)
		map_range(s_map[CPU_SLAVE], 0x8000, 0xc000,
		          s_slave_rom + ((uint32_t)s_slave_bank % (s_slave_size / 0x4000)) * 0x4000);
	if (s_audio_rom && s_m->kind != M_KURIKINT && s_m->kind != M_EVILSTON && s_audio_size >= 0x4000)
		map_range(s_map[CPU_AUDIO], 0x4000, 0x8000,
		          s_audio_rom + ((uint32_t)s_audio_bank % (s_audio_size / 0x4000)) * 0x4000);
}

static void map_build(void)
{
	memset(s_map, 0, sizeof(s_map));
	const uint8_t **m = s_map[CPU_MAIN];
	if (s_lvc->rom_mask >= 0x5fff)
		map_range(m, 0x0000, 0x6000, s_lvc->rom);
	switch (s_m->kind) {
	case M_FHAWK: map_range(m, 0x8000, 0xa000, s_share); map_range(m, 0xa000, 0xc000, s_ram); break;
	case M_CHAMPWR: map_range(m, 0x8000, 0xa000, s_ram); map_range(m, 0xa000, 0xc000, s_share); break;
	case M_RAIMAIS: map_range(m, 0xa000, 0xc000, s_ram); break;
	default: map_range(m, 0x8000, 0xa000, s_ram); break;   // (kurikint, evilston, the one-CPU boards)
	}
	m = s_map[CPU_SLAVE];
	if (s_slave_rom && s_m->kind == M_RAIMAIS) {
		map_rom(m, 0xc000, s_slave_rom, s_slave_size);
		map_range(m, 0xc000, 0xe000, s_slave_ram);
	} else if (s_slave_rom && s_slave_size >= 0x8000) {
		map_range(m, 0x0000, 0x8000, s_slave_rom);
		if (s_m->kind == M_FHAWK)
			map_range(m, 0xe000, 0x10000, s_share);
		else if (s_m->kind == M_CHAMPWR)
			map_range(m, 0xc000, 0xe000, s_share);
	}
	m = s_map[CPU_AUDIO];
	if (s_audio_rom) {
		if (s_m->kind == M_KURIKINT || s_m->kind == M_EVILSTON) {
			map_rom(m, s_m->kind == M_KURIKINT ? 0x8000 : 0xc000, s_audio_rom, s_audio_size);
			map_range(m, 0xc000, 0xe000, s_audio_ram);
			if (s_m->kind == M_EVILSTON && s_audio_size >= 0x800)
				for (uint32_t a = 0xf000; a < 0xf800; a += 0x100)
					m[a >> 8] = s_audio_rom + (a & 0x7ff);
		} else {
			map_rom(m, 0x4000, s_audio_rom, s_audio_size);
			if (s_m->kind == M_FHAWK)
				map_range(m, 0x8000, 0xa000, s_audio_ram);
			else if (s_m->kind == M_CHAMPWR)
				map_range(m, 0x8000, 0x9000, s_audio_ram);
			else if (s_m->kind == M_RAIMAIS)
				map_range(m, 0xc000, 0xe000, s_audio_ram);
		}
	}
	map_banks();
}

static uint8_t z80_in(z80 *z, uint8_t p) { (void)z; (void)p; return 0xff; }
static void z80_out(z80 *z, uint8_t p, uint8_t v) { (void)z; (void)p; (void)v; }

static void cpu_bind(int i)
{
	z80 *z = &s_cpu[i].z;
	z->read_byte = i == CPU_MAIN ? main_read : i == CPU_SLAVE ? slave_read : audio_read;
	z->write_byte = i == CPU_MAIN ? main_write : i == CPU_SLAVE ? slave_write : audio_write;
	z->port_in = z80_in;
	z->port_out = z80_out;
	z->userdata = NULL;
	z->read_map = s_map[i];
}

static void cpu_start(int i)
{
	unsigned long cyc = s_cpu[i].z.cyc;
	z80_init(&s_cpu[i].z);
	cpu_bind(i);
	s_cpu[i].z.cyc = cyc;
}

static void syt_nmi(int state)
{
	if (state)
		z80_gen_nmi(&s_cpu[CPU_AUDIO].z);
}

static void syt_reset(int state)
{
	cpu_t *c = &s_cpu[CPU_AUDIO];
	if (state && !c->reset)
		cpu_start(CPU_AUDIO);
	c->reset = state;
}

static void cpu_run(int i, unsigned long target)
{
	cpu_t *c = &s_cpu[i];
	z80 *z = &c->z;
	if (c->reset) {
		if ((long)(z->cyc - target) < 0)
			z->cyc = target;
		return;
	}
	if (i == CPU_MAIN) {
		while ((long)(z->cyc - target) < 0) {
			z->int_pending = s_irq_line ? 1 : 0;
			z->int_data = s_lvc->irq_vector[s_last_irq_level];
			z80_step(z);
			if (s_irq_line && !z->int_pending)
				s_irq_line = 0;   // irq_callback: acknowledged, line cleared
		}
		return;
	}
	int ymline = i == s_ym_owner && i != CPU_MAIN;
	while ((long)(z->cyc - target) < 0) {
		int line = c->hold || (ymline && s_ym_irq);
		z->int_pending = line ? 1 : 0;
		z->int_data = 0xff;
		z80_step(z);
		if (line && !z->int_pending)
			c->hold = 0;
	}
}

// ---- video ----
static void tl_render(void)
{
	tc0090lvc_render(s_view, rbx_video.bitmap, rbx_video.pitch, rbx_video.palette);
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	tc0090lvc_capture(s_lvc, s_view);
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

static void tl_reset(void)
{
	memset(s_ram, 0, sizeof(s_ram));
	memset(s_share, 0, sizeof(s_share));
	memset(s_dpram, 0, sizeof(s_dpram));
	memset(s_slave_ram, 0, sizeof(s_slave_ram));
	memset(s_audio_ram, 0, sizeof(s_audio_ram));
	tc0090lvc_reset(s_lvc);
	for (int i = 0; i < 3; i++) {
		memset(&s_cpu[i].z, 0, sizeof(z80));
		cpu_start(i);
		s_cpu[i].frac = 0;
		s_cpu[i].hold = 0;
		s_cpu[i].reset = 0;
	}
	tc0140syt_reset(&s_syt);
	s_syt.reset_line = 0;
	s_irq_line = s_last_irq_level = 0;
	s_slave_bank = 0;
	s_audio_bank = (s_m->kind == M_FHAWK || s_m->kind == M_CHAMPWR) ? 1 : 0;
	map_build();
	s_ym_irq = s_intl = 0;
	memset(s_ioc_regs, 0, sizeof(s_ioc_regs));
	s_ioc_port = 0;
	s_mux_sel = 0;
	s_ym_addr = s_ym_reg7 = s_ym_portb = 0;
	s_upd_x = s_upd_y = s_upd_sx = s_upd_sy = 0;
	rbx_ym_reset(0);
	msm5205_reset(&s_msm);
	s_adpcm_pos = 0;
	s_adpcm_data = -1;
	s_msm_gain = 205;
	s_msm_acc = 0;
	s_ym_frac = 0;
	s_mix_n = 0;
	s_frame = 0;
}

static void tl_exit(void);

static int tl_init(const rbx_game *g)
{
	s_dbg = getenv("RBX_DBG") != NULL;
	s_m = find_machine(g->machine);
	if (!s_m) {
		rbx_fail("%s: machine %s is not emulated", g->set, g->machine);
		return -1;
	}
	uint32_t rsize, gsize;
	uint8_t *rom = rbx_region("maincpu", &rsize);
	uint8_t *gfx = rbx_region("maincpu:gfx", &gsize);
	if (!rom || !gfx || (rsize & (rsize - 1))) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	if (!strcmp(g->set, "plottinga")) {   // init_plottinga: bits 7..0 => bits 0..7
		for (uint32_t i = 0; i < rsize; i++) {
			uint8_t b = rom[i], r = 0;
			for (int k = 0; k < 8; k++)
				r |= (uint8_t)(((b >> k) & 1) << (7 - k));
			rom[i] = r;
		}
	}
	s_ncpu = s_m->ncpu;
	s_ym_owner = s_ncpu == 1 ? CPU_MAIN : CPU_AUDIO;
	if (s_ncpu >= 2) {
		s_audio_rom = rbx_region("audiocpu", &s_audio_size);
		if (!s_audio_rom || s_audio_size < 0x4000) {
			rbx_fail("%s: audiocpu ROM missing", g->set);
			return -1;
		}
	}
	if (s_ncpu == 3) {
		s_slave_rom = rbx_region("slave", &s_slave_size);
		if (!s_slave_rom || s_slave_size < 0x8000) {
			rbx_fail("%s: slave ROM missing", g->set);
			return -1;
		}
	}
	if (s_m->kind == M_CHAMPWR) {
		s_adpcm = rbx_region("adpcm", &s_adpcm_size);
		if (!s_adpcm || !s_adpcm_size) {
			rbx_fail("%s: adpcm ROM missing", g->set);
			return -1;
		}
	}
	s_lvc = (tc0090lvc_t *)calloc(1, sizeof(tc0090lvc_t));
	s_view = (tc0090lvc_view *)calloc(1, sizeof(tc0090lvc_view));
	if (!s_lvc || !s_view || tc0090lvc_view_init(s_view, gfx, gsize)) {
		rbx_fail("%s: out of memory (graphics)", g->set);
		tl_exit();
		return -1;
	}
	tc0090lvc_init(s_lvc, rom, rsize);
	s_port_dswa = rbx_port_find("DSWA");
	s_port_dswb = rbx_port_find("DSWB");
	s_port_in[0] = rbx_port_find("IN0");
	s_port_in[1] = rbx_port_find("IN1");
	s_port_in[2] = rbx_port_find("IN2");
	s_port_an[0] = rbx_port_find("AN0");
	s_port_an[1] = rbx_port_find("AN1");

	rbx_ym_config yc;
	memset(&yc, 0, sizeof(yc));
	yc.type = s_m->ym;
	yc.clock = s_m->ym_clock;
	memcpy(yc.gain, s_m->gain, sizeof(yc.gain));
	if (s_m->kind == M_FHAWK || s_m->kind == M_CHAMPWR || s_m->kind == M_RAIMAIS)
		yc.irq = ym_irq_cb;
	if (s_m->kind == M_FHAWK || s_m->kind == M_CHAMPWR)
		yc.port_a_write = ym_port_a_w;
	if (s_ncpu == 1) {
		yc.port_a_read = ym_port_a_r;
		yc.port_b_read = ym_port_b_r;
	}
	if (s_m->ym == RBX_YM2610) {
		yc.adpcma = rbx_region("ymsnd:adpcma", &yc.adpcma_size);
		yc.adpcmb = rbx_region("ymsnd:adpcmb", &yc.adpcmb_size);
		if (!yc.adpcmb) {   // one sample ROM: ADPCM-B reads the ADPCM-A region (MAME)
			yc.adpcmb = yc.adpcma;
			yc.adpcmb_size = yc.adpcma_size;
		}
	}
	if (rbx_ym_init(0, &yc)) {
		rbx_fail("%s: sound init failed", g->set);
		tl_exit();
		return -1;
	}
	if (rbx_ym_set_async(0, 1)) {   // (its samples made on the worker: rbx_ym_async_take at the frame's end)
		tl_exit();
		return -1;
	}
	msm5205_init(&s_msm, MSM_CLOCK, MSM5205_S48_4B, msm_vck);
	s_mix_cap = (int)(rbx_ym_rate(0) / 50) + 64;
	s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	if (!s_mix || rbx_audio_setup(rbx_ym_rate(0))) {
		tl_exit();
		return -1;
	}
	if (rbx_video_setup(LVC_SCREEN_W, LVC_VIS_Y1 - LVC_VIS_Y0 + 1, 512)) {   // (power of two: the core masks indices)
		tl_exit();
		return -1;
	}
	s_cpu[CPU_MAIN].clock = MAIN_CLOCK;
	s_cpu[CPU_SLAVE].clock = SUB_CLOCK;
	s_cpu[CPU_AUDIO].clock = SUB_CLOCK;
	s_syt.nmi = syt_nmi;
	s_syt.reset = syt_reset;
	s_syt.sync = NULL;
	tl_reset();
	return 0;
}

static void tl_exit(void)
{
	if (s_view)
		tc0090lvc_view_exit(s_view);
	free(s_view);
	free(s_lvc);
	s_view = NULL;
	s_lvc = NULL;
	free(s_mix);
	s_mix = NULL;
	rbx_ym_exit(0);
}

// MAME vbl_interrupt (scanline timer)
static void scanline_irq(int line)
{
	if (s_cpu[CPU_MAIN].z.interrupt_mode != 2)   // kludge to make plgirls boot
		return;
	uint8_t en = s_lvc->irq_enable;
	if (line == 120 && (en & 1)) { s_last_irq_level = 0; s_irq_line = 1; }
	else if (line == 0 && (en & 2)) { s_last_irq_level = 1; s_irq_line = 1; }
	else if (line == 240 && (en & 4)) { s_last_irq_level = 2; s_irq_line = 1; }
}

static void trackball_frame(void)
{
	// uPD4701A: X counter from AN0 (trackball Y, reversed), Y counter from AN1 (trackball X), sensitivity 50
	s_upd_x = (uint16_t)(s_upd_x - rbx_analog_dpad(1, RBX_IPT_TRACKBALL_Y, 30) / 2);   // (d-pad: key delta 30)
	s_upd_y = (uint16_t)(s_upd_y + rbx_analog_dpad(1, RBX_IPT_TRACKBALL_X, 30) / 2);
}

static void tl_frame(void)
{
	static const int order[3] = { CPU_MAIN, CPU_SLAVE, CPU_AUDIO };
	int ncpu = s_ncpu;
	if (s_m->kind == M_HORSHOES)
		trackball_frame();
	for (int line = 0; line < LINES; line++) {
		scanline_irq(line);
		if (line == LVC_VIS_Y1 + 1) {
			vblank_capture();
			if (ncpu == 3) s_cpu[CPU_SLAVE].hold = 1;
			if (ncpu == 2) s_cpu[CPU_AUDIO].hold = 1;
		}
		for (int k = 0; k < ncpu; k++) {
			cpu_t *c = &s_cpu[order[k == 1 && ncpu == 2 ? 2 : k]];
			c->start = c->z.cyc;
			c->cycles = rbx_line_cycles(&c->frac, c->clock, 60000, LINES);
		}
		s_ym_line = (uint32_t)rbx_line_cycles(&s_ym_frac, s_m->ym_clock, 60000, LINES);
		s_ym_done = 0;
		int slices = ncpu == 3 ? SLICES : 1;
		for (int s = 1; s <= slices; s++)
			for (int k = 0; k < ncpu; k++) {
				int i = order[k == 1 && ncpu == 2 ? 2 : k];
				cpu_t *c = &s_cpu[i];
				cpu_run(i, c->start + (unsigned long)((int64_t)c->cycles * s / slices));
			}
		ym_sync();
	}
	rbx_ym_async_take(0, s_mix, s_mix_n);
	for (int i = 0; i < s_mix_n; i++)
		rbx_audio_put(s_mix[i]);
	memset(s_mix, 0, sizeof(int32_t) * (size_t)s_mix_n);
	s_mix_n = 0;
	s_frame++;
	if (s_dbg && s_frame % 120 == 0)
		fprintf(stderr, "f%u main pc %04x im %d iff %d en %02x | slave pc %04x | audio pc %04x rst %d\n", s_frame,
		        s_cpu[0].z.pc, s_cpu[0].z.interrupt_mode, s_cpu[0].z.iff1, s_lvc->irq_enable, s_cpu[1].z.pc,
		        s_cpu[2].z.pc, s_cpu[2].reset), fprintf(stderr, "  msm starts %d gain %d\n", s_msm_starts, s_msm_gain);
	if (s_dbg && s_frame % 120 == 0) {
		int nzp = 0, nzv = 0;
		for (int i = 0; i < 0x200; i++) nzp += s_lvc->pal[i] != 0;
		for (int i = 0x4000; i < 0x10000; i++) nzv += s_lvc->vram[i] != 0;
		fprintf(stderr, "  vregs %02x %02x %02x %02x %02x banks %02x %02x %02x %02x rom %02x pal nz %d vram nz %d\n",
		        s_lvc->vregs[0], s_lvc->vregs[1], s_lvc->vregs[2], s_lvc->vregs[3], s_lvc->vregs[4], s_lvc->ram_bank[0],
		        s_lvc->ram_bank[1], s_lvc->ram_bank[2], s_lvc->ram_bank[3], s_lvc->rom_bank, nzp, nzv);
	}
}

static int tl_state(void *ctx, int load)
{
	cpu_t cpu[3];
	memcpy(cpu, s_cpu, sizeof(cpu));
	int32_t misc[24] = {
		s_irq_line, s_last_irq_level, s_slave_bank, s_audio_bank, s_ym_irq, s_intl, s_ioc_port, s_mux_sel,
		s_ym_addr, s_ym_reg7, s_ym_portb, s_upd_x, s_upd_y, s_upd_sx, s_upd_sy, (int32_t)s_adpcm_pos,
		s_adpcm_data, s_msm_gain, (int32_t)s_msm_acc, (int32_t)s_frame, 0, 0, 0, 0 };
	uint64_t yfrac = s_ym_frac;
	int rc = 0;
	rc |= rbx_state_io(ctx, "cpus", cpu, sizeof(cpu));
	rc |= rbx_state_io(ctx, "lvc", s_lvc, offsetof(tc0090lvc_t, rom));
	rc |= rbx_state_io(ctx, "ram", s_ram, sizeof(s_ram));
	rc |= rbx_state_io(ctx, "share", s_share, sizeof(s_share));
	rc |= rbx_state_io(ctx, "dpram", s_dpram, sizeof(s_dpram));
	rc |= rbx_state_io(ctx, "sram", s_slave_ram, sizeof(s_slave_ram));
	rc |= rbx_state_io(ctx, "aram", s_audio_ram, sizeof(s_audio_ram));
	rc |= rbx_state_io(ctx, "ioc", s_ioc_regs, sizeof(s_ioc_regs));
	rc |= rbx_state_io(ctx, "syt", &s_syt, offsetof(tc0140syt_t, nmi));
	rc |= rbx_state_io(ctx, "msm", &s_msm, offsetof(msm5205_t, vck));
	rc |= rbx_state_io(ctx, "misc", misc, sizeof(misc));
	rc |= rbx_state_io(ctx, "yfrac", &yfrac, sizeof(yfrac));
	rc |= rbx_ym_state(0, ctx, load);
	if (rc)
		return -1;
	if (load) {
		memcpy(s_cpu, cpu, sizeof(cpu));
		for (int i = 0; i < 3; i++)
			cpu_bind(i);
		s_irq_line = misc[0]; s_last_irq_level = misc[1]; s_slave_bank = misc[2]; s_audio_bank = misc[3];
		s_ym_irq = misc[4]; s_intl = misc[5]; s_ioc_port = (uint8_t)misc[6]; s_mux_sel = misc[7];
		s_ym_addr = (uint8_t)misc[8]; s_ym_reg7 = (uint8_t)misc[9]; s_ym_portb = (uint8_t)misc[10];
		s_upd_x = (uint16_t)misc[11]; s_upd_y = (uint16_t)misc[12]; s_upd_sx = (uint16_t)misc[13]; s_upd_sy = (uint16_t)misc[14];
		s_adpcm_pos = (uint32_t)misc[15]; s_adpcm_data = misc[16]; s_msm_gain = misc[17]; s_msm_acc = (uint32_t)misc[18];
		s_frame = (uint32_t)misc[19];
		s_ym_frac = yfrac;
		map_build();
		memset(s_lvc->chr_dirty, 0xff, sizeof(s_lvc->chr_dirty));
	}
	return 0;
}

static int tl_save(void *ctx) { return tl_state(ctx, 0); }
static int tl_load(void *ctx) { return tl_state(ctx, 1); }
static int tl_supported(const char *machine) { return find_machine(machine) != NULL; }

const rbx_driver rbx_drv_taito_l = { "Taito L System", tl_init, tl_exit, tl_reset, tl_frame, tl_save, tl_load, tl_supported, tl_render };
