/*  Operation Wolf, ported from MAME src/mame/taito/opwolf.cpp (GPL-2.0+, copyright-holders: David Graves,
    Jarek Burczynski). 68000 + C-Chip (machine/taitocchip.c) + PC080SN + PC090OJ; sound board: Z80 + YM2151 + PC060HA
    (machine/taito_sound.c, map 9000) with two MSM5205 ADPCM channels and two TC0060DCA volume controls.
    opwolfp (prototype) has no protection; opwolfb (bootleg) replaces the C-Chip with a second Z80.
    Light gun: the crosshair (MAME P1X / P1Y, 0-255) follows rbx_analog(1, RBX_IPT_LIGHTGUN_X / Y) deltas.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m68k_bus.h"
#include "../../cpu/m68k/m68k.h"
#include "../../cpu/z80/z80.h"
#include "../machine/taito_sound.h"
#include "../machine/taitocchip.h"
#include "../../sound/msm5205u.h"
#include "../video/pc080sn.h"
#include "../video/pc090oj.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#define LINES 256
#define SCREEN_W 320
#define VIS_Y0 8
#define VIS_Y1 247
#define VIS_H (VIS_Y1 - VIS_Y0 + 1)

enum { OW_CCHIP, OW_PROTO, OW_BOOTLEG };

static int s_kind;
static uint8_t *s_rom;
static uint32_t s_rom_size;
static uint8_t s_ram[0x8000];
static uint8_t s_pal[0x1000];
static pc080sn_t *s_sn;
static pc090oj_t *s_oj;
static struct {
	pc080sn_regs sn;
	pc090oj_regs oj;
	uint8_t pal[0x1000];
} *s_snap;
static uint8_t s_pri[SCREEN_W * VIS_H];
static int s_port_dswa, s_port_dswb, s_port_in[4];   // IN0..IN3
static uint64_t s_cpu_frac, s_cc_frac, s_sub_frac;
static int s_gun_xoffs, s_gun_yoffs;
static int s_gun[2] = { 0x80, 0x80 };   // P1X / P1Y
// bootleg: Z80 in place of the C-Chip
static z80 s_sub;
static uint8_t *s_sub_rom;
static uint32_t s_sub_rom_size;
static uint8_t s_cchip_ram[0x800];
// ADPCM: two MSM5205 with the register files of MAME adpcm_w; TC0060DCA gains
static msm5205u_t s_msm[2];
static const uint8_t *s_adpcm;
static uint32_t s_adpcm_size;
static struct {
	uint8_t regs[2][8];
	uint32_t pos[2], end[2];
	int32_t data[2];
	uint8_t dca[2][2];   // TC0060DCA volume registers: [0] = ADPCM channels, [1] = output (YM + ADPCM)
} s_ad;
static int s_atten[256];   // TC0060DCA attenuation table, 1/256

// ---- sound ----
static void msm_vck(msm5205u_t *m, void *user)   // MAME msm5205u_vck_w<N>
{
	int n = (int)(intptr_t)user;
	if (s_ad.data[n] != -1) {
		msm5205u_data_w(m, (uint8_t)(s_ad.data[n] & 0x0f));
		s_ad.data[n] = -1;
		if (s_ad.pos[n] == s_ad.end[n])
			msm5205u_reset_w(m, 1);
	} else {
		s_ad.data[n] = s_ad.pos[n] < s_adpcm_size ? s_adpcm[s_ad.pos[n]] : 0;
		s_ad.pos[n] = (s_ad.pos[n] + 1) & 0x7ffff;
		msm5205u_data_w(m, (uint8_t)(s_ad.data[n] >> 4));
	}
}

static void update_volume(void)
{
	// YM2151 -> TC0060DCA #1 (both channels); mono = the mean of its two outputs
	rbx_ym_set_volume(0, (s_atten[s_ad.dca[1][0]] + s_atten[s_ad.dca[1][1]]) / 2);
}

static void snd_ext_write(uint16_t a, uint8_t v)
{
	int n;
	switch (a & 0xf000) {
	case 0xb000: case 0xc000:   // adpcm_w<N>
		n = (a & 0xf000) == 0xc000;
		if ((a & 0xfff) > 6)
			return;
		s_ad.regs[n][a & 7] = v;
		if ((a & 7) == 4) {
			uint16_t start = (uint16_t)(s_ad.regs[n][1] << 8 | s_ad.regs[n][0]);
			uint16_t end = (uint16_t)(s_ad.regs[n][3] << 8 | s_ad.regs[n][2]);
			s_ad.pos[n] = (uint32_t)start << 4;
			s_ad.end[n] = (uint32_t)end << 4;
			msm5205u_reset_w(&s_msm[n], 0);
			s_ad.dca[0][n] = s_ad.regs[n][5];
		}
		break;
	case 0xd000: if (a == 0xd000) { s_ad.dca[1][0] = v; update_volume(); } break;
	case 0xe000: if (a == 0xe000) { s_ad.dca[1][1] = v; update_volume(); } break;
	}
}

static void snd_ext_mix(int32_t *buf, int n, uint32_t rate)
{
	int out = (s_atten[s_ad.dca[1][0]] + s_atten[s_ad.dca[1][1]]) / 2;
	msm5205u_render(&s_msm[0], buf, n, rate, s_atten[s_ad.dca[0][0]] * out / 256);
	msm5205u_render(&s_msm[1], buf, n, rate, s_atten[s_ad.dca[0][1]] * out / 256);
}

// ---- inputs ----
static uint16_t gun_r(int axis)
{
	uint32_t v = rbx_port_read(s_port_in[2 + axis]) & 0xffff;
	int g = axis ? s_gun[1] - 0x24 + s_gun_yoffs : s_gun[0] * 320 / 256 + 0x15 + s_gun_xoffs;   // gun_x_r / gun_y_r
	return (uint16_t)((v & ~0x1ffu) | ((uint32_t)g & 0x1ff));
}

static void update_gun(void)
{
	for (int axis = 0; axis < 2; axis++) {
		int v = s_gun[axis] + rbx_analog(1, axis ? RBX_IPT_LIGHTGUN_Y : RBX_IPT_LIGHTGUN_X);
		s_gun[axis] = v < 0 ? 0 : v > 255 ? 255 : v;
	}
}

static uint8_t cc_pb(void) { return (uint8_t)rbx_port_read(s_port_in[0]); }
static uint8_t cc_pc(void) { return (uint8_t)rbx_port_read(s_port_in[1]); }
static void cc_counters(uint8_t v) { (void)v; }

// ---- bootleg sub CPU ----
static uint8_t sub_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x8000) return a < s_sub_rom_size ? s_sub_rom[a] : 0xff;
	if (a == 0x8800) return (uint8_t)rbx_port_read(s_port_in[1]);
	if (a == 0x9800) return (uint8_t)rbx_port_read(s_port_in[0]);
	if (a >= 0xc000 && a < 0xc800) return s_cchip_ram[a & 0x7ff];
	return 0xff;
}
static void sub_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0xc000 && a < 0xc800) s_cchip_ram[a & 0x7ff] = v;
}
static uint8_t sub_in(z80 *z, uint8_t p) { (void)z; (void)p; return 0xff; }
static void sub_out(z80 *z, uint8_t p, uint8_t v) { (void)z; (void)p; (void)v; }

static void sub_start(void)
{
	z80_init(&s_sub);
	s_sub.read_byte = sub_read;
	s_sub.write_byte = sub_write;
	s_sub.port_in = sub_in;
	s_sub.port_out = sub_out;
}

// ---- main CPU devices ----
static uint16_t main_read16(uint32_t a)
{
	if (a >= 0x0f0000 && a <= 0x0fffff) {
		uint32_t o = a & 0xfff;
		if (s_kind == OW_CCHIP)
			return o < 0x800 ? cchip_mem68_r(o >> 1) : cchip_asic_r((o - 0x800) >> 1);
		if (s_kind == OW_BOOTLEG)
			return s_cchip_ram[(o >> 1) & 0x7ff];
		return 0;
	}
	if (a == 0x380000) return (uint16_t)(rbx_port_read(s_port_dswa) & 0xff);
	if (a == 0x380002) return (uint16_t)(rbx_port_read(s_port_dswb) & 0xff);
	if (a == 0x3a0000) return gun_r(0);
	if (a == 0x3a0002) return gun_r(1);
	if (a == 0x3e0002) return (uint16_t)(tc0140syt_master_comm_r(tsnd_syt()) << 8);
	return 0;
}

static uint8_t main_read8(uint32_t a)
{
	if (a == 0x3e0002) return tc0140syt_master_comm_r(tsnd_syt());
	return (uint8_t)(main_read16(a & ~1u) >> ((a & 1) ? 0 : 8));
}

static void main_write16m(uint32_t a, uint16_t v, uint16_t mask)
{
	if (a >= 0x0f0000 && a <= 0x0fffff) {
		uint32_t o = a & 0xfff;
		if (s_kind == OW_CCHIP && (mask & 0xff)) {
			if (o < 0x800)
				cchip_mem68_w(o >> 1, (uint8_t)v);
			else
				cchip_asic68_w((o - 0x800) >> 1, (uint8_t)v);
		} else if (s_kind == OW_BOOTLEG) {
			s_cchip_ram[(o >> 1) & 0x7ff] = (uint8_t)v;   // cchip_w: data & 0xff
		}
		return;
	}
	if (a >= 0xc20000 && a <= 0xc20003) { pc080sn_yscroll_w(s_sn, (a >> 1) & 1, v, mask); return; }
	if (a >= 0xc40000 && a <= 0xc40003) { pc080sn_xscroll_w(s_sn, (a >> 1) & 1, v, mask); return; }
	if (a >= 0xc50000 && a <= 0xc50003) { pc080sn_ctrl_w(s_sn, (a >> 1) & 1, v, mask); return; }
	if (a == 0x380000) { s_oj->sprite_ctrl = (uint16_t)((s_oj->sprite_ctrl & ~mask) | (v & mask)); return; }   // spritectrl_w
	if (a == 0x3e0000 && (mask & 0xff00)) { tc0140syt_master_port_w(tsnd_syt(), (uint8_t)(v >> 8)); return; }
	if (a == 0x3e0002 && (mask & 0xff00)) { tc0140syt_master_comm_w(tsnd_syt(), (uint8_t)(v >> 8)); return; }
}

static void main_write16(uint32_t a, uint16_t v) { main_write16m(a, v, 0xffff); }
static void main_write8(uint32_t a, uint8_t v)
{
	if (a & 1)
		main_write16m(a & ~1u, v, 0x00ff);
	else
		main_write16m(a, (uint16_t)(v << 8), 0xff00);
}

// ---- video ----
static inline int pal5(int v) { return (v << 3) | (v >> 2); }

static void opwolf_render(void)
{
	const uint16_t *p = (const uint16_t *)s_snap->pal;
	for (int i = 0; i < 2048; i++) {   // xRGBRRRRGGGGBBBB_bit0
		uint32_t raw = p[i];
		int r = (int)(((raw >> 7) & 0x1e) | ((raw >> 14) & 1));
		int g = (int)(((raw >> 3) & 0x1e) | ((raw >> 13) & 1));
		int b = (int)(((raw << 1) & 0x1e) | ((raw >> 12) & 1));
		rbx_video.palette[i] = (uint32_t)(pal5(r) << 16 | pal5(g) << 8 | pal5(b));
	}
	uint16_t *bm = rbx_video.bitmap;
	int pitch = rbx_video.pitch;
	pc080sn_draw(s_sn, &s_snap->sn, 0, bm, pitch, s_pri, SCREEN_W, VIS_Y0, VIS_H, 1, 1);
	pc080sn_draw(s_sn, &s_snap->sn, 1, bm, pitch, s_pri, SCREEN_W, VIS_Y0, VIS_H, 0, 2);
	// colpri_cb: palette bank in bits 5-7, sprites under the top layer
	pc090oj_draw(s_oj, &s_snap->oj, bm, pitch, s_pri, SCREEN_W, VIS_Y0, VIS_H, 1, (s_snap->oj.sprite_ctrl & 0xe0) >> 1, 0xfc);
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	s_snap->sn = s_sn->r;
	pc090oj_snapshot(s_oj, &s_snap->oj);
	memcpy(s_snap->pal, s_pal, sizeof(s_pal));
	rbx_video_frame_done();
}

// ---- init / frame ----
static void opwolf_reset(void)
{
	// (the palette / video RAM too: a game that has not written it yet showed the last game's, as at start it is 0)
	memset(s_pal, 0, sizeof(s_pal));
	memset(s_ram, 0, sizeof(s_ram));
	memset(s_cchip_ram, 0, sizeof(s_cchip_ram));
	pc080sn_reset(s_sn);
	pc090oj_reset(s_oj);
	s_oj->sprite_ctrl = 0;
	if (s_kind == OW_CCHIP)
		cchip_reset();
	if (s_kind == OW_BOOTLEG)
		sub_start();
	memset(&s_ad, 0, sizeof(s_ad));
	s_ad.data[0] = s_ad.data[1] = -1;
	s_ad.dca[0][0] = s_ad.dca[0][1] = s_ad.dca[1][0] = s_ad.dca[1][1] = 0xff;
	for (int i = 0; i < 2; i++) {
		msm5205u_reset(&s_msm[i]);
		msm5205u_reset_w(&s_msm[i], 1);
	}
	m68k_bus_irq_reset();
	m68k_pulse_reset();
	tsnd_reset();
	update_volume();
	s_cpu_frac = s_cc_frac = s_sub_frac = 0;
	s_gun[0] = s_gun[1] = 0x80;
}

static void opwolf_exit(void);

static int opwolf_init(const rbx_game *g)
{
	uint32_t sn_size, oj_size;
	s_kind = !strcmp(g->machine, "opwolfp") ? OW_PROTO : !strcmp(g->machine, "opwolfb") ? OW_BOOTLEG : OW_CCHIP;
	s_rom = rbx_region("maincpu", &s_rom_size);
	uint8_t *sn_gfx = rbx_region("pc080sn", &sn_size);
	uint8_t *oj_gfx = rbx_region("pc090oj", &oj_size);
	s_adpcm = rbx_region("adpcm", &s_adpcm_size);
	s_sub_rom = rbx_region("sub", &s_sub_rom_size);
	if (!s_rom || !sn_gfx || !oj_gfx || s_rom_size < 0x40000 || (s_kind == OW_BOOTLEG && !s_sub_rom)) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	m68k_bus_swap16(s_rom, s_rom_size);
	// init_opwolf: World and US versions have different gun offsets; init_opwolfb / init_opwolfp
	if (s_kind == OW_CCHIP) {
		s_gun_xoffs = 0xec - (m68k_word(s_rom, 0x03ffb0) & 0xff);
		s_gun_yoffs = 0x1c - (m68k_word(s_rom, 0x03ffae) & 0xff);
	} else if (s_kind == OW_BOOTLEG) {
		s_gun_xoffs = -2;
		s_gun_yoffs = 17;
	} else {
		s_gun_xoffs = 5;
		s_gun_yoffs = 30;
	}
	s_sn = (pc080sn_t *)calloc(1, sizeof(pc080sn_t));
	s_oj = (pc090oj_t *)calloc(1, sizeof(pc090oj_t));
	s_snap = calloc(1, sizeof(*s_snap));
	if (!s_sn || !s_oj || !s_snap || pc080sn_init(s_sn, sn_gfx, sn_size, PC080SN_GFX_PACKED_MSB) ||
	    pc090oj_init(s_oj, oj_gfx, oj_size, 2048)) {
		rbx_fail("%s: out of memory (graphics)", g->set);
		opwolf_exit();
		return -1;
	}
	s_oj->direct = 1;
	s_port_dswa = rbx_port_find("DSWA");
	s_port_dswb = rbx_port_find("DSWB");
	static const char *const tags[4] = { "IN0", "IN1", "IN2", "IN3" };
	for (int i = 0; i < 4; i++)
		s_port_in[i] = rbx_port_find(tags[i]);
	if (s_kind == OW_CCHIP) {
		cchip_config cc;
		memset(&cc, 0, sizeof(cc));
		cc.clock = 12000000;
		cc.in_pb = cc_pb;
		cc.in_pc = cc_pc;
		cc.out_pb = cc_counters;
		if (cchip_init(&cc)) {
			rbx_fail("%s: C-Chip ROMs missing", g->set);
			opwolf_exit();
			return -1;
		}
	}
	for (int i = 0; i < 256; i++)   // TC0060DCA: 1 / (1 + exp(-10 * (x / 256 - 0.6)))
		s_atten[i] = (int)(256.0 / (1.0 + exp(-10.0 * (i / 256.0 - 0.6))) + 0.5);
	// sound: YM2151 0.35 to both inputs of TC0060DCA #1; MSM5205 1.0 through TC0060DCA #0 into #1
	tsnd_config sc;
	memset(&sc, 0, sizeof(sc));
	sc.map = TSND_MAP_9000;
	sc.z80_clock = 4000000;
	sc.frame_rate_mhz = 60000;
	sc.lines = LINES;
	sc.ym.type = RBX_YM2151;
	sc.ym.clock = 4000000;
	sc.ym.gain[0] = sc.ym.gain[1] = 90;
	sc.ym_port_bank = 1;
	sc.ext_write = snd_ext_write;
	sc.ext_mix = snd_ext_mix;
	for (int i = 0; i < 2; i++)
		msm5205u_init(&s_msm[i], 384000, MSM5205_S48_4B, msm_vck, (void *)(intptr_t)i);
	if (tsnd_init(&sc)) {
		rbx_fail("%s: sound init failed", g->set);
		opwolf_exit();
		return -1;
	}
	if (rbx_video_setup(SCREEN_W, VIS_H, 2048)) {
		opwolf_exit();
		return -1;
	}
	m68k_init();
	m68k_set_cpu_type(M68K_CPU_TYPE_68000);
	m68k_bus_reset_map();
	m68k_bus_map(0, 0x03ffff, s_rom, s_rom_size, 1, 0);
	m68k_bus_map(0x100000, 0x107fff, s_ram, sizeof(s_ram), 1, 1);
	m68k_bus_map(0x200000, 0x200fff, s_pal, sizeof(s_pal), 1, 1);
	m68k_bus_map(0xc00000, 0xc0ffff, s_sn->r.ram, sizeof(s_sn->r.ram), 1, 1);
	m68k_bus_map(0xd00000, 0xd03fff, s_oj->ram, sizeof(s_oj->ram), 1, 1);
	m68k_bus.read8 = main_read8;
	m68k_bus.read16 = main_read16;
	m68k_bus.write8 = main_write8;
	m68k_bus.write16 = main_write16;
	opwolf_reset();
	return 0;
}

static void opwolf_exit(void)
{
	if (s_sn)
		pc080sn_exit(s_sn);
	if (s_oj)
		pc090oj_exit(s_oj);
	free(s_sn);
	free(s_oj);
	free(s_snap);
	s_sn = NULL;
	s_oj = NULL;
	s_snap = NULL;
	tsnd_exit();
}

static void opwolf_frame(void)
{
	update_gun();
	for (int line = 0; line < LINES; line++) {
		if (line == VIS_Y1 + 1) {
			vblank_capture();
			m68k_bus_hold_irq(5);
			if (s_kind == OW_CCHIP) {
				cchip_ext_interrupt(1);
				cchip_ext_interrupt(0);
			}
			if (s_kind == OW_BOOTLEG)
				z80_gen_int(&s_sub, 0xff);   // irq0_line_hold
		}
		m68k_execute(tsnd_line_cycles(&s_cpu_frac, 8000000));
		if (s_kind == OW_CCHIP)
			cchip_run_clocks(&s_cc_frac, 60000, LINES);
		if (s_kind == OW_BOOTLEG) {
			unsigned long end = s_sub.cyc + (unsigned long)tsnd_line_cycles(&s_sub_frac, 4000000);
			while ((long)(s_sub.cyc - end) < 0)
				z80_step(&s_sub);
		}
		tsnd_line();
	}
	tsnd_frame_end();
}

static int opwolf_state(void *ctx, int load)
{
	uint8_t cpu[1024];
	uint32_t csize = m68k_context_size();
	if (csize > sizeof(cpu))
		return -1;
	if (!load)
		m68k_get_context(cpu);
	uint32_t irq = m68k_bus_irq_pending();
	uint64_t fr[3] = { s_cpu_frac, s_cc_frac, s_sub_frac };
	int gun[2] = { s_gun[0], s_gun[1] };
	z80 sub = s_sub;
	int rc = 0;
	rc |= rbx_state_io(ctx, "m68k", cpu, csize);
	rc |= rbx_state_io(ctx, "irq", &irq, 4);
	rc |= rbx_state_io(ctx, "ram", s_ram, sizeof(s_ram));
	rc |= rbx_state_io(ctx, "pal", s_pal, sizeof(s_pal));
	rc |= rbx_state_io(ctx, "pc080sn", &s_sn->r, sizeof(s_sn->r));
	rc |= rbx_state_io(ctx, "pc090oj", s_oj->ram, sizeof(s_oj->ram));
	rc |= rbx_state_io(ctx, "ojctrl", &s_oj->sprite_ctrl, sizeof(s_oj->sprite_ctrl));
	rc |= rbx_state_io(ctx, "msm0", &s_msm[0], MSM5205_STATE_SIZE);
	rc |= rbx_state_io(ctx, "msm1", &s_msm[1], MSM5205_STATE_SIZE);
	rc |= rbx_state_io(ctx, "adpcm", &s_ad, sizeof(s_ad));
	rc |= rbx_state_io(ctx, "gun", gun, sizeof(gun));
	rc |= rbx_state_io(ctx, "frac", fr, sizeof(fr));
	if (s_kind == OW_CCHIP)
		rc |= cchip_state(ctx, load);
	if (s_kind == OW_BOOTLEG) {
		rc |= rbx_state_io(ctx, "sub", &sub, sizeof(sub));
		rc |= rbx_state_io(ctx, "cchipram", s_cchip_ram, sizeof(s_cchip_ram));
	}
	rc |= tsnd_state(ctx, load);
	if (rc)
		return -1;
	if (load) {
		m68k_set_context(cpu);
		m68k_bus_set_irq_pending(irq);
		s_cpu_frac = fr[0];
		s_cc_frac = fr[1];
		s_sub_frac = fr[2];
		s_gun[0] = gun[0];
		s_gun[1] = gun[1];
		if (s_kind == OW_BOOTLEG) {
			sub.read_byte = sub_read;
			sub.write_byte = sub_write;
			sub.port_in = sub_in;
			sub.port_out = sub_out;
			sub.userdata = NULL;
			s_sub = sub;
		}
		update_volume();
	}
	return 0;
}

static int opwolf_save(void *ctx) { return opwolf_state(ctx, 0); }
static int opwolf_load(void *ctx) { return opwolf_state(ctx, 1); }
static int opwolf_supported(const char *machine)
{
	return !strcmp(machine, "opwolf") || !strcmp(machine, "opwolfp") || !strcmp(machine, "opwolfb");
}

const rbx_driver rbx_drv_opwolf = { "Operation Wolf", opwolf_init, opwolf_exit, opwolf_reset, opwolf_frame, opwolf_save,
                                    opwolf_load, opwolf_supported, opwolf_render };
