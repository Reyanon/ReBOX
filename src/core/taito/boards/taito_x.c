/*  Taito X System, ported from MAME src/mame/taito/taito_x.cpp (GPL-2.0+, copyright-holders: Howie Cohen,
    Yochizo; with help from Richard Bush, Nicola Salmoria, stephh and others). Memory maps, machine configurations and
    sprite offsets follow MAME. Sprites: X1-001 (video/x1_001.c); sound board: machine/taito_sound.c.
    Superman (taitox_cchip_state): C-Chip (machine/taitocchip.c) at 0x900000, IRQ 6, 57.43 Hz.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m68k_bus.h"
#include "../../cpu/m68k/m68k.h"
#include "../machine/taito_sound.h"
#include "../machine/taitocchip.h"
#include "../../video/x1_001.h"

#include <stdlib.h>
#include <string.h>

enum { TX_DAISENPU, TX_GIGANDES, TX_BALLBROS, TX_KYUSTRKR, TX_SUPERMAN };

typedef struct {
	const char *name;
	int kind;
	uint32_t rom_end;
	int ym;                 // RBX_YM2610 / RBX_YM2151
	int vis_y0, vis_y1;     // visible lines
	int fg_flipy, fg_noflipy, bg_flipy, bg_noflipy;
	uint32_t rate_mhz;      // refresh in milli-Hz
} machine_t;

static const machine_t s_machines[] = {
	{ "daisenpu", TX_DAISENPU, 0x3ffff, RBX_YM2151, 16, 239, -0x12, 0x0e, 0x1, -0x1, 60000 },
	{ "gigandes", TX_GIGANDES, 0x7ffff, RBX_YM2610, 8, 247, -0x0a, 0x0e, 0x1, -0x1, 60000 },
	{ "ballbros", TX_BALLBROS, 0x3ffff, RBX_YM2610, 8, 247, -0x0a, 0x0e, 0x1, -0x1, 60000 },
	{ "kyustrkr", TX_KYUSTRKR, 0x3ffff, RBX_YM2610, 8, 247, -0x0a, 0x0e, 0x1, -0x1, 60000 },
	{ "superman", TX_SUPERMAN, 0x7ffff, RBX_YM2610, 8, 247, -0x12, 0x0e, 0x1, -0x1, 57430 },
};

#define LINES 256
#define SCREEN_W 384

static const machine_t *s_m;
static uint8_t *s_rom;
static uint32_t s_rom_size;
static uint8_t s_ram[0x4000];
static uint8_t s_pal[0x1000];
static x1_001_t *s_x1, *s_snap;
static uint8_t s_snap_pal[0x1000];
static int s_port_dswa, s_port_dswb, s_port_in[3];
static uint64_t s_cpu_frac, s_cc_frac;
static int s_cchip;   // superman

// ---- main CPU devices ----
static uint16_t dsw_r(int offset)
{
	uint32_t a = rbx_port_read(s_port_dswa), b = rbx_port_read(s_port_dswb);
	switch (offset) {
	case 0: return a & 0x0f;
	case 1: return (a & 0xf0) >> 4;
	case 2: return b & 0x0f;
	case 3: return (b & 0xf0) >> 4;
	}
	return 0;
}

// ---- C-Chip (superman_map: 0x900000-0x9007ff RAM window, 0x900800-0x900fff ASIC; low byte) ----
static uint8_t cc_pa(void) { return (uint8_t)rbx_port_read(s_port_in[0]); }
static uint8_t cc_pb(void) { return (uint8_t)rbx_port_read(s_port_in[1]); }
static uint8_t cc_ad(void) { return (uint8_t)rbx_port_read(s_port_in[2]); }

static uint16_t main_read16(uint32_t a)
{
	if (s_cchip && a >= 0x900000 && a <= 0x900fff)
		return a <= 0x9007ff ? cchip_mem68_r((a - 0x900000) >> 1) : cchip_asic_r((a - 0x900800) >> 1);
	if (a >= 0xd00000 && a <= 0xd005ff) return s_x1->ylow[(a - 0xd00000) >> 1];
	if (a >= 0xd00600 && a <= 0xd00607) return s_x1->ctrl[(a - 0xd00600) >> 1];
	if (a >= 0x500000 && a <= 0x50000f) return dsw_r((int)((a - 0x500000) >> 1));
	if (a >= 0x900000 && a <= 0x90000f) {
		int o = (int)((a - 0x900000) >> 1);
		return o < 3 ? (uint16_t)rbx_port_read(s_port_in[o]) : 0;
	}
	if (a == 0x800002) return tc0140syt_master_comm_r(tsnd_syt());
	return 0;
}

static uint8_t main_read8(uint32_t a)
{
	if (a == 0x800003) return tc0140syt_master_comm_r(tsnd_syt());
	if (a >= 0x800000 && a <= 0x800003) return 0;
	return (uint8_t)(main_read16(a & ~1u) >> ((a & 1) ? 0 : 8));
}

static void main_write16m(uint32_t a, uint16_t v, uint16_t mask)
{
	if (s_cchip && a >= 0x900000 && a <= 0x900fff) {
		if (mask & 0xff) {
			if (a <= 0x9007ff)
				cchip_mem68_w((a - 0x900000) >> 1, (uint8_t)v);
			else
				cchip_asic68_w((a - 0x900800) >> 1, (uint8_t)v);
		}
		return;
	}
	if (a >= 0xd00000 && a <= 0xd005ff) { if (mask & 0xff) s_x1->ylow[(a - 0xd00000) >> 1] = (uint8_t)v; return; }
	if (a >= 0xd00600 && a <= 0xd00607) { if (mask & 0xff) s_x1->ctrl[(a - 0xd00600) >> 1] = (uint8_t)v; return; }
	if (a == 0x800000 && (mask & 0xff)) { tc0140syt_master_port_w(tsnd_syt(), (uint8_t)v); return; }
	if (a == 0x800002 && (mask & 0xff)) { tc0140syt_master_comm_w(tsnd_syt(), (uint8_t)v); return; }
	// 0x900008: coin counters / lockout; 0x400000 / 0x600000: written every frame (watchdog?)
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
static void tx_render(void)
{
	const uint16_t *p = (const uint16_t *)s_snap_pal;
	for (int i = 0; i < 2048; i++) {   // xRGB_555
		int r = (p[i] >> 10) & 31, g = (p[i] >> 5) & 31, b = p[i] & 31;
		r = (r << 3) | (r >> 2); g = (g << 3) | (g >> 2); b = (b << 3) | (b >> 2);
		rbx_video.palette[i] = (uint32_t)(r << 16 | g << 8 | b);
	}
	uint16_t *bm = rbx_video.bitmap;
	int h = rbx_video.height;
	for (int i = 0; i < SCREEN_W * h; i++)
		bm[i] = 0x1f0;
	x1_draw(s_snap, bm, rbx_video.pitch, SCREEN_W, h, 0, s_m->vis_y0, LINES, s_m->vis_y1, 0x1000);
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap->ctrl, s_x1->ctrl, sizeof(s_x1->ctrl));
	memcpy(s_snap->ylow, s_x1->ylow, sizeof(s_x1->ylow));
	memcpy(s_snap->code, s_x1->code, sizeof(s_x1->code));
	s_snap->bgflag = s_x1->bgflag;
	memcpy(s_snap_pal, s_pal, sizeof(s_pal));
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

static void tx_reset(void)
{
	// (the palette / video RAM too: a game that has not written it yet showed the last game's, as at start it is 0)
	memset(s_pal, 0, sizeof(s_pal));
	memset(s_ram, 0, sizeof(s_ram));
	x1_reset(s_x1);
	m68k_bus_irq_reset();
	m68k_pulse_reset();
	tsnd_reset();
	if (s_cchip)
		cchip_reset();
	s_cpu_frac = s_cc_frac = 0;
}

static void tx_exit(void);

static int tx_init(const rbx_game *g)
{
	s_m = find_machine(g->machine);
	if (!s_m) {
		rbx_fail("%s: machine %s is not emulated", g->set, g->machine);
		return -1;
	}
	s_rom = rbx_region("maincpu", &s_rom_size);
	uint32_t gsize;
	uint8_t *gfx = rbx_region("gfx1", &gsize);
	if (!s_rom || !gfx) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	m68k_bus_swap16(s_rom, s_rom_size);
	s_x1 = (x1_001_t *)calloc(1, sizeof(x1_001_t));
	s_snap = (x1_001_t *)calloc(1, sizeof(x1_001_t));
	if (!s_x1 || !s_snap || x1_init(s_x1, gfx, gsize, 128)) {
		rbx_fail("%s: out of memory (graphics)", g->set);
		tx_exit();
		return -1;
	}
	s_x1->fg_flipyoffs = s_m->fg_flipy;
	s_x1->fg_noflipyoffs = s_m->fg_noflipy;
	s_x1->bg_flipyoffs = s_m->bg_flipy;
	s_x1->bg_noflipyoffs = s_m->bg_noflipy;
	*s_snap = *s_x1;   // (configuration and the decoded tiles; RAM is copied at each vblank)
	s_port_dswa = rbx_port_find("DSWA");
	s_port_dswb = rbx_port_find("DSWB");
	s_port_in[0] = rbx_port_find("IN0");
	s_port_in[1] = rbx_port_find("IN1");
	s_port_in[2] = rbx_port_find("IN2");
	s_cchip = s_m->kind == TX_SUPERMAN;
	if (s_cchip) {   // inputs through the C-Chip: PA = IN0, PB = IN1, AN = IN2
		cchip_config cc;
		memset(&cc, 0, sizeof(cc));
		cc.clock = 8000000;
		cc.in_pa = cc_pa;
		cc.in_pb = cc_pb;
		cc.in_ad = cc_ad;
		if (cchip_init(&cc)) {
			rbx_fail("%s: C-Chip ROMs missing", g->set);
			tx_exit();
			return -1;
		}
	}
	// sound: MAME's stereo routes: YM2610 SSG 0.75 to both, FM L / R 1.0; YM2151 0.45 per side
	tsnd_config sc;
	memset(&sc, 0, sizeof(sc));
	sc.stereo = 1;
	sc.map = TSND_MAP_E000;
	sc.z80_clock = 4000000;
	sc.frame_rate_mhz = s_m->rate_mhz;
	sc.lines = LINES;
	sc.ym.type = s_m->ym;
	if (s_m->ym == RBX_YM2151) {
		sc.ym.clock = 4000000;
		sc.ym.gain[0] = sc.ym.gain[1] = 115;
	} else {
		sc.ym.clock = 8000000;
		sc.ym.gain[0] = 192;
		sc.ym.gain[1] = sc.ym.gain[2] = 256;
	}
	if (tsnd_init(&sc)) {
		rbx_fail("%s: sound init failed", g->set);
		tx_exit();
		return -1;
	}
	if (rbx_video_setup(SCREEN_W, s_m->vis_y1 - s_m->vis_y0 + 1, 2048)) {
		tx_exit();
		return -1;
	}
	m68k_init();
	m68k_set_cpu_type(M68K_CPU_TYPE_68000);
	m68k_bus_reset_map();
	m68k_bus_map(0, s_m->rom_end, s_rom, s_rom_size, 1, 0);
	m68k_bus_map(0xb00000, 0xb00fff, s_pal, sizeof(s_pal), 1, 1);
	m68k_bus_map(0xe00000, 0xe03fff, s_x1->code, sizeof(s_x1->code), 1, 1);
	m68k_bus_map(0xf00000, 0xf03fff, s_ram, sizeof(s_ram), 1, 1);
	m68k_bus.read8 = main_read8;
	m68k_bus.read16 = main_read16;
	m68k_bus.write8 = main_write8;
	m68k_bus.write16 = main_write16;
	tx_reset();
	return 0;
}

static void tx_exit(void)
{
	if (s_x1)
		x1_exit(s_x1);
	free(s_x1);
	free(s_snap);
	s_x1 = s_snap = NULL;
	tsnd_exit();
}

static void tx_frame(void)
{
	rbx_audio.frame_mhz = s_m->rate_mhz;   // (the board's frame rate: a frame's sound is played in a 60 Hz frame, as its picture)
	int vblank = s_m->vis_y1 + 1;
	for (int line = 0; line < LINES; line++) {
		if (line == vblank) {
			vblank_capture();
			if (s_cchip) {   // taitox_cchip_state::interrupt
				m68k_bus_hold_irq(6);
				cchip_ext_interrupt(1);
				cchip_ext_interrupt(0);
			} else {
				m68k_bus_hold_irq(2);
			}
		}
		m68k_execute(tsnd_line_cycles(&s_cpu_frac, 8000000));
		if (s_cchip)
			cchip_run_clocks(&s_cc_frac, s_m->rate_mhz, LINES);
		tsnd_line();
	}
	tsnd_frame_end();
}

static int tx_state(void *ctx, int load)
{
	uint8_t cpu[1024];
	uint32_t csize = m68k_context_size();
	if (csize > sizeof(cpu))
		return -1;
	if (!load)
		m68k_get_context(cpu);
	uint32_t irq = m68k_bus_irq_pending();
	uint64_t fr = s_cpu_frac;
	int rc = 0;
	rc |= rbx_state_io(ctx, "m68k", cpu, csize);
	rc |= rbx_state_io(ctx, "irq", &irq, 4);
	rc |= rbx_state_io(ctx, "ram", s_ram, sizeof(s_ram));
	rc |= rbx_state_io(ctx, "pal", s_pal, sizeof(s_pal));
	rc |= rbx_state_io(ctx, "x1ctrl", s_x1->ctrl, sizeof(s_x1->ctrl));
	rc |= rbx_state_io(ctx, "x1ylow", s_x1->ylow, sizeof(s_x1->ylow));
	rc |= rbx_state_io(ctx, "x1code", s_x1->code, sizeof(s_x1->code));
	rc |= rbx_state_io(ctx, "x1bg", &s_x1->bgflag, 1);
	rc |= rbx_state_io(ctx, "frac", &fr, sizeof(fr));
	if (s_cchip) {
		rc |= rbx_state_io(ctx, "ccfrac", &s_cc_frac, sizeof(s_cc_frac));
		rc |= cchip_state(ctx, load);
	}
	rc |= tsnd_state(ctx, load);
	if (rc)
		return -1;
	if (load) {
		m68k_set_context(cpu);
		m68k_bus_set_irq_pending(irq);
		s_cpu_frac = fr;
	}
	return 0;
}

static int tx_save(void *ctx) { return tx_state(ctx, 0); }
static int tx_load(void *ctx) { return tx_state(ctx, 1); }
static int tx_supported(const char *machine) { return find_machine(machine) != NULL; }

const rbx_driver rbx_drv_taito_x = { "Taito X System", tx_init, tx_exit, tx_reset, tx_frame, tx_save, tx_load, tx_supported, tx_render };
