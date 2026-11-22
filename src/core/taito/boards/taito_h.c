/*  Taito H System (Syvalion, Record Breaker / Go For The Gold, Tetris, Dynamite League), ported from MAME
    src/mame/taito/taito_h.cpp, taito_h_v.cpp and taito_h.h (BSD-3-Clause, copyright-holders: Yochizo; thanks to
    Richard Bush), with the TC0040IOC / TC0220IOC I/O chips of taitoio.cpp (BSD-3-Clause).
    Memory maps, machine configurations, interrupts and the per game sprite priority loops follow MAME.
    Video: TC0080VCO (video/tc0080vco.c); sound board (Z80, YM2610, TC0140SYT): machine/taito_sound.c.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m68k_bus.h"
#include "../../cpu/m68k/m68k.h"
#include "../machine/taito_sound.h"
#include "../video/tc0080vco.h"

#include <stdlib.h>
#include <string.h>

// ---- machine configurations (MAME taitoh_state::recordbr ...) ----
enum { TH_SYVALION, TH_RECORDBR, TH_TETRISTH, TH_DLEAGUE };

typedef struct {
	const char *name;
	int kind;
	uint32_t rom_end;
	uint32_t ioc, syt;      // TC0040IOC / TC0220IOC base, TC0140SYT base
	int irq;                // vblank interrupt level (irq2_line_hold / irq1_line_hold)
	int vis_x0, vis_y0, vis_w, vis_h;
} machine_t;

static const machine_t s_machines[] = {
	{ "syvalion", TH_SYVALION, 0x7ffff, 0x200000, 0x300000, 2, 0, 3 * 16, 32 * 16, 25 * 16 },
	{ "recordbr", TH_RECORDBR, 0x7ffff, 0x200000, 0x300000, 2, 16, 2 * 16, 20 * 16, 15 * 16 },
	{ "tetristh", TH_TETRISTH, 0x3ffff, 0x300000, 0x200000, 2, 16, 2 * 16, 20 * 16, 15 * 16 },
	{ "dleague",  TH_DLEAGUE,  0x5ffff, 0x200000, 0x300000, 1, 16, 2 * 16, 20 * 16, 15 * 16 },
};

// the screen is 1024 lines at 60 Hz; the CPUs run in 256 slices of 4 lines (MAME interleave is 600 Hz), vblank
// (MAME set_vblank_int: the first line after the visible area) falls on a slice boundary
#define SLICES 256
#define LINES_PER_SLICE 4
#define CPU_CLOCK 12000000
#define PAL_ENTRIES 1024

static const machine_t *s_m;
static uint8_t *s_rom;
static uint32_t s_rom_size;
static uint8_t (*s_ram_hp)[0x10000];
#define s_ram (*s_ram_hp)
static uint8_t s_palpage[0x1000];      // 0x500000-0x500fff page; palette RAM at +0x800
static tc0080vco_t *s_vco, *s_snap;
static uint8_t s_snap_pal[0x800];
static uint8_t s_ioc_regs[8], s_ioc_port;
static int s_port_ids[8];              // I/O chip inputs 0, 1, 2, 3, 7
static int s_track_swap;               // syvalionp: trackball x and y swapped, not reversed
static uint8_t s_track[4];             // P2Y, P2X, P1Y, P1X of this frame (PORT_RESET: per frame deltas)
static uint64_t s_cpu_frac;

// ---- I/O chips ----
static uint8_t ioc_read(int offset)
{
	switch (offset) {
	case 0: case 1: case 2: case 3: case 7: return (uint8_t)rbx_port_read(s_port_ids[offset]);
	case 4: return s_ioc_regs[4];
	}
	return 0xff;
}

static void read_trackballs(void)
{
	// MAME syvalion_state m_io_track = { P2Y, P2X, P1Y, P1X }; Y is PORT_REVERSE (the prototype swaps x / y instead)
	for (int i = 0; i < 4; i++) {
		int player = i < 2 ? 2 : 1, yport = !(i & 1);
		int v;
		if (!s_track_swap)
			v = yport ? -rbx_analog_dpad(player, RBX_IPT_TRACKBALL_Y, 9) : rbx_analog_dpad(player, RBX_IPT_TRACKBALL_X, 9);
		else   // (d-pad: key delta 30 x sensitivity 30%)
			v = yport ? rbx_analog_dpad(player, RBX_IPT_TRACKBALL_X, 9) : rbx_analog_dpad(player, RBX_IPT_TRACKBALL_Y, 9);
		s_track[i] = (uint8_t)v;
	}
}

static uint8_t syvalion_input_bypass_r(void)
{
	uint8_t port = s_ioc_port;
	switch (port) {
	case 0x08: case 0x0a: case 0x0c: case 0x0e:   // trackball coords, bottom 8 bits
		return s_track[(port - 8) >> 1];
	case 0x09: case 0x0b: case 0x0d: case 0x0f:   // top 8 bits
		return (s_track[(port - 8) >> 1] & 0x80) ? 0xff : 0x00;
	}
	return ioc_read(port);
}

// ---- main CPU devices (byte lanes of umask16(0x00ff) devices are the odd addresses) ----
static uint8_t dev_read8(uint32_t a)
{
	if (s_m->kind == TH_DLEAGUE && a >= 0x200000 && a <= 0x20000f)   // TC0220IOC
		return (a & 1) ? ioc_read((int)((a - 0x200000) >> 1)) : 0;
	if (s_m->kind != TH_DLEAGUE && a >= s_m->ioc && a <= s_m->ioc + 3) {   // TC0040IOC
		if (!(a & 1))
			return 0;
		if (a == s_m->ioc + 1)
			return s_m->kind == TH_SYVALION ? syvalion_input_bypass_r() : ioc_read(s_ioc_port);
		return s_m->kind == TH_SYVALION ? s_ioc_port : 0;   // port_r / watchdog
	}
	if (a == s_m->syt + 3)
		return tc0140syt_master_comm_r(tsnd_syt());
	return 0;
}

static void dev_write8(uint32_t a, uint8_t v)
{
	if (s_m->kind == TH_DLEAGUE && a >= 0x200000 && a <= 0x20000f) {
		if (a & 1)
			s_ioc_regs[((a - 0x200000) >> 1) & 7] = v;   // 0 = watchdog, 4 = coin lockout / counters
		return;
	}
	if (s_m->kind != TH_DLEAGUE && a >= s_m->ioc && a <= s_m->ioc + 3) {
		if (a == s_m->ioc + 1) {
			if (s_ioc_port < 8)
				s_ioc_regs[s_ioc_port] = v;
		} else if (a == s_m->ioc + 3)
			s_ioc_port = v;
		return;
	}
	if (a == s_m->syt + 1) { tc0140syt_master_port_w(tsnd_syt(), v); return; }
	if (a == s_m->syt + 3) { tc0140syt_master_comm_w(tsnd_syt(), v); return; }
	// 0x430000 (TC0160ROM addressing mode?), 0x600000 (dleague, once per frame): ignored as in MAME
}

static uint8_t main_read8(uint32_t a) { return dev_read8(a); }
static uint16_t main_read16(uint32_t a) { return (uint16_t)(dev_read8(a) << 8 | dev_read8(a | 1)); }
static void main_write8(uint32_t a, uint8_t v) { dev_write8(a, v); }
static void main_write16(uint32_t a, uint16_t v)
{
	dev_write8(a, (uint8_t)(v >> 8));
	dev_write8(a | 1, (uint8_t)v);
}

// ---- video (MAME screen_update_syvalion / recordbr / dleague) ----
static void th_render(void)
{
	const uint16_t *p = (const uint16_t *)s_snap_pal;
	for (int i = 0; i < PAL_ENTRIES; i++) {   // xBGR_555
		int b = (p[i] >> 10) & 31, g = (p[i] >> 5) & 31, r = p[i] & 31;
		r = (r << 3) | (r >> 2); g = (g << 3) | (g >> 2); b = (b << 3) | (b >> 2);
		rbx_video.palette[i] = (uint32_t)(r << 16 | g << 8 | b);
	}
	tc0080vco_t *v = s_snap;
	uint16_t *bm = rbx_video.bitmap;
	int pitch = rbx_video.pitch, w = s_m->vis_w, h = s_m->vis_h, x0 = s_m->vis_x0, y0 = s_m->vis_y0;
	vco_sprite_t s;
	// (bitmap.fill(0) is left out: the opaque bg0 covers every pixel)
	vco_draw_bg0(v, bm, pitch, w, h, x0, y0, 1);
	switch (s_m->kind) {
	case TH_SYVALION:
		vco_draw_bg1(v, bm, pitch, w, h, x0, y0);
		vco_draw_sprites(v, bm, pitch, w, h, x0, y0, 0x3f8 / 2, 0, 0);
		break;
	case TH_RECORDBR:
	case TH_TETRISTH:
		for (int pri = 0; pri < 2; pri++) {
			if (pri)
				vco_draw_bg1(v, bm, pitch, w, h, x0, y0);
			for (int offs = 0x3f8 / 2; offs >= 0; offs -= 4) {
				if ((offs < 0x1b0) == (pri == 0))
					continue;
				if (vco_sprite_params(v, offs, 1, &s))
					vco_sprite_draw(v, bm, pitch, w, h, x0, y0, &s);
			}
		}
		break;
	case TH_DLEAGUE:
		for (int pri = 0; pri < 2; pri++) {
			if (pri)
				vco_draw_bg1(v, bm, pitch, w, h, x0, y0);
			for (int offs = 0x3f8 / 2; offs >= 0; offs -= 4) {
				if (!vco_sprite_params(v, offs, 0, &s))
					continue;
				int pribit = (v->ram[VCO_SPRITES + offs] & 0x1000) >> 12;
				if (s.zoomx < 63)
					pribit = 0;
				if (v->ram[VCO_SCROLL + 2] & 0x8000)
					pribit = 1;
				if (pri == pribit)
					vco_sprite_draw(v, bm, pitch, w, h, x0, y0, &s);
			}
		}
		break;
	}
	vco_draw_tx(v, bm, pitch, w, h, x0, y0);
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap->ram, s_vco->ram, sizeof(s_vco->ram));
	memcpy(s_snap_pal, s_palpage + 0x800, sizeof(s_snap_pal));
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

static void th_reset(void)
{
	memset(s_ram, 0, sizeof(s_ram));
	memset(s_palpage, 0, sizeof(s_palpage));
	memset(s_ioc_regs, 0, sizeof(s_ioc_regs));
	s_ioc_port = 0;
	vco_reset(s_vco);
	m68k_bus_irq_reset();
	m68k_pulse_reset();
	tsnd_reset();
	s_cpu_frac = 0;
}

static void th_exit(void);

static int port_type(const rbx_game *g, const char *tag)
{
	for (int i = 0; g->ports[i].tag; i++)
		if (!strcmp(g->ports[i].tag, tag) && g->ports[i].count)
			return g->fields[g->ports[i].first].type;
	return -1;
}

static int th_init(const rbx_game *g)
{
	if (!s_ram_hp && !(s_ram_hp = calloc(1, sizeof(*s_ram_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	s_m = find_machine(g->machine);
	if (!s_m) {
		rbx_fail("%s: machine %s is not emulated", g->set, g->machine);
		return -1;
	}
	s_rom = rbx_region("maincpu", &s_rom_size);
	uint32_t gsize;
	uint8_t *gfx = rbx_region("tc0080vco", &gsize);
	if (!s_rom || !gfx) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	m68k_bus_swap16(s_rom, s_rom_size);
	s_vco = (tc0080vco_t *)calloc(1, sizeof(tc0080vco_t));
	s_snap = (tc0080vco_t *)calloc(1, sizeof(tc0080vco_t));
	// MAME taitoh_base: set_offsets(1, 1), set_bgflip_yoffs(-2)
	if (!s_vco || !s_snap || vco_init(s_vco, gfx, gsize, 1, 1, -2)) {
		rbx_fail("%s: out of memory (graphics)", g->set);
		th_exit();
		return -1;
	}
	vco_share(s_snap, s_vco);
	static const char *const ioc_tags[8] = { "DSWA", "DSWB", "IN0", "IN1", 0, 0, 0, "IN2" };
	for (int i = 0; i < 8; i++)
		s_port_ids[i] = ioc_tags[i] ? rbx_port_find(ioc_tags[i]) : -1;
	s_track_swap = port_type(g, "P1X") == RBX_IPT_TRACKBALL_Y;
	memset(s_track, 0, sizeof(s_track));
	// sound: Z80 4 MHz, YM2610 8 MHz, MAME routes SSG 0.75, FM 1.0 / 1.0 to mono
	tsnd_config sc;
	memset(&sc, 0, sizeof(sc));
	sc.map = TSND_MAP_E000;
	sc.z80_clock = 4000000;
	sc.frame_rate_mhz = 60000;
	sc.lines = SLICES;
	sc.ym.type = RBX_YM2610;
	sc.ym.clock = 8000000;
	sc.ym.gain[0] = 192;
	sc.ym.gain[1] = sc.ym.gain[2] = 256;
	if (tsnd_init(&sc)) {
		rbx_fail("%s: sound init failed", g->set);
		th_exit();
		return -1;
	}
	if (rbx_video_setup(s_m->vis_w, s_m->vis_h, PAL_ENTRIES)) {
		th_exit();
		return -1;
	}
	m68k_init();
	m68k_set_cpu_type(M68K_CPU_TYPE_68000);
	m68k_bus_reset_map();
	m68k_bus_map(0, s_m->rom_end, s_rom, s_rom_size, 1, 0);
	m68k_bus_map(0x100000, 0x11ffff, s_ram, sizeof(s_ram), 1, 1);   // 64KB, mirror 0x010000
	m68k_bus_map(0x400000, 0x420fff, s_vco->ram, sizeof(s_vco->ram), 1, 1);
	m68k_bus_map(0x500000, 0x500fff, s_palpage, sizeof(s_palpage), 1, 1);
	m68k_bus.read8 = main_read8;
	m68k_bus.read16 = main_read16;
	m68k_bus.write8 = main_write8;
	m68k_bus.write16 = main_write16;
	th_reset();
	return 0;
}

static void th_exit(void)
{
	if (s_vco)
		vco_exit(s_vco);
	free(s_vco);
	free(s_snap);
	s_vco = s_snap = NULL;
	tsnd_exit();
	free(s_ram_hp);
	s_ram_hp = NULL;
}

static void th_frame(void)
{
	if (s_m->kind == TH_SYVALION)
		read_trackballs();
	int vblank = (s_m->vis_y0 + s_m->vis_h) / LINES_PER_SLICE;
	for (int slice = 0; slice < SLICES; slice++) {
		if (slice == vblank) {
			vblank_capture();
			m68k_bus_hold_irq(s_m->irq);
		}
		m68k_execute(tsnd_line_cycles(&s_cpu_frac, CPU_CLOCK));
		tsnd_line();
	}
	tsnd_frame_end();
}

static int th_state(void *ctx, int load)
{
	uint8_t cpu[1024];
	uint32_t csize = m68k_context_size();
	if (csize > sizeof(cpu))
		return -1;
	if (!load)
		m68k_get_context(cpu);
	uint32_t irq = m68k_bus_irq_pending();
	uint64_t fr = s_cpu_frac;
	uint8_t io[9];
	memcpy(io, s_ioc_regs, 8);
	io[8] = s_ioc_port;
	int rc = 0;
	rc |= rbx_state_io(ctx, "m68k", cpu, csize);
	rc |= rbx_state_io(ctx, "irq", &irq, 4);
	rc |= rbx_state_io(ctx, "ram", s_ram, sizeof(s_ram));
	rc |= rbx_state_io(ctx, "pal", s_palpage, sizeof(s_palpage));
	rc |= rbx_state_io(ctx, "vco", s_vco->ram, sizeof(s_vco->ram));
	rc |= rbx_state_io(ctx, "ioc", io, sizeof(io));
	rc |= rbx_state_io(ctx, "frac", &fr, sizeof(fr));
	rc |= tsnd_state(ctx, load);
	if (rc)
		return -1;
	if (load) {
		m68k_set_context(cpu);
		m68k_bus_set_irq_pending(irq);
		s_cpu_frac = fr;
		memcpy(s_ioc_regs, io, 8);
		s_ioc_port = io[8];
	}
	return 0;
}

static int th_save(void *ctx) { return th_state(ctx, 0); }
static int th_load(void *ctx) { return th_state(ctx, 1); }
static int th_supported(const char *machine) { return find_machine(machine) != NULL; }

const rbx_driver rbx_drv_taito_h = { "Taito H System", th_init, th_exit, th_reset, th_frame, th_save, th_load, th_supported, th_render };
