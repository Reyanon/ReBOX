/*  Taito O System (medal / gambling: Parent Jack, Eibise), ported from MAME src/mame/taito/taito_o.cpp
    (BSD-3-Clause, copyright-holders: Tomasz Slanina, Angelo Salese, Roberto Fresca, Grull Osgo), with the opto coin
    sensor (taitoio_opto.cpp, BSD-3-Clause, Angelo Salese) and the medal hopper (machine/ticket.cpp, BSD-3-Clause)
    modelled after MAME. Video: TC0080VCO (video/tc0080vco.c); sound: a YM2203 on the 68000 bus.
    The lamps, the hopper and the medal counts are given to the frontend (rbx_medal_*) for the cabinet overlay.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../rbx_core.h"
#include "../../cpu/m68k_bus.h"
#include "../../cpu/m68k/m68k.h"
#include "../../sound/rbx_ym.h"
#include "../video/tc0080vco.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define LINES 1024             // MAME screen: 1024 x 1024 at 60 Hz, scanline timer
#define VIS_Y0 48
#define VIS_Y1 495
#define SCREEN_W 512
#define CPU_CLOCK 12000000
#define YM_CLOCK 2000000
#define HOPPER_PERIOD (LINES * 6)   // 100 ms in scanlines

static uint8_t *s_rom;
static uint32_t s_rom_size;
static uint8_t s_nvram[0x10000];
static uint8_t s_palpage[0x1000];      // 0x500000-0x500fff page; palette at +0x800
static tc0080vco_t *s_vco, *s_snap;
static uint8_t s_snap_pal[0x800];
static int s_port_in0, s_port_in1, s_port_coin, s_port_dswa, s_port_dswb;
static uint64_t s_cpu_frac, s_ym_frac;
static int32_t *s_mix;
static int s_mix_cap, s_mix_n;
static char s_nv_path[256];
static int s_nv_fresh;                 // no NVRAM file: init it (MAME "Init NVRAM" key, IRQ 7)
static uint32_t s_frame;

// opto coin sensors (two slots) and the hopper, in scanlines
typedef struct { int sense, h, l; int64_t start, len, clear_at; int stage; } opto_t;
static opto_t s_opto[2];
static int64_t s_now;                  // scanlines since reset
static struct { int power, status, ff; int64_t next; uint32_t paid; } s_hopper;
static uint16_t s_outa, s_outb;
static uint32_t s_medals_in;
static int s_coin_prev[2];
static int64_t s_opto_off[2];       // scanline the passing medal leaves the sensor
#define MEDAL_LINES (LINES * 3)          // 3 frames per sensor phase (the game reads the sensors once a frame)
static volatile int s_medal_reset;     // set by the frontend, done at the next frame

// ---- opto (MAME taitoio_opto: -- -> H- -> HL -> -L -> --) ----
static void opto_sense(opto_t *o, int state)
{
	if (!o->sense && state) {
		o->start = s_now;
		o->sense = 1;
		o->h = 1;
		o->l = 0;
	} else if (o->sense && !state) {
		o->len = s_now - o->start;
		o->clear_at = s_now + o->len;
		o->stage = 1;
		o->sense = 0;
		o->l = 1;
	}
}

static void opto_tick(opto_t *o)
{
	if (o->stage && s_now >= o->clear_at) {
		if (o->stage == 1) {
			o->h = 0;
			o->clear_at = s_now + o->len;
			o->stage = 2;
		} else {
			o->l = 0;
			o->stage = 0;
		}
	}
}

// ---- hopper (MAME ticket_dispenser, hopper type, 100 ms) ----
static void hopper_motor(int state)
{
	if (state && !s_hopper.power) {
		s_hopper.next = s_now + HOPPER_PERIOD;
		s_hopper.power = 1;
		s_hopper.status = 0;
	} else if (!state && s_hopper.power) {
		if (!s_hopper.status)
			s_hopper.next = -1;
		s_hopper.power = 0;
	}
}

static void hopper_tick(void)
{
	if (s_hopper.next < 0 || s_now < s_hopper.next)
		return;
	int status = !s_hopper.status;
	s_hopper.next = s_hopper.power ? s_now + HOPPER_PERIOD : -1;
	if (status != s_hopper.status) {
		s_hopper.status = status;
		if (status)
			s_hopper.paid++;
		// coin out sensor flip flop: one IRQ 6 per medal (taitoo_state::hopper_int_cb)
		if (s_hopper.ff != status && !status)
			m68k_bus_hold_irq(6);
		s_hopper.ff = status;
	}
}

static void update_custom(void)
{
	rbx_port_custom(s_port_in0, 0xc000, (s_opto[0].h ? 0x4000 : 0) | (s_opto[0].l ? 0x8000 : 0));
	rbx_port_custom(s_port_in1, 0x0206, (s_opto[1].h ? 0x0002 : 0) | (s_opto[1].l ? 0x0004 : 0) | (s_hopper.status ? 0x0200 : 0));
}

// ---- sound: YM2203 straight on the 68000 bus ----
static uint8_t dswa_r(void) { return (uint8_t)rbx_port_read(s_port_dswa); }
static uint8_t dswb_r(void) { return (uint8_t)rbx_port_read(s_port_dswb); }

// the YM runs late: its clocks are kept until the CPU touches it or the frame ends (same output as running it after
// every scanline, at a fraction of the calls)
static uint32_t s_ym_pending;
static void ym_sync(void)
{
	if (s_ym_pending) {
		s_mix_n += rbx_ym_advance(0, s_ym_pending, s_mix + s_mix_n, s_mix_cap - s_mix_n);
		s_ym_pending = 0;
	}
}

// ---- main CPU ----
static uint16_t main_read16(uint32_t a)
{
	if (a == 0x200000) { update_custom(); return (uint16_t)rbx_port_read(s_port_in0); }
	if (a == 0x200002) { update_custom(); return (uint16_t)rbx_port_read(s_port_in1); }
	if (a >= 0x300000 && a <= 0x300003) { ym_sync(); return rbx_ym_read(0, (a >> 1) & 1); }
	return 0;
}

static uint8_t main_read8(uint32_t a)
{
	return (uint8_t)(main_read16(a & ~1u) >> ((a & 1) ? 0 : 8));
}

static void main_write16m(uint32_t a, uint16_t v, uint16_t mask)
{
	if (a == 0x200000) {
		s_outa = (uint16_t)((s_outa & ~mask) | (v & mask));
		if (mask & 0xff00)
			hopper_motor((v >> 9) & 1);
		return;
	}
	if (a == 0x200002) { s_outb = (uint16_t)((s_outb & ~mask) | (v & mask)); return; }   // lamps
	if (a >= 0x300000 && a <= 0x300003 && (mask & 0xff)) { ym_sync(); rbx_ym_write(0, (a >> 1) & 1, (uint8_t)v); return; }
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
static void to_render(void)
{
	const uint16_t *p = (const uint16_t *)s_snap_pal;
	for (int i = 0; i < 528; i++) {   // xBGR_555
		int b = (p[i] >> 10) & 31, g = (p[i] >> 5) & 31, r = p[i] & 31;
		r = (r << 3) | (r >> 2); g = (g << 3) | (g >> 2); b = (b << 3) | (b >> 2);
		rbx_video.palette[i] = (uint32_t)(r << 16 | g << 8 | b);
	}
	uint16_t *bm = rbx_video.bitmap;
	int pitch = rbx_video.pitch, w = SCREEN_W, h = VIS_Y1 - VIS_Y0 + 1;
	// MAME taitoo_state::screen_update (bitmap.fill(0) is left out: the opaque bg0 covers every pixel)
	vco_draw_bg0(s_snap, bm, pitch, w, h, 0, VIS_Y0, 1);
	int next = vco_draw_sprites(s_snap, bm, pitch, w, h, 0, VIS_Y0, 0x3f8 / 2, 1, 1);
	vco_draw_bg1(s_snap, bm, pitch, w, h, 0, VIS_Y0);
	vco_draw_tx(s_snap, bm, pitch, w, h, 0, VIS_Y0);
	vco_draw_sprites(s_snap, bm, pitch, w, h, 0, VIS_Y0, next, 1, 1);
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap->ram, s_vco->ram, sizeof(s_vco->ram));
	memcpy(s_snap_pal, s_palpage + 0x800, sizeof(s_snap_pal));
	rbx_video_frame_done();
}

// ---- init / frame ----
static void to_reset(void)
{
	vco_reset(s_vco);
	memset(s_palpage, 0, sizeof(s_palpage));
	memset(s_opto, 0, sizeof(s_opto));
	memset(s_coin_prev, 0, sizeof(s_coin_prev));
	memset(s_opto_off, 0, sizeof(s_opto_off));
	memset(&s_hopper, 0, sizeof(s_hopper));
	s_hopper.next = -1;
	s_outa = s_outb = 0;
	s_now = 0;
	s_frame = 0;
	rbx_ym_reset(0);
	m68k_bus_irq_reset();
	m68k_pulse_reset();
	s_cpu_frac = s_ym_frac = 0;
	s_ym_pending = 0;
	s_mix_n = 0;
}

static void to_exit(void);

static int to_init(const rbx_game *g)
{
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
	if (!s_vco || !s_snap || vco_init(s_vco, gfx, gsize, 1, 1, -2)) {
		rbx_fail("%s: out of memory (graphics)", g->set);
		to_exit();
		return -1;
	}
	// the snapshot shares the decoded tiles
	vco_share(s_snap, s_vco);
	s_port_in0 = rbx_port_find("IN0");
	s_port_in1 = rbx_port_find("IN1");
	s_port_coin = rbx_port_find("COIN");
	s_port_dswa = rbx_port_find("DSWA");
	s_port_dswb = rbx_port_find("DSWB");
	rbx_ym_config yc;
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM2203;
	yc.clock = YM_CLOCK;
	yc.gain[0] = yc.gain[1] = yc.gain[2] = yc.gain[3] = 256;
	yc.port_a_read = dswb_r;   // MAME: port A = DSWB, port B = DSWA
	yc.port_b_read = dswa_r;
	if (rbx_ym_init(0, &yc)) {
		to_exit();
		return -1;
	}
	s_mix_cap = (int)(rbx_ym_rate(0) / 50) + 64;
	s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	if (!s_mix || rbx_audio_setup(rbx_ym_rate(0)) || rbx_video_setup(SCREEN_W, VIS_Y1 - VIS_Y0 + 1, 528)) {
		to_exit();
		return -1;
	}
	// NVRAM (MAME default: all 0)
	memset(s_nvram, 0, sizeof(s_nvram));
	s_nv_path[0] = 0;
	s_nv_fresh = 1;
	extern const char *rbx_nvram_dir(void);
	const char *dir = rbx_nvram_dir();
	if (dir) {
		snprintf(s_nv_path, sizeof(s_nv_path), "%s%s.nv", dir, g->set);
		FILE *f = fopen(s_nv_path, "rb");
		if (f) {
			s_nv_fresh = fread(s_nvram, 1, sizeof(s_nvram), f) != sizeof(s_nvram);
			fclose(f);
		}
	}
	m68k_bus_swap16(s_nvram, sizeof(s_nvram));   // (file in MAME .nv order: big-endian words)
	m68k_init();
	m68k_set_cpu_type(M68K_CPU_TYPE_68000);
	m68k_bus_reset_map();
	m68k_bus_map(0, 0x01ffff, s_rom, s_rom_size, 1, 0);
	m68k_bus_map(0x100000, 0x11ffff, s_nvram, sizeof(s_nvram), 1, 1);
	m68k_bus_map(0x400000, 0x420fff, s_vco->ram, sizeof(s_vco->ram), 1, 1);
	m68k_bus_map(0x500000, 0x500fff, s_palpage, sizeof(s_palpage), 1, 1);
	m68k_bus.read8 = main_read8;
	m68k_bus.read16 = main_read16;
	m68k_bus.write8 = main_write8;
	m68k_bus.write16 = main_write16;
	to_reset();
	return 0;
}

static void to_exit(void)
{
	if (s_nv_path[0]) {
		FILE *f = fopen(s_nv_path, "wb");
		if (f) {
			static uint8_t be[0x10000];
			memcpy(be, s_nvram, sizeof(be));
			m68k_bus_swap16(be, sizeof(be));
			fwrite(be, 1, sizeof(be), f);
			fclose(f);
		}
		s_nv_path[0] = 0;
	}
	if (s_vco)
		vco_exit(s_vco);
	free(s_vco);
	free(s_snap);
	s_vco = s_snap = NULL;
	free(s_mix);
	s_mix = NULL;
	rbx_ym_exit(0);
}

static void to_frame(void)
{
	// coins: the COIN port drives the opto sensors (MAME PORT_WRITE_LINE coin_sense_w)
	// A press starts one medal passing the sensor for a fixed time (a real medal takes some 20 ms): holding the key or
	// the touch as long as the sensor phases made the game report a stuck medal ("COIN IN TIME OUT")
	// medal reset from the frontend: counters and NVRAM (credits, bookkeeping) cleared, then the NVRAM init as on a first start
	if (s_medal_reset) {
		s_medal_reset = 0;
		s_medals_in = 0;
		memset(s_nvram, 0, sizeof(s_nvram));
		to_reset();
		s_nv_fresh = 1;
	}
	uint32_t coin = rbx_port_read(s_port_coin);
	for (int i = 0; i < 2; i++) {
		int down = (coin >> i) & 1;
		// coin lock (MAME coin_lockout_w(0, !BIT(outa, 11))): a locked slot returns the medal
		if (down && !s_coin_prev[i] && !s_opto[i].sense && !s_opto[i].stage && (s_outa & 0x0800)) {
			opto_sense(&s_opto[i], 1);
			s_opto_off[i] = s_now + MEDAL_LINES;
			s_medals_in++;
		}
		s_coin_prev[i] = down;
	}
	// a first start without NVRAM: MAME's "Init NVRAM" key (IRQ 7) once the program runs
	// (the IRQ 7 routine clears the NVRAM and then loops until the machine is restarted: restart it, NVRAM kept)
	if (s_nv_fresh == 1 && s_frame == 60) {
		m68k_bus_hold_irq(7);
		s_nv_fresh = 2;
	} else if (s_nv_fresh == 2 && s_frame == 90) {
		m68k_bus_irq_reset();
		m68k_pulse_reset();
		s_nv_fresh = 0;
	}
	for (int line = 0; line < LINES; line++, s_now++) {
		if (line == 0)
			m68k_bus_hold_irq(5);    // I/O
		if (line == 448)
			m68k_bus_hold_irq(4);    // vblank
		if (line == VIS_Y1 + 1)
			vblank_capture();
		for (int i = 0; i < 2; i++) {
			if (s_opto[i].sense && s_now >= s_opto_off[i])
				opto_sense(&s_opto[i], 0);
			opto_tick(&s_opto[i]);
		}
		hopper_tick();
		m68k_execute(rbx_line_cycles(&s_cpu_frac, CPU_CLOCK, 60000, LINES));
		s_ym_pending += (uint32_t)rbx_line_cycles(&s_ym_frac, YM_CLOCK, 60000, LINES);
	}
	ym_sync();
	for (int i = 0; i < s_mix_n; i++)
		rbx_audio_put(s_mix[i]);
	memset(s_mix, 0, sizeof(int32_t) * (size_t)s_mix_n);
	s_mix_n = 0;
	s_frame++;
	if (getenv("RBX_DBG") && s_frame % 120 == 0)
		fprintf(stderr, "f%u pc %06x sr %04x irq %x\n", s_frame, m68k_get_reg(NULL, M68K_REG_PC), m68k_get_reg(NULL, M68K_REG_SR), m68k_bus_irq_pending());
}

static int to_state(void *ctx, int load)
{
	uint8_t cpu[1024];
	uint32_t csize = m68k_context_size();
	if (csize > sizeof(cpu))
		return -1;
	if (!load)
		m68k_get_context(cpu);
	uint32_t irq = m68k_bus_irq_pending();
	uint64_t fr[2] = { s_cpu_frac, s_ym_frac };
	int64_t misc[3] = { s_now, (int64_t)s_outa | (int64_t)s_outb << 16, (int64_t)s_frame };
	int rc = 0;
	rc |= rbx_state_io(ctx, "m68k", cpu, csize);
	rc |= rbx_state_io(ctx, "irq", &irq, 4);
	rc |= rbx_state_io(ctx, "nvram", s_nvram, sizeof(s_nvram));
	rc |= rbx_state_io(ctx, "pal", s_palpage, sizeof(s_palpage));
	rc |= rbx_state_io(ctx, "vco", s_vco->ram, sizeof(s_vco->ram));
	rc |= rbx_state_io(ctx, "opto", s_opto, sizeof(s_opto));
	rc |= rbx_state_io(ctx, "hopper", &s_hopper, sizeof(s_hopper));
	rc |= rbx_state_io(ctx, "frac", fr, sizeof(fr));
	rc |= rbx_state_io(ctx, "misc", misc, sizeof(misc));
	rc |= rbx_ym_state(0, ctx, load);
	if (rc)
		return -1;
	if (load) {
		m68k_set_context(cpu);
		m68k_bus_set_irq_pending(irq);
		s_cpu_frac = fr[0];
		s_ym_frac = fr[1];
		s_now = misc[0];
		s_outa = (uint16_t)misc[1];
		s_outb = (uint16_t)(misc[1] >> 16);
		s_frame = (uint32_t)misc[2];
	}
	return 0;
}

static int to_save(void *ctx) { return to_state(ctx, 0); }
static int to_load(void *ctx) { return to_state(ctx, 1); }
static int to_supported(const char *machine) { return !strcmp(machine, "taitoo"); }

const rbx_driver rbx_drv_taito_o = { "Taito O System", to_init, to_exit, to_reset, to_frame, to_save, to_load, to_supported, to_render };

// ---- medal cabinet state for the frontend overlay ----
uint32_t rbx_medal_lamps(void) { return s_outb; }
uint32_t rbx_medal_outputs(void) { return s_outa; }
uint32_t rbx_medal_paid(void) { return s_hopper.paid; }
uint32_t rbx_medal_inserted(void) { return s_medals_in; }
int rbx_medal_hopper_running(void) { return s_hopper.power; }
void rbx_medal_reset(void) { s_medal_reset = 1; }
