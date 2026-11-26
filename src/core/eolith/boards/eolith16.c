/*  Eolith 16-bit hardware: KlonDike+. Ported from MAME 0.289 src/mame/eolith/eolith16.cpp (BSD-3-Clause,
    copyright-holders: Pierpaolo Prazzoli) and eolith_speedup.cpp (BSD-3-Clause, copyright-holders: Tomasz Slanina,
    Pierpaolo Prazzoli).
    Hyperstone E1-16T at 60 MHz (cpu/e132xs, 16-bit bus), an 8-bit frame buffer in two banks (the screen shows the
    first, 320x200, a fixed 3-3-2 palette), OKI M6295 at 1 MHz, 93C66 EEPROM (8-bit); MAME's vblank speedup.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/e132xs/e132xs.h"
#include "../../sound/okim6295.h"
#include "../../machine/eepromser.h"

#include <stdlib.h>
#include <string.h>

#define CPU_CLOCK 60000000
#define FRAME_MHZ 60000
#define LINES 262
#define VIS_W 320
#define VIS_H 200

enum { E16_PA_MAIN = RBX_PA_DRV, E16_PA_OKI, E16_PA_END };
static const char *const s_pa_names[] = { "e1-16t", "oki" };

static e132xs_t s_cpu;
static okim6295_t s_oki;
static eepser_t s_eeprom;
static uint8_t *s_ram;          // 2 MB at 0 (host-order 32-bit words)
static uint8_t *s_vram;         // 0x20000 bytes: two 0x10000 banks at 0x50000000 (host-order 32-bit words)
static uint8_t s_iram[0x1000];
static uint8_t *s_snap;         // the frame shown (320x200 pens)
static uint32_t s_spd_addr;
static int s_port_special, s_port_system, s_port_inputs;
static int s_line;
static struct {
	uint32_t vrambank, spd_vblank, spinning;
	uint64_t cpu_frac, aud_frac;
} s_st;

static void map_vram(void)
{
	e132xs_map(&s_cpu, 0x50000000, 0x5000ffff, s_vram + (s_st.vrambank & 1) * 0x10000, 0x10000, 1, 1);
}

// KlonDike+'s two vblank waits in RAM (LDW.D / LDHU.D 0xffea0000 / ANDNI 0xff7f / BE, BNE back; MAME runs them too,
// its speedup pc is the second one's, inside vblank): ~87K rounds a frame. A round that reads what the last one did,
// exactly a round later (no interrupt, nothing else between), repeats until the line's segment ends: those rounds are
// taken at once (e132xs_skip_rounds; the same cycles, registers and reads)
static const uint16_t s_wait_code[2][6] = {
	{ 0x9112, 0xefea, 0x0000, 0x7723, 0xff7f, 0xf275 },   // 0x1a034: while (!(IN & 0x80))
	{ 0x9112, 0xefea, 0x0000, 0x7723, 0xff7f, 0xf375 },   // 0x1a040: while (IN & 0x80)
};
static struct { uint32_t pc, v; int64_t t; } s_poll;   // the last wait round's read (pc 0: none)

static void wait_skip(uint32_t v)
{
	const uint32_t pc = e132xs_pc(&s_cpu);
	const int w = pc == 0x1a03a ? 0 : pc == 0x1a046 ? 1 : -1;
	if (w < 0 || s_cpu.abort) {
		s_poll.pc = 0;
		return;
	}
	v = v << 16 | (uint16_t)rbx_port_read(s_port_system);   // (a 32-bit load reads that half too)
	const int64_t t = e132xs_total_cycles(&s_cpu);
	const int round = (int)(2 * s_cpu.cc1 + s_cpu.cc2);   // (LD, ANDNI, the branch taken)
	if (s_poll.pc == pc && s_poll.v == v && t - s_poll.t == round &&
	    eepser_do_read_93cxx(&s_eeprom, t + s_cpu.icount) == ((v >> 20) & 1)) {   // (the ready bit: monotonic)
		int ok = 1;
		for (int i = 0; i < 6 && ok; i++)
			ok = e132xs_read16(&s_cpu, pc - 6 + 2 * (uint32_t)i) == s_wait_code[w][i];
		if (ok)
			e132xs_skip_rounds(&s_cpu, round);
	}
	s_poll.pc = pc;
	s_poll.v = v;
	s_poll.t = e132xs_total_cycles(&s_cpu);
}

static uint16_t custom_r(void)
{
	if (!s_st.spd_vblank && s_spd_addr && e132xs_pc(&s_cpu) == s_spd_addr) {
		s_st.spinning = 1;
		e132xs_abort(&s_cpu);
	}
	uint32_t v = rbx_port_read(s_port_special) & ~0x90u;
	v |= (uint32_t)eepser_do_read_93cxx(&s_eeprom, e132xs_total_cycles(&s_cpu)) << 4;
	v |= (uint32_t)(s_line >= 240) << 7;   // speedup_vblank_r
	if (s_spd_addr)
		wait_skip(v);
	return (uint16_t)v;
}

static void eeprom_w(uint16_t data)
{
	const uint32_t b = ((data & 0x80) >> 7) ^ 1;
	if (b != s_st.vrambank) {
		s_st.vrambank = b;
		map_vram();
	}
	const int64_t now = e132xs_total_cycles(&s_cpu);
	eepser_cs_write(&s_eeprom, (data >> 4) & 1, now);
	eepser_clk_write(&s_eeprom, (data >> 5) & 1, now);
	eepser_di_write(&s_eeprom, (data >> 6) & 1);
}

static uint16_t bus_r16(void *user, uint32_t a)
{
	(void)user;
	switch (a) {
	case 0xffe40000: s_poll.pc = 0; return oki_read(&s_oki);   // (the OKI on the low byte, the upper unmapped)
	case 0xffea0000: return custom_r();
	case 0xffea0002: return (uint16_t)rbx_port_read(s_port_system);
	case 0xffec0002: s_poll.pc = 0; return (uint16_t)rbx_port_read(s_port_inputs);
	}
	return 0;
}
static uint32_t bus_r32(void *user, uint32_t a) { return (uint32_t)bus_r16(user, a) << 16 | bus_r16(user, a + 2); }
static uint8_t bus_r8(void *user, uint32_t a)
{
	const uint16_t v = bus_r16(user, a & ~1u);
	return (uint8_t)((a & 1) ? v : v >> 8);
}
static void bus_w16(void *user, uint32_t a, uint16_t v)
{
	(void)user;
	s_poll.pc = 0;
	if (a == 0xffe40000)
		oki_write(&s_oki, (uint8_t)v);
	else if (a == 0xffe80000)
		eeprom_w(v);
}
static void bus_w32(void *user, uint32_t a, uint32_t v)
{
	bus_w16(user, a, (uint16_t)(v >> 16));
	bus_w16(user, a + 2, (uint16_t)v);
}
static void bus_w8(void *user, uint32_t a, uint8_t v)
{
	(void)user;
	s_poll.pc = 0;
	if (a == 0xffe40001)
		oki_write(&s_oki, v);
	else if (a == 0xffe80001)
		eeprom_w(v);   // (an 8-bit write: the upper byte reads 0 to the handler)
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	for (int i = 0; i < VIS_W * VIS_H; i++)   // (m_vram from its start: the first bank)
		s_snap[i] = s_vram[i ^ 3];
	rbx_video_frame_done();
}

static void e16_render(void)
{
	for (int y = 0; y < VIS_H; y++) {
		uint16_t *d = rbx_video.bitmap + y * rbx_video.pitch;
		const uint8_t *s = s_snap + y * VIS_W;
		for (int x = 0; x < VIS_W; x++)
			d[x] = s[x];
	}
}

static void e16_reset(void)
{
	memset(&s_st, 0, sizeof(s_st));
	s_poll.pc = 0;
	map_vram();
	e132xs_reset(&s_cpu);
	oki_reset(&s_oki);
	s_line = 0;
}

static void e16_exit(void);

static int e16_init(const rbx_game *g)
{
	uint32_t rom_size, data_size, oki_size;
	uint8_t *rom = rbx_region("maincpu", &rom_size), *data = rbx_region("maindata", &data_size);
	uint8_t *oki = rbx_region("oki", &oki_size);
	if (!rom || !data || !oki)
		return -1;
	s_ram = calloc(1, 0x200000);
	s_vram = calloc(1, 0x20000);
	s_snap = calloc(1, VIS_W * VIS_H);
	if (!s_ram || !s_vram || !s_snap || e132xs_init(&s_cpu, E132XS_E1, 1)) {
		rbx_fail("%s: out of memory", g->set);
		e16_exit();
		return -1;
	}
	memset(s_iram, 0, sizeof(s_iram));
	e132xs_swap32(rom, rom_size);
	e132xs_swap32(data, data_size);
	s_spd_addr = !strcmp(g->set, "klondkp") ? 0x0001a046 : 0;
	e132xs_t *c = &s_cpu;
	e132xs_map(c, 0x00000000, 0x001fffff, s_ram, 0x200000, 1, 1);
	e132xs_map(c, 0xc0000000, 0xdfffffff, s_iram, sizeof(s_iram), 1, 1);
	e132xs_map(c, 0xff000000, 0xff1fffff, data, data_size, 1, 0);
	e132xs_map(c, 0xfff80000, 0xffffffff, rom, rom_size, 1, 0);
	c->read8 = bus_r8;
	c->read16 = bus_r16;
	c->read32 = bus_r32;
	c->write8 = bus_w8;
	c->write16 = bus_w16;
	c->write32 = bus_w32;
	oki_init(&s_oki, 1000000, 1, oki, oki_size, 256);
	eepser_init(&s_eeprom, EEPSER_93CXX, 512, 8, CPU_CLOCK);
	s_eeprom.op_time[0] = s_eeprom.op_time[2] = CPU_CLOCK / 4000;   // erase / write time 250 us
	eepser_nv_load(&s_eeprom, g->set);
	s_port_special = rbx_port_find("SPECIAL");
	s_port_system = rbx_port_find("SYSTEM");
	s_port_inputs = rbx_port_find("INPUTS");
	if (rbx_video_setup(VIS_W, VIS_H, 256) || rbx_audio_setup(s_oki.rate)) {
		e16_exit();
		return -1;
	}
	for (int cc = 0; cc < 256; cc++) {   // eolith16_palette
		const int r = 0x21 * (cc & 1) + 0x47 * ((cc >> 1) & 1) + 0x97 * ((cc >> 2) & 1);
		const int gg = 0x21 * ((cc >> 3) & 1) + 0x47 * ((cc >> 4) & 1) + 0x97 * ((cc >> 5) & 1);
		const int b = 0x55 * ((cc >> 6) & 1) + 0xaa * ((cc >> 7) & 1);
		rbx_video.palette[cc] = (uint32_t)(r << 16 | gg << 8 | b);
	}
	rbx_prof_name(RBX_PA_DRV, s_pa_names, E16_PA_END - RBX_PA_DRV);
	e16_reset();
	return 0;
}

static void e16_exit(void)
{
	eepser_nv_save(&s_eeprom);
	e132xs_exit(&s_cpu);
	free(s_ram);
	free(s_vram);
	free(s_snap);
	s_ram = s_vram = s_snap = NULL;
}

static void e16_frame(void)
{
	for (int line = 0; line < LINES; line++) {
		s_line = line;
		if (line == 0)
			s_st.spd_vblank = 0;
		if (line == 240) {
			s_st.spinning = 0;
			s_st.spd_vblank = 1;
		}
		if (line == VIS_H)
			vblank_capture();
		rbx_prof_set(E16_PA_MAIN);
		const int n = rbx_line_cycles(&s_st.cpu_frac, CPU_CLOCK, FRAME_MHZ, LINES);
		if (s_st.spinning) {
			e132xs_eat(&s_cpu, n);
		} else {
			e132xs_execute(&s_cpu, n);
			if (s_st.spinning)
				e132xs_eat(&s_cpu, 0);
		}
		rbx_prof_set(E16_PA_OKI);
		const int na = rbx_line_cycles(&s_st.aud_frac, s_oki.rate, FRAME_MHZ, LINES);
		for (int i = 0; i < na; i++)
			rbx_audio_put(oki_sample(&s_oki));
	}
	rbx_prof_set(RBX_PA_DRIVER);
}

static int e16_state(void *ctx, int load)
{
	if (rbx_state_io(ctx, "e132xs", &s_cpu, E132XS_STATE_SIZE) || rbx_state_io(ctx, "ram", s_ram, 0x200000) ||
	    rbx_state_io(ctx, "vram", s_vram, 0x20000) || rbx_state_io(ctx, "iram", s_iram, sizeof(s_iram)) ||
	    rbx_state_io(ctx, "misc", &s_st, sizeof(s_st)) ||
	    rbx_state_io(ctx, "oki", s_oki.v, sizeof(s_oki.v) + sizeof(int)) || eepser_state(&s_eeprom, ctx, load, "eeprom"))
		return -1;
	if (load) {
		e132xs_post_load(&s_cpu);
		s_poll.pc = 0;
		map_vram();
	}
	return 0;
}

static int e16_save(void *ctx) { return e16_state(ctx, 0); }
static int e16_load(void *ctx) { return e16_state(ctx, 1); }
static int e16_supported(const char *machine) { return !strcmp(machine, "eolith16"); }

const rbx_driver rbx_drv_eolith16 = { "Eolith 16-bit", e16_init, e16_exit, e16_reset, e16_frame, e16_save, e16_load,
                                      e16_supported, e16_render, NULL };
