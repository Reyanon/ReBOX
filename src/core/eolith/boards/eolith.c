/*  Eolith 32-bit hardware, "Gradation 2D system": Hidden Catch 1 / 2 / 2000 / 3, New Hidden Catch, Iron Fortress, Linky
    Pipe, Raccoon World, Puzzle King, Candy Candy, Land Breaker, Penfan Girls, Steal See, Fortress 2 Blue Arcade.
    Ported from MAME 0.289 src/mame/eolith/eolith.cpp (BSD-3-Clause, copyright-holders: Tomasz Slanina, Pierpaolo
    Prazzoli) and eolith_speedup.cpp / eolith_speedup.h (the same).
    Main CPU: Hyperstone E1-32N at 45 / 50 MHz (cpu/e132xs), a 16-bit frame buffer drawn by the CPU (two 320x240
    buffers in a 336-pixel pitch, RGB555), 93C66 EEPROM (8-bit). Sound: an 80C32 sound CPU (music data banked, a latch
    from the main CPU) sends MIDI-like bytes over its serial port to the QS1000 (its own 8052 and the wavetable engine,
    sound/qs1000). MAME's speedup is kept: the vblank poll at the set's pc spins the main CPU until line 240.
    Hidden Catch 3's touch pens are light guns here (the d-pad moves them), drawn as crosshairs (MAME PORT_CROSSHAIR).
    The protected sets (the AT89C52's internal ROM not dumped) run the substitute ROM with MAME's patch of the check.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/e132xs/e132xs.h"
#include "../../cpu/mcs51/mcs51.h"
#include "../../sound/qs1000.h"
#include "../../machine/eepromser.h"

#include <stdlib.h>
#include <string.h>

#define FRAME_MHZ 60000      // set_refresh_hz(60)
#define LINES 262            // set_size(512, 262)
#define VIS_W 320
#define VIS_H 240
#define SND_CLOCK 12000000   // I8032
#define QS_CLOCK 24000000    // QS1000 (its 8052 and the wavetable engine)
#define AUD_RATE (QS_CLOCK / QS1000_RATE_DIV)

enum { EO_PA_MAIN = RBX_PA_DRV, EO_PA_SNDCPU, EO_PA_QSCPU, EO_PA_QSWAVE, EO_PA_END };
static const char *const s_pa_names[] = { "e1-32n", "80c32", "qs1000 8052", "qs1000 voices" };

// MAME eolith_speedup_table (eolith.cpp's sets)
typedef struct { const char *set; uint32_t addr; } eo_speedup;
static const eo_speedup s_speedups[] = {
	{ "linkypip", 0x4000825c }, { "ironfort", 0x40020854 }, { "ironfortc", 0x40020234 }, { "hidnctch", 0x4000bba0 },
	{ "hidnctcha", 0x4000bba0 }, { "raccoon", 0x40008204 }, { "puzzlekg", 0x40029458 }, { "hidctch2", 0x40009524 },
	{ "hidctch2a", 0x40029B58 }, { "landbrk", 0x40023574 }, { "landbrka", 0x4002446c }, { "landbrkb", 0x40023B28 },
	{ "nhidctch", 0x40012778 }, { "hidctch3", 0x4001f6a0 }, { "fort2b", 0x000081e0 }, { "fort2ba", 0x000081e0 },
	{ "penfan", 0x4001FA66 }, { "penfana", 0x4001FAb6 }, { "candy", 0x4001990C }, { "hidnc2k", 0x40016824 },
};

static e132xs_t s_cpu;
static mcs51_t s_snd;
static qs1000_t s_qs;
static eepser_t s_eeprom;
static uint8_t *s_ram0, *s_ram4;      // 0x00000000 / 0x40000000, 2 MB each (host-order 32-bit words)
static uint8_t *s_vram;               // two buffers of 0x40000 bytes (host-order 32-bit words)
static uint8_t *s_vram_cur;           // the buffer the CPU sees
static uint8_t s_iram[0x1000];        // the E1-32's 4 KB internal RAM
static uint8_t *s_maincpu, *s_maindata, *s_sounddata;
static uint32_t s_maincpu_size, s_maindata_size, s_sounddata_size;
static uint16_t *s_snap;              // the frame shown (320x240)
static int s_pen_snap[2][2];
static uint32_t s_cpu_clock;
static int s_hidctch3, s_stealsee;
static uint32_t s_spd_addr;           // the speedup pc (0 = none)
static int s_port_in0, s_port_dsw;
static int s_line;
static struct {
	uint32_t buffer, latch, latch_pending, sndbank;
	uint32_t spd_vblank, spinning, rand;
	int32_t pen[2][2];
	uint64_t cpu_frac, snd_frac, qs_frac, aud_frac;
} s_st;
static int32_t s_mix_l[1024], s_mix_r[1024];

static void map_vram(void)
{
	s_vram_cur = s_vram + (s_st.buffer & 1) * 0x40000;
	e132xs_map(&s_cpu, 0x90000000, 0x9003ffff, s_vram_cur, 0x40000, 1, 0);
	e132xs_map_skip15(&s_cpu, 0x90000000, 0x9003ffff, s_vram_cur, 0x40000);   // (eolith_vram_w; allocated in eo_init)
}

// ---- main CPU I/O ----
static uint32_t custom_r(void)
{
	// eolith_e1_speedup_state_base::speedup_read
	if (!s_st.spd_vblank && s_spd_addr && e132xs_pc(&s_cpu) == s_spd_addr) {
		s_st.spinning = 1;   // spin_until_trigger(1000): until line 240
		e132xs_abort(&s_cpu);
	}
	const int vblank = s_line >= 240;   // speedup_vblank_r: screen vpos() >= 240
	if (s_stealsee && e132xs_pc(&s_cpu) == 0x400081ec && !s_st.spd_vblank)   // stealsee_speedup_vblank_r
		e132xs_burn(&s_cpu, 500);
	uint32_t v = rbx_port_read(s_port_in0) & ~0x48u;
	v |= (uint32_t)eepser_do_read_93cxx(&s_eeprom, e132xs_total_cycles(&s_cpu)) << 3;
	v |= (uint32_t)vblank << 6;   // (custom read lines are not inverted)
	s_st.rand = s_st.rand * 1103515245u + 12345u;   // (machine().rand() & 0x300)
	return (v & ~0x300u) | ((s_st.rand >> 8) & 0x300);
}

static void systemcontrol_w(uint32_t data)
{
	const uint32_t b = (data & 0x80) >> 7;
	if (b != s_st.buffer) {
		s_st.buffer = b;
		map_vram();
	}
	// EEPROMOUT (fields in their order: cs, clk, di)
	const int64_t now = e132xs_total_cycles(&s_cpu);
	eepser_cs_write(&s_eeprom, (data >> 1) & 1, now);
	eepser_clk_write(&s_eeprom, (data >> 2) & 1, now);
	eepser_di_write(&s_eeprom, (data >> 3) & 1);
}

static void soundlatch_w(uint8_t v)
{
	s_st.latch = v;
	s_st.latch_pending = 1;
	mcs51_set_input(&s_snd, MCS51_INT0_LINE, 1);
}

static uint32_t pen_r(int p)
{
	// 320 x 240
	return (uint32_t)(s_st.pen[p][0] + s_st.pen[p][1] * 168 * 2);
}

static uint32_t bus_r32(void *user, uint32_t a)
{
	(void)user;
	if (a == 0xfc000000)
		return custom_r();
	if (a == 0xfca00000)
		return rbx_port_read(s_port_dsw);
	if (s_hidctch3) {
		if ((a & ~0x80000u) == 0xfce00000)
			return pen_r(0);
		if ((a & ~0x80000u) == 0xfcf00000)
			return pen_r(1);
	}
	return 0;
}
static uint16_t bus_r16(void *user, uint32_t a)
{
	const uint32_t v = bus_r32(user, a & ~3u);
	return (uint16_t)((a & 2) ? v : v >> 16);
}
static uint8_t bus_r8(void *user, uint32_t a)
{
	return (uint8_t)(bus_r32(user, a & ~3u) >> ((3 - (a & 3)) * 8));
}

static void bus_w32(void *user, uint32_t a, uint32_t v)
{
	(void)user;
	// (VRAM: e132xs_map_skip15, MAME eolith_vram_w: whole 16-bit writes with bit 15 clear only)
	if (a == 0xfc400000) {
		systemcontrol_w(v);
	} else if (a == 0xfc800000) {
		soundlatch_w((uint8_t)v);
	}
}
static void bus_w16(void *user, uint32_t a, uint16_t v)
{
	(void)user;
	if ((a & ~3u) == 0xfc400000)
		systemcontrol_w((a & 2) ? v : (uint32_t)v << 16);
	else if ((a & ~3u) == 0xfc800000 && (a & 2))
		soundlatch_w((uint8_t)v);
}
static void bus_w8(void *user, uint32_t a, uint8_t v)
{
	(void)user;
	if ((a & ~3u) == 0xfc400000)
		systemcontrol_w((uint32_t)v << ((3 - (a & 3)) * 8));
	else if (a == 0xfc800003)
		soundlatch_w(v);
	// (VRAM: byte writes are not whole 16-bit writes)
}

// ---- sound CPU ----
static uint8_t snd_xdata_r(void *user, uint16_t a)
{
	(void)user;
	if (a < 0x8000) {
		const uint32_t o = (s_st.sndbank << 15) | a;
		return o < s_sounddata_size ? s_sounddata[o] : 0;
	}
	if (a == 0x8000) {   // generic_latch_8 read: the pending flag (INT0) goes down
		s_st.latch_pending = 0;
		mcs51_set_input(&s_snd, MCS51_INT0_LINE, 0);
		return (uint8_t)s_st.latch;
	}
	return 0;
}
static void snd_port_out(void *user, int port, uint8_t data)
{
	(void)user;
	if (port == 1)
		s_st.sndbank = data & 0x0f;   // .... xxxx - Data ROM bank (32kB)
	// (P3: the serial TxD goes to the QS1000 byte by byte, snd_uart_tx)
}
static void snd_uart_tx(void *user, int data)
{
	(void)user;
	qs1000_serial_in(&s_qs, data);
}
static uint8_t qs_p1_in(void *user) { (void)user; return 1; }   // Sound banking? (must be 1)
static uint8_t qs_p3_in(void *user) { (void)user; return 1; }   // (RxD: the idle line; the bytes come whole)

// ---- video ----
static void vblank_capture(void)
{
	rbx_video_wait_idle();
	const uint8_t *src = s_vram + ((s_st.buffer & 1) ^ 1) * 0x40000;
	for (int y = 0; y < VIS_H; y++) {
		uint16_t *d = s_snap + y * VIS_W;
		const uint32_t row = (uint32_t)y * 336 * 2;
		for (int x = 0; x < VIS_W; x++)
			d[x] = *(const uint16_t *)(src + ((row + (uint32_t)x * 2) ^ 2)) & 0x7fff;
	}
	memcpy(s_pen_snap, s_st.pen, sizeof(s_pen_snap));
	rbx_video_frame_done();
}

static void draw_cross(int cx, int cy, uint16_t col)
{
	for (int i = -6; i <= 6; i++) {
		const int x = cx + i, y = cy + i;
		if (x >= 0 && x < VIS_W && cy >= 0 && cy < VIS_H)
			rbx_video.bitmap[cy * rbx_video.pitch + x] = col;
		if (y >= 0 && y < VIS_H && cx >= 0 && cx < VIS_W)
			rbx_video.bitmap[y * rbx_video.pitch + cx] = col;
	}
}

static void eo_render(void)
{
	for (int y = 0; y < VIS_H; y++)
		memcpy(rbx_video.bitmap + y * rbx_video.pitch, s_snap + y * VIS_W, VIS_W * 2);
	if (s_hidctch3) {   // the pens' crosshairs (MAME PORT_CROSSHAIR: the pen x 2 on the 320x240 screen)
		draw_cross(s_pen_snap[0][0] * 2, s_pen_snap[0][1] * 2, 0x7fff);
		draw_cross(s_pen_snap[1][0] * 2, s_pen_snap[1][1] * 2, 0x7c1f);
	}
}

// ---- frame ----
static void eo_reset(void)
{
	memset(&s_st, 0, sizeof(s_st));
	s_st.rand = 0x1234567;
	for (int p = 0; p < 2; p++) {
		s_st.pen[p][0] = 80;
		s_st.pen[p][1] = 60;
	}
	memcpy(s_pen_snap, s_st.pen, sizeof(s_pen_snap));
	map_vram();
	e132xs_reset(&s_cpu);
	mcs51_reset(&s_snd);
	qs1000_reset(&s_qs);
	mcs51_set_input(&s_snd, MCS51_INT1_LINE, 1);   // (eolith_state::machine_reset)
	mcs51_set_input(&s_snd, MCS51_INT0_LINE, 0);
	s_line = 0;
}

static void eo_exit(void);

static int eo_init(const rbx_game *g)
{
	uint32_t snd_size, qcpu_size, qrom_size;
	s_maincpu = rbx_region("maincpu", &s_maincpu_size);
	s_maindata = rbx_region("maindata", &s_maindata_size);
	uint8_t *snd = rbx_region("soundcpu", &snd_size);
	s_sounddata = rbx_region("sounddata", &s_sounddata_size);
	uint8_t *qcpu = rbx_region("qs1000:cpu", &qcpu_size);
	uint8_t *qrom = rbx_region("qs1000", &qrom_size);
	if (!s_maincpu || !s_maindata || !snd || !s_sounddata || !qcpu || !qrom)
		return -1;
	s_ram0 = calloc(1, 0x200000);
	s_ram4 = calloc(1, 0x200000);
	s_vram = calloc(1, 0x80000);
	s_snap = calloc(1, VIS_W * VIS_H * 2);
	if (!s_ram0 || !s_ram4 || !s_vram || !s_snap || e132xs_init(&s_cpu, E132XS_E1, 0) ||
	    e132xs_map_skip15(&s_cpu, 0x90000000, 0x9003ffff, s_vram, 0x40000)) {
		rbx_fail("%s: out of memory", g->set);
		eo_exit();
		return -1;
	}
	memset(s_iram, 0, sizeof(s_iram));
	// the CPU's big-endian images as host-order 32-bit words
	e132xs_swap32(s_maincpu, s_maincpu_size);
	e132xs_swap32(s_maindata, s_maindata_size);
	// the protected sets: MAME patch_mcu_protection (the BR of the MCU check made a NOP)
	const char *init = strchr(g->machine, ':');
	init = init ? init + 1 : "";
	uint32_t patch = 0;
	if (!strcmp(init, "landbrka")) patch = 0x14f00;
	else if (!strcmp(init, "landbrkb")) patch = 0x14da8;
	else if (!strcmp(init, "hidctch2")) patch = 0x0bcc8;
	else if (!strcmp(init, "hidnc2k")) patch = 0x17b2c;
	if (patch && patch + 4 <= s_maincpu_size) {
		uint32_t *w = (uint32_t *)(s_maincpu + patch);
		*w = (*w & 0xffff) | 0x03000000;
	}
	s_hidctch3 = !strncmp(g->machine, "hidctch3", 8);
	s_stealsee = !strcmp(g->set, "stealsee");
	s_cpu_clock = !strncmp(g->machine, "eolith50", 8) || s_hidctch3 ? 50000000 : !strncmp(g->machine, "ironfort", 8) ? 44900000
	                                                                                                         : 45000000;
	s_spd_addr = 0;
	for (size_t i = 0; i < sizeof(s_speedups) / sizeof(s_speedups[0]); i++)
		if (!strcmp(s_speedups[i].set, g->set))
			s_spd_addr = s_speedups[i].addr;

	// main CPU map (eolith_map)
	e132xs_t *c = &s_cpu;
	e132xs_map(c, 0x00000000, 0x001fffff, s_ram0, 0x200000, 1, 1);   // fort2b wants ram here
	e132xs_map(c, 0x40000000, 0x401fffff, s_ram4, 0x200000, 1, 1);
	e132xs_map(c, 0xc0000000, 0xdfffffff, s_iram, sizeof(s_iram), 1, 1);   // (iram_4k_map, mirror 0x1ffff000)
	e132xs_map(c, 0xfd000000, 0xfeffffff, s_maindata, s_maindata_size, 1, 0);
	e132xs_map(c, 0xfff80000, 0xffffffff, s_maincpu, s_maincpu_size, 1, 0);
	c->read8 = bus_r8;
	c->read16 = bus_r16;
	c->read32 = bus_r32;
	c->write8 = bus_w8;
	c->write16 = bus_w16;
	c->write32 = bus_w32;

	// sound CPU (I8032: an 8052 without ROM) and the QS1000
	mcs51_init(&s_snd, snd, snd_size);
	s_snd.is_8052 = 1;
	s_snd.xdata_r = snd_xdata_r;
	s_snd.port_out = snd_port_out;
	s_snd.uart_tx = snd_uart_tx;
	qs1000_init(&s_qs, QS_CLOCK, qcpu, qcpu_size, qrom, qrom_size);
	s_qs.p1_in = qs_p1_in;
	s_qs.p3_in = qs_p3_in;

	// EEPROM_93C66_8BIT, erase / write time 250 us
	eepser_init(&s_eeprom, EEPSER_93CXX, 512, 8, s_cpu_clock);
	s_eeprom.op_time[0] = s_eeprom.op_time[2] = (int64_t)s_cpu_clock / 4000;
	eepser_nv_load(&s_eeprom, g->set);

	s_port_in0 = rbx_port_find("IN0");
	s_port_dsw = rbx_port_find("DSW1");

	if (rbx_video_setup(VIS_W, VIS_H, 0x8000) || rbx_audio_setup_stereo(AUD_RATE)) {
		eo_exit();
		return -1;
	}
	for (int i = 0; i < 0x8000; i++) {   // PALETTE RGB_555 (pal5bit)
		const int r = (i >> 10) & 31, gg = (i >> 5) & 31, b = i & 31;
		rbx_video.palette[i] = (uint32_t)((r << 3 | r >> 2) << 16 | (gg << 3 | gg >> 2) << 8 | (b << 3 | b >> 2));
	}
	rbx_prof_name(RBX_PA_DRV, s_pa_names, EO_PA_END - RBX_PA_DRV);
	eo_reset();
	return 0;
}

static void eo_exit(void)
{
	eepser_nv_save(&s_eeprom);
	e132xs_exit(&s_cpu);
	free(s_ram0);
	free(s_ram4);
	free(s_vram);
	free(s_snap);
	s_ram0 = s_ram4 = s_vram = NULL;
	s_snap = NULL;
}

static void pens_frame(void)
{
	for (int p = 0; p < 2; p++) {
		// PORT_MINMAX(0, 159) / (0, 119): the analog device's moves at half speed, the d-pad's 2 a frame
		int x = s_st.pen[p][0] + rbx_analog_dpad(p + 1, RBX_IPT_LIGHTGUN_X, 4) / 2;
		int y = s_st.pen[p][1] + rbx_analog_dpad(p + 1, RBX_IPT_LIGHTGUN_Y, 4) / 2;
		s_st.pen[p][0] = x < 0 ? 0 : x > 159 ? 159 : x;
		s_st.pen[p][1] = y < 0 ? 0 : y > 119 ? 119 : y;
	}
}

static void eo_frame(void)
{
	if (s_hidctch3)
		pens_frame();
	for (int line = 0; line < LINES; line++) {
		s_line = line;
		// the scanline timer (eolith_speedup): vblank flag, the trigger at the resume line (240)
		if (line == 0)
			s_st.spd_vblank = 0;
		if (line == 240) {
			s_st.spinning = 0;
			s_st.spd_vblank = 1;
			vblank_capture();   // (screen update at the start of vblank)
		}
		rbx_prof_set(EO_PA_MAIN);
		const int n = rbx_line_cycles(&s_st.cpu_frac, s_cpu_clock, FRAME_MHZ, LINES);
		if (s_st.spinning) {
			e132xs_eat(&s_cpu, n);
		} else {
			e132xs_execute(&s_cpu, n);
			if (s_st.spinning)
				e132xs_eat(&s_cpu, 0);   // (the rest of the line spun)
		}
		rbx_prof_set(EO_PA_SNDCPU);
		mcs51_execute(&s_snd, rbx_line_cycles(&s_st.snd_frac, SND_CLOCK / 12, FRAME_MHZ, LINES));
		rbx_prof_set(EO_PA_QSCPU);
		qs1000_run(&s_qs, rbx_line_cycles(&s_st.qs_frac, QS_CLOCK / 12, FRAME_MHZ, LINES));
		rbx_prof_set(EO_PA_QSWAVE);
		int na = rbx_line_cycles(&s_st.aud_frac, AUD_RATE, FRAME_MHZ, LINES);
		if (na > (int)(sizeof(s_mix_l) / sizeof(s_mix_l[0])))
			na = (int)(sizeof(s_mix_l) / sizeof(s_mix_l[0]));
		memset(s_mix_l, 0, (size_t)na * sizeof(int32_t));
		memset(s_mix_r, 0, (size_t)na * sizeof(int32_t));
		qs1000_render(&s_qs, s_mix_l, s_mix_r, na);
		for (int i = 0; i < na; i++)   // (MAME: add_int(..., 32768 << 12))
			rbx_audio_put_stereo(s_mix_l[i] >> 12, s_mix_r[i] >> 12);
	}
	rbx_prof_set(RBX_PA_DRIVER);
}

static int eo_state(void *ctx, int load)
{
	if (rbx_state_io(ctx, "e132xs", &s_cpu, E132XS_STATE_SIZE) || rbx_state_io(ctx, "ram0", s_ram0, 0x200000) ||
	    rbx_state_io(ctx, "ram4", s_ram4, 0x200000) || rbx_state_io(ctx, "vram", s_vram, 0x80000) ||
	    rbx_state_io(ctx, "iram", s_iram, sizeof(s_iram)) || rbx_state_io(ctx, "misc", &s_st, sizeof(s_st)) ||
	    rbx_state_io(ctx, "snd", &s_snd.st, sizeof(s_snd.st)) || rbx_state_io(ctx, "snd52", &s_snd.st2, sizeof(s_snd.st2)) ||
	    rbx_state_io(ctx, "snduart", &s_snd.uart, sizeof(s_snd.uart)) || qs1000_state(&s_qs, ctx, load) ||
	    eepser_state(&s_eeprom, ctx, load, "eeprom"))
		return -1;
	if (load) {
		e132xs_post_load(&s_cpu);
		map_vram();
	}
	return 0;
}

static int eo_save(void *ctx) { return eo_state(ctx, 0); }
static int eo_load(void *ctx) { return eo_state(ctx, 1); }

static int eo_supported(const char *machine)
{
	return !strncmp(machine, "eolith45", 8) || !strncmp(machine, "eolith50", 8) || !strncmp(machine, "ironfort", 8) ||
	       !strncmp(machine, "hidctch3", 8);
}

const rbx_driver rbx_drv_eolith = { "Eolith Gradation 2D", eo_init, eo_exit, eo_reset, eo_frame, eo_save, eo_load,
                                    eo_supported, eo_render, NULL };
