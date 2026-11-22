/*  IGS PGM (PolyGame Master), ported from MAME src/mame/igs/pgm.cpp (BSD-3-Clause, copyright-holders: David Haywood,
    ElSemi), with the V3021 RTC of src/devices/machine/v3021.cpp (BSD-3-Clause, copyright-holders: Angelo Salese).
    68000 at 20 MHz with the BIOS at 0 and the cartridge at 0x100000; Z80 at 8.4672 MHz in 64 KB of RAM the 68000
    uploads, with three latches and the ICS2115 (sound/ics2115.c); IGS023 video (video/igs023.c); the cartridge's
    protection device as a pgm_prot (pgm_prot_*.c).
    Timing: 264 lines of 64 us (59.19 Hz), the 68000 and the Z80 a scanline at a time; the 68000's accesses to the Z80's
    RAM and the latches first run the Z80 up to the 68000's time (MAME's retry_access), and the Z80 runs in pieces
    that end at the ICS2115's timer interrupts.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "pgm.h"

#include "../../cpu/m68k_bus.h"
#include "../../cpu/m68k/m68k.h"
#include "../../cpu/z80/z80.h"
#include "../../sound/ics2115.h"
#include "../video/igs023.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define LINES 264
#define VBLANK_LINE 224
#define FRAME_MHZ 59186               // 50 MHz / 5 / (640 x 264)
#define M68K_CLOCK 20000000
#define Z80_CLOCK 8467200             // 33.8688 MHz / 4 (the ICS2115 runs at 4 ticks a Z80 cycle)
#define OUT_RATE 33075                // the ICS2115 with 32 voices: 33.8688 MHz / (32 x 32)

uint8_t *pgm_rom;
uint32_t pgm_rom_size;
uint16_t pgm_mainram[0x10000];
const rbx_game *pgm_game;
int pgm_irq4_disabled;

static const pgm_prot *s_prot;
static const char *s_init;
static igs023_t *s_vid;
static igs023_snap *s_snap;
static uint16_t s_pal[0xa00];
static int s_port[6];   // P1P2, P3P4, Service, DSW, Region, RegionHack
static char s_nv_path[256];

// sound
static z80 s_z80;
static uint8_t s_zram[0x10000];
static const uint8_t *s_zmap[256];
static int s_zint;
static uint8_t s_latch[3];
static uint8_t s_z80_halt;
static ics2115_t s_ics;
static unsigned long s_ics_zcyc;   // the Z80 time the ICS2115 has been run up to
static int32_t *s_mix;
static uint32_t s_rs_frac;

// timing
static uint64_t s_m_frac, s_z_frac;
static unsigned long s_zline_len;
static int s_mline_len, s_line;
static int s_m68k_frame;   // 68000 cycles of the lines before the current one

// V3021 RTC
static struct {
	uint8_t cs, io, addr, data, cnt, mode, pad[2];
	uint8_t ram[16];
	uint8_t sec, min, hour, day, wday, month, year, wnum;
	uint32_t frames;
} s_rtc;

// ---- shared helpers ----
uint32_t pgm_region_port(void) { return s_port[4] >= 0 ? rbx_port_read(s_port[4]) : 0; }
int pgm_port(const char *tag) { return rbx_port_find(tag); }
uint32_t pgm_region_hack(void) { return s_port[5] >= 0 ? rbx_port_read(s_port[5]) & 0xff : 0xff; }
int pgm_line_cycles(uint64_t *frac, uint32_t clock) { return rbx_line_cycles(frac, clock, FRAME_MHZ, LINES); }
uint8_t *pgm_region(const char *name, uint32_t *size) { return rbx_region(name, size); }
int pgm_m68k_cycles(void) { return s_m68k_frame + m68k_cycles_run(); }
void pgm_m68k_yield(void) { m68k_end_timeslice(); }
static int s_m68k_spin;   // pgm_m68k_spin: the 68000 sleeps until the next interrupt
void pgm_m68k_spin(void)
{
	s_m68k_spin = 1;
	m68k_end_timeslice();
}
void pgm_map(uint32_t start, uint32_t end, void *mem, uint32_t size, int writable)
{
	m68k_bus_map(start, end, mem, size, 1, writable);
}

// ---- ICS2115 / Z80 ----
static void ics_irq(int state) { s_zint = state; }

static void ics_sync(void)
{
	if (s_z80.cyc > s_ics_zcyc) {
		ics2115_advance(&s_ics, (uint32_t)(s_z80.cyc - s_ics_zcyc) * 4);
		s_ics_zcyc = s_z80.cyc;
	}
}

static uint8_t z80_read(void *u, uint16_t a)
{
	(void)u;
	return s_zram[a];
}

static void z80_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	s_zram[a] = v;
}

static uint8_t z80_in(z80 *z, uint8_t port)
{
	(void)port;
	const uint16_t a = z->mem_ptr;
	if (a >= 0x8000 && a <= 0x8003) {
		ics_sync();
		return ics2115_read(&s_ics, a & 3);
	}
	if (a >= 0x8100 && a <= 0x81ff) return s_latch[2];
	if (a >= 0x8200 && a <= 0x82ff) return s_latch[0];
	if (a >= 0x8400 && a <= 0x84ff) return s_latch[1];
	return 0;
}

static void z80_out(z80 *z, uint8_t port, uint8_t v)
{
	(void)port;
	const uint16_t a = z->mem_ptr;
	if (a >= 0x8000 && a <= 0x8003) {
		ics_sync();
		ics2115_write(&s_ics, a & 3, v);
	} else if (a >= 0x8100 && a <= 0x81ff)
		s_latch[2] = v;
	else if (a >= 0x8200 && a <= 0x82ff)
		s_latch[0] = v;
	else if (a >= 0x8400 && a <= 0x84ff)
		s_latch[1] = v;
}

static void z80_hook(void)
{
	s_z80.read_byte = z80_read;
	s_z80.write_byte = z80_write;
	s_z80.port_in = z80_in;
	s_z80.port_out = z80_out;
	s_z80.read_map = s_zmap;
}

static void z80_reset_cpu(void)
{
	const unsigned long cyc = s_z80.cyc;
	z80_init(&s_z80);
	z80_hook();
	s_z80.cyc = cyc;
}

// run the Z80 (and the ICS2115 with it) up to Z80 time `target`
static void z80_run_to(unsigned long target)
{
	while (s_z80.cyc < target) {
		if (s_z80_halt) {
			s_z80.cyc = target;
			break;
		}
		unsigned long next = target;
		const uint64_t ev = ics2115_next_event(&s_ics);
		if (ev != UINT64_MAX) {
			const uint64_t evz = s_ics_zcyc + (ev + 3) / 4;
			if (evz < next)
				next = evz > s_z80.cyc ? (unsigned long)evz : s_z80.cyc + 1;
		}
		z80_run(&s_z80, next, &s_zint, 0);
		ics_sync();
	}
	ics_sync();
}

// the 68000 touches the sound side: the Z80 (on the worker, rbx_aux) first ends the current line. (MAME runs it up to
// the 68000's time, retry_access; here it is up to a line ahead, the same for every run)
static void z80_catch_up(void) { rbx_aux_sync(); }

// the worker's job: the Z80 and the ICS2115 up to the line's end
static unsigned long s_ztarget;
static void z80_job(void) { z80_run_to(s_ztarget); }

// ---- V3021 ----
static uint8_t bcd(int v) { return (uint8_t)(((v / 10) << 4) | (v % 10)); }

static void rtc_copy_clock_to_ram(void)
{
	const uint8_t *c = &s_rtc.sec;
	s_rtc.ram[1] = 0;
	static const uint8_t order[8] = { 0, 1, 2, 3, 5, 6, 4, 7 };   // sec min hour day month year wday wnum
	for (int i = 0; i < 8; i++)
		if (s_rtc.ram[2 + i] != c[order[i]])
			s_rtc.ram[1] |= (uint8_t)(1 << i);
	for (int i = 0; i < 8; i++)
		s_rtc.ram[2 + i] = c[order[i]];
}

static void rtc_set_now(void)
{
	// (a fixed start, as the board would keep it: the same run gives the same picture)
	s_rtc.sec = 0;
	s_rtc.min = 0;
	s_rtc.hour = 0;
	s_rtc.day = bcd(1);
	s_rtc.month = bcd(1);
	s_rtc.year = bcd(0);
	s_rtc.wday = bcd(6);
	s_rtc.wnum = bcd(2);
	s_rtc.frames = 0;
}

static int from_bcd(uint8_t v) { return (v >> 4) * 10 + (v & 15); }

static void rtc_tick(void)   // a second
{
	int s = from_bcd(s_rtc.sec) + 1, m = from_bcd(s_rtc.min), h = from_bcd(s_rtc.hour);
	if (s >= 60) { s = 0; m++; }
	if (m >= 60) { m = 0; h++; }
	if (h >= 24) {
		h = 0;
		int d = from_bcd(s_rtc.day) + 1, w = from_bcd(s_rtc.wday) % 7 + 1;
		if (d > 28) {   // (months are not followed: a day past the 28th starts the next month)
			d = 1;
			int mo = from_bcd(s_rtc.month) + 1;
			if (mo > 12) {
				mo = 1;
				s_rtc.year = bcd((from_bcd(s_rtc.year) + 1) % 100);
			}
			s_rtc.month = bcd(mo);
		}
		s_rtc.day = bcd(d);
		s_rtc.wday = bcd(w);
	}
	s_rtc.sec = bcd(s);
	s_rtc.min = bcd(m);
	s_rtc.hour = bcd(h);
}

static void rtc_cs(void)   // cs_w(1); cs_w(0)
{
	if (!s_rtc.mode) {
		s_rtc.addr = (uint8_t)(((s_rtc.addr >> 1) | (s_rtc.io << 3)) & 0xf);
		if (++s_rtc.cnt >= 4) {
			if (s_rtc.addr == 0xe) {
				if (!(s_rtc.ram[0] & 0x10)) {   // copy RAM to clock
					s_rtc.ram[1] = 0;
					s_rtc.sec = s_rtc.ram[2];
					s_rtc.min = s_rtc.ram[3];
					s_rtc.hour = s_rtc.ram[4];
					s_rtc.day = s_rtc.ram[5];
					s_rtc.month = s_rtc.ram[6];
					s_rtc.year = s_rtc.ram[7];
					s_rtc.wday = s_rtc.ram[8];
					s_rtc.wnum = s_rtc.ram[9];
				}
			} else if (s_rtc.addr == 0xf)
				rtc_copy_clock_to_ram();
			else {
				s_rtc.data = s_rtc.ram[s_rtc.addr & 0xf];
				s_rtc.mode = 1;
			}
			s_rtc.cnt = 0;
		}
	} else {
		s_rtc.data = (uint8_t)((s_rtc.data >> 1) | (s_rtc.io << 7));
		if (++s_rtc.cnt >= 8) {
			if (s_rtc.addr != 1 && s_rtc.addr <= 9)
				s_rtc.ram[s_rtc.addr & 0xf] = s_rtc.data;
			s_rtc.mode = 0;
			s_rtc.cnt = 0;
		}
	}
}

static uint8_t rtc_read(void)
{
	const uint8_t r = s_rtc.data & 1;
	rtc_cs();
	return r;
}

static void rtc_write(uint8_t v)
{
	s_rtc.io = v & 1;
	rtc_cs();
}

// ---- 68000 ----
static uint16_t main_read16(uint32_t a)
{
	uint16_t v;
	if (s_prot && s_prot->read16 && s_prot->read16(a, &v))
		return v;
	if (a >= 0xb00000 && a <= 0xb0ffff)
		return igs023_reg_r(s_vid, a - 0xb00000, s_line);
	if (a >= 0xc10000 && a <= 0xc1ffff) {
		z80_catch_up();
		const uint32_t o = a & 0xfffe;
		return (uint16_t)(s_zram[o] << 8 | s_zram[o + 1]);
	}
	switch (a & ~1u) {
	case 0xc00002: z80_catch_up(); return s_latch[0];
	case 0xc00004: z80_catch_up(); return s_latch[1];
	case 0xc00006: return rtc_read();
	case 0xc0000c: z80_catch_up(); return s_latch[2];
	case 0xc08000: return (uint16_t)rbx_port_read(s_port[0]);
	case 0xc08002: return (uint16_t)rbx_port_read(s_port[1]);
	case 0xc08004: return (uint16_t)rbx_port_read(s_port[2]);
	case 0xc08006: return (uint16_t)rbx_port_read(s_port[3]);
	}
	return 0;
}

static uint8_t main_read8(uint32_t a)
{
	if (a >= 0xc10000 && a <= 0xc1ffff) {
		z80_catch_up();
		return s_zram[a & 0xffff];
	}
	const uint16_t w = main_read16(a & ~1u);
	return (uint8_t)((a & 1) ? w : w >> 8);
}

static void main_write16m(uint32_t a, uint16_t v, uint16_t mask)
{
	if (s_prot && s_prot->write16 && s_prot->write16(a, v, mask))
		return;
	if (a >= 0xb00000 && a <= 0xb0ffff) {
		igs023_reg_w(s_vid, a - 0xb00000, v, mask);
		return;
	}
	if (a >= 0xc10000 && a <= 0xc1ffff) {
		z80_catch_up();
		const uint32_t o = a & 0xfffe;
		if (mask & 0xff00)
			s_zram[o] = (uint8_t)(v >> 8);
		if (mask & 0x00ff)
			s_zram[o + 1] = (uint8_t)v;
		return;
	}
	const int lo = (mask & 0xff) != 0;
	switch (a & ~1u) {
	case 0xc00002:
		if (lo) {
			z80_catch_up();
			s_latch[0] = (uint8_t)v;
			z80_gen_nmi(&s_z80);
		}
		break;
	case 0xc00004:
		if (lo) {
			z80_catch_up();
			s_latch[1] = (uint8_t)v;
		}
		break;
	case 0xc00006:
		if (lo)
			rtc_write((uint8_t)v);
		break;
	case 0xc00008:   // z80_reset_w
		z80_catch_up();
		if (v == 0x5050) {
			ics2115_reset(&s_ics);
			s_zint = 0;
			s_z80_halt = 0;
			z80_reset_cpu();
		} else
			s_z80_halt = 1;   // (the games stop the Z80 during uploads)
		break;
	case 0xc0000c:
		if (lo) {
			z80_catch_up();
			s_latch[2] = (uint8_t)v;
		}
		break;
	}
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
#define BANDS IGS023_BANDS
static int s_band_next, s_band_left, s_band_active, s_band_helpers;
static uint8_t s_band_pri[IGS023_W * IGS023_H];

static void run_band(int b)
{
	const int y0 = IGS023_H * b / BANDS, y1 = IGS023_H * (b + 1) / BANDS;
	igs023_draw_lines(s_vid, s_snap, rbx_video.bitmap, rbx_video.pitch, y0, y1, s_band_pri + y0 * IGS023_W);
}

static void pgm_render(void)
{
	for (int i = 0; i < 0xa00; i++) {   // xRGB_555
		const uint16_t p = s_snap->pal[i];
		int r = (p >> 10) & 31, g = (p >> 5) & 31, b = p & 31;
		r = (r << 3) | (r >> 2);
		g = (g << 3) | (g >> 2);
		b = (b << 3) | (b >> 2);
		rbx_video.palette[i] = (uint32_t)(r << 16 | g << 8 | b);
	}
	igs023_prepass(s_vid, s_snap);
	// the frame in bands: the render thread takes them, and the threads that wait for it help (pgm_render_help)
	__atomic_store_n(&s_band_next, 0, __ATOMIC_SEQ_CST);
	__atomic_store_n(&s_band_left, BANDS, __ATOMIC_SEQ_CST);
	__atomic_store_n(&s_band_active, 1, __ATOMIC_SEQ_CST);
	for (;;) {
		const int b = __atomic_fetch_add(&s_band_next, 1, __ATOMIC_ACQ_REL);
		if (b >= BANDS)
			break;
		run_band(b);
		__atomic_sub_fetch(&s_band_left, 1, __ATOMIC_ACQ_REL);
	}
	while (__atomic_load_n(&s_band_left, __ATOMIC_ACQUIRE) > 0)
		;
	__atomic_store_n(&s_band_active, 0, __ATOMIC_SEQ_CST);
	while (__atomic_load_n(&s_band_helpers, __ATOMIC_ACQUIRE) > 0)
		;
}

static int pgm_render_help(int helper)
{
	const int one = helper & 0x100;
	helper &= 0xff;
	if (helper < 1 || helper > 2 || !__atomic_load_n(&s_band_active, __ATOMIC_SEQ_CST))
		return 0;
	__atomic_add_fetch(&s_band_helpers, 1, __ATOMIC_SEQ_CST);
	int did = 0;
	if (__atomic_load_n(&s_band_active, __ATOMIC_SEQ_CST))
		for (;;) {
			const int b = __atomic_fetch_add(&s_band_next, 1, __ATOMIC_ACQ_REL);
			if (b >= BANDS)
				break;
			run_band(b);
			__atomic_sub_fetch(&s_band_left, 1, __ATOMIC_ACQ_REL);
			did = 1;
			if (one)
				break;
		}
	__atomic_sub_fetch(&s_band_helpers, 1, __ATOMIC_SEQ_CST);
	return did;
}

static void vblank(void)
{
	// MAME: the screen is drawn first (with the sprite list of the last vblank), then screen_vblank takes the new list
	rbx_video_wait_idle();
	igs023_snapshot(s_vid, s_snap, s_pal);
	rbx_video_frame_done();
	igs023_get_sprites(s_vid, pgm_mainram);
	m68k_bus_hold_irq(6);
	s_m68k_spin = 0;
	if (s_prot && s_prot->vblank)
		s_prot->vblank();
}

// ---- setup ----
static const pgm_prot *prot_of(const char *machine)
{
	if (!strcmp(machine, "pgm")) return NULL;
	if (!strcmp(machine, "pgm_asic3")) return &pgm_prot_asic3;
	if (!strcmp(machine, "pgm_012_025_drgw2")) return &pgm_prot_025_012;
	if (!strncmp(machine, "pgm_022_025", 11)) return &pgm_prot_025_022;
	if (!strcmp(machine, "pgm_028_025_ol")) return &pgm_prot_025_028;
	if (!strncmp(machine, "pgm_arm_type1", 13)) return &pgm_prot_arm1;
	if (!strncmp(machine, "pgm_arm_type2", 13)) return &pgm_prot_arm2;
	if (!strncmp(machine, "pgm_arm_type3", 13)) return &pgm_prot_arm3;
	return (const pgm_prot *)-1;   // not emulated
}

static const char *init_of(const char *set)
{
	static const struct { const char *set, *machine, *init; } t[] = {
#include "pgm_sets.inc"
	};
	for (size_t i = 0; i < sizeof(t) / sizeof(t[0]); i++)
		if (!strcmp(t[i].set, set))
			return t[i].init;
	return "pgm";
}

static void pgm_exit(void);

static void pgm_reset(void)
{
	rbx_aux_sync();
	// (main RAM is the battery-backed NVRAM: kept)
	memset(s_pal, 0, sizeof(s_pal));
	igs023_reset(s_vid);
	memset(s_zram, 0, sizeof(s_zram));
	memset(s_latch, 0, sizeof(s_latch));
	ics2115_reset(&s_ics);
	z80_init(&s_z80);
	z80_hook();
	s_zint = 0;
	s_z80_halt = 1;   // (machine_reset: the Z80 halted until the 68000 starts it)
	s_ics_zcyc = 0;
	s_rs_frac = 0;
	s_rtc.cs = s_rtc.io = s_rtc.addr = s_rtc.data = s_rtc.cnt = s_rtc.mode = 0;
	s_rtc.ram[0] = s_rtc.ram[1] = 0;
	m68k_bus_irq_reset();
	m68k_pulse_reset();
	s_m68k_spin = 0;
	s_m_frac = s_z_frac = 0;
	if (s_prot && s_prot->reset)
		s_prot->reset();
}

static int pgm_init(const rbx_game *g)
{
	pgm_game = g;
	s_prot = prot_of(g->machine);
	if (s_prot == (const pgm_prot *)-1) {
		rbx_fail("%s: protection (%s) not emulated", g->set, g->machine);
		s_prot = NULL;
		return -1;
	}
	s_init = init_of(g->set);
	uint32_t gfx_size, a_size, b_size, ics_size;
	pgm_rom = rbx_region("maincpu", &pgm_rom_size);
	uint8_t *gfx = rbx_region("igs023", &gfx_size);
	uint8_t *a = rbx_region("igs023:sprcol", &a_size);
	uint8_t *b = rbx_region("igs023:sprmask", &b_size);
	uint8_t *ics = rbx_region("ics", &ics_size);
	if (!pgm_rom || !gfx || !ics) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	m68k_bus_swap16(pgm_rom, pgm_rom_size);
	s_vid = (igs023_t *)calloc(1, sizeof(igs023_t));
	s_snap = (igs023_snap *)calloc(1, sizeof(igs023_snap));
	s_mix = (int32_t *)calloc(4096, sizeof(int32_t));
	if (!s_vid || !s_snap || !s_mix || ics2115_init(&s_ics, ics, ics_size, ics_irq, 4096)) {
		rbx_fail("%s: out of memory", g->set);
		pgm_exit();
		return -1;
	}
	igs023_init(s_vid, gfx, gfx_size, a, a_size, b, b_size);
	static const char *const tags[6] = { "P1P2", "P3P4", "Service", "DSW", "Region", "RegionHack" };
	for (int i = 0; i < 6; i++)
		s_port[i] = rbx_port_find(tags[i]);
	for (int i = 0; i < 256; i++)
		s_zmap[i] = s_zram + i * 256;
	if (rbx_audio_setup(OUT_RATE) || rbx_video_setup(IGS023_W, IGS023_H, 0xa00)) {
		pgm_exit();
		return -1;
	}
	rbx_audio.frame_mhz = FRAME_MHZ;
	// main RAM: the NVRAM file, else zeros (MAME's nvram DEFAULT_ALL_0)
	memset(pgm_mainram, 0, sizeof(pgm_mainram));
	s_nv_path[0] = 0;
	{
		extern const char *rbx_nvram_dir(void);
		const char *dir = rbx_nvram_dir();
		if (dir) {
			snprintf(s_nv_path, sizeof(s_nv_path), "%s%s.nv", dir, g->set);
			FILE *f = fopen(s_nv_path, "rb");
			if (f) {
				if (fread(pgm_mainram, 1, sizeof(pgm_mainram), f) != sizeof(pgm_mainram))
					memset(pgm_mainram, 0, sizeof(pgm_mainram));
				fclose(f);
			}
		}
	}
	rtc_set_now();
	rtc_copy_clock_to_ram();
	// the 68000 map (pgm_basic_mem); a protection device adds to it
	m68k_init();
	m68k_set_cpu_type(M68K_CPU_TYPE_68000);
	m68k_bus_reset_map();
	uint32_t rom_end = s_prot && s_prot->rom_end ? s_prot->rom_end : 0x3fffff;
	if (rom_end > pgm_rom_size - 1)
		rom_end = pgm_rom_size - 1;
	m68k_bus_map(0, rom_end, pgm_rom, pgm_rom_size, 1, 0);
	m68k_bus_map(0x800000, 0x8fffff, pgm_mainram, sizeof(pgm_mainram), 1, 1);
	for (uint32_t base = 0x900000; base < 0xa00000; base += 0x8000) {
		m68k_bus_map(base, base + 0x3fff, s_vid->bg, sizeof(s_vid->bg), 1, 1);
		m68k_bus_map(base + 0x4000, base + 0x6fff, s_vid->tx, sizeof(s_vid->tx), 1, 1);
		m68k_bus_map(base + 0x7000, base + 0x7fff, s_vid->rowscroll, sizeof(s_vid->rowscroll), 1, 1);
	}
	m68k_bus_map(0xa00000, 0xa013ff, s_pal, sizeof(s_pal), 1, 1);
	m68k_bus_map(0xb00000, 0xb00fff, s_vid->spritebuffer, sizeof(s_vid->spritebuffer), 1, 1);
	m68k_bus.read8 = main_read8;
	m68k_bus.read16 = main_read16;
	m68k_bus.write8 = main_write8;
	m68k_bus.write16 = main_write16;
	pgm_irq4_disabled = 0;
	if (s_prot && s_prot->init && s_prot->init(s_init)) {
		pgm_exit();
		return -1;
	}
	rbx_aux_set_job(z80_job);
	rbx_aux_set_spin(300);
	pgm_reset();
	return 0;
}

static void pgm_exit(void)
{
	rbx_aux_set_job(NULL);
	if (s_prot && s_prot->exit)
		s_prot->exit();
	s_prot = NULL;
	if (s_nv_path[0]) {
		FILE *f = fopen(s_nv_path, "wb");
		if (f) {
			fwrite(pgm_mainram, 1, sizeof(pgm_mainram), f);
			fclose(f);
		}
	}
	s_nv_path[0] = 0;
	ics2115_exit(&s_ics);
	if (s_vid)
		igs023_exit(s_vid);
	free(s_vid);
	free(s_snap);
	free(s_mix);
	s_vid = NULL;
	s_snap = NULL;
	s_mix = NULL;
}

static void audio_out(void)
{
	// the chip's samples at its rate, to OUT_RATE; MAME: both outputs to the mono speaker at 0.1
	const uint32_t rate = 33868800u / ics2115_sample_ticks(&s_ics);
	const int n = s_ics.out_n;
	if (rate == OUT_RATE) {
		for (int i = 0; i < n; i++)
			rbx_audio_put(s_ics.out[i] / 10);
	} else if (n) {
		const uint32_t step = (uint32_t)(((uint64_t)rate << 16) / OUT_RATE);
		for (; (s_rs_frac >> 16) < (uint32_t)n; s_rs_frac += step)
			rbx_audio_put(s_ics.out[s_rs_frac >> 16] / 10);
		s_rs_frac -= (uint32_t)n << 16;
	}
	s_ics.out_n = 0;
}

static void pgm_frame(void)
{
	rbx_audio.count = 0;
	unsigned long zt = 0;
	s_m68k_frame = 0;
	for (int line = 0; line < LINES; line++) {
		s_line = line;
		if (line == 0) {
			if (!pgm_irq4_disabled)
				m68k_bus_hold_irq(4);   // (vblank end; MAME m_irq4_disabled: puzzli2, pgm3in1)
			s_m68k_spin = 0;
		}
		if (line == VBLANK_LINE)
			vblank();
		s_mline_len = rbx_line_cycles(&s_m_frac, M68K_CLOCK, FRAME_MHZ, LINES);
		s_zline_len = (unsigned long)rbx_line_cycles(&s_z_frac, Z80_CLOCK, FRAME_MHZ, LINES);
		zt += s_zline_len;
		rbx_aux_sync();   // (the last line's sound done)
		s_ztarget = zt;
		rbx_aux_kick();   // (this line's on the worker, beside the 68000)
		if (!s_m68k_spin)
			m68k_execute(s_mline_len);
		s_m68k_frame += s_mline_len;
		if (s_prot && s_prot->line)
			s_prot->line(line);
	}
	rbx_aux_sync();
	// (Z80 time from 0 again each frame: its counter is a 32-bit long on the Vita)
	s_z80.cyc -= zt;
	s_ics_zcyc -= zt;
	s_ztarget -= zt;
	if (++s_rtc.frames >= 59) {
		s_rtc.frames = 0;
		rtc_tick();
	}
	audio_out();
}

static int pgm_state(void *ctx, int load)
{
	rbx_aux_sync();
	uint8_t cpu[1024];
	const uint32_t csize = m68k_context_size();
	if (csize > sizeof(cpu))
		return -1;
	if (!load)
		m68k_get_context(cpu);
	uint32_t irq = m68k_bus_irq_pending();
	uint64_t fr[3] = { s_m_frac, s_z_frac, s_rs_frac };
	z80 z = s_z80;
	int rc = 0;
	rc |= rbx_state_io(ctx, "m68k", cpu, csize);
	rc |= rbx_state_io(ctx, "irq", &irq, 4);
	rc |= rbx_state_io(ctx, "ram", pgm_mainram, sizeof(pgm_mainram));
	rc |= rbx_state_io(ctx, "pal", s_pal, sizeof(s_pal));
	rc |= rbx_state_io(ctx, "vid", s_vid, (uint32_t)offsetof(igs023_t, gfx));
	rc |= rbx_state_io(ctx, "z80", &z, sizeof(z));
	rc |= rbx_state_io(ctx, "zram", s_zram, sizeof(s_zram));
	rc |= rbx_state_io(ctx, "latch", s_latch, sizeof(s_latch));
	rc |= rbx_state_io(ctx, "zhalt", &s_z80_halt, 1);
	rc |= rbx_state_io(ctx, "zint", &s_zint, sizeof(s_zint));
	rc |= rbx_state_io(ctx, "icszc", &s_ics_zcyc, sizeof(s_ics_zcyc));
	rc |= rbx_state_io(ctx, "ics", &s_ics, ICS2115_STATE_SIZE);
	rc |= rbx_state_io(ctx, "rtc", &s_rtc, sizeof(s_rtc));
	rc |= rbx_state_io(ctx, "frac", fr, sizeof(fr));
	if (!rc && s_prot && s_prot->state)
		rc |= s_prot->state(ctx, load);
	if (rc)
		return -1;
	if (load) {
		m68k_set_context(cpu);
		m68k_bus_set_irq_pending(irq);
		s_z80 = z;
		z80_hook();
		s_m_frac = fr[0];
		s_z_frac = fr[1];
		s_rs_frac = (uint32_t)fr[2];
		s_ics.out_n = 0;
	}
	return 0;
}

static int pgm_save(void *ctx) { return pgm_state(ctx, 0); }
static int pgm_load(void *ctx) { return pgm_state(ctx, 1); }
static int pgm_supported(const char *machine) { return prot_of(machine) != (const pgm_prot *)-1; }

const rbx_driver rbx_drv_pgm = { "IGS PGM", pgm_init, pgm_exit, pgm_reset, pgm_frame, pgm_save, pgm_load,
                                 pgm_supported, pgm_render, pgm_render_help };
