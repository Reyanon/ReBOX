/*  Technos Block Out and Palco Agress. Ported from MAME src/mame/technos/blockout.cpp (BSD-3-Clause, copyright-holders:
    Nicola Salmoria).
    68000 at 10 MHz, IRQ 6 at line 250 and 5 at line 0 (held until acknowledged); two 512x256 byte bitmaps (the front
    one over the back one, which uses the palette's upper half) and a 1-bit overlay in one register color. Sound: Z80
    at 3.58 MHz, YM2151, OKI6295 at 1.056 MHz, the latch on the NMI.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m68k_bus.h"
#include "../../cpu/m68k/m68k.h"
#include "../../cpu/z80/z80.h"
#include "../../sound/okim6295.h"
#include "../../sound/rbx_ym.h"

#include <stdlib.h>
#include <string.h>

#define CPU_CLOCK 10000000
#define Z80_CLOCK 3579545
#define YM_CLOCK 3579545
#define OKI_CLOCK 1056000
#define FRAME_MHZ 57445          // set_raw(28 MHz / 4, 448, 0, 320, 272, 10, 250)
#define LINES 272
#define VIS_Y0 10
#define VIS_H 240
#define VIS_W 320

static uint8_t *s_rom, *s_zrom;
static uint32_t s_rom_size, s_zrom_size;
static uint16_t (*s_vram_hp)[0x20000];        // 0x180000 .. 0x1bffff: bytes (host words: byte a at a ^ 1)
#define s_vram (*s_vram_hp)
static uint16_t (*s_ram1_hp)[0x6000];         // 0x1d4000 .. 0x1dffff
#define s_ram1 (*s_ram1_hp)
static uint16_t (*s_ram2_hp)[0x6000];         // 0x1f4000 .. 0x1fffff
#define s_ram2 (*s_ram2_hp)
static uint16_t (*s_front_hp)[0x4000];        // 0x200000 .. 0x207fff
#define s_front (*s_front_hp)
static uint16_t (*s_ram3_hp)[0xc000];         // 0x208000 .. 0x21ffff
#define s_ram3 (*s_ram3_hp)
static uint16_t s_pal[0x200];           // 0x280200 .. 0x2805ff
static uint16_t s_color;                // 0x280002: the overlay's color (pen 512)
static int s_port[5];
static uint64_t s_cpu_frac, s_z80_frac, s_ym_frac;
static struct {
	uint16_t vram[0x20000], front[0x4000], pal[0x200], color;
} *s_snap;

// ---- sound ----
static z80 s_z80;
static uint8_t s_zram[0x800];
static uint8_t s_latch;
static int s_latch_pending;
static okim6295_t s_oki;
static int s_ym_irq;
static uint32_t s_oki_acc;
static int32_t s_oki_hold;
static int32_t *s_mix;
static int s_mix_n, s_mix_cap;
static const uint8_t *s_zmap[256];

static void ym_irq(int state) { s_ym_irq = state; }

static uint8_t z80_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x8000) return a < s_zrom_size ? s_zrom[a] : 0;
	if (a < 0x8800) return s_zram[a - 0x8000];
	if (a == 0x8800 || a == 0x8801) return rbx_ym_read(0, a & 1);
	if (a == 0x9800) return oki_read(&s_oki);
	if (a == 0xa000) {
		s_latch_pending = 0;
		return s_latch;
	}
	return 0;
}
static void z80_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0x8000 && a < 0x8800) s_zram[a - 0x8000] = v;
	else if (a == 0x8800 || a == 0x8801) rbx_ym_write(0, a & 1, v);
	else if (a == 0x9800) oki_write(&s_oki, v);
}
static uint8_t z80_in(z80 *z, uint8_t port) { (void)z; (void)port; return 0xff; }
static void z80_out(z80 *z, uint8_t port, uint8_t v) { (void)z; (void)port; (void)v; }

static void z80_start(void)
{
	z80_init(&s_z80);
	s_z80.read_byte = z80_read;
	s_z80.write_byte = z80_write;
	s_z80.port_in = z80_in;
	s_z80.port_out = z80_out;
	s_z80.read_map = s_zmap;
}

static void sound_line(void)
{
	const int cycles = rbx_line_cycles(&s_z80_frac, Z80_CLOCK, FRAME_MHZ, LINES);
	z80_run(&s_z80, s_z80.cyc + (unsigned long)cycles, &s_ym_irq, 0);
	const uint32_t clocks = (uint32_t)rbx_line_cycles(&s_ym_frac, YM_CLOCK, FRAME_MHZ, LINES);
	const int n = rbx_ym_advance(0, clocks, s_mix + s_mix_n, s_mix_cap - s_mix_n);
	const uint32_t yr = rbx_ym_rate(0);
	for (int i = 0; i < n; i++) {
		s_oki_acc += s_oki.rate;
		while (s_oki_acc >= yr) {
			s_oki_acc -= yr;
			s_oki_hold = oki_sample(&s_oki);
		}
		s_mix[s_mix_n + i] += s_oki_hold;
	}
	s_mix_n += n;
}

static void sound_frame_end(void)
{
	rbx_ym_async_take(0, s_mix, s_mix_n);
	for (int i = 0; i < s_mix_n; i++)
		rbx_audio_put(s_mix[i]);
	memset(s_mix, 0, sizeof(int32_t) * (size_t)s_mix_n);
	s_mix_n = 0;
}

// ---- bus ----
static uint16_t main_read16(uint32_t a)
{
	a &= 0xffffff;
	if (a >= 0x100000 && a < 0x10000a)   // P1, P2, SYSTEM, DSW1, DSW2
		return (uint16_t)rbx_port_read(s_port[(a - 0x100000) >> 1]);
	return 0;
}
static uint8_t main_read8(uint32_t a)
{
	const uint16_t v = main_read16(a & ~1u);
	return (uint8_t)(a & 1 ? v : v >> 8);
}
static void main_write16m(uint32_t a, uint16_t v, uint16_t mask)
{
	a &= 0xffffff;
	if (a == 0x100010)
		m68k_bus_set_line(6, 0);
	else if (a == 0x100012)
		m68k_bus_set_line(5, 0);
	else if (a == 0x100014 && (mask & 0x00ff)) {
		s_latch = (uint8_t)v;
		if (!s_latch_pending)
			z80_gen_nmi(&s_z80);
		s_latch_pending = 1;
	} else if (a == 0x280002)
		s_color = (uint16_t)((s_color & ~mask) | (v & mask));
}
static void main_write16(uint32_t a, uint16_t v) { main_write16m(a, v, 0xffff); }
static void main_write8(uint32_t a, uint8_t v)
{
	const int sh = a & 1 ? 0 : 8;
	main_write16m(a & ~1u, (uint16_t)(v << sh), (uint16_t)(0xff << sh));
}

// ---- video ----
static uint32_t pal_444(uint32_t raw)   // xBGR_444 with the board's resistor weights
{
	uint32_t c[3];
	for (int k = 0; k < 3; k++) {
		const uint32_t n = raw >> (k * 4);
		c[k] = 0x0e * (n & 1) + 0x1f * ((n >> 1) & 1) + 0x43 * ((n >> 2) & 1) + 0x8f * ((n >> 3) & 1);
	}
	return c[0] << 16 | c[1] << 8 | c[2];
}

static void bo_render(void)
{
	for (int i = 0; i < 0x200; i++)
		rbx_video.palette[i] = pal_444(s_snap->pal[i]);
	rbx_video.palette[512] = pal_444(s_snap->color);
	const uint8_t *vram = (const uint8_t *)s_snap->vram;
	for (int y = VIS_Y0; y < VIS_Y0 + VIS_H; y++) {
		uint16_t *d = rbx_video.bitmap + (y - VIS_Y0) * rbx_video.pitch;
		const uint32_t row = (uint32_t)(y & 0xff) << 9;
		const uint16_t *fr = s_snap->front + ((y & 0xff) << 6);
		for (int x = 0; x < VIS_W; x++) {
			const uint32_t o = row | (uint32_t)x;
			const uint8_t front = vram[o ^ 1];
			uint16_t c = front ? front : (uint16_t)(vram[(0x20000 | o) ^ 1] | 0x100);
			if (fr[x >> 3] & (1 << (7 - (x & 7))))
				c = 512;
			d[x] = c;
		}
	}
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap->vram, s_vram, sizeof(s_vram));
	memcpy(s_snap->front, s_front, sizeof(s_front));
	memcpy(s_snap->pal, s_pal, sizeof(s_pal));
	s_snap->color = s_color;
	rbx_video_frame_done();
}

// ---- init / frame ----
static void bo_reset(void)
{
	memset(s_vram, 0, sizeof(s_vram));
	memset(s_ram1, 0, sizeof(s_ram1));
	memset(s_ram2, 0, sizeof(s_ram2));
	memset(s_front, 0, sizeof(s_front));
	memset(s_ram3, 0, sizeof(s_ram3));
	memset(s_pal, 0, sizeof(s_pal));
	s_color = 0;
	s_latch = 0;
	s_latch_pending = 0;
	s_ym_irq = 0;
	memset(s_zram, 0, sizeof(s_zram));
	z80_start();
	rbx_ym_reset(0);
	oki_reset(&s_oki);
	s_oki_acc = 0;
	s_oki_hold = 0;
	s_mix_n = 0;
	m68k_bus_irq_reset();
	m68k_pulse_reset();
	s_cpu_frac = s_z80_frac = s_ym_frac = 0;
}

static void bo_exit(void);

static int bo_init(const rbx_game *g)
{
	if (!s_front_hp && !(s_front_hp = calloc(1, sizeof(*s_front_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	if (!s_ram1_hp && !(s_ram1_hp = calloc(1, sizeof(*s_ram1_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	if (!s_ram2_hp && !(s_ram2_hp = calloc(1, sizeof(*s_ram2_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	if (!s_ram3_hp && !(s_ram3_hp = calloc(1, sizeof(*s_ram3_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	if (!s_vram_hp && !(s_vram_hp = calloc(1, sizeof(*s_vram_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	uint32_t osize;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_zrom = rbx_region("audiocpu", &s_zrom_size);
	uint8_t *oki = rbx_region("oki", &osize);
	if (!s_rom || s_rom_size < 0x40000 || !s_zrom || !oki) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	m68k_bus_swap16(s_rom, s_rom_size);
	if (strstr(g->machine, ":agress"))   // init_agress: the TRACE exception's bogus mirror check patched out
		((uint16_t *)s_rom)[0x82 / 2] = 0x2700;
	s_snap = calloc(1, sizeof(*s_snap));
	rbx_ym_config yc;
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM2151;
	yc.clock = YM_CLOCK;
	yc.gain[0] = yc.gain[1] = 77;   // (0.60 to both sides: (L + R) / 2)
	yc.irq = ym_irq;
	if (!s_snap || rbx_ym_init(0, &yc) || rbx_ym_set_async(0, 1)) {
		rbx_fail("%s: out of memory", g->set);
		bo_exit();
		return -1;
	}
	s_mix_cap = (int)(rbx_ym_rate(0) / 50) + 256;
	s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	if (!s_mix || rbx_audio_setup(rbx_ym_rate(0)) || rbx_video_setup(VIS_W, VIS_H, 513)) {
		rbx_fail("%s: out of memory", g->set);
		bo_exit();
		return -1;
	}
	oki_init(&s_oki, OKI_CLOCK, 1, oki, osize, 128);
	memset(s_zmap, 0, sizeof(s_zmap));
	for (uint32_t p = 0; p < 0x80 && p * 256 + 256 <= s_zrom_size; p++)
		s_zmap[p] = s_zrom + p * 256;
	static const char *const names[5] = { "P1", "P2", "SYSTEM", "DSW1", "DSW2" };
	for (int i = 0; i < 5; i++)
		s_port[i] = rbx_port_find(names[i]);
	m68k_init();
	m68k_set_cpu_type(M68K_CPU_TYPE_68000);
	m68k_set_mame_timing(1);   // (MAME's DIVx / MULx / ADDQ.W An times)
	m68k_bus_reset_map();
	m68k_bus_map(0, 0x3ffff, s_rom, 0x40000, 1, 0);
	m68k_bus_map(0x180000, 0x1bffff, s_vram, sizeof(s_vram), 1, 1);
	m68k_bus_map(0x1d4000, 0x1dffff, s_ram1, sizeof(s_ram1), 1, 1);
	m68k_bus_map(0x1f4000, 0x1fffff, s_ram2, sizeof(s_ram2), 1, 1);
	m68k_bus_map(0x200000, 0x207fff, s_front, sizeof(s_front), 1, 1);
	m68k_bus_map(0x208000, 0x21ffff, s_ram3, sizeof(s_ram3), 1, 1);
	m68k_bus_map(0x280200, 0x2805ff, s_pal, sizeof(s_pal), 1, 1);
	m68k_bus.read8 = main_read8;
	m68k_bus.read16 = main_read16;
	m68k_bus.write8 = main_write8;
	m68k_bus.write16 = main_write16;
	bo_reset();
	return 0;
}

static void bo_exit(void)
{
	m68k_set_mame_timing(0);
	rbx_ym_exit(0);
	free(s_mix);
	s_mix = NULL;
	free(s_snap);
	s_snap = NULL;
	free(s_front_hp);
	s_front_hp = NULL;
	free(s_ram1_hp);
	s_ram1_hp = NULL;
	free(s_ram2_hp);
	s_ram2_hp = NULL;
	free(s_ram3_hp);
	s_ram3_hp = NULL;
	free(s_vram_hp);
	s_vram_hp = NULL;
}

static void bo_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	for (int line = 0; line < LINES; line++) {
		if (line == 0)
			m68k_bus_set_line(5, 1);
		if (line == VIS_Y0 + VIS_H) {   // (line 250: the vblank IRQ; the frame drawn at its start)
			vblank_capture();
			m68k_bus_set_line(6, 1);
		}
		m68k_execute(rbx_line_cycles(&s_cpu_frac, CPU_CLOCK, FRAME_MHZ, LINES));
		sound_line();
	}
	sound_frame_end();
}

static int bo_state(void *ctx, int load)
{
	const uint32_t csize = m68k_context_size();
	uint8_t *cpu = (uint8_t *)malloc(csize);
	if (!cpu)
		return -1;
	if (!load)
		m68k_get_context(cpu);
	uint32_t irq[2] = { m68k_bus_irq_pending(), m68k_bus_cur->irq_asserted };
	uint64_t fr[3] = { s_cpu_frac, s_z80_frac, s_ym_frac };
	int32_t misc[6] = { s_latch, s_latch_pending, s_ym_irq, (int32_t)s_oki_acc, s_oki_hold, s_color };
	z80 z = s_z80;
	int rc = 0;
	rc |= rbx_state_io(ctx, "m68k", cpu, csize);
	rc |= rbx_state_io(ctx, "irq", irq, sizeof(irq));
	rc |= rbx_state_io(ctx, "vram", s_vram, sizeof(s_vram));
	rc |= rbx_state_io(ctx, "ram1", s_ram1, sizeof(s_ram1));
	rc |= rbx_state_io(ctx, "ram2", s_ram2, sizeof(s_ram2));
	rc |= rbx_state_io(ctx, "front", s_front, sizeof(s_front));
	rc |= rbx_state_io(ctx, "ram3", s_ram3, sizeof(s_ram3));
	rc |= rbx_state_io(ctx, "pal", s_pal, sizeof(s_pal));
	rc |= rbx_state_io(ctx, "z80", &z, sizeof(z));
	rc |= rbx_state_io(ctx, "zram", s_zram, sizeof(s_zram));
	rc |= rbx_state_io(ctx, "oki", s_oki.v, sizeof(s_oki.v) + sizeof(int));
	rc |= rbx_ym_state(0, ctx, load);
	rc |= rbx_state_io(ctx, "frac", fr, sizeof(fr));
	rc |= rbx_state_io(ctx, "misc", misc, sizeof(misc));
	if (!rc && load) {
		m68k_set_context(cpu);
		m68k_bus_set_irq_pending(irq[0]);
		m68k_bus_cur->irq_asserted = irq[1];
		m68k_bus_update_irq();
		z.read_byte = z80_read;
		z.write_byte = z80_write;
		z.port_in = z80_in;
		z.port_out = z80_out;
		z.userdata = NULL;
		z.read_map = s_zmap;
		s_z80 = z;
		s_cpu_frac = fr[0];
		s_z80_frac = fr[1];
		s_ym_frac = fr[2];
		s_latch = (uint8_t)misc[0];
		s_latch_pending = misc[1];
		s_ym_irq = misc[2];
		s_oki_acc = (uint32_t)misc[3];
		s_oki_hold = misc[4];
		s_color = (uint16_t)misc[5];
	}
	free(cpu);
	return rc ? -1 : 0;
}

static int bo_save(void *ctx) { return bo_state(ctx, 0); }
static int bo_load(void *ctx) { return bo_state(ctx, 1); }
static int bo_supported(const char *machine) { return !strncmp(machine, "blockout", 8); }

const rbx_driver rbx_drv_blockout = { "Technos Block Out", bo_init, bo_exit, bo_reset, bo_frame, bo_save, bo_load,
                                      bo_supported, bo_render, NULL };
