/*  Irem M10, M11 and M15: IPM Invader, Andromeda SS, Sky Chuter, Head On, Space Beam and Green Beret. Ported from MAME
    src/mame/irem/m10.cpp (BSD-3-Clause, copyright-holders: Lee Taylor, Couriersud) with the pulse lengths of
    src/devices/machine/74123.cpp (BSD-3-Clause, copyright-holders: Couriersud). A 6502 under 1 MHz, a 32x32 layer of 1 bpp
    characters (from RAM on the M15), four bars of RAM graphics behind it on the M10 and M11. The sound is MAME's sample
    set of discrete recordings: silent here.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m6502/m6502.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define VY0 16
#define VH 224
#define SND_RATE 48000

enum { K_M10, K_M11, K_M15 };

static uint8_t *s_rom, *s_tiles;
static uint32_t s_rom_size, s_tiles_size;
static m6502_t s_cpu;
static int s_kind, s_lines, s_vx0, s_vw, s_htotal, s_port_cab, s_port_dsw, s_port_in, s_port_p2, s_port_fake;
static uint32_t s_line_x2;            // (a line's CPU cycles x 2)
static int64_t s_period, s_pulse;     // (the 74123s: the free running one, the IRQ's, in CPU cycles)
static uint8_t s_back_color[4], s_back_xpos[4];
static uint32_t s_pix, s_cpu_clock;

static struct {
	uint8_t ram[0x300], vram[0x400], cram[0x400], chargen[0x800];
	uint8_t flip, bottomline, coin, irq;
	uint32_t frac;
	int64_t end, next, irq_off;
	uint32_t frame;
	uint8_t state_end;
} t;

// ---- the drawing's snapshot ----
static struct { uint8_t vram[0x400], cram[0x400], chargen[0x800], flip, bottomline; } s_vs;

static void capture(void)
{
	memcpy(s_vs.vram, t.vram, sizeof(s_vs.vram));
	memcpy(s_vs.cram, t.cram, sizeof(s_vs.cram));
	memcpy(s_vs.chargen, t.chargen, sizeof(s_vs.chargen));
	s_vs.flip = t.flip;
	s_vs.bottomline = t.bottomline;
}

static void draw_back(uint16_t *bm, int pitch)
{
	for (int i = 0; i < 4; i++) {
		const uint8_t *src = s_vs.chargen + i * 256;
		const int x0 = (s_vs.flip ? 31 * 8 - s_back_xpos[i] : s_back_xpos[i]) - s_vx0;
		const uint32_t base = (uint32_t)s_back_color[i] * 2;
		for (int y = 0; y < VH; y++) {
			const int sy = y + VY0, v = src[s_vs.flip ? 255 - sy : sy];
			uint16_t *d = bm + y * pitch;
			for (int k = 0; k < 8; k++) {
				const int x = x0 + k;
				if (x >= 0 && x < s_vw)
					d[x] = (uint16_t)(base + ((v >> (s_vs.flip ? k : 7 - k)) & 1));
			}
		}
	}
}

static void m10_render(void)
{
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch, flip = s_vs.flip, m15 = s_kind == K_M15;
	if (!m15) {
		for (int y = 0; y < VH; y++)
			memset(bm + y * pitch, 0, (size_t)s_vw * 2);
		draw_back(bm, pitch);
		if (s_vs.bottomline) {
			const int x = flip ? s_vw - 1 - (16 - s_vx0) : 16 - s_vx0;
			for (int y = 0; y < VH; y++)
				bm[y * pitch + x] = 1;
		}
	}
	const uint8_t *font = m15 ? s_vs.chargen : s_tiles;
	const int vx = flip ? s_htotal - 256 : 0, vy = flip ? s_lines - 256 : 0;
	for (int pc = 0; pc < 32; pc++)
		for (int pr = 0; pr < 32; pr++) {
			const int c = flip ? 31 - pc : pc, r = flip ? 31 - pr : pr, idx = (31 - c) * 32 + r;
			const int x0 = ((pc * 8 + vx) & 255) - s_vx0, y0 = ((pr * 8 + vy) & 255) - VY0;
			if (x0 <= -8 || x0 >= s_vw || y0 <= -8 || y0 >= VH)
				continue;
			const uint8_t *f = font + s_vs.vram[idx] * 8;
			const uint32_t base = (uint32_t)(s_vs.cram[idx] & 7) * 2;
			for (int j = 0; j < 8; j++) {
				const int y = y0 + j;
				if (y < 0 || y >= VH)
					continue;
				const int v = f[flip ? 7 - j : j];
				uint16_t *d = bm + y * pitch;
				for (int k = 0; k < 8; k++) {
					const int x = x0 + k, p = (v >> (flip ? k : 7 - k)) & 1;
					if (x >= 0 && x < s_vw && (m15 || p))
						d[x] = (uint16_t)(base + (uint32_t)p);
				}
			}
		}
}

// ---- the 6502's bus ----
static uint8_t rd(void *c, uint16_t a)
{
	(void)c;
	if (a == 0xa200)
		return (uint8_t)rbx_port_read(s_port_dsw);
	if (a == 0xa300)
		return (uint8_t)rbx_port_read(s_port_in);
	if (a == 0xa000 && s_kind == K_M15)
		return (uint8_t)rbx_port_read(s_port_p2);
	if (a == 0xa700 && s_kind != K_M15) {   // (clear_74123_r: the IRQ's pulse over)
		t.irq_off = 0;
		t.irq = 0;
		m6502_set_irq(&s_cpu, 0);
	}
	return 0;
}

static void wr(void *c, uint16_t a, uint8_t v)
{
	(void)c;
	if (a == 0xa400) {
		const int cab = (int)rbx_port_read(s_port_cab) & 1;
		if (s_kind == K_M15) {
			if (cab)
				t.flip = !(v & 0x04);
		} else {
			t.bottomline = !(v & 0x20);
			if (cab)
				t.flip = !(v & 0x10);
		}
	}
}

// ---- init / frame ----
static void m10_reset(void);

static int m10_init(const rbx_game *g)
{
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_tiles = rbx_region("tiles", &s_tiles_size);
	s_kind = !strcmp(g->machine, "m10") ? K_M10 : !strcmp(g->machine, "m11") ? K_M11 : K_M15;
	if (!s_rom || s_rom_size < 0x10000 || (s_kind != K_M15 && (!s_tiles || s_tiles_size < 0x800))) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	memset(&t, 0, sizeof(t));
	s_pix = s_kind == K_M10 ? 6250000 : 5865000;
	s_cpu_clock = s_pix / 8;
	s_htotal = s_kind == K_M10 ? 360 : 372;
	s_lines = s_kind == K_M10 ? 281 : 262;
	s_vx0 = s_kind == K_M10 ? 8 : 0;
	s_vw = s_kind == K_M10 ? 240 : 256;
	s_line_x2 = (uint32_t)s_htotal / 4;   // (the pixel clock / 8: 45 and 46.5 cycles)
	// (TTL74123_NOT_GROUNDED_DIODE: 0.25 R C (1 + 700 / R), R 16k C 2.2u and R 1k C 1u)
	s_period = (int64_t)(0.25 * 16000.0 * 2.2e-6 * (1.0 + 700.0 / 16000.0) * s_cpu_clock);
	s_pulse = (int64_t)(0.25 * 1000.0 * 1e-6 * (1.0 + 700.0 / 1000.0) * s_cpu_clock);
	const int ipm = !strncmp(g->set, "ipminvad", 8);
	static const uint8_t col[2][4] = { { 3, 0, 3, 3 }, { 3, 3, 5, 5 } }, xp[2][4] = { { 4, 26, 2, 3 }, { 4, 26, 7, 6 } };
	for (int i = 0; i < 4; i++) {
		s_back_color[i] = col[ipm][i];
		s_back_xpos[i] = (uint8_t)(xp[ipm][i] * 8);
	}
	s_port_cab = rbx_port_find("CAB");
	s_port_dsw = rbx_port_find("DSW");
	s_port_in = rbx_port_find(s_kind == K_M15 ? "P1" : "INPUTS");
	s_port_p2 = rbx_port_find("P2");
	s_port_fake = rbx_port_find("FAKE");
	if (rbx_audio_setup(SND_RATE) || rbx_video_setup(s_vw, VH, 16)) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	rbx_audio.frame_mhz = (uint32_t)((uint64_t)s_pix * 1000 / ((uint32_t)s_htotal * (uint32_t)s_lines));
	for (int i = 0; i < 16; i++)
		rbx_video.palette[i] = (i & 1) ? (uint32_t)((((~i >> 3) & 1) * 0xff) << 16 | ((~i >> 2) & 1) * 0xff00 | ((~i >> 1) & 1) * 0xff) : 0;
	m10_reset();
	return 0;
}

static void m10_exit(void) { }

static void m10_reset(void)
{
	memset(&t, 0, offsetof(__typeof__(t), state_end));
	memset(&s_vs, 0, sizeof(s_vs));
	m6502_init(&s_cpu, M6502_NMOS);
	s_cpu.read = rd;
	s_cpu.write = wr;
	const int top = s_kind == K_M15 ? 0x34 : 0x30;
	for (int p = 0x10; p < top; p++)
		s_cpu.rpage[p] = s_rom + p * 256;
	for (int p = 0xfc; p < 0x100; p++)
		s_cpu.rpage[p] = s_rom + p * 256;
	for (int p = 0; p < 3; p++)
		s_cpu.rpage[p] = s_cpu.wpage[p] = t.ram + p * 256;
	for (int p = 0; p < 4; p++) {
		s_cpu.rpage[0x40 + p] = s_cpu.wpage[0x40 + p] = t.vram + p * 256;
		s_cpu.rpage[0x48 + p] = s_cpu.wpage[0x48 + p] = t.cram + p * 256;
	}
	for (int p = 0; p < (s_kind == K_M15 ? 8 : 4); p++)
		s_cpu.rpage[0x50 + p] = s_cpu.wpage[0x50 + p] = t.chargen + p * 256;
	m6502_reset(&s_cpu);
	t.next = 0;   // (the oscillator starts at once)
}

// (the 74123s up to the CPU's target: one runs free, each of its pulses starts the other, whose output is the IRQ)
static void run_to(int64_t target)
{
	if (s_kind == K_M15) {
		m6502_run_until(&s_cpu, target);
		return;
	}
	while (t.end < target) {
		int64_t e = target;
		if (t.next < e)
			e = t.next;
		if (t.irq_off && t.irq_off < e)
			e = t.irq_off;
		if (e > t.end) {
			t.end = e;
			m6502_run_until(&s_cpu, e);
		}
		if (t.irq_off && t.end >= t.irq_off) {
			t.irq_off = 0;
			t.irq = 0;
			m6502_set_irq(&s_cpu, 0);
		}
		if (t.end >= t.next) {
			t.next += s_period;
			t.irq = 1;
			t.irq_off = t.next - s_period + s_pulse;
			m6502_set_irq(&s_cpu, 1);
		}
	}
}

static void m10_frame(void)
{
	const int coin = (int)rbx_port_read(s_port_fake) & 1;
	if (coin != t.coin) {   // (coin_inserted)
		t.coin = (uint8_t)coin;
		m6502_set_nmi(&s_cpu, coin);
	}
	for (int line = 0; line < s_lines; line++) {
		if (line == VY0 + VH) {   // (the update; the M15's IRQ for 80 pixels)
			rbx_video_wait_idle();
			capture();
			rbx_video_frame_done();
			if (s_kind == K_M15) {
				m6502_set_irq(&s_cpu, 1);
				t.frac += 20;
				const uint32_t c = t.frac >> 1;
				t.frac &= 1;
				t.end += c;
				run_to(t.end);
				m6502_set_irq(&s_cpu, 0);
				t.frac += s_line_x2 * 2 - 20;
				const uint32_t c2 = t.frac >> 1;
				t.frac &= 1;
				t.end += c2;
				run_to(t.end);
				continue;
			}
		}
		t.frac += s_line_x2 * 2;
		const uint32_t c = t.frac >> 1;
		t.frac &= 1;
		if (s_kind == K_M15) {
			t.end += c;
			run_to(t.end);
		} else
			run_to(t.end + c);
	}
	for (int i = 0; i < SND_RATE / 60; i++)   // (the samples aren't played: silence)
		rbx_audio_put(0);
	t.frame++;
}

static int m10_state(void *ctx, int load)
{
	int rc = 0;
	rc |= rbx_state_io(ctx, "m10", &t, (uint32_t)offsetof(__typeof__(t), state_end));
	rc |= rbx_state_io(ctx, "m10_cpu", &s_cpu, M6502_STATE_SIZE);
	(void)load;
	return rc ? -1 : 0;
}

static int m10_save(void *ctx) { return m10_state(ctx, 0); }
static int m10_load(void *ctx) { return m10_state(ctx, 1); }
static int m10_supported(const char *machine) { return !strcmp(machine, "m10") || !strcmp(machine, "m11") || !strcmp(machine, "m15"); }

const rbx_driver rbx_drv_m10 = { "Irem M10", m10_init, m10_exit, m10_reset, m10_frame, m10_save, m10_load, m10_supported,
                                 m10_render, NULL };
