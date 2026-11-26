/*  Data East Side Pocket, from MAME src/mame/dataeast/sidepckt.cpp (BSD-3-Clause, copyright-holders: Nicola Salmoria).
    A 6809 at 2 MHz (NMI the vblank; FIRQ from the MCU's P3.0), the i8751 (its P1 the reply, its P2 the command with
    INT0, P3.1 the INT0 ack; MAME: perfect sync, here sixteenths of a line; the bootleg without it), an 8x8 background
    (y scrolled; mirrored in x, its tiles flipped back; two transparency groups put pens over the sprites), sprites,
    the PROM palette. The sound: a 6502 at 1.5 MHz with YM2203 and YM3526 (on its IRQ), the latch on its NMI. MAME's
    screen: 58 Hz, 256x256 with a 2.5 ms vblank (here 300 lines, 240 .. 283 the vblank), 256x224 shown.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../cheat_cpu.h"
#include "../../rbx.h"
#include "../../cpu/m6502/m6502.h"
#include "../../cpu/m6809/m6809.h"
#include "../../cpu/mcs51/mcs51.h"
#include "../../sound/rbx_ym.h"
#include "../../video/gfxlayout.h"

#include <stdlib.h>
#include <string.h>

#define CPU_CLOCK 2000000        // 12 MHz / 6
#define SND_CLOCK 1500000
#define MCU_CLOCK (8000000 / 12)
#define YM1_CLOCK 1500000        // YM2203
#define YM2_CLOCK 3000000        // YM3526
#define FRAME_MHZ 58000
#define LINES 300
#define VIS_Y0 16
#define VIS_H 224
#define VIS_W 256
#define VBL_START 240
#define VBL_END 284
#define SLICES 16

static int s_boot;
static uint8_t *s_rom, *s_mcu_rom;
static uint32_t s_rom_size, s_mcu_size;
static uint8_t s_ram[0x2100];              // 0x0000 .. 0x20ff (RAM, the tiles, their colours, the sprites)
static uint8_t s_scroll_y;
static m6809_t s_cpu;
static int s_firq;
static rbx_gfx s_gfx8, s_gfxspr;
static uint32_t s_palette[256];
static int s_port_p[2], s_port_dsw[2];
static int s_line;
static uint64_t s_cpu_frac, s_snd_frac, s_mcu_frac, s_ym1_frac, s_ym2_frac;
static struct {
	uint8_t vram[0x400], cram[0x400], spr[0x100];
	uint8_t scroll_y;
} *s_snap;

#define VRAM 0x1000   // (s_ram offsets)
#define CRAM 0x1800
#define SPR 0x2000

// ---- the i8751 ----
static mcs51_t s_mcu;
static uint8_t s_mcu_p1, s_mcu_p2, s_mcu_p3;

static uint8_t mcu_in(void *u, int port)
{
	(void)u;
	return port == 2 ? s_mcu_p2 : 0xff;
}
static void mcu_out(void *u, int port, uint8_t data)
{
	(void)u;
	if (port == 1)
		s_mcu_p1 = data;
	else if (port == 3) {   // mcu_p3_w: P3.0 the CPU's FIRQ (low), P3.1 falling: the INT0 ack
		if (!(data & 1) && (s_mcu_p3 & 1)) { s_firq = 1; m6809_set_firq(&s_cpu, 1); }
		if ((data & 1) && !(s_mcu_p3 & 1)) { s_firq = 0; m6809_set_firq(&s_cpu, 0); }
		if (!(data & 2) && (s_mcu_p3 & 2))
			mcs51_set_input(&s_mcu, MCS51_INT0_LINE, 0);
		s_mcu_p3 = data;
	}
}

// ---- sound: 6502, YM2203, YM3526 ----
static m6502_t s_snd;
static uint8_t s_sram[0x1000];
static uint8_t *s_zrom;
static uint32_t s_zrom_size;
static uint8_t s_latch;
static int32_t *s_mix, *s_ym2buf;
static int s_mix_n, s_mix_cap, s_ym2_n, s_ym2_cap;

static void ym2_irq(int state) { m6502_set_irq(&s_snd, state); }
static uint8_t snd_r(void *ctx, uint16_t a)
{
	(void)ctx;
	if (a == 0x3000) {   // (reading the latch clears its NMI)
		m6502_set_nmi(&s_snd, 0);
		return s_latch;
	}
	return 0;
}
static void snd_w(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	if (a == 0x1000 || a == 0x1001) rbx_ym_write(0, a & 1, v);
	else if (a == 0x2000 || a == 0x2001) rbx_ym_write(1, a & 1, v);
}
static void sound_line(void)
{
	s_snd_frac += (uint64_t)SND_CLOCK * 1000;
	const uint64_t den = (uint64_t)FRAME_MHZ * LINES;
	const int64_t cyc = (int64_t)(s_snd_frac / den);
	s_snd_frac -= (uint64_t)cyc * den;
	m6502_run_until(&s_snd, s_snd.cycles + cyc);
	s_mix_n += rbx_ym_advance(0, (uint32_t)rbx_line_cycles(&s_ym1_frac, YM1_CLOCK, FRAME_MHZ, LINES), s_mix + s_mix_n,
	                          s_mix_cap - s_mix_n);
	s_ym2_n += rbx_ym_advance(1, (uint32_t)rbx_line_cycles(&s_ym2_frac, YM2_CLOCK, FRAME_MHZ, LINES), s_ym2buf + s_ym2_n,
	                          s_ym2_cap - s_ym2_n);
}
static void sound_frame_end(void)
{
	rbx_ym_async_take(0, s_mix, s_mix_n);
	rbx_ym_async_take(1, s_ym2buf, s_ym2_n);
	if (s_ym2_n > 0)
		for (int i = 0; i < s_mix_n; i++)
			s_mix[i] += s_ym2buf[(int)((int64_t)i * s_ym2_n / s_mix_n)];
	for (int i = 0; i < s_mix_n; i++)
		rbx_audio_put(s_mix[i]);
	memset(s_mix, 0, sizeof(int32_t) * (size_t)s_mix_n);
	memset(s_ym2buf, 0, sizeof(int32_t) * (size_t)s_ym2_n);
	s_mix_n = s_ym2_n = 0;
}

// ---- bus ----
static uint8_t cpu_r(void *ctx, uint16_t a)
{
	(void)ctx;
	switch (a) {
	case 0x3000: return (uint8_t)rbx_port_read(s_port_p[0]);
	case 0x3001: return (uint8_t)rbx_port_read(s_port_p[1]);
	case 0x3002: return (uint8_t)rbx_port_read(s_port_dsw[0]);
	case 0x3003: return (uint8_t)rbx_port_read(s_port_dsw[1]);
	case 0x300c: return s_scroll_y;
	case 0x3014: return s_boot ? 0 : s_mcu_p1;   // mcu_r
	}
	return 0;
}
static void cpu_w(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	switch (a) {
	case 0x3004: s_latch = v; m6502_set_nmi(&s_snd, 1); break;
	case 0x300c: s_scroll_y = v & 0x3f; break;
	case 0x3018:   // mcu_w: the command, INT0
		if (!s_boot) {
			s_mcu_p2 = v;
			mcs51_set_input(&s_mcu, MCS51_INT0_LINE, 1);
		}
		break;
	}
}

// ---- video ----
static void spr_blit(uint32_t code, int colour, int fx, int fy, int sx, int sy, uint16_t *bm, int pitch)
{
	if (!s_gfxspr.count)
		return;
	code %= s_gfxspr.count;
	const uint8_t *src = s_gfxspr.pix + code * 256;
	for (int y = 0; y < 16; y++) {
		const int dy = sy + y - VIS_Y0;
		if (dy < 0 || dy >= VIS_H)
			continue;
		const uint8_t *row = src + (fy ? 15 - y : y) * 16;
		for (int x = 0; x < 16; x++) {
			const int dx = sx + x;
			if (dx < 0 || dx >= VIS_W)
				continue;
			const uint8_t p = row[fx ? 15 - x : x];
			if (p)
				bm[dy * pitch + dx] = (uint16_t)(colour * 8 + p);
		}
	}
}
// the background's pixels of one layer (MAME transmasks: LAYER1 = group 0's pens and group 1's pen 0, LAYER0 =
// group 1's pens 1-7); the map mirrored in x with its tiles flipped back
static void bg_draw(uint16_t *bm, int pitch, int layer0)
{
	for (int y = 0; y < VIS_H; y++) {
		uint16_t *d = bm + y * pitch;
		const int py = (y + VIS_Y0 + s_snap->scroll_y) & 0xff;
		for (int x = 0; x < VIS_W; x++) {
			const int idx = (py >> 3) * 32 + (31 - (x >> 3)), attr = s_snap->cram[idx];
			const uint32_t code = s_gfx8.count ? rbx_wrap((uint32_t)(s_snap->vram[idx] + ((attr & 7) << 8)), s_gfx8.count) : 0;
			const uint8_t p = s_gfx8.pix[(code * 8 + (uint32_t)(py & 7)) * 8 + (uint32_t)(x & 7)];
			const int in0 = (attr & 0x80) && p != 0;
			if (in0 != layer0)
				continue;
			d[x] = (uint16_t)(128 + (((attr & 0x10) >> 3) | ((attr & 0x20) >> 5)) * 8 + p);
		}
	}
}

static void sp_render(void)
{
	memcpy(rbx_video.palette, s_palette, sizeof(s_palette));
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	bg_draw(bm, pitch, 0);
	for (int offs = 0; offs < 0x100; offs += 4) {   // draw_sprites (in the list's order, wrapping in x)
		const uint8_t *s = s_snap->spr + offs;
		const int attr = s[1];
		const uint32_t code = (uint32_t)(((attr & 3) << 8) | s[3]);
		const int colour = (attr & 0xf0) >> 4, sx = s[2] - 2, sy = s[0];
		const int fx = (attr & 8) != 0, fy = (attr & 4) != 0;
		spr_blit(code, colour, fx, fy, sx, sy, bm, pitch);
		spr_blit(code, colour, fx, fy, sx - 256, sy, bm, pitch);
	}
	bg_draw(bm, pitch, 1);
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap->vram, s_ram + VRAM, sizeof(s_snap->vram));
	memcpy(s_snap->cram, s_ram + CRAM, sizeof(s_snap->cram));
	memcpy(s_snap->spr, s_ram + SPR, sizeof(s_snap->spr));
	s_snap->scroll_y = s_scroll_y;
	rbx_video_frame_done();
}

// ---- init / frame ----
static void sp_reset(void)
{
	memset(s_ram, 0, sizeof(s_ram));
	s_scroll_y = 0;
	m6809_reset(&s_cpu);
	m6809_set_nmi(&s_cpu, 0);
	m6809_set_firq(&s_cpu, 0);
	s_firq = 0;
	s_mcu_p1 = s_mcu_p2 = 0;
	s_mcu_p3 = 0xff;
	if (!s_boot)
		mcs51_reset(&s_mcu);
	memset(s_sram, 0, sizeof(s_sram));
	s_latch = 0;
	m6502_reset(&s_snd);
	m6502_set_irq(&s_snd, 0);
	m6502_set_nmi(&s_snd, 0);
	rbx_ym_reset(0);
	rbx_ym_reset(1);
	s_mix_n = s_ym2_n = 0;
	s_line = 0;
	s_cpu_frac = s_snd_frac = s_mcu_frac = s_ym1_frac = s_ym2_frac = 0;
}

static void sp_exit(void);

static int sp_init(const rbx_game *g)
{
	rbx_cheat_prefer("m6809");   // (the cheats' main CPU: another CPU is set up first)
	s_boot = !strcmp(g->machine, "sidepcktb");
	uint32_t csize, ssize, psize;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_zrom = rbx_region("audiocpu", &s_zrom_size);
	s_mcu_rom = s_boot ? NULL : rbx_region("mcu", &s_mcu_size);
	uint8_t *chars = rbx_region("chars", &csize), *spr = rbx_region("sprites", &ssize), *prom = rbx_region("proms", &psize);
	if (!s_rom || s_rom_size < 0x10000 || !s_zrom || s_zrom_size < 0x10000 || (!s_boot && (!s_mcu_rom || s_mcu_size < 0x1000)) ||
	    !chars || csize < 0x18000 || !spr || ssize < 0x18000 || !prom || psize < 0x200) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	{	// charlayout (2048, 3 bpp: planes 0, 0x8000, 0x10000), spritelayout (1024, the same planes)
		static uint32_t x8[8], y8[8], xs[16], y16[16];
		for (int i = 0; i < 8; i++) {
			x8[i] = (uint32_t)i;
			y8[i] = (uint32_t)(i * 8);
			xs[i] = (uint32_t)(128 + i);
			xs[8 + i] = (uint32_t)i;
		}
		for (int i = 0; i < 16; i++)
			y16[i] = (uint32_t)(i * 8);
		static const uint32_t pl[3] = { 0, 0x8000 * 8, 0x10000 * 8 };
		const rbx_gfx_layout l8 = { 8, 8, 3, pl, x8, y8, 8 * 8 }, ls = { 16, 16, 3, pl, xs, y16, 32 * 8 };
		if (rbx_gfx_decode(&s_gfx8, &l8, chars, 2048 * 8) || rbx_gfx_decode(&s_gfxspr, &ls, spr, 1024 * 32)) {
			rbx_fail("%s: out of memory (graphics)", g->set);
			sp_exit();
			return -1;
		}
	}
	for (int i = 0; i < 256; i++) {   // palette: red the high nibble, green the low; blue the second PROM
		static const uint8_t w[4] = { 0x0e, 0x1f, 0x43, 0x8f };
		int r = 0, gg = 0, b = 0;
		for (int k = 0; k < 4; k++) {
			r += ((prom[i] >> (4 + k)) & 1) * w[k];
			gg += ((prom[i] >> k) & 1) * w[k];
			b += ((prom[256 + i] >> k) & 1) * w[k];
		}
		s_palette[i] = (uint32_t)(r << 16 | gg << 8 | b);
	}
	if (!s_boot) {
		mcs51_init(&s_mcu, s_mcu_rom, 0x1000);
		s_mcu.port_in = mcu_in;
		s_mcu.port_out = mcu_out;
	}
	s_snap = calloc(1, sizeof(*s_snap));
	// sound: YM2203 1.5 MHz 0.25, YM3526 3 MHz 1.0
	rbx_ym_config yc;
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM2203;
	yc.clock = YM1_CLOCK;
	yc.gain[0] = yc.gain[1] = yc.gain[2] = yc.gain[3] = 64;
	int rc = !s_snap || rbx_ym_init(0, &yc) || rbx_ym_set_async(0, 1);
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM3526;
	yc.clock = YM2_CLOCK;
	yc.gain[0] = 256;
	yc.irq = ym2_irq;
	rc = rc || rbx_ym_init(1, &yc) || rbx_ym_set_async(1, 1);
	if (!rc) {
		s_mix_cap = (int)(rbx_ym_rate(0) / 50) + 256;
		s_ym2_cap = (int)(rbx_ym_rate(1) / 50) + 256;
		s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
		s_ym2buf = (int32_t *)calloc((size_t)s_ym2_cap, sizeof(int32_t));
		rc = !s_mix || !s_ym2buf || rbx_audio_setup(rbx_ym_rate(0)) || rbx_video_setup(VIS_W, VIS_H, 256);
	}
	if (rc) {
		rbx_fail("%s: out of memory", g->set);
		sp_exit();
		return -1;
	}
	m6502_init(&s_snd, M6502_NMOS);
	for (int p = 0; p < 0x10; p++)
		s_snd.rpage[p] = s_snd.wpage[p] = s_sram + p * 256;
	for (int p = 0x80; p < 0x100; p++)
		s_snd.rpage[p] = s_zrom + p * 256;
	s_snd.read = snd_r;
	s_snd.write = snd_w;
	m6809_init(&s_cpu);
	for (int p = 0; p < 0x20; p++) {   // (RAM; the tiles and their colours mirrored at 0x1400 and 0x1c00)
		const int o = p < 0x10 ? p * 256 : ((p & ~4) * 256);
		s_cpu.rpage[p] = s_cpu.wpage[p] = s_ram + o;
	}
	s_cpu.rpage[0x20] = s_cpu.wpage[0x20] = s_ram + SPR;
	for (int p = 0x40; p < 0x100; p++)
		s_cpu.rpage[p] = s_rom + p * 256;
	s_cpu.read = cpu_r;
	s_cpu.write = cpu_w;
	s_port_p[0] = rbx_port_find("P1");
	s_port_p[1] = rbx_port_find("P2");
	s_port_dsw[0] = rbx_port_find("DSW1");
	s_port_dsw[1] = rbx_port_find("DSW2");
	sp_reset();
	return 0;
}

static void sp_exit(void)
{
	rbx_ym_exit(0);
	rbx_ym_exit(1);
	free(s_mix);
	free(s_ym2buf);
	s_mix = s_ym2buf = NULL;
	rbx_gfx_free(&s_gfx8);
	rbx_gfx_free(&s_gfxspr);
	free(s_snap);
	s_snap = NULL;
}

static void sp_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	for (int line = 0; line < LINES; line++) {
		s_line = line;
		if (line == VBL_START) {   // screen_vblank on the NMI
			vblank_capture();
			m6809_set_nmi(&s_cpu, 1);
		} else if (line == VBL_END)
			m6809_set_nmi(&s_cpu, 0);
		s_cpu_frac += (uint64_t)CPU_CLOCK * 1000;
		const uint64_t den = (uint64_t)FRAME_MHZ * LINES;
		const int64_t cyc = (int64_t)(s_cpu_frac / den);
		s_cpu_frac -= (uint64_t)cyc * den;
		if (s_boot)
			m6809_run_until(&s_cpu, s_cpu.cycles + cyc);
		else {   // (MAME: perfect sync with the MCU)
			const int mc = rbx_line_cycles(&s_mcu_frac, MCU_CLOCK, FRAME_MHZ, LINES);
			const int64_t t0 = s_cpu.cycles;
			for (int k = 0; k < SLICES; k++) {
				m6809_run_until(&s_cpu, t0 + cyc * (k + 1) / SLICES);
				mcs51_execute(&s_mcu, mc * (k + 1) / SLICES - mc * k / SLICES);
			}
		}
		sound_line();
	}
	sound_frame_end();
}

static int sp_state(void *ctx, int load)
{
	uint8_t cpu[M6809_STATE_SIZE], snd[M6502_STATE_SIZE];
	if (!load) {
		memcpy(cpu, &s_cpu, M6809_STATE_SIZE);
		memcpy(snd, &s_snd, M6502_STATE_SIZE);
	}
	uint64_t fr[5] = { s_cpu_frac, s_snd_frac, s_mcu_frac, s_ym1_frac, s_ym2_frac };
	int32_t misc[6] = { s_scroll_y, s_firq, s_mcu_p1, s_mcu_p2, s_mcu_p3, s_latch };
	int rc = 0;
	rc |= rbx_state_io(ctx, "m6809", cpu, M6809_STATE_SIZE);
	rc |= rbx_state_io(ctx, "ram", s_ram, sizeof(s_ram));
	if (!s_boot) {
		rc |= rbx_state_io(ctx, "mcu", &s_mcu.st, sizeof(s_mcu.st));
		rc |= rbx_state_io(ctx, "mcuf", s_mcu.forced_inputs, sizeof(s_mcu.forced_inputs));
	}
	rc |= rbx_state_io(ctx, "m6502", snd, M6502_STATE_SIZE);
	rc |= rbx_state_io(ctx, "sram", s_sram, sizeof(s_sram));
	rc |= rbx_ym_state(0, ctx, load);
	rc |= rbx_ym_state(1, ctx, load);
	rc |= rbx_state_io(ctx, "frac", fr, sizeof(fr));
	rc |= rbx_state_io(ctx, "misc", misc, sizeof(misc));
	if (!rc && load) {
		memcpy(&s_cpu, cpu, M6809_STATE_SIZE);
		memcpy(&s_snd, snd, M6502_STATE_SIZE);
		s_cpu_frac = fr[0];
		s_snd_frac = fr[1];
		s_mcu_frac = fr[2];
		s_ym1_frac = fr[3];
		s_ym2_frac = fr[4];
		s_scroll_y = (uint8_t)misc[0];
		s_firq = misc[1];
		s_mcu_p1 = (uint8_t)misc[2];
		s_mcu_p2 = (uint8_t)misc[3];
		s_mcu_p3 = (uint8_t)misc[4];
		s_latch = (uint8_t)misc[5];
	}
	return rc ? -1 : 0;
}

static int sp_save(void *ctx) { return sp_state(ctx, 0); }
static int sp_load(void *ctx) { return sp_state(ctx, 1); }
static int sp_supported(const char *machine) { return !strcmp(machine, "sidepckt") || !strcmp(machine, "sidepcktb"); }

const rbx_driver rbx_drv_sidepckt = { "DECO Side Pocket", sp_init, sp_exit, sp_reset, sp_frame, sp_save, sp_load,
                                      sp_supported, sp_render, NULL };
