/*  Data East Pocket Gal / Pocket Gal 2 / Super Pool III, from MAME src/mame/dataeast/pcktgal.cpp (BSD-3-Clause,
    copyright-holders: Bryan McPhail).
    A 6502 at 2 MHz (NMI at vblank), a BAC06 playfield of 8x8 tiles (8-bit, big-endian), 2 bpp 16x16 sprites, the PROM
    palette. The sound: DECO 222 (Pocket Gal 2: a plain 6502) at 1.5 MHz with YM2203 and YM3812 and an MSM5205 fed a
    byte at a time (MAME's HOLD_LINE IRQ on every second VCK), the latch on an NMI pulse, a banked ROM. The bootleg:
    other graphics layouts, its map drawn without the playfield's registers (MAME draw_bootleg).
    MAME's screen: 60 Hz, 256x256 without a vblank time, 256x224 shown.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m6502/m6502.h"
#include "../../sound/msm5205u.h"
#include "../../sound/rbx_ym.h"
#include "../../video/gfxlayout.h"
#include "../video/bac06.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define CPU_CLOCK 2000000
#define SND_CLOCK 1500000
#define YM1_CLOCK 1500000        // YM2203
#define YM2_CLOCK 3000000        // YM3812
#define FRAME_MHZ 60000
#define LINES 256
#define VIS_Y0 16
#define VIS_H 224
#define VIS_W 256

enum { V_ORIGINAL, V_BOOTLEG, V_PG2 };
static int s_var;
static uint8_t *s_rom;
static uint32_t s_rom_size;
static uint8_t s_ram[0x800], s_spr[0x200];
static int s_bank;
static m6502_t s_cpu;
static bac06_t s_tile;
static rbx_gfx s_gfx8, s_gfxspr;
static uint32_t s_palette[512];
static int s_port_p[2], s_port_dsw;
static int s_line;
static uint64_t s_cpu_frac, s_snd_frac, s_ym1_frac, s_ym2_frac;
static struct {
	bac06_t tile;
	uint8_t spr[0x200];
} *s_snap;

// ---- sound: DECO 222 / 6502, YM2203, YM3812, MSM5205 ----
static m6502_t s_snd;
static uint8_t s_sram[0x800];
static uint8_t *s_zrom;
static uint32_t s_zrom_size;
static uint8_t s_latch, s_msm_next;
static int s_snd_bank, s_toggle;
static msm5205u_t s_msm;
static int32_t *s_mix, *s_ym2buf;
static int s_mix_n, s_mix_cap, s_ym2_n, s_ym2_cap;

static void msm_vck(msm5205u_t *m, void *user)   // adpcm_int: the next nibble; the IRQ (held until taken) every second
{
	(void)user;
	msm5205u_data_w(m, (uint8_t)(s_msm_next >> 4));
	s_msm_next = (uint8_t)(s_msm_next << 4);
	s_toggle = 1 - s_toggle;
	if (s_toggle)
		m6502_set_irq(&s_snd, 1);
}
static void set_snd_bank(int b)
{
	s_snd_bank = b & 1;
	for (int p = 0; p < 0x40; p++)
		s_snd.rpage[0x40 + p] = s_zrom + s_snd_bank * 0x4000 + p * 256;
}
static uint8_t snd_r(void *ctx, uint16_t a)
{
	(void)ctx;
	if (a == 0x3000) return s_latch;
	return 0;   // (0x3400: sound_unk_r)
}
static void snd_w(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	switch (a) {
	case 0x0800: case 0x0801: rbx_ym_write(0, a & 1, v); break;
	case 0x1000: case 0x1001: rbx_ym_write(1, a & 1, v); break;
	case 0x1800: s_msm_next = v; break;   // adpcm_data_w
	case 0x2000:   // sound_bank_w: the bank (bit 2), the MSM's reset (bit 1)
		set_snd_bank((v >> 2) & 1);
		msm5205u_reset_w(&s_msm, (v >> 1) & 1);
		break;
	}
}

// ---- bus ----
static void set_bank(int v)   // bank_w: 0x4000 from 0x0000 / 0x4000, 0x6000 from 0x2000 / 0x6000 of the ROM
{
	s_bank = v & 3;
	for (int p = 0; p < 0x20; p++) {
		s_cpu.rpage[0x40 + p] = s_rom + ((s_bank & 1) ? 0x4000 : 0x0000) + p * 256;
		s_cpu.rpage[0x60 + p] = s_rom + ((s_bank & 2) ? 0x6000 : 0x2000) + p * 256;
	}
}
// the BAC06's 8-bit accessors (MAME ctrlreg8_w, scrollreg8_r / w<false>, vram8_r / w<false>)
static void ctrl8_w(uint32_t o, uint8_t v) { bac06_ctrl_w(&s_tile, (int)(o >> 1), v, 0x00ff); }
static void scroll8_w(uint32_t o, uint8_t v)
{
	const uint32_t sh = o < 4 ? (~o & 1) << 3 : 0;
	bac06_scroll_w(&s_tile, (int)(o >> 1), (uint16_t)(v << sh), (uint16_t)(0xff << sh));
}
static uint8_t scroll8_r(uint32_t o) { return (uint8_t)(s_tile.scrollreg[(o >> 1) & 7] >> ((~o & 1) << 3)); }
static uint8_t vram8_r(uint32_t o) { return (uint8_t)(bac06_vram_r(&s_tile, (int)(o >> 1)) >> ((~o & 1) << 3)); }
static void vram8_w(uint32_t o, uint8_t v)
{
	bac06_vram_w(&s_tile, (int)(o >> 1), (uint16_t)(v << ((~o & 1) << 3)), (uint16_t)(0xff << ((~o & 1) << 3)));
}
static uint8_t cpu_r(void *ctx, uint16_t a)
{
	(void)ctx;
	if (a >= 0x0800 && a < 0x1000) return vram8_r(a - 0x0800u);
	if (a >= 0x1810 && a < 0x1820) return scroll8_r(a & 0xfu);
	switch (a) {
	case 0x1800: return (uint8_t)rbx_port_read(s_port_p[0]);
	case 0x1a00: return (uint8_t)rbx_port_read(s_port_p[1]);
	case 0x1c00: return (uint8_t)rbx_port_read(s_port_dsw);
	}
	return 0;
}
static void cpu_w(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	if (a >= 0x0800 && a < 0x1000) vram8_w(a - 0x0800u, v);
	else if (a >= 0x1800 && a < 0x1808) ctrl8_w(a & 7u, v);
	else if (a >= 0x1810 && a < 0x1820) scroll8_w(a & 0xfu, v);
	else if (a == 0x1a00) {   // sound_w: the latch, an NMI pulse
		s_latch = v;
		m6502_set_nmi(&s_snd, 1);
		m6502_set_nmi(&s_snd, 0);
	} else if (a == 0x1c00)
		set_bank(v);
}

// ---- video ----
static void pg_render(void)
{
	memcpy(rbx_video.palette, s_palette, sizeof(s_palette));
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	bac06_t *t = &s_snap->tile;
	const int flip = (t->ctrlreg[0] & 0x80) != 0;
	t->flip_screen = flip;
	bac06_draw(t, bm, NULL, pitch, VIS_W, VIS_Y0, 0, VIS_H, BAC06_OPAQUE, 0, 0xff);
	const uint8_t *sr = s_snap->spr;
	for (int offs = 0; offs < 0x200; offs += 4) {   // draw_sprites (in the list's order; 0xf8: off)
		if (sr[offs] == 0xf8)
			continue;
		int sx = 240 - sr[offs + 2], sy = 240 - sr[offs];
		int fx = (sr[offs + 1] & 4) != 0, fy = (sr[offs + 1] & 2) != 0;
		if (flip) {
			sx = 240 - sx;
			sy = 240 - sy;
			fx = !fx;
			fy = !fy;
		}
		if (!s_gfxspr.count)
			continue;
		const uint32_t code = rbx_wrap((uint32_t)(sr[offs + 3] + ((sr[offs + 1] & 1) << 8)), s_gfxspr.count);
		const int colour = (sr[offs + 1] & 0x70) >> 4;
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
					bm[dy * pitch + dx] = (uint16_t)(colour * 4 + p);
			}
		}
	}
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	s_snap->tile = s_tile;
	memcpy(s_snap->spr, s_spr, sizeof(s_spr));
	rbx_video_frame_done();
}

// ---- init / frame ----
static void pg_reset(void)
{
	memset(s_ram, 0, sizeof(s_ram));
	memset(s_spr, 0, sizeof(s_spr));
	memset(s_tile.vram, 0, sizeof(s_tile.vram));
	memset(s_tile.rowscroll, 0, sizeof(s_tile.rowscroll));
	memset(s_tile.colscroll, 0, sizeof(s_tile.colscroll));
	bac06_reset(&s_tile);
	set_bank(0);
	m6502_reset(&s_cpu);
	m6502_set_irq(&s_cpu, 0);
	m6502_set_nmi(&s_cpu, 0);
	memset(s_sram, 0, sizeof(s_sram));
	s_latch = s_msm_next = 0;
	s_toggle = 0;
	set_snd_bank(0);
	m6502_reset(&s_snd);
	m6502_set_irq(&s_snd, 0);
	m6502_set_nmi(&s_snd, 0);
	rbx_ym_reset(0);
	rbx_ym_reset(1);
	msm5205u_reset(&s_msm);
	s_mix_n = s_ym2_n = 0;
	s_line = 0;
	s_cpu_frac = s_snd_frac = s_ym1_frac = s_ym2_frac = 0;
}

static void pg_exit(void);

static int pg_init(const rbx_game *g)
{
	s_var = !strcmp(g->machine, "bootleg") ? V_BOOTLEG : !strcmp(g->machine, "pcktgal2") ? V_PG2 : V_ORIGINAL;
	uint32_t csize, ssize, psize;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_zrom = rbx_region("audiocpu", &s_zrom_size);
	uint8_t *chars = rbx_region("chars", &csize), *spr = rbx_region("sprites", &ssize), *prom = rbx_region("proms", &psize);
	if (!s_rom || s_rom_size < 0x10000 || !s_zrom || s_zrom_size < 0x10000 || !chars || csize < 0x20000 || !spr ||
	    ssize < 0x10000 || !prom || psize < 0x400) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	if (s_var != V_BOOTLEG)   // init_original: the tile ROMs' 16-byte halves of each 32 swapped
		for (uint32_t i = 0; i < csize; i += 32)
			for (int j = 0; j < 16; j++) {
				const uint8_t t = chars[i + j + 16];
				chars[i + j + 16] = chars[i + j];
				chars[i + j] = t;
			}
	{	// charlayout (4096, 4 bpp: planes 0x10000, 0, 0x18000, 0x8000; the bootleg's 0x18000, 0x8000, 0x10000, 0),
		// spritelayout (1024, 2 bpp: 0x8000, 0; the bootleg's bits reversed)
		static uint32_t x8[8], y8[8], xs[16], xb[16], y16[16];
		for (int i = 0; i < 8; i++) {
			x8[i] = (uint32_t)i;
			y8[i] = (uint32_t)(i * 8);
			xs[i] = (uint32_t)(128 + i);
			xs[8 + i] = (uint32_t)i;
			xb[i] = (uint32_t)(128 + 7 - i);
			xb[8 + i] = (uint32_t)(7 - i);
		}
		for (int i = 0; i < 16; i++)
			y16[i] = (uint32_t)(i * 8);
		static const uint32_t pc[4] = { 0x10000 * 8, 0, 0x18000 * 8, 0x8000 * 8 },
		                      pb[4] = { 0x18000 * 8, 0x8000 * 8, 0x10000 * 8, 0 }, ps[2] = { 0x8000 * 8, 0 };
		const int boot = s_var == V_BOOTLEG;
		const rbx_gfx_layout l8 = { 8, 8, 4, boot ? pb : pc, x8, y8, 8 * 8 };
		const rbx_gfx_layout ls = { 16, 16, 2, ps, boot ? xb : xs, y16, 32 * 8 };
		if (rbx_gfx_decode(&s_gfx8, &l8, chars, 4096 * 8) || rbx_gfx_decode(&s_gfxspr, &ls, spr, 1024 * 32)) {
			rbx_fail("%s: out of memory (graphics)", g->set);
			pg_exit();
			return -1;
		}
	}
	for (int i = 0; i < 512; i++) {   // palette: red the low nibble, green the high; blue the second PROM
		static const uint8_t w[4] = { 0x0e, 0x1f, 0x43, 0x8f };
		int r = 0, gg = 0, b = 0;
		for (int k = 0; k < 4; k++) {
			r += ((prom[i] >> k) & 1) * w[k];
			gg += ((prom[i] >> (4 + k)) & 1) * w[k];
			b += ((prom[512 + i] >> k) & 1) * w[k];
		}
		s_palette[i] = (uint32_t)(r << 16 | gg << 8 | b);
	}
	memset(&s_tile, 0, sizeof(s_tile));   // set_gfx_region_wide(0, 0, 0): the 8x8 chars from 256
	s_tile.gfx8 = &s_gfx8;
	s_tile.base8 = 256;
	s_tile.bootleg8 = s_var == V_BOOTLEG ? 3 : 0;   // (draw_bootleg: 8x8, shape 2)
	s_snap = calloc(1, sizeof(*s_snap));
	// sound: YM2203 1.5 MHz 0.60, YM3812 3 MHz 1.0 (their IRQs not wired), MSM5205 384 kHz 0.70
	rbx_ym_config yc;
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM2203;
	yc.clock = YM1_CLOCK;
	yc.gain[0] = yc.gain[1] = yc.gain[2] = yc.gain[3] = 154;
	int rc = !s_snap || rbx_ym_init(0, &yc) || rbx_ym_set_async(0, 1);
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM3812;
	yc.clock = YM2_CLOCK;
	yc.gain[0] = 256;
	rc = rc || rbx_ym_init(1, &yc) || rbx_ym_set_async(1, 1);
	if (!rc) {
		s_mix_cap = (int)(rbx_ym_rate(0) / 50) + 256;
		s_ym2_cap = (int)(rbx_ym_rate(1) / 50) + 256;
		s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
		s_ym2buf = (int32_t *)calloc((size_t)s_ym2_cap, sizeof(int32_t));
		rc = !s_mix || !s_ym2buf || rbx_audio_setup(rbx_ym_rate(0)) || rbx_video_setup(VIS_W, VIS_H, 512);
	}
	if (rc) {
		rbx_fail("%s: out of memory", g->set);
		pg_exit();
		return -1;
	}
	msm5205u_init(&s_msm, 384000, MSM5205_S48_4B, msm_vck, NULL);
	m6502_init(&s_snd, M6502_NMOS);
	s_snd.deco222 = s_var != V_PG2;
	s_snd.irq_hold = 1;
	for (int p = 0; p < 8; p++)
		s_snd.rpage[p] = s_snd.wpage[p] = s_sram + p * 256;
	for (int p = 0x80; p < 0x100; p++)
		s_snd.rpage[p] = s_zrom + p * 256;
	s_snd.read = snd_r;
	s_snd.write = snd_w;
	m6502_init(&s_cpu, M6502_NMOS);
	for (int p = 0; p < 8; p++)
		s_cpu.rpage[p] = s_cpu.wpage[p] = s_ram + p * 256;
	s_cpu.rpage[0x10] = s_cpu.wpage[0x10] = s_spr;
	s_cpu.rpage[0x11] = s_cpu.wpage[0x11] = s_spr + 0x100;
	for (int p = 0x80; p < 0x100; p++)
		s_cpu.rpage[p] = s_rom + p * 256;
	s_cpu.read = cpu_r;
	s_cpu.write = cpu_w;
	s_port_p[0] = rbx_port_find("P1");
	s_port_p[1] = rbx_port_find("P2");
	s_port_dsw = rbx_port_find("DSW");
	pg_reset();
	return 0;
}

static void pg_exit(void)
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

static void pg_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	for (int line = 0; line < LINES; line++) {
		s_line = line;
		if (line == VIS_Y0 + VIS_H) {   // screen_vblank (no vblank time): an NMI pulse
			vblank_capture();
			m6502_set_nmi(&s_cpu, 1);
			m6502_set_nmi(&s_cpu, 0);
		}
		m6502_run_until(&s_cpu, s_cpu.cycles + rbx_line_cycles(&s_cpu_frac, CPU_CLOCK, FRAME_MHZ, LINES));
		m6502_run_until(&s_snd, s_snd.cycles + rbx_line_cycles(&s_snd_frac, SND_CLOCK, FRAME_MHZ, LINES));
		const int n = rbx_ym_advance(0, (uint32_t)rbx_line_cycles(&s_ym1_frac, YM1_CLOCK, FRAME_MHZ, LINES), s_mix + s_mix_n,
		                             s_mix_cap - s_mix_n);
		msm5205u_render(&s_msm, s_mix + s_mix_n, n, rbx_ym_rate(0), 179);   // 0.70 (its VCKs in step with the CPU)
		s_mix_n += n;
		s_ym2_n += rbx_ym_advance(1, (uint32_t)rbx_line_cycles(&s_ym2_frac, YM2_CLOCK, FRAME_MHZ, LINES),
		                          s_ym2buf + s_ym2_n, s_ym2_cap - s_ym2_n);
	}
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

static int pg_state(void *ctx, int load)
{
	uint8_t cpu[M6502_STATE_SIZE], snd[M6502_STATE_SIZE];
	if (!load) {
		memcpy(cpu, &s_cpu, M6502_STATE_SIZE);
		memcpy(snd, &s_snd, M6502_STATE_SIZE);
	}
	uint64_t fr[4] = { s_cpu_frac, s_snd_frac, s_ym1_frac, s_ym2_frac };
	int32_t misc[5] = { s_bank, s_latch, s_msm_next, s_snd_bank, s_toggle };
	int rc = 0;
	rc |= rbx_state_io(ctx, "m6502", cpu, M6502_STATE_SIZE);
	rc |= rbx_state_io(ctx, "ram", s_ram, sizeof(s_ram));
	rc |= rbx_state_io(ctx, "spr", s_spr, sizeof(s_spr));
	rc |= rbx_state_io(ctx, "bac0", &s_tile, offsetof(bac06_t, gfx8));
	rc |= rbx_state_io(ctx, "m6502s", snd, M6502_STATE_SIZE);
	rc |= rbx_state_io(ctx, "sram", s_sram, sizeof(s_sram));
	rc |= rbx_state_io(ctx, "msm", &s_msm, MSM5205_STATE_SIZE);
	rc |= rbx_ym_state(0, ctx, load);
	rc |= rbx_ym_state(1, ctx, load);
	rc |= rbx_state_io(ctx, "frac", fr, sizeof(fr));
	rc |= rbx_state_io(ctx, "misc", misc, sizeof(misc));
	if (!rc && load) {
		memcpy(&s_cpu, cpu, M6502_STATE_SIZE);
		memcpy(&s_snd, snd, M6502_STATE_SIZE);
		s_cpu_frac = fr[0];
		s_snd_frac = fr[1];
		s_ym1_frac = fr[2];
		s_ym2_frac = fr[3];
		set_bank(misc[0]);
		s_latch = (uint8_t)misc[1];
		s_msm_next = (uint8_t)misc[2];
		set_snd_bank(misc[3]);
		s_toggle = misc[4];
	}
	return rc ? -1 : 0;
}

static int pg_save(void *ctx) { return pg_state(ctx, 0); }
static int pg_load(void *ctx) { return pg_state(ctx, 1); }
static int pg_supported(const char *machine)
{
	return !strcmp(machine, "pcktgal") || !strcmp(machine, "bootleg") || !strcmp(machine, "pcktgal2");
}

const rbx_driver rbx_drv_pcktgal = { "DECO Pocket Gal", pg_init, pg_exit, pg_reset, pg_frame, pg_save, pg_load,
                                     pg_supported, pg_render, NULL };
