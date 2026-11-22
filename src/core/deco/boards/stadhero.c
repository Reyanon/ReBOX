/*  Data East Stadium Hero. Ported from MAME src/mame/dataeast/stadhero.cpp (BSD-3-Clause, copyright-holders: Bryan
    McPhail).
    68000 at 10 MHz (IRQ 5 at vblank until its acknowledge write), a BAC06 tilemap chip (3 bpp 16x16 tiles, wide 2, its
    RAM banked), an 8x8 text layer, the MXC06 sprites (the live list), the sound: 6502 at 1.5 MHz with YM2203, YM3812
    (on its IRQ) and an OKI, the latch on its NMI. A port reads the beam's horizontal position (the game's random
    numbers).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m68k_bus.h"
#include "../../cpu/m68k/m68k.h"
#include "../../cpu/m6502/m6502.h"
#include "../../sound/okim6295.h"
#include "../../sound/rbx_ym.h"
#include "../../video/gfxlayout.h"
#include "../video/bac06.h"
#include "../video/decmxc06.h"

#include <stdlib.h>
#include <string.h>

#define CPU_CLOCK 10000000       // 20 MHz / 2
#define SND_CLOCK 1500000        // 24 MHz / 16
#define YM1_CLOCK 1500000
#define YM2_CLOCK 3000000
#define FRAME_MHZ 58000          // set_refresh(58 Hz)
#define LINES 256                // set_size(32*8, 32*8)
#define VIS_Y0 8
#define VIS_H 240
#define VIS_W 256
#define VBL_LINES 8              // vblank 529 us

static uint8_t *s_rom;
static uint32_t s_rom_size;
static uint16_t (*s_ram_hp)[0x2000];             // 0xff8000 .. 0xffbfff
#define s_ram (*s_ram_hp)
static uint16_t s_spriteram[0x400];        // 0xffc000 (mirror 0x800)
static uint16_t s_pal[0x400];              // xBGR_444
static uint16_t s_pf1[0x400];              // the text layer (0x200000)
static bac06_t s_tile;
static decmxc06_t s_spr;
static rbx_gfx s_gfx8, s_gfx16, s_gfxspr;
static int s_port_in, s_port_dsw, s_port_coin;
static int s_line, s_line_cyc;
static uint32_t s_frame;
static uint64_t s_cpu_frac, s_snd_frac, s_ym1_frac, s_ym2_frac;
static struct {
	bac06_t tile;
	uint16_t spriteram[0x400], pal[0x400], pf1[0x400];
	uint32_t frame;
} *s_snap;

// ---- sound: 6502, YM2203, YM3812, OKI ----
static m6502_t s_snd;
static uint8_t s_sram[0x600];
static uint8_t *s_zrom;
static uint32_t s_zrom_size;
static okim6295_t s_oki;
static uint8_t s_latch;
static uint32_t s_oki_acc;
static int32_t s_oki_hold;
static int32_t *s_mix, *s_ym2buf;
static int s_mix_n, s_mix_cap, s_ym2_n, s_ym2_cap;

static void ym2_irq(int state) { m6502_set_irq(&s_snd, state); }
static uint8_t snd_r(void *ctx, uint16_t a)
{
	(void)ctx;
	if (a < 0x0600) return s_sram[a];
	if (a == 0x3000) {
		m6502_set_nmi(&s_snd, 0);
		return s_latch;
	}
	if (a == 0x3800) return oki_read(&s_oki);
	if (a >= 0x8000) return a < s_zrom_size ? s_zrom[a] : 0;
	return 0;
}
static void snd_w(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	if (a < 0x0600) s_sram[a] = v;
	else if (a == 0x0800 || a == 0x0801) rbx_ym_write(0, a & 1, v);
	else if (a == 0x1000 || a == 0x1001) rbx_ym_write(1, a & 1, v);
	else if (a == 0x3800) oki_write(&s_oki, v);
}
static void sound_line(void)
{
	s_snd_frac += (uint64_t)SND_CLOCK * 1000;
	const uint64_t den = (uint64_t)FRAME_MHZ * LINES;
	const int64_t cyc = (int64_t)(s_snd_frac / den);
	s_snd_frac -= (uint64_t)cyc * den;
	m6502_run_until(&s_snd, s_snd.cycles + cyc);
	const int n = rbx_ym_advance(0, (uint32_t)rbx_line_cycles(&s_ym1_frac, YM1_CLOCK, FRAME_MHZ, LINES), s_mix + s_mix_n,
	                             s_mix_cap - s_mix_n);
	s_ym2_n += rbx_ym_advance(1, (uint32_t)rbx_line_cycles(&s_ym2_frac, YM2_CLOCK, FRAME_MHZ, LINES), s_ym2buf + s_ym2_n,
	                          s_ym2_cap - s_ym2_n);
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
static void wmask(uint16_t *p, uint16_t v, uint16_t mask) { *p = (uint16_t)((*p & ~mask) | (v & mask)); }

static uint16_t main_read16(uint32_t a)
{
	a &= 0xffffff;
	if (a >= 0x260000 && a < 0x262000) return bac06_vram_r(&s_tile, (int)((a - 0x260000) >> 1));
	switch (a) {
	case 0x30c000: return (uint16_t)rbx_port_read(s_port_in);
	case 0x30c002: {   // the coins (high byte); mystery_r: the beam's horizontal position / 2
		const int vbl = s_line >= VIS_Y0 + VIS_H && s_line < VIS_Y0 + VIS_H + VBL_LINES;
		rbx_port_custom(s_port_coin, 0x80, vbl ? 0x80 : 0);
		const int hpos = s_line_cyc ? (int)((int64_t)m68k_cycles_run() * 256 / s_line_cyc) : 0;
		return (uint16_t)((rbx_port_read(s_port_coin) & 0xff) << 8 | ((hpos > 255 ? 255 : hpos) / 2));
	}
	case 0x30c004: return (uint16_t)rbx_port_read(s_port_dsw);
	}
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
	if (a >= 0x240000 && a < 0x240008) bac06_ctrl_w(&s_tile, (int)((a >> 1) & 3), v, mask);
	else if (a >= 0x240010 && a < 0x240018) bac06_scroll_w(&s_tile, (int)((a >> 1) & 3), v, mask);
	else if (a >= 0x260000 && a < 0x262000) bac06_vram_w(&s_tile, (int)((a - 0x260000) >> 1), v, mask);
	else if (a == 0x30c004) m68k_bus_set_line(5, 0);   // int_ack_w
	else if (a == 0x30c006) {
		if (mask & 0xff) {
			s_latch = (uint8_t)v;
			m6502_set_nmi(&s_snd, 1);
		}
	}
}
static void main_write16(uint32_t a, uint16_t v) { main_write16m(a, v, 0xffff); }
static void main_write8(uint32_t a, uint8_t v)
{
	const int sh = a & 1 ? 0 : 8;
	main_write16m(a & ~1u, (uint16_t)(v << sh), (uint16_t)(0xff << sh));
}

// ---- video ----
static void sh_render(void)
{
	for (int i = 0; i < 0x400; i++) {   // xBGR_444
		const uint16_t p = s_snap->pal[i];
		rbx_video.palette[i] = (uint32_t)(((p & 15) * 17) << 16 | (((p >> 4) & 15) * 17) << 8 | (((p >> 8) & 15) * 17));
	}
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	bac06_t *t = &s_snap->tile;
	const int flip = (t->ctrlreg[0] & 0x80) != 0;
	t->flip_screen = flip;
	decmxc06_t spr = s_spr;
	spr.flip_screen = flip;
	bac06_draw(t, bm, NULL, pitch, VIS_W, VIS_Y0, 0, VIS_H, BAC06_OPAQUE, 0, 0xff);
	decmxc06_draw(&spr, s_snap->spriteram, 0x400, bm, NULL, pitch, VIS_W, VIS_H, VIS_Y0, s_snap->frame);
	for (int y = 0; y < VIS_H; y++) {   // the text layer: 32x32 of 8x8 by rows, 3 bpp, pen 0 clear
		uint16_t *d = bm + y * pitch;
		for (int x = 0; x < VIS_W; x++) {
			int px = x, py = y + VIS_Y0;
			if (flip) {
				px = 255 - px;
				py = 255 - py;
			}
			const uint16_t tw = s_snap->pf1[(py >> 3) * 32 + (px >> 3)];
			const uint32_t code = s_gfx8.count ? (tw & 0xfffu) % s_gfx8.count : 0;
			const uint8_t p = s_gfx8.pix[(code * 8 + (uint32_t)(py & 7)) * 8 + (uint32_t)(px & 7)];
			if (p)
				d[x] = (uint16_t)((tw >> 12) * 8 + p);
		}
	}
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	s_snap->tile = s_tile;
	memcpy(s_snap->spriteram, s_spriteram, sizeof(s_spriteram));
	memcpy(s_snap->pal, s_pal, sizeof(s_pal));
	memcpy(s_snap->pf1, s_pf1, sizeof(s_pf1));
	s_snap->frame = s_frame;
	rbx_video_frame_done();
}

// ---- init / frame ----
static void sh_reset(void)
{
	memset(s_ram, 0, sizeof(s_ram));
	memset(s_spriteram, 0, sizeof(s_spriteram));
	memset(s_pal, 0, sizeof(s_pal));
	memset(s_pf1, 0, sizeof(s_pf1));
	memset(s_tile.vram, 0, sizeof(s_tile.vram));
	bac06_reset(&s_tile);
	memset(s_sram, 0, sizeof(s_sram));
	s_latch = 0;
	m6502_reset(&s_snd);
	m6502_set_irq(&s_snd, 0);
	m6502_set_nmi(&s_snd, 0);
	rbx_ym_reset(0);
	rbx_ym_reset(1);
	oki_reset(&s_oki);
	s_oki_acc = 0;
	s_oki_hold = 0;
	s_mix_n = s_ym2_n = 0;
	m68k_bus_irq_reset();
	m68k_pulse_reset();
	s_line = 0;
	s_frame = 0;
	s_cpu_frac = s_snd_frac = s_ym1_frac = s_ym2_frac = 0;
}

static void sh_exit(void);

static int sh_init(const rbx_game *g)
{
	if (!s_ram_hp && !(s_ram_hp = calloc(1, sizeof(*s_ram_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	uint32_t csize, tsize, ssize, osize;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_zrom = rbx_region("audiocpu", &s_zrom_size);
	uint8_t *chars = rbx_region("chars", &csize), *tiles = rbx_region("tiles", &tsize),
	        *spr = rbx_region("sprites", &ssize), *oki = rbx_region("oki", &osize);
	if (!s_rom || !s_zrom || !chars || !tiles || !spr || !oki) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	{	// charlayout (3 bpp: the thirds 0, 1, 2), tile_3bpp (the thirds 2, 1, 0), spritelayout (the quarters 3 .. 0)
		static uint32_t x8[8], y8[8], x16[16], y16[16];
		for (int i = 0; i < 8; i++) {
			x8[i] = (uint32_t)i;
			y8[i] = (uint32_t)(i * 8);
			x16[i] = (uint32_t)(16 * 8 + i);
			x16[8 + i] = (uint32_t)i;
		}
		for (int i = 0; i < 16; i++)
			y16[i] = (uint32_t)(i * 8);
		const uint32_t tc = csize / 3 * 8, tt = tsize / 3 * 8, qs = ssize * 2;
		const uint32_t pc[3] = { 0, tc, 2 * tc }, pt[3] = { 2 * tt, tt, 0 }, ps[4] = { 3 * qs, 2 * qs, qs, 0 };
		const rbx_gfx_layout l8 = { 8, 8, 3, pc, x8, y8, 8 * 8 };
		const rbx_gfx_layout lt = { 16, 16, 3, pt, x16, y16, 16 * 16 }, ls = { 16, 16, 4, ps, x16, y16, 16 * 16 };
		if (rbx_gfx_decode(&s_gfx8, &l8, chars, csize / 3) || rbx_gfx_decode(&s_gfx16, &lt, tiles, tsize / 3) ||
		    rbx_gfx_decode(&s_gfxspr, &ls, spr, ssize / 4)) {
			rbx_fail("%s: out of memory (graphics)", g->set);
			sh_exit();
			return -1;
		}
	}
	memset(&s_tile, 0, sizeof(s_tile));   // (set_gfx_region_wide(1, 1, 2): the 3 bpp tiles from 512)
	s_tile.gfx16 = &s_gfx16;
	s_tile.base16 = 512;
	s_tile.gran16 = 3;
	s_tile.wide = 2;
	memset(&s_spr, 0, sizeof(s_spr));
	s_spr.gfx = &s_gfxspr;
	s_spr.color_base = 256;
	s_snap = calloc(1, sizeof(*s_snap));
	// sound: YM2203 1.5 MHz (SSG 0.95, FM 0.40), YM3812 3 MHz 0.80, OKI 1.056 MHz 0.80
	rbx_ym_config yc;
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM2203;
	yc.clock = YM1_CLOCK;
	yc.gain[0] = yc.gain[1] = yc.gain[2] = 243;
	yc.gain[3] = 102;
	int rc = !s_snap || rbx_ym_init(0, &yc) || rbx_ym_set_async(0, 1);
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM3812;
	yc.clock = YM2_CLOCK;
	yc.gain[0] = 205;
	yc.irq = ym2_irq;
	rc = rc || rbx_ym_init(1, &yc) || rbx_ym_set_async(1, 1);
	if (!rc) {
		s_mix_cap = (int)(rbx_ym_rate(0) / 50) + 256;
		s_ym2_cap = (int)(rbx_ym_rate(1) / 50) + 256;
		s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
		s_ym2buf = (int32_t *)calloc((size_t)s_ym2_cap, sizeof(int32_t));
		rc = !s_mix || !s_ym2buf || rbx_audio_setup(rbx_ym_rate(0)) || rbx_video_setup(VIS_W, VIS_H, 0x400);
	}
	if (rc) {
		rbx_fail("%s: out of memory", g->set);
		sh_exit();
		return -1;
	}
	oki_init(&s_oki, 1056000, 1, oki, osize, 205);
	m6502_init(&s_snd, M6502_NMOS);
	for (int p = 0; p < 6; p++)
		s_snd.rpage[p] = s_snd.wpage[p] = s_sram + p * 256;
	for (int p = 0x80; p < 0x100; p++)
		if ((uint32_t)p * 256 + 256 <= s_zrom_size)
			s_snd.rpage[p] = s_zrom + p * 256;
	s_snd.read = snd_r;
	s_snd.write = snd_w;
	s_port_in = rbx_port_find("INPUTS");
	s_port_dsw = rbx_port_find("DSW");
	s_port_coin = rbx_port_find("COIN");
	m68k_bus_swap16(s_rom, s_rom_size);
	m68k_init();
	m68k_set_cpu_type(M68K_CPU_TYPE_68000);
	m68k_bus_reset_map();
	m68k_bus_map(0, 0x1ffff, s_rom, s_rom_size, 1, 0);
	m68k_bus_map(0x200000, 0x2007ff, s_pf1, sizeof(s_pf1), 1, 1);
	m68k_bus_map(0x310000, 0x3107ff, s_pal, sizeof(s_pal), 1, 1);
	m68k_bus_map(0xff8000, 0xffbfff, s_ram, sizeof(s_ram), 1, 1);
	m68k_bus_map(0xffc000, 0xffc7ff, s_spriteram, sizeof(s_spriteram), 1, 1);
	m68k_bus_map(0xffc800, 0xffcfff, s_spriteram, sizeof(s_spriteram), 1, 1);
	m68k_bus.read8 = main_read8;
	m68k_bus.read16 = main_read16;
	m68k_bus.write8 = main_write8;
	m68k_bus.write16 = main_write16;
	sh_reset();
	return 0;
}

static void sh_exit(void)
{
	rbx_ym_exit(0);
	rbx_ym_exit(1);
	free(s_mix);
	free(s_ym2buf);
	s_mix = s_ym2buf = NULL;
	rbx_gfx_free(&s_gfx8);
	rbx_gfx_free(&s_gfx16);
	rbx_gfx_free(&s_gfxspr);
	free(s_snap);
	s_snap = NULL;
	free(s_ram_hp);
	s_ram_hp = NULL;
}

static void sh_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	for (int line = 0; line < LINES; line++) {
		s_line = line;
		if (line == VIS_Y0 + VIS_H) {   // irq5_line_assert at vblank (until int_ack_w)
			vblank_capture();
			m68k_bus_set_line(5, 1);
		}
		s_line_cyc = rbx_line_cycles(&s_cpu_frac, CPU_CLOCK, FRAME_MHZ, LINES);
		m68k_execute(s_line_cyc);
		sound_line();
	}
	sound_frame_end();
	s_frame++;
}

static int sh_state(void *ctx, int load)
{
	const uint32_t csize = m68k_context_size();
	uint8_t *cpu = (uint8_t *)malloc(csize);
	if (!cpu)
		return -1;
	if (!load)
		m68k_get_context(cpu);
	uint32_t irq[2] = { m68k_bus_irq_pending(), m68k_bus_cur->irq_asserted };
	uint64_t fr[4] = { s_cpu_frac, s_snd_frac, s_ym1_frac, s_ym2_frac };
	int32_t misc[5] = { (int32_t)s_frame, s_latch, (int32_t)s_oki_acc, s_oki_hold, 0 };
	uint8_t snd[M6502_STATE_SIZE];
	if (!load)
		memcpy(snd, &s_snd, M6502_STATE_SIZE);
	int rc = 0;
	rc |= rbx_state_io(ctx, "m68k", cpu, csize);
	rc |= rbx_state_io(ctx, "irq", irq, sizeof(irq));
	rc |= rbx_state_io(ctx, "ram", s_ram, sizeof(s_ram));
	rc |= rbx_state_io(ctx, "spr", s_spriteram, sizeof(s_spriteram));
	rc |= rbx_state_io(ctx, "pal", s_pal, sizeof(s_pal));
	rc |= rbx_state_io(ctx, "pf1", s_pf1, sizeof(s_pf1));
	rc |= rbx_state_io(ctx, "bac", &s_tile, offsetof(bac06_t, gfx8));
	rc |= rbx_state_io(ctx, "m6502", snd, M6502_STATE_SIZE);
	rc |= rbx_state_io(ctx, "sram", s_sram, sizeof(s_sram));
	rc |= rbx_state_io(ctx, "oki", s_oki.v, sizeof(s_oki.v) + sizeof(int));
	rc |= rbx_ym_state(0, ctx, load);
	rc |= rbx_ym_state(1, ctx, load);
	rc |= rbx_state_io(ctx, "frac", fr, sizeof(fr));
	rc |= rbx_state_io(ctx, "misc", misc, sizeof(misc));
	if (!rc && load) {
		m68k_set_context(cpu);
		m68k_bus_set_irq_pending(irq[0]);
		m68k_bus_cur->irq_asserted = irq[1];
		m68k_bus_update_irq();
		memcpy(&s_snd, snd, M6502_STATE_SIZE);
		s_cpu_frac = fr[0];
		s_snd_frac = fr[1];
		s_ym1_frac = fr[2];
		s_ym2_frac = fr[3];
		s_frame = (uint32_t)misc[0];
		s_latch = (uint8_t)misc[1];
		s_oki_acc = (uint32_t)misc[2];
		s_oki_hold = misc[3];
	}
	free(cpu);
	return rc ? -1 : 0;
}

static int sh_save(void *ctx) { return sh_state(ctx, 0); }
static int sh_load(void *ctx) { return sh_state(ctx, 1); }
static int sh_supported(const char *machine) { return !strcmp(machine, "stadhero"); }

const rbx_driver rbx_drv_stadhero = { "DECO Stadium Hero", sh_init, sh_exit, sh_reset, sh_frame, sh_save, sh_load,
                                      sh_supported, sh_render, NULL };
