/*  Data East Act-Fancer / Trio The Punch. Ported from MAME src/mame/dataeast/actfancr.cpp (BSD-3-Clause,
    copyright-holders: Bryan McPhail).
    HuC6280 at 7.16 MHz as the main CPU (IRQ1 held at vblank; Trio The Punch reads its inputs through the CPU's port),
    two BAC06 tilemap chips as 8-bit registers, the MXC06 sprites (an 8-bit list copied to 16-bit words by a write),
    xBGR_444 palette written by bytes. The sound: 6502 at 1.5 MHz with YM2203, YM3812 (on its IRQ) and an OKI, the
    latch on its NMI.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../cheat_cpu.h"
#include "../../rbx.h"
#include "../../cpu/h6280/h6280.h"
#include "../../cpu/m6502/m6502.h"
#include "../../sound/okim6295.h"
#include "../../sound/rbx_ym.h"
#include "../../video/gfxlayout.h"
#include "../video/bac06.h"
#include "../video/decmxc06.h"

#include <stdlib.h>
#include <string.h>

#define CPU_CLOCK (21477200 / 3)
#define SND_CLOCK 1500000
#define YM1_CLOCK 1500000
#define YM2_CLOCK 3000000
#define FRAME_MHZ 60000          // set_refresh_hz(60)
#define LINES 256                // set_size(32*8, 32*8)
#define VIS_Y0 8
#define VIS_H 240
#define VIS_W 256
#define VBL_LINES 8              // vblank 529 us

static int s_trio;
static uint8_t *s_rom;
static uint32_t s_rom_size;
static uint8_t (*s_ram_hp)[0x4000];              // 0x1f0000 .. 0x1f3fff
#define s_ram (*s_ram_hp)
static uint8_t s_spriteram[0x800];
static uint16_t s_spriteram16[0x400];      // (the copy the MXC06 draws)
static uint8_t s_pal[0x800];               // xBGR_444, little-endian byte pairs (0x600 used)
static uint8_t s_control_select;           // (Trio The Punch)
static h6280_t s_cpu;
static bac06_t (*s_tile_hp)[2];
#define s_tile (*s_tile_hp)
static decmxc06_t s_spr;
static rbx_gfx s_gfx8, s_gfx16, s_gfxspr;
static int s_port_p1, s_port_p2, s_port_dsw1, s_port_dsw2, s_port_sys;
static int s_line;
static uint32_t s_frame;
static uint64_t s_cpu_frac, s_snd_frac, s_ym1_frac, s_ym2_frac;
static struct {
	bac06_t tile[2];
	uint16_t spriteram[0x400];
	uint8_t pal[0x800];
	uint32_t frame;
} *s_snap;

// ---- sound: 6502, YM2203, YM3812, OKI ----
static m6502_t s_snd;
static uint8_t s_sram[0x800];
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
	if (a < 0x0800) return s_sram[a];
	if (a == 0x3000) {
		m6502_set_nmi(&s_snd, 0);
		return s_latch;
	}
	if (a == 0x3800) return oki_read(&s_oki);
	if (a >= 0x4000) return a < s_zrom_size ? s_zrom[a] : 0;
	return 0;
}
static void snd_w(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	if (a < 0x0800) s_sram[a] = v;
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

// ---- bus (the HuC6280's physical addresses) ----
static int vblank(void) { return s_line >= VIS_Y0 + VIS_H && s_line < VIS_Y0 + VIS_H + VBL_LINES; }
static uint8_t system_r(void)
{
	rbx_port_custom(s_port_sys, 0x80, vblank() ? 0x80 : 0);
	return (uint8_t)rbx_port_read(s_port_sys);
}
static void buffer_spriteram(void)
{
	for (int i = 0; i < 0x400; i++)
		s_spriteram16[i] = (uint16_t)(s_spriteram[i * 2] | (s_spriteram[i * 2 + 1] << 8));
}
// the BAC06's 8-bit accessors (MAME ctrlreg8_w, scrollreg8_w<true>, vram8_r / w<true>, rowscroll8_r / w<true>)
static void ctrl8_w(bac06_t *t, uint32_t o, uint8_t v) { bac06_ctrl_w(t, (int)(o >> 1), v, 0x00ff); }
static void scroll8_w(bac06_t *t, uint32_t o, uint8_t v)
{
	const uint32_t sh = o < 4 ? (o & 1) << 3 : 0;
	bac06_scroll_w(t, (int)(o >> 1), (uint16_t)(v << sh), (uint16_t)(0xff << sh));
}
static uint8_t vram8_r(const bac06_t *t, uint32_t o) { return (uint8_t)(bac06_vram_r(t, (int)(o >> 1)) >> ((o & 1) << 3)); }
static void vram8_w(bac06_t *t, uint32_t o, uint8_t v)
{
	bac06_vram_w(t, (int)(o >> 1), (uint16_t)(v << ((o & 1) << 3)), (uint16_t)(0xff << ((o & 1) << 3)));
}
static uint8_t row8_r(const bac06_t *t, uint32_t o) { return (uint8_t)(t->rowscroll[(o >> 1) & 0xfff] >> ((o & 1) << 3)); }
static void row8_w(bac06_t *t, uint32_t o, uint8_t v)
{
	uint16_t *p = &t->rowscroll[(o >> 1) & 0xfff];
	const uint16_t m = (uint16_t)(0xff << ((o & 1) << 3));
	*p = (uint16_t)((*p & ~m) | ((uint16_t)(v << ((o & 1) << 3)) & m));
}

static uint8_t cpu_r(void *ctx, uint32_t pa)
{
	(void)ctx;
	if (s_trio) {
		if (pa >= 0x044000 && pa < 0x046000) return vram8_r(&s_tile[1], pa - 0x044000);
		if (pa >= 0x046400 && pa < 0x046800) return row8_r(&s_tile[1], pa - 0x046400);
		if (pa >= 0x064000 && pa < 0x064800) return vram8_r(&s_tile[0], pa - 0x064000);
		if (pa >= 0x066400 && pa < 0x066800) return row8_r(&s_tile[0], pa - 0x066400);
		if (pa >= 0x120000 && pa < 0x120800) return s_spriteram[pa - 0x120000];
		if (pa >= 0x130000 && pa < 0x130600) return s_pal[pa - 0x130000];
		return 0;
	}
	if (pa >= 0x062000 && pa < 0x064000) return vram8_r(&s_tile[0], pa - 0x062000);
	if (pa >= 0x072000 && pa < 0x072800) return vram8_r(&s_tile[1], pa - 0x072000);
	if (pa >= 0x100000 && pa < 0x100800) return s_spriteram[pa - 0x100000];
	if (pa >= 0x120000 && pa < 0x120600) return s_pal[pa - 0x120000];
	switch (pa) {
	case 0x130000: return (uint8_t)rbx_port_read(s_port_p1);
	case 0x130001: return (uint8_t)rbx_port_read(s_port_p2);
	case 0x130002: return (uint8_t)rbx_port_read(s_port_dsw1);
	case 0x130003: return (uint8_t)rbx_port_read(s_port_dsw2);
	case 0x140000: return system_r();
	case 0x140001: return 0;
	}
	return 0;
}
static void cpu_w(void *ctx, uint32_t pa, uint8_t v)
{
	(void)ctx;
	if (s_trio) {
		if (pa >= 0x040000 && pa < 0x040008) ctrl8_w(&s_tile[1], pa & 7, v);
		else if (pa >= 0x040010 && pa < 0x040020) scroll8_w(&s_tile[1], pa & 0xf, v);
		else if (pa >= 0x044000 && pa < 0x046000) vram8_w(&s_tile[1], pa - 0x044000, v);
		else if (pa >= 0x046400 && pa < 0x046800) row8_w(&s_tile[1], pa - 0x046400, v);
		else if (pa >= 0x060000 && pa < 0x060008) ctrl8_w(&s_tile[0], pa & 7, v);
		else if (pa >= 0x060010 && pa < 0x060020) scroll8_w(&s_tile[0], pa & 0xf, v);
		else if (pa >= 0x064000 && pa < 0x064800) vram8_w(&s_tile[0], pa - 0x064000, v);
		else if (pa >= 0x066400 && pa < 0x066800) row8_w(&s_tile[0], pa - 0x066400, v);
		else if (pa == 0x100000) { s_latch = v; m6502_set_nmi(&s_snd, 1); }
		else if (pa == 0x110000 || pa == 0x110001) buffer_spriteram();
		else if (pa >= 0x120000 && pa < 0x120800) s_spriteram[pa - 0x120000] = v;
		else if (pa >= 0x130000 && pa < 0x130600) s_pal[pa - 0x130000] = v;
		return;
	}
	if (pa >= 0x060000 && pa < 0x060008) ctrl8_w(&s_tile[0], pa & 7, v);
	else if (pa >= 0x060010 && pa < 0x060020) scroll8_w(&s_tile[0], pa & 0xf, v);
	else if (pa >= 0x062000 && pa < 0x064000) vram8_w(&s_tile[0], pa - 0x062000, v);
	else if (pa >= 0x070000 && pa < 0x070008) ctrl8_w(&s_tile[1], pa & 7, v);
	else if (pa >= 0x070010 && pa < 0x070020) scroll8_w(&s_tile[1], pa & 0xf, v);
	else if (pa >= 0x072000 && pa < 0x072800) vram8_w(&s_tile[1], pa - 0x072000, v);
	else if (pa >= 0x100000 && pa < 0x100800) s_spriteram[pa - 0x100000] = v;
	else if (pa == 0x110000 || pa == 0x110001) buffer_spriteram();
	else if (pa >= 0x120000 && pa < 0x120600) s_pal[pa - 0x120000] = v;
	else if (pa == 0x150000) { s_latch = v; m6502_set_nmi(&s_snd, 1); }
}
// Trio The Punch: the inputs through the CPU's port (control_select_w / control_r)
static uint8_t port_in(void *ctx)
{
	(void)ctx;
	switch (s_control_select) {
	case 0: return (uint8_t)rbx_port_read(s_port_p1);
	case 1: return (uint8_t)rbx_port_read(s_port_p2);
	case 2: return (uint8_t)rbx_port_read(s_port_dsw1);
	case 3: return (uint8_t)rbx_port_read(s_port_dsw2);
	case 4: return system_r();
	}
	return 0xff;
}
static void port_out(void *ctx, uint8_t v) { (void)ctx; s_control_select = v; }

// ---- video ----
static void af_render(void)
{
	const int ncol = 0x300;   // (768 entries)
	for (int i = 0; i < ncol; i++) {   // xBGR_444 (little-endian bytes)
		const uint16_t p = (uint16_t)(s_snap->pal[2 * i] | (s_snap->pal[2 * i + 1] << 8));
		rbx_video.palette[i] = (uint32_t)(((p & 15) * 17) << 16 | (((p >> 4) & 15) * 17) << 8 | (((p >> 8) & 15) * 17));
	}
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	bac06_t *t0 = &s_snap->tile[0], *t1 = &s_snap->tile[1];
	const int flip = (t1->ctrlreg[0] & 0x80) != 0;
	t0->flip_screen = t1->flip_screen = flip;
	decmxc06_t spr = s_spr;
	spr.flip_screen = flip;
	bac06_draw(t0, bm, NULL, pitch, VIS_W, VIS_Y0, 0, VIS_H, BAC06_OPAQUE, 0, 0xff);
	decmxc06_draw(&spr, s_snap->spriteram, 0x400, bm, NULL, pitch, VIS_W, VIS_H, VIS_Y0, s_snap->frame);
	bac06_draw(t1, bm, NULL, pitch, VIS_W, VIS_Y0, 0, VIS_H, 0, 0, 0xff);
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap->tile, s_tile, sizeof(s_tile));
	memcpy(s_snap->spriteram, s_spriteram16, sizeof(s_spriteram16));
	memcpy(s_snap->pal, s_pal, sizeof(s_pal));
	s_snap->frame = s_frame;
	rbx_video_frame_done();
}

// ---- init / frame ----
static void af_reset(void)
{
	memset(s_ram, 0, sizeof(s_ram));
	memset(s_spriteram, 0, sizeof(s_spriteram));
	memset(s_spriteram16, 0, sizeof(s_spriteram16));
	memset(s_pal, 0, sizeof(s_pal));
	s_control_select = 0;
	for (int i = 0; i < 2; i++) {
		memset(s_tile[i].vram, 0, sizeof(s_tile[i].vram));
		memset(s_tile[i].rowscroll, 0, sizeof(s_tile[i].rowscroll));
		memset(s_tile[i].colscroll, 0, sizeof(s_tile[i].colscroll));
		bac06_reset(&s_tile[i]);
	}
	h6280_reset(&s_cpu);
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
	s_line = 0;
	s_frame = 0;
	s_cpu_frac = s_snd_frac = s_ym1_frac = s_ym2_frac = 0;
}

static void af_exit(void);

static int af_init(const rbx_game *g)
{
	rbx_cheat_prefer("h6280");   // (the cheats' main CPU: another CPU is set up first)
	s_trio = !strcmp(g->machine, "triothep");
	uint32_t csize, tsize, ssize, osize;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_zrom = rbx_region("audiocpu", &s_zrom_size);
	uint8_t *chars = rbx_region("chars", &csize), *tiles = rbx_region("tiles", &tsize),
	        *spr = rbx_region("sprites", &ssize), *oki = rbx_region("oki", &osize);
	if (!s_rom || !s_zrom || !chars || !tiles || !spr || !oki) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	{	// layout_8x8x4 (planes the quarters 1, 3, 0, 2), layout_16x16x4 (0, 1, 2, 3)
		static uint32_t x8[8], y8[8], x16[16], y16[16];
		for (int i = 0; i < 8; i++) {
			x8[i] = (uint32_t)i;
			y8[i] = (uint32_t)(i * 8);
			x16[i] = (uint32_t)(16 * 8 + i);
			x16[8 + i] = (uint32_t)i;
		}
		for (int i = 0; i < 16; i++)
			y16[i] = (uint32_t)(i * 8);
		const uint32_t qc = csize * 2, qt = tsize * 2, qs = ssize * 2;
		const uint32_t pc[4] = { qc, 3 * qc, 0, 2 * qc }, pt[4] = { 0, qt, 2 * qt, 3 * qt }, ps[4] = { 0, qs, 2 * qs, 3 * qs };
		const rbx_gfx_layout l8 = { 8, 8, 4, pc, x8, y8, 8 * 8 };
		const rbx_gfx_layout lt = { 16, 16, 4, pt, x16, y16, 32 * 8 }, ls = { 16, 16, 4, ps, x16, y16, 32 * 8 };
		if (rbx_gfx_decode(&s_gfx8, &l8, chars, csize / 4) || rbx_gfx_decode(&s_gfx16, &lt, tiles, tsize / 4) ||
		    rbx_gfx_decode(&s_gfxspr, &ls, spr, ssize / 4)) {
			rbx_fail("%s: out of memory (graphics)", g->set);
			af_exit();
			return -1;
		}
	}
	memset(s_tile, 0, sizeof(s_tile));
	// chip 0: set_gfx_region_wide(1, 1, 2) (Trio: wide 0), the tiles from 256 (Trio: 512); chip 1: the chars
	s_tile[0].gfx16 = &s_gfx16;
	s_tile[0].base16 = s_trio ? 512 : 256;
	s_tile[0].wide = s_trio ? 0 : 2;
	s_tile[1].gfx8 = &s_gfx8;
	memset(&s_spr, 0, sizeof(s_spr));
	s_spr.gfx = &s_gfxspr;
	s_spr.color_base = s_trio ? 256 : 512;
	s_snap = calloc(1, sizeof(*s_snap));
	// sound: YM2203 1.5 MHz (SSG 0.90, FM 0.50), YM3812 3 MHz 0.90, OKI 1.024 MHz (Trio 1.056) 0.85
	rbx_ym_config yc;
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM2203;
	yc.clock = YM1_CLOCK;
	yc.gain[0] = yc.gain[1] = yc.gain[2] = 230;
	yc.gain[3] = 128;
	int rc = !s_snap || rbx_ym_init(0, &yc) || rbx_ym_set_async(0, 1);
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM3812;
	yc.clock = YM2_CLOCK;
	yc.gain[0] = 230;
	yc.irq = ym2_irq;
	rc = rc || rbx_ym_init(1, &yc) || rbx_ym_set_async(1, 1);
	if (!rc) {
		s_mix_cap = (int)(rbx_ym_rate(0) / 50) + 256;
		s_ym2_cap = (int)(rbx_ym_rate(1) / 50) + 256;
		s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
		s_ym2buf = (int32_t *)calloc((size_t)s_ym2_cap, sizeof(int32_t));
		rc = !s_mix || !s_ym2buf || rbx_audio_setup(rbx_ym_rate(0)) || rbx_video_setup(VIS_W, VIS_H, 0x300);
	}
	if (rc) {
		rbx_fail("%s: out of memory", g->set);
		af_exit();
		return -1;
	}
	oki_init(&s_oki, s_trio ? 1056000 : 1024188, 1, oki, osize, 218);
	m6502_init(&s_snd, M6502_NMOS);
	for (int p = 0; p < 8; p++)
		s_snd.rpage[p] = s_snd.wpage[p] = s_sram + p * 256;
	for (int p = 0x40; p < 0x100; p++)
		if ((uint32_t)p * 256 + 256 <= s_zrom_size)
			s_snd.rpage[p] = s_zrom + p * 256;
	s_snd.read = snd_r;
	s_snd.write = snd_w;
	h6280_init(&s_cpu);
	for (uint32_t pg = 0; pg * 0x2000 < s_rom_size && pg < 0x20; pg++)
		s_cpu.rpage[pg] = s_rom + pg * 0x2000;
	s_cpu.rpage[0xf8] = s_cpu.wpage[0xf8] = s_ram;
	s_cpu.rpage[0xf9] = s_cpu.wpage[0xf9] = s_ram + 0x2000;
	s_cpu.read = cpu_r;
	s_cpu.write = cpu_w;
	if (s_trio) {
		s_cpu.port_in = port_in;
		s_cpu.port_out = port_out;
	}
	s_port_p1 = rbx_port_find("P1");
	s_port_p2 = rbx_port_find("P2");
	s_port_dsw1 = rbx_port_find("DSW1");
	s_port_dsw2 = rbx_port_find("DSW2");
	s_port_sys = rbx_port_find("SYSTEM");
	af_reset();
	return 0;
}

static void af_exit(void)
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
	free(s_tile_hp);
	s_tile_hp = NULL;
}

static void af_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	for (int line = 0; line < LINES; line++) {
		s_line = line;
		if (line == VIS_Y0 + VIS_H) {   // the vblank on IRQ1 (HOLD_LINE: until taken)
			vblank_capture();
			h6280_set_irq(&s_cpu, H6280_IRQ1, 1);
		}
		s_cpu_frac += (uint64_t)CPU_CLOCK * 1000;
		const uint64_t den = (uint64_t)FRAME_MHZ * LINES;
		const int64_t cyc = (int64_t)(s_cpu_frac / den);
		s_cpu_frac -= (uint64_t)cyc * den;
		h6280_run_until(&s_cpu, s_cpu.cycles + cyc);
		sound_line();
	}
	sound_frame_end();
	s_frame++;
}

// HOLD_LINE: the IRQ1 line dropped when the CPU takes it
static void irq_ack(void *ctx, int line)
{
	(void)ctx;
	if (line == H6280_IRQ1)
		h6280_set_irq(&s_cpu, H6280_IRQ1, 0);
}

static int af_state(void *ctx, int load)
{
	uint8_t cpu[H6280_STATE_SIZE], snd[M6502_STATE_SIZE];
	if (!load) {
		memcpy(cpu, &s_cpu, H6280_STATE_SIZE);
		memcpy(snd, &s_snd, M6502_STATE_SIZE);
	}
	uint64_t fr[4] = { s_cpu_frac, s_snd_frac, s_ym1_frac, s_ym2_frac };
	int32_t misc[6] = { (int32_t)s_frame, s_control_select, s_latch, (int32_t)s_oki_acc, s_oki_hold, 0 };
	int rc = 0;
	rc |= rbx_state_io(ctx, "h6280", cpu, H6280_STATE_SIZE);
	rc |= rbx_state_io(ctx, "ram", s_ram, sizeof(s_ram));
	rc |= rbx_state_io(ctx, "spr", s_spriteram, sizeof(s_spriteram));
	rc |= rbx_state_io(ctx, "spr16", s_spriteram16, sizeof(s_spriteram16));
	rc |= rbx_state_io(ctx, "pal", s_pal, sizeof(s_pal));
	rc |= rbx_state_io(ctx, "bac0", &s_tile[0], offsetof(bac06_t, gfx8));
	rc |= rbx_state_io(ctx, "bac1", &s_tile[1], offsetof(bac06_t, gfx8));
	rc |= rbx_state_io(ctx, "m6502", snd, M6502_STATE_SIZE);
	rc |= rbx_state_io(ctx, "sram", s_sram, sizeof(s_sram));
	rc |= rbx_state_io(ctx, "oki", s_oki.v, sizeof(s_oki.v) + sizeof(int));
	rc |= rbx_ym_state(0, ctx, load);
	rc |= rbx_ym_state(1, ctx, load);
	rc |= rbx_state_io(ctx, "frac", fr, sizeof(fr));
	rc |= rbx_state_io(ctx, "misc", misc, sizeof(misc));
	if (!rc && load) {
		memcpy(&s_cpu, cpu, H6280_STATE_SIZE);
		memcpy(&s_snd, snd, M6502_STATE_SIZE);
		s_cpu_frac = fr[0];
		s_snd_frac = fr[1];
		s_ym1_frac = fr[2];
		s_ym2_frac = fr[3];
		s_frame = (uint32_t)misc[0];
		s_control_select = (uint8_t)misc[1];
		s_latch = (uint8_t)misc[2];
		s_oki_acc = (uint32_t)misc[3];
		s_oki_hold = misc[4];
	}
	return rc ? -1 : 0;
}

static int af_save(void *ctx) { return af_state(ctx, 0); }
static int af_load(void *ctx) { return af_state(ctx, 1); }
static int af_supported(const char *machine) { return !strcmp(machine, "actfancr") || !strcmp(machine, "triothep"); }

static int af_init_ack(const rbx_game *g)
{
	rbx_cheat_prefer("h6280");   // (the cheats' main CPU: another CPU is set up first)
	if (!s_ram_hp && !(s_ram_hp = calloc(1, sizeof(*s_ram_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	if (!s_tile_hp && !(s_tile_hp = calloc(1, sizeof(*s_tile_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	const int rc = af_init(g);
	if (!rc)
		s_cpu.irq_ack = irq_ack;
	return rc;
}

const rbx_driver rbx_drv_actfancr = { "DECO Act-Fancer", af_init_ack, af_exit, af_reset, af_frame, af_save, af_load,
                                      af_supported, af_render, NULL };
