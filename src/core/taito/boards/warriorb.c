/*  Warrior Blade / Darius II (dual screen) / Sagaia, ported from MAME src/mame/taito/warriorb.cpp (BSD-3-Clause,
    copyright-holders: David Graves), with TC0220IOC / TC0510NIO (src/mame/taito/taitoio.cpp, BSD-3-Clause) and
    TC0110PCR (src/mame/taito/tc0110pcr.cpp, BSD-3-Clause, Nicola Salmoria).
    One 68000, two TC0100SCN (video/tc0100scn.c; writes to the first one go to both), two TC0110PCR palettes,
    one sprite plane over the two screens; sound board Z80 + YM2610 (B) + TC0140SYT (machine/taito_sound.c) with
    the pan latch at e400.
    The two 320 pixel screens are drawn side by side into one 640 pixel wide bitmap; screen k uses palette
    entries 4096 * k (its TC0110PCR), index 8192 is the black pen.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m68k_bus.h"
#include "../../cpu/m68k/m68k.h"
#include "../machine/taito_sound.h"
#include "../video/tc0100scn.h"
#include "../../video/gfxlayout.h"

#include <stdlib.h>
#include <string.h>

#define LINES 256
#define SCREENS 2
#define SCREEN_W 320
#define W (SCREENS * SCREEN_W)
#define CPU_CLOCK 16000000
#define BLACK_PEN (SCREENS * 4096)

static int s_wb;                     // warriorb (else darius2d)
static int s_vis_y0, s_vis_h;        // darius2d 3*8..255, warriorb 2*8..255
static uint32_t s_scn_base[SCREENS], s_ctrl_base[SCREENS];
static uint8_t *s_rom;
static uint32_t s_rom_size;
static uint8_t s_ram[0x14000];
static uint8_t s_spr[0x2000];        // 600000-6013ff (page sized)
static uint16_t s_pcr[SCREENS][0x1000];
static uint16_t s_pcr_addr[SCREENS];
static tc0100scn_t *s_scn[SCREENS], *s_snap_scn[SCREENS];
static rbx_gfx s_sprgfx;
static int s_port[5];   // DSWA, DSWB, IN0, IN1, IN2
static uint64_t s_cpu_frac;
static struct {
	uint8_t ioc_regs[8];
	uint8_t pan[4];
} s_m;
static struct {
	uint16_t pcr[SCREENS][0x1000];
	uint8_t spr[0x1400];
} *s_snap;
static uint8_t *s_pri;
static uint16_t s_pal_last[SCREENS][0x1000];   // palette RAM converted into rbx_video.palette
static int s_pal_valid;

// ---- sound: pancontrol_w (e400-e403), original volume * 3 ----
static void update_pan(void)
{
	int pd[4];
	for (int i = 0; i < 4; i++)
		pd[i] = s_m.pan[i] * 3;
	rbx_ym_set_gain(0, 1, (pd[0] + pd[1]) * 128 / 100);   // mean of the left and right filters
	rbx_ym_set_gain(0, 2, (pd[2] + pd[3]) * 128 / 100);   // mean of the left and right filters
}

static void snd_ext_write(uint16_t a, uint8_t v)
{
	if (a >= 0xe400 && a <= 0xe403) {
		s_m.pan[(a & 3) ^ 1] = v;
		update_pan();
	}
}

// ---- TC0220IOC / TC0510NIO ----
static uint8_t ioc_read(uint32_t off)
{
	switch (off) {
	case 0: return (uint8_t)rbx_port_read(s_port[0]);
	case 1: return (uint8_t)rbx_port_read(s_port[1]);
	case 2: return (uint8_t)rbx_port_read(s_port[2]);
	case 3: return (uint8_t)rbx_port_read(s_port[3]);
	case 4: return s_m.ioc_regs[4];
	case 7: return (uint8_t)rbx_port_read(s_port[4]);
	}
	return 0xff;
}

// ---- 68000 bus ----
static int ctrl_chip(uint32_t a)
{
	for (int i = 0; i < SCREENS; i++)
		if (a >= s_ctrl_base[i] && a <= s_ctrl_base[i] + 0xf)
			return i;
	return -1;
}

static uint16_t main_read16(uint32_t a)
{
	if (a >= 0x800000 && a <= 0x80000f) return ioc_read((a - 0x800000) >> 1);
	if (a == 0x830002) return tc0140syt_master_comm_r(tsnd_syt());
	int c = ctrl_chip(a);
	if (c >= 0) return tc0100scn_ctrl_r(s_scn[c], (a >> 1) & 7);
	if (a >= 0x400000 && a <= 0x400007) return ((a >> 1) & 3) == 1 ? s_pcr[0][s_pcr_addr[0]] : 0xff;
	if (a >= 0x420000 && a <= 0x420007) return ((a >> 1) & 3) == 1 ? s_pcr[1][s_pcr_addr[1]] : 0xff;
	return 0;
}

static uint8_t main_read8(uint32_t a)
{
	if (a == 0x830003) return tc0140syt_master_comm_r(tsnd_syt());
	return (uint8_t)(main_read16(a & ~1u) >> ((a & 1) ? 0 : 8));
}

static void main_write16m(uint32_t a, uint16_t v, uint16_t mask)
{
	if (a >= s_scn_base[0] && a < s_scn_base[0] + TC0100SCN_RAM_BYTES) {   // tc0100scn_dual_screen_w
		uint32_t o = (a - s_scn_base[0]) >> 1;
		for (int i = 0; i < SCREENS; i++)
			s_scn[i]->ram[o] = (uint16_t)((s_scn[i]->ram[o] & ~mask) | (v & mask));
		return;
	}
	if (a >= 0x800000 && a <= 0x80000f) {
		if (mask & 0xff)
			s_m.ioc_regs[(a - 0x800000) >> 1] = (uint8_t)v;   // 4: coin counters / lockout, 0: watchdog
		return;
	}
	if (a == 0x830000 && (mask & 0xff)) { tc0140syt_master_port_w(tsnd_syt(), (uint8_t)v); return; }
	if (a == 0x830002 && (mask & 0xff)) { tc0140syt_master_comm_w(tsnd_syt(), (uint8_t)v); return; }
	int c = ctrl_chip(a);
	if (c >= 0) { tc0100scn_ctrl_w(s_scn[c], (a >> 1) & 7, v, mask); return; }
	for (int p = 0; p < SCREENS; p++)
		if (a >= 0x400000u + 0x20000u * p && a <= 0x400007u + 0x20000u * p) {
			switch ((a >> 1) & 3) {
			case 0: s_pcr_addr[p] = v & 0xfff; break;
			case 1: s_pcr[p][s_pcr_addr[p]] = v; break;
			}
			return;
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
static void draw_sprites(int chip, uint16_t *bm, uint8_t *pri, int pitch)
{
	const uint16_t *spr = (const uint16_t *)s_snap->spr;
	const int x_offs = SCREEN_W * chip, y_offs = 8;
	for (int offs = 0; offs < 0x1400 / 2; offs += 4) {
		uint32_t tilenum = spr[offs + 1] & 0x7fff;
		uint16_t d0 = spr[offs], d2 = spr[offs + 2], d3 = spr[offs + 3];
		int y = (-(d0 & 0x1ff) - 24) & 0x1ff;
		int flipy = (d0 >> 9) & 1;
		uint32_t pmask = (d2 & 0x100) ? 0xfffe : 0;
		int color = d2 & 0x7f;
		int x = d3 & 0x3ff, flipx = (d3 >> 10) & 1;
		x -= x_offs;
		y += y_offs;
		if (x > 0x3c0) x -= 0x400;
		if (y > 0x180) y -= 0x200;
		rbx_gfx_prio_transpen(&s_sprgfx, tilenum, (uint16_t)((color << 4) + 4096 * chip), flipx, flipy, x, y - s_vis_y0, bm, pri, pitch,
		                      SCREEN_W, s_vis_h, pmask);
	}
}

static void warriorb_render(void)
{
	for (int k = 0; k < SCREENS; k++)
		for (int i = 0; i < 4096; i++) {   // color_xbgr555, entries changed since the last frame
			uint16_t c = s_snap->pcr[k][i];
			if (c == s_pal_last[k][i] && s_pal_valid)
				continue;
			s_pal_last[k][i] = c;
			int r = c & 31, g = (c >> 5) & 31, b = (c >> 10) & 31;
			r = (r << 3) | (r >> 2); g = (g << 3) | (g >> 2); b = (b << 3) | (b >> 2);
			rbx_video.palette[k * 4096 + i] = (uint32_t)(r << 16 | g << 8 | b);
		}
	rbx_video.palette[BLACK_PEN] = 0;
	s_pal_valid = 1;
	uint16_t *bm0 = rbx_video.bitmap;
	int pitch = rbx_video.pitch;
	memset(s_pri, 0, (size_t)pitch * s_vis_h);
	for (int k = 0; k < SCREENS; k++) {
		const tc0100scn_t *scn = s_snap_scn[k];
		uint16_t *bm = bm0 + k * SCREEN_W;
		uint8_t *pri = s_pri + k * SCREEN_W;
		int layer0 = tc0100scn_bottomlayer(scn);
		if (tc0100scn_draw_ex(scn, layer0, bm, pri, pitch, s_vis_y0, s_vis_h, 0, TC0100SCN_OPAQUE))
			for (int y = 0; y < s_vis_h; y++)
				for (int x = 0; x < SCREEN_W; x++)
					bm[y * pitch + x] = BLACK_PEN;
		tc0100scn_draw(scn, layer0 ^ 1, bm, pri, pitch, s_vis_y0, s_vis_h, 1);
		draw_sprites(k, bm, pri, pitch);
		tc0100scn_draw(scn, 2, bm, pri, pitch, s_vis_y0, s_vis_h, 0);
	}
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	for (int i = 0; i < SCREENS; i++)
		tc0100scn_copy(s_snap_scn[i], s_scn[i]);
	memcpy(s_snap->pcr, s_pcr, sizeof(s_pcr));
	memcpy(s_snap->spr, s_spr, sizeof(s_snap->spr));
	rbx_video_frame_done();
}

// ---- init / frame ----
static void warriorb_reset(void)
{
	// (the palette / video RAM too: a game that has not written it yet showed the last game's, as at start it is 0)
	memset(s_pcr, 0, sizeof(s_pcr));
	memset(s_ram, 0, sizeof(s_ram));
	memset(s_spr, 0, sizeof(s_spr));
	memset(s_pcr_addr, 0, sizeof(s_pcr_addr));
	for (int i = 0; i < SCREENS; i++)
		tc0100scn_reset(s_scn[i]);
	memset(&s_m, 0, sizeof(s_m));
	tsnd_reset();
	update_pan();
	m68k_bus_irq_reset();
	m68k_pulse_reset();
	s_cpu_frac = 0;
}

static void warriorb_exit(void);

static int warriorb_init(const rbx_game *g)
{
	uint32_t scn_size, spr_size;
	s_wb = !strcmp(g->machine, "warriorb");
	s_vis_y0 = s_wb ? 16 : 24;
	s_vis_h = 256 - s_vis_y0;
	s_scn_base[0] = s_wb ? 0x300000 : 0x200000;
	s_scn_base[1] = s_wb ? 0x340000 : 0x240000;
	s_ctrl_base[0] = s_wb ? 0x320000 : 0x220000;
	s_ctrl_base[1] = s_wb ? 0x360000 : 0x260000;
	s_rom = rbx_region("maincpu", &s_rom_size);
	uint8_t *scn_gfx = rbx_region("tc0100scn_1", &scn_size);
	uint8_t *spr_gfx = rbx_region("sprites", &spr_size);
	if (!s_rom || !scn_gfx || !spr_gfx) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	m68k_bus_swap16(s_rom, s_rom_size);
	s_snap = calloc(1, sizeof(*s_snap));
	s_pri = (uint8_t *)malloc((size_t)W * 256);
	if (!s_snap || !s_pri)
		goto mem_fail;
	// gfx_16x16x4_packed_lsb
	{
		uint32_t n = spr_size / 128;
		s_sprgfx.pix = (uint8_t *)malloc((size_t)n * 257);
		if (!s_sprgfx.pix)
			goto mem_fail;
		s_sprgfx.used = s_sprgfx.pix + (size_t)n * 256;
		s_sprgfx.count = n;
		s_sprgfx.width = s_sprgfx.height = 16;
		for (uint32_t i = 0; i < n; i++) {
			uint8_t *d = s_sprgfx.pix + (size_t)i * 256, any = 0;
			for (int j = 0; j < 256; j++) {
				uint8_t b = spr_gfx[i * 128 + (j >> 1)];
				d[j] = (j & 1) ? (uint8_t)(b >> 4) : (uint8_t)(b & 15);
				any |= d[j];
			}
			s_sprgfx.used[i] = any != 0;
		}
	}
	for (int i = 0; i < SCREENS; i++) {
		s_scn[i] = (tc0100scn_t *)calloc(1, sizeof(tc0100scn_t));
		s_snap_scn[i] = (tc0100scn_t *)calloc(1, sizeof(tc0100scn_t));
		if (!s_scn[i] || !s_snap_scn[i])
			goto mem_fail;
	}
	// both SCR ROM sets are identical: decode once, the second chip shares the tiles
	if (tc0100scn_init(s_scn[0], scn_gfx, scn_size, TC0100SCN_4BPP))
		goto mem_fail;
	for (int i = 0; i < SCREENS; i++) {
		tc0100scn_t *c = s_scn[i];
		if (i) {
			c->tiles = s_scn[0]->tiles;
			c->tile_used = s_scn[0]->tile_used;
			c->ntiles = s_scn[0]->ntiles;
			c->code_mask = 0xffff;
			c->owned = NULL;
		}
		c->x_offset = 4;   // set_offsets(4, 0)
		c->y_offset = 0;
		c->multiscrn_xoffs = (i && s_wb) ? 1 : 0;
		c->multiscrn_hack = i ? 1 : 0;
		c->screen_w = SCREEN_W;
		c->screen_h = 256;
		c->pen_base = (uint16_t)(4096 * i);   // this screen's TC0110PCR
		*s_snap_scn[i] = *c;
		s_snap_scn[i]->owned = NULL;
	}
	static const char *const tags[5] = { "DSWA", "DSWB", "IN0", "IN1", "IN2" };
	for (int i = 0; i < 5; i++)
		s_port[i] = rbx_port_find(tags[i]);
	tsnd_config sc;
	memset(&sc, 0, sizeof(sc));
	sc.map = TSND_MAP_E000;
	sc.z80_clock = 4000000;
	sc.frame_rate_mhz = 60000;
	sc.lines = LINES;
	sc.ym.type = s_wb ? RBX_YM2610B : RBX_YM2610;
	sc.ym.clock = 8000000;
	sc.ym.gain[0] = 192;   // SSG 0.75 to both speakers (warriorb, mono = mean) / the subwoofer
	sc.bank_mask = 7;
	sc.ext_write = snd_ext_write;
	if (tsnd_init(&sc)) {
		rbx_fail("%s: sound init failed", g->set);
		warriorb_exit();
		return -1;
	}
	if (rbx_video_setup(W, s_vis_h, 16384)) {
		warriorb_exit();
		return -1;
	}
	s_pal_valid = 0;
	m68k_init();
	m68k_set_cpu_type(M68K_CPU_TYPE_68000);
	m68k_bus_reset_map();
	if (s_wb) {
		m68k_bus_map(0, 0x1fffff, s_rom, s_rom_size, 1, 0);
		m68k_bus_map(0x200000, 0x213fff, s_ram, sizeof(s_ram), 1, 1);
	} else {
		m68k_bus_map(0, 0x0fffff, s_rom, s_rom_size, 1, 0);
		m68k_bus_map(0x100000, 0x10ffff, s_ram, 0x10000, 1, 1);
	}
	m68k_bus_map(s_scn_base[0], s_scn_base[0] + TC0100SCN_RAM_BYTES - 1, s_scn[0]->ram, TC0100SCN_RAM_BYTES, 1, 0);
	m68k_bus_map(s_scn_base[1], s_scn_base[1] + TC0100SCN_RAM_BYTES - 1, s_scn[1]->ram, TC0100SCN_RAM_BYTES, 1, 1);
	m68k_bus_map(0x600000, 0x601fff, s_spr, sizeof(s_spr), 1, 1);
	m68k_bus.read8 = main_read8;
	m68k_bus.read16 = main_read16;
	m68k_bus.write8 = main_write8;
	m68k_bus.write16 = main_write16;
	warriorb_reset();
	return 0;
mem_fail:
	rbx_fail("%s: out of memory (graphics)", g->set);
	warriorb_exit();
	return -1;
}

static void warriorb_exit(void)
{
	for (int i = 0; i < SCREENS; i++) {
		if (s_scn[i])
			tc0100scn_exit(s_scn[i]);
		free(s_scn[i]);
		free(s_snap_scn[i]);
		s_scn[i] = s_snap_scn[i] = NULL;
	}
	free(s_snap);
	free(s_pri);
	s_snap = NULL;
	s_pri = NULL;
	rbx_gfx_free(&s_sprgfx);
	tsnd_exit();
}

static void warriorb_frame(void)
{
	for (int line = 0; line < LINES; line++) {
		m68k_execute(tsnd_line_cycles(&s_cpu_frac, CPU_CLOCK));
		tsnd_line();
	}
	// vblank starts after the last visible line (255)
	vblank_capture();
	m68k_bus_hold_irq(4);
	tsnd_frame_end();
}

static int warriorb_state(void *ctx, int load)
{
	uint8_t cpu[1024];
	uint32_t csize = m68k_context_size();
	if (csize > sizeof(cpu))
		return -1;
	if (!load)
		m68k_get_context(cpu);
	uint32_t irq = m68k_bus_irq_pending();
	uint64_t fr = s_cpu_frac;
	int rc = 0;
	rc |= rbx_state_io(ctx, "m68k", cpu, csize);
	rc |= rbx_state_io(ctx, "irq", &irq, 4);
	rc |= rbx_state_io(ctx, "ram", s_ram, sizeof(s_ram));
	rc |= rbx_state_io(ctx, "spr", s_spr, sizeof(s_spr));
	rc |= rbx_state_io(ctx, "pcr", s_pcr, sizeof(s_pcr));
	rc |= rbx_state_io(ctx, "pcra", s_pcr_addr, sizeof(s_pcr_addr));
	for (int i = 0; i < SCREENS; i++) {
		char tag[8] = "scn0", tagc[8] = "scnc0";
		tag[3] = tagc[4] = (char)('0' + i);
		rc |= rbx_state_io(ctx, tag, s_scn[i]->ram, sizeof(s_scn[i]->ram));
		rc |= rbx_state_io(ctx, tagc, s_scn[i]->ctrl, sizeof(s_scn[i]->ctrl));
	}
	rc |= rbx_state_io(ctx, "misc", &s_m, sizeof(s_m));
	rc |= rbx_state_io(ctx, "frac", &fr, sizeof(fr));
	rc |= tsnd_state(ctx, load);
	if (rc)
		return -1;
	if (load) {
		m68k_set_context(cpu);
		m68k_bus_set_irq_pending(irq);
		s_cpu_frac = fr;
		update_pan();
	}
	return 0;
}

static int warriorb_save(void *ctx) { return warriorb_state(ctx, 0); }
static int warriorb_load(void *ctx) { return warriorb_state(ctx, 1); }
static int warriorb_supported(const char *machine) { return !strcmp(machine, "warriorb") || !strcmp(machine, "darius2d"); }

const rbx_driver rbx_drv_warriorb = { "Warrior Blade", warriorb_init, warriorb_exit, warriorb_reset, warriorb_frame,
                                      warriorb_save, warriorb_load, warriorb_supported, warriorb_render };
