/*  Top Speed / Full Throttle, ported from MAME src/mame/taito/topspeed.cpp (BSD-3-Clause, copyright-holders:
    David Graves; thanks to Richard Bush). Two 68000s sharing 64 KB (CPU A: video, CPU B: inputs and the motor CPU
    stub), two PC080SN (the road layers are their row-scrolled planes, MAME tilemap_draw_special without the
    unused road colour control), zoomed 16x8 sprite chunks built into 128x128 sprites through a RAM sprite map.
    Sound: Z80 + YM2151 + PC060HA (machine/taito_sound.c, map 9000), a Z80 CTC (channel 0 timer, src/devices/
    machine/z80ctc.cpp, BSD-3-Clause, Aaron Giles) clocking the second MSM5205 in slave mode, the first MSM5205 at
    8 kHz, and the two TC0060DCA volumes. Mono mix of MAME's stereo routes.
    Controls (rbx_analog with digital fallback; MAME default cabinet "Deluxe": racing wheel and analog pedals):
      steering  = rbx_analog(1, RBX_IPT_PADDLE) (x delta), else d-pad left / right turn the wheel, released = centre
      gas       = button 1 (pedal fully down) / brake = button 2, nitro = button 3, shifter = button 4 (toggle)
    The "Standard" cabinet (DIP) reads the same buttons as switches and the steering as a relative dial.
    The motor CPU (27c256.ic17) is not emulated (MAME stubs it the same way).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../rbx_core.h"
#include "../../cpu/m68k_bus.h"
#include "../../cpu/m68k/m68k.h"
#include "../machine/taito_sound.h"
#include "../../sound/msm5205.h"
#include "../video/pc080sn.h"
#include "../../video/zoomgfx.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define LINES 256
#define RATE_MHZ 60053          // 60.0532 Hz measured on real hardware
#define SCREEN_W 320
#define VIS_Y0 16
#define VIS_Y1 255
#define VIS_H (VIS_Y1 - VIS_Y0 + 1)
#define CPU_CLOCK 8000000

static uint8_t *s_rom, *s_sub_rom;
static uint32_t s_rom_size, s_sub_rom_size;
static uint8_t s_shared[0x10000];
static uint8_t s_ram[0x10000];          // 0x800000-0x80ffff (raster control + work RAM)
static uint16_t s_pal[0x2000];
static uint16_t s_spr[0x800];
static uint16_t s_sprmap[0x8000];
static pc080sn_t *s_sn[2];
static zoomgfx_t s_gfx;
// CPU B (second Musashi context)
static m68k_bus_t s_bus_b;
static uint8_t s_ctx[2][2048];
static uint32_t s_irq[2];
static int s_cur;
static uint64_t s_frac[2];
// I/O (TC0040IOC)
static uint8_t s_ioc_regs[8], s_ioc_port;
static int s_port[8], s_port_steer;
static int s_steer, s_dial;
static uint32_t s_rand;
// sound: ADPCM and CTC
static msm5205_t s_msm[2];
static const uint8_t *s_adpcm[2];
static uint32_t s_adpcm_size[2];
static struct {
	uint16_t pos[2];
	uint8_t reset[2], nibble[2], vck, vck2, vclk;
	uint8_t vol[4];              // TC0060DCA: MSM 1, MSM 2, YM L, YM R (0-255)
	// CTC channel 0
	uint8_t ctc_ctrl, ctc_tc_next, ctc_running;
	uint16_t ctc_tc;
	uint32_t ctc_count;          // Z80 clocks to the next zero count
	uint32_t msm_frac;
} s_snd;

static struct {
	pc080sn_regs sn[2];
	uint16_t spr[0x800];
	uint16_t sprmap[0x8000];
	uint16_t pal[0x2000];
} *s_snap;
static uint8_t *s_pribuf;

// ---- two 68000s on one Musashi core ----
static void cpu_switch(int n)
{
	if (n == s_cur)
		return;
	m68k_get_context(s_ctx[s_cur]);
	s_irq[s_cur] = m68k_bus_irq_pending();
	m68k_set_context(s_ctx[n]);
	m68k_bus_select(n ? &s_bus_b : NULL);
	m68k_bus_set_irq_pending(s_irq[n]);
	s_cur = n;
}

// ---- sound ----
static void msm_update(int chip)   // MAME msm5205_update
{
	if (s_snd.reset[chip])
		return;
	uint16_t pos = s_snd.pos[chip];
	uint8_t data = pos < s_adpcm_size[chip] ? s_adpcm[chip][pos] : 0;
	msm5205_data_w(&s_msm[chip], (uint8_t)((s_snd.nibble[chip] ? data : data >> 4) & 0xf));
	if (s_snd.nibble[chip])
		s_snd.pos[chip]++;
	s_snd.nibble[chip] ^= 1;
}

static void msm1_vck(void) { msm_update(0); }

static void msm2_vclk(int state)   // msm5205 vclk_w (slave mode): decode on the falling edge
{
	if (s_snd.vclk != state) {
		s_snd.vclk = (uint8_t)state;
		if (!state)
			msm5205_clock(&s_msm[1]);
	}
}

static void ctc_zero_count(void)   // z80ctc_to0 on the ZC/TO pulse (one falling edge)
{
	if (s_snd.vck) {
		msm2_vclk(1);
	} else {
		uint16_t oldpos = s_snd.pos[1];
		msm_update(1);
		if ((oldpos >> 8) == 0x0f && (s_snd.pos[1] >> 8) == 0x10) {
			s_snd.pos[1] = 0;
			msm5205_reset_w(&s_msm[1], 1);
			msm2_vclk(0);
			msm5205_reset_w(&s_msm[1], 0);
		} else {
			msm2_vclk(0);
		}
	}
	s_snd.vck ^= 1;
}

static uint32_t ctc_period(void)
{
	return (uint32_t)((s_snd.ctc_ctrl & 0x20) ? 256 : 16) * (s_snd.ctc_tc ? s_snd.ctc_tc : 256);
}

static void ctc_write(uint8_t port, uint8_t v)   // channel 0 only (MAME maps port 0)
{
	if (port != 0)
		return;
	if (s_snd.ctc_tc_next) {   // time constant
		s_snd.ctc_tc_next = 0;
		s_snd.ctc_tc = v;
		if (!(s_snd.ctc_ctrl & 0x40)) {   // timer mode (trigger start not modelled: starts now)
			s_snd.ctc_running = 1;
			s_snd.ctc_count = ctc_period();
		}
		return;
	}
	if (!(v & 1))   // interrupt vector
		return;
	s_snd.ctc_ctrl = v;
	if (v & 0x04)
		s_snd.ctc_tc_next = 1;
	if (v & 0x02)   // software reset
		s_snd.ctc_running = 0;
}

static uint8_t ctc_read(uint8_t port) { (void)port; return 0xff; }

static void snd_ext_write(uint16_t a, uint8_t v)
{
	if (a >= 0xb000 && a < 0xd000) {   // msm5205_command_w
		uint32_t offset = a - 0xb000;
		int chip = (offset >> 12) & 1;
		switch ((offset >> 8) & 0x2e) {
		case 0x00: s_snd.pos[chip] = (uint16_t)(v << 8); break;
		case 0x04: s_snd.reset[chip] = 0; msm5205_reset_w(&s_msm[chip], 0); break;
		case 0x08: s_snd.reset[chip] = 1; s_snd.nibble[chip] = 0; msm5205_reset_w(&s_msm[chip], 1); break;
		}
		return;
	}
	if (a >= 0xd000 && a < 0xe000) {   // volume_w
		switch (a - 0xd000) {
		case 0x000: s_snd.vol[0] = v; break;
		case 0x200: s_snd.vol[1] = v; break;
		case 0x400: s_snd.vol[2] = v; break;
		case 0x600: s_snd.vol[3] = v; break;
		}
		rbx_ym_set_volume(0, (s_snd.vol[2] + s_snd.vol[3]) * 256 / 510);
	}
}

static void snd_ext_mix(int32_t *buf, int n, uint32_t rate)
{
	// Z80 / CTC clock 4 MHz, MSM 1 384 kHz; mono = (L + R) / 2 of MAME's routes (the MSMs go to both speakers)
	int g1 = s_snd.vol[0], g2 = s_snd.vol[1];
	uint32_t ctc_step = 4000000 / rate;
	for (int i = 0; i < n; i++) {
		s_snd.msm_frac += 384000;
		uint32_t k = s_snd.msm_frac / rate;
		s_snd.msm_frac -= k * rate;
		msm5205_advance(&s_msm[0], k);
		if (s_snd.ctc_running && !(s_snd.ctc_ctrl & 0x40)) {
			uint32_t c = ctc_step;
			while (c >= s_snd.ctc_count) {
				c -= s_snd.ctc_count;
				s_snd.ctc_count = ctc_period();
				ctc_zero_count();
			}
			s_snd.ctc_count -= c;
		}
		buf[i] += (msm5205_output(&s_msm[0]) * g1 + msm5205_output(&s_msm[1]) * g2) / 255;
	}
}

// ---- inputs (CPU B, TC0040IOC with MAME input_bypass_r) ----
static const uint8_t s_pedal_gray[8] = { 0, 1, 3, 2, 6, 7, 5, 4 };

static int cabinet(void) { return (int)(rbx_port_read(s_port[0]) & 3); }

static void update_controls(void)
{
	uint32_t in = rbx_player_input(1);
	int a = rbx_analog(1, RBX_IPT_PADDLE);
	if (cabinet() == 2) {   // continuous wheel: relative dial (MAME keydelta 2)
		if (!a)
			a = ((in & RBX_IN_RIGHT) ? 2 : 0) - ((in & RBX_IN_LEFT) ? 2 : 0);
		s_dial = (s_dial + a) & 0xffff;
		return;
	}
	if (a) {
		s_steer += a;
	} else if (in & (RBX_IN_LEFT | RBX_IN_RIGHT)) {
		s_steer += (in & RBX_IN_RIGHT) ? 10 : -10;   // MAME keydelta 10
	} else {   // released: back to the centre
		if (s_steer > 0x80) s_steer = s_steer - 10 < 0x80 ? 0x80 : s_steer - 10;
		if (s_steer < 0x80) s_steer = s_steer + 10 > 0x80 ? 0x80 : s_steer + 10;
	}
	s_steer = s_steer < 0 ? 0 : s_steer > 0xff ? 0xff : s_steer;
}

static uint8_t ioc_portreg_r(void)
{
	uint32_t in = rbx_player_input(1);
	switch (s_ioc_port) {
	case 0: return (uint8_t)rbx_port_read(s_port[0]);
	case 1: return (uint8_t)rbx_port_read(s_port[1]);
	case 2: case 3: {
		uint8_t v = (uint8_t)rbx_port_read(s_port[s_ioc_port]);
		if (cabinet() != 2) {   // analog pedals (custom bits 5-7): button 1 = gas, button 2 = brake, fully down
			int pedal = (in & (s_ioc_port == 2 ? RBX_IN_B2 : RBX_IN_B1)) ? 7 : 0;
			v = (uint8_t)((v & 0x1f) | s_pedal_gray[pedal] << 5);
		}
		return v;
	}
	case 4: return s_ioc_regs[4];
	case 7: return (uint8_t)rbx_port_read(s_port[7]);
	}
	return 0xff;
}

static uint8_t input_bypass_r(void)
{
	uint16_t steer = (uint16_t)(0xff80 + (cabinet() == 2 ? s_dial : s_steer));
	if (s_ioc_port == 0x0c) return (uint8_t)steer;
	if (s_ioc_port == 0x0d) return (uint8_t)(steer >> 8);
	return ioc_portreg_r();
}

// ---- CPU A ----
static uint16_t a_read16(uint32_t a)
{
	if (a == 0x7e0002) return tc0140syt_master_comm_r(tsnd_syt());
	if (a >= 0xa00000 && a <= 0xa0ffff) return pc080sn_word_r(s_sn[0], (a - 0xa00000) >> 1);
	if (a >= 0xb00000 && a <= 0xb0ffff) return pc080sn_word_r(s_sn[1], (a - 0xb00000) >> 1);
	return 0;
}

static uint8_t a_read8(uint32_t a)
{
	uint16_t v = a_read16(a & ~1u);
	return (uint8_t)((a & 1) ? v : v >> 8);
}

static void a_write16m(uint32_t a, uint16_t v, uint16_t mask)
{
	if (a == 0x7e0000 && (mask & 0xff)) { tc0140syt_master_port_w(tsnd_syt(), (uint8_t)v); return; }
	if (a == 0x7e0002 && (mask & 0xff)) { tc0140syt_master_comm_w(tsnd_syt(), (uint8_t)v); return; }
	for (int i = 0; i < 2; i++) {
		uint32_t base = i ? 0xb00000 : 0xa00000;
		if (a >= base + 0x20000 && a <= base + 0x20003) { pc080sn_yscroll_w(s_sn[i], (a >> 1) & 1, v, mask); return; }
		if (a >= base + 0x40000 && a <= base + 0x40003) { pc080sn_xscroll_w(s_sn[i], (a >> 1) & 1, v, mask); return; }
		if (a >= base + 0x50000 && a <= base + 0x50003) { pc080sn_ctrl_w(s_sn[i], (a >> 1) & 1, v, mask); return; }
	}
	// 0x600002 cpua_ctrl_w and 0x880000 lamps: no effect
}

static void a_write16(uint32_t a, uint16_t v) { a_write16m(a, v, 0xffff); }
static void a_write8(uint32_t a, uint8_t v)
{
	if (a & 1)
		a_write16m(a & ~1u, v, 0x00ff);
	else
		a_write16m(a, (uint16_t)(v << 8), 0xff00);
}

// ---- CPU B ----
static uint16_t b_read16(uint32_t a)
{
	if (a == 0x880000) return input_bypass_r();
	if (a == 0x880002) return s_ioc_port;
	if (a >= 0x900000 && a <= 0x9003ff) {   // motor_r
		uint32_t o = (a - 0x900000) >> 1;
		if (o == 0) {
			s_rand = s_rand * 1103515245u + 12345u;
			return (uint16_t)((s_rand >> 16) & 0xff);
		}
		return o == 0x101 ? 0x55 : 0;
	}
	return 0;
}

static uint8_t b_read8(uint32_t a)
{
	uint16_t v = b_read16(a & ~1u);
	return (uint8_t)((a & 1) ? v : v >> 8);
}

static void b_write16m(uint32_t a, uint16_t v, uint16_t mask)
{
	if (a == 0x880000 && (mask & 0xff)) {   // portreg_w
		if (s_ioc_port < 8)
			s_ioc_regs[s_ioc_port] = (uint8_t)v;
		return;
	}
	if (a == 0x880002 && (mask & 0xff)) { s_ioc_port = (uint8_t)v; return; }
}

static void b_write16(uint32_t a, uint16_t v) { b_write16m(a, v, 0xffff); }
static void b_write8(uint32_t a, uint8_t v)
{
	if (a & 1)
		b_write16m(a & ~1u, v, 0x00ff);
	else
		b_write16m(a, (uint16_t)(v << 8), 0xff00);
}

// ---- video ----
static void draw_sprites(uint16_t *bm, int pitch)
{
	static const uint32_t primasks[2] = { 0xff00, 0xfffc };
	const uint16_t *sp = s_snap->spr, *map = s_snap->sprmap;
	zoomgfx_target t = { bm, s_pribuf, pitch, 0, VIS_Y0, SCREEN_W - 1, VIS_Y1, VIS_Y0, SCREEN_W };
	for (int offs = 0; offs < 0x800; offs += 4) {
		uint16_t data = sp[offs + 2];
		int tilenum = sp[offs + 3] & 0xff;
		uint32_t color = (sp[offs + 3] & 0xff00) >> 8;
		int flipx = (data & 0x4000) >> 14;
		int flipy = (sp[offs + 1] & 0x8000) >> 15;
		int x = data & 0x1ff;
		int y = sp[offs] & 0x1ff;
		int zoomx = sp[offs + 1] & 0x7f;
		int zoomy = (sp[offs] & 0xfe00) >> 9;
		int priority = (data & 0x8000) >> 15;
		if (y == 0x180)
			break;
		int map_offset = tilenum << 7;
		zoomx += 1;
		zoomy += 1;
		y += 3 + (128 - zoomy);
		if (x > 0x140) x -= 0x200;
		if (y > 0x140) y -= 0x200;
		for (int chunk = 0; chunk < 128; chunk++) {
			int k = chunk % 8, j = chunk / 8;
			int px = flipx ? 7 - k : k;
			int py = flipy ? 15 - j : j;
			uint16_t code = map[(map_offset + (py << 3) + px) & 0x7fff];
			if (code & 0x8000)
				continue;
			int curx = x + ((k * zoomx) / 8);
			int cury = y + ((j * zoomy) / 16);
			int zx = x + (((k + 1) * zoomx) / 8) - curx;
			int zy = y + (((j + 1) * zoomy) / 16) - cury;
			zoomgfx_draw(&s_gfx, &t, code, color * 16, flipx, flipy, curx, cury, (uint32_t)zx << 12, (uint32_t)zy << 13,
			             1, primasks[priority]);
		}
	}
}

static void topspeed_render(void)
{
	for (int i = 0; i < 0x2000; i++) {   // xBGR_555
		uint16_t d = s_snap->pal[i];
		int r = d & 31, g = (d >> 5) & 31, b = (d >> 10) & 31;
		rbx_video.palette[i] = (uint32_t)(((r << 3) | (r >> 2)) << 16 | ((g << 3) | (g >> 2)) << 8 | ((b << 3) | (b >> 2)));
	}
	uint16_t *bm = rbx_video.bitmap;
	int pitch = rbx_video.pitch;
	memset(s_pribuf, 0, SCREEN_W * VIS_H);
	// layer order is hardwired: chip 2 BG1 (opaque), chip 2 BG0, chip 1 BG1, chip 1 BG0, sprites
	pc080sn_draw(s_sn[1], &s_snap->sn[1], 1, bm, pitch, s_pribuf, SCREEN_W, VIS_Y0, VIS_H, 1, 1);
	pc080sn_draw(s_sn[1], &s_snap->sn[1], 0, bm, pitch, s_pribuf, SCREEN_W, VIS_Y0, VIS_H, 0, 2);
	pc080sn_draw(s_sn[0], &s_snap->sn[0], 1, bm, pitch, s_pribuf, SCREEN_W, VIS_Y0, VIS_H, 0, 4);
	pc080sn_draw(s_sn[0], &s_snap->sn[0], 0, bm, pitch, s_pribuf, SCREEN_W, VIS_Y0, VIS_H, 0, 8);
	draw_sprites(bm, pitch);
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	s_snap->sn[0] = s_sn[0]->r;
	s_snap->sn[1] = s_sn[1]->r;
	memcpy(s_snap->spr, s_spr, sizeof(s_spr));
	memcpy(s_snap->sprmap, s_sprmap, sizeof(s_sprmap));
	memcpy(s_snap->pal, s_pal, sizeof(s_pal));
	rbx_video_frame_done();
}

// ---- init / frame ----
static void topspeed_reset(void)
{
	// (the palette / video RAM too: a game that has not written it yet showed the last game's, as at start it is 0)
	memset(s_pal, 0, sizeof(s_pal));
	memset(s_shared, 0, sizeof(s_shared));
	memset(s_ram, 0, sizeof(s_ram));
	memset(s_spr, 0, sizeof(s_spr));
	memset(s_sprmap, 0, sizeof(s_sprmap));
	memset(s_ioc_regs, 0, sizeof(s_ioc_regs));
	s_ioc_port = 0;
	pc080sn_reset(s_sn[0]);
	pc080sn_reset(s_sn[1]);
	// CPU B, then CPU A (left running)
	s_cur = 0;
	m68k_bus_select(&s_bus_b);
	m68k_bus_irq_reset();
	m68k_pulse_reset();
	m68k_get_context(s_ctx[1]);
	s_irq[1] = 0;
	m68k_bus_select(NULL);
	m68k_bus_irq_reset();
	m68k_pulse_reset();
	s_irq[0] = 0;
	tsnd_reset();
	memset(&s_snd, 0, sizeof(s_snd));
	for (int i = 0; i < 4; i++)
		s_snd.vol[i] = 255;
	for (int i = 0; i < 2; i++) {
		msm5205_reset(&s_msm[i]);
		msm5205_reset_w(&s_msm[i], 1);
	}
	rbx_ym_set_volume(0, 256);
	s_frac[0] = s_frac[1] = 0;
	s_steer = 0x80;
	s_dial = 0;
	s_rand = 1;
}

static void topspeed_exit(void);

static int topspeed_init(const rbx_game *g)
{
	uint32_t sn_size, spr_size;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_sub_rom = rbx_region("subcpu", &s_sub_rom_size);
	uint8_t *sn_gfx = rbx_region("pc080sn", &sn_size);
	uint8_t *spr = rbx_region("sprites", &spr_size);
	s_adpcm[0] = rbx_region("adpcm_0", &s_adpcm_size[0]);
	s_adpcm[1] = rbx_region("adpcm_1", &s_adpcm_size[1]);
	if (!s_rom || !s_sub_rom || !sn_gfx || !spr || !s_adpcm[0] || !s_adpcm[1]) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	m68k_bus_swap16(s_rom, s_rom_size);
	m68k_bus_swap16(s_sub_rom, s_sub_rom_size);
	s_sn[0] = (pc080sn_t *)calloc(1, sizeof(pc080sn_t));
	s_sn[1] = (pc080sn_t *)calloc(1, sizeof(pc080sn_t));
	s_snap = calloc(1, sizeof(*s_snap));
	s_pribuf = (uint8_t *)calloc(SCREEN_W, VIS_H);
	if (!s_sn[0] || !s_sn[1] || !s_snap || !s_pribuf || pc080sn_init(s_sn[0], sn_gfx, sn_size, PC080SN_GFX_PACKED_MSB) ||
	    zoomgfx_decode(&s_gfx, spr, spr_size, ZGFX_16x8_PLANAR64)) {
		rbx_fail("%s: out of memory (graphics)", g->set);
		topspeed_exit();
		return -1;
	}
	s_sn[0]->x_offset = 0;
	s_sn[0]->y_offset = 8;
	s_sn[0]->colors = 512;
	*s_sn[1] = *s_sn[0];   // both chips use the same tile ROM (decoded once, owned by chip 1)
	static const char *const tags[8] = { "DSWA", "DSWB", "IN0", "IN1", 0, 0, 0, "IN2" };
	for (int i = 0; i < 8; i++)
		s_port[i] = tags[i] ? rbx_port_find(tags[i]) : -1;
	s_port_steer = rbx_port_find("STEER");
	// sound: YM2151 (bank from its CT pins; bank 0 is the erased area at 0x10000), two MSM5205, CTC on port 0
	msm5205_init(&s_msm[0], 384000, MSM5205_S48_4B, msm1_vck);
	msm5205_init(&s_msm[1], 384000, MSM5205_SEX_4B, NULL);
	tsnd_config sc;
	memset(&sc, 0, sizeof(sc));
	sc.map = TSND_MAP_9000;
	sc.z80_clock = 4000000;
	sc.frame_rate_mhz = RATE_MHZ;
	sc.lines = LINES;
	sc.ym.type = RBX_YM2151;
	sc.ym.clock = 4000000;
	sc.ym.gain[0] = sc.ym.gain[1] = 128;   // (L + R) / 2
	sc.ym_port_bank = 1;
	sc.bank0_offset = 0x10000;
	sc.ext_write = snd_ext_write;
	sc.ext_mix = snd_ext_mix;
	sc.port_in = ctc_read;
	sc.port_out = ctc_write;
	if (tsnd_init(&sc)) {
		rbx_fail("%s: sound init failed", g->set);
		topspeed_exit();
		return -1;
	}
	if (rbx_video_setup(SCREEN_W, VIS_H, 8192)) {
		topspeed_exit();
		return -1;
	}
	if (m68k_context_size() > sizeof(s_ctx[0])) {
		topspeed_exit();
		return -1;
	}
	m68k_init();
	m68k_set_cpu_type(M68K_CPU_TYPE_68000);
	// CPU A
	m68k_bus_select(NULL);
	m68k_bus_reset_map();
	m68k_bus_map(0, 0x0fffff, s_rom, s_rom_size, 1, 0);
	m68k_bus_map(0x400000, 0x40ffff, s_shared, sizeof(s_shared), 1, 1);
	m68k_bus_map(0x500000, 0x503fff, s_pal, sizeof(s_pal), 1, 1);
	m68k_bus_map(0x800000, 0x80ffff, s_ram, sizeof(s_ram), 1, 1);
	m68k_bus_map(0xa00000, 0xa0ffff, s_sn[0]->r.ram, sizeof(s_sn[0]->r.ram), 1, 1);
	m68k_bus_map(0xb00000, 0xb0ffff, s_sn[1]->r.ram, sizeof(s_sn[1]->r.ram), 1, 1);
	m68k_bus_map(0xd00000, 0xd00fff, s_spr, sizeof(s_spr), 1, 1);
	m68k_bus_map(0xe00000, 0xe0ffff, s_sprmap, sizeof(s_sprmap), 1, 1);
	m68k_bus.read8 = a_read8;
	m68k_bus.read16 = a_read16;
	m68k_bus.write8 = a_write8;
	m68k_bus.write16 = a_write16;
	// CPU B
	m68k_bus_select(&s_bus_b);
	m68k_bus_reset_map();
	m68k_bus_map(0, 0x01ffff, s_sub_rom, s_sub_rom_size, 1, 0);
	m68k_bus_map(0x400000, 0x40ffff, s_shared, sizeof(s_shared), 1, 1);
	s_bus_b.read8 = b_read8;
	s_bus_b.read16 = b_read16;
	s_bus_b.write8 = b_write8;
	s_bus_b.write16 = b_write16;
	m68k_bus_select(NULL);
	topspeed_reset();
	return 0;
}

static void topspeed_exit(void)
{
	m68k_bus_select(NULL);
	if (s_sn[0])
		pc080sn_exit(s_sn[0]);
	free(s_sn[0]);
	free(s_sn[1]);
	s_sn[0] = s_sn[1] = NULL;
	free(s_snap);
	s_snap = NULL;
	free(s_pribuf);
	s_pribuf = NULL;
	zoomgfx_free(&s_gfx);
	tsnd_exit();
}

static void topspeed_frame(void)
{
	rbx_audio.frame_mhz = RATE_MHZ;   // (the board's frame rate: a frame's sound is played in a 60 Hz frame, as its picture)
	update_controls();
	for (int line = 0; line < LINES; line++) {
		if (line == 0) {   // vblank (vblank time 0: at the end of the visible frame)
			vblank_capture();
			cpu_switch(1);
			m68k_bus_hold_irq(5);
			cpu_switch(0);
			m68k_bus_hold_irq(6);
		}
		cpu_switch(0);
		m68k_execute(rbx_line_cycles(&s_frac[0], CPU_CLOCK, RATE_MHZ, LINES));
		cpu_switch(1);
		m68k_execute(rbx_line_cycles(&s_frac[1], CPU_CLOCK, RATE_MHZ, LINES));
		cpu_switch(0);
		tsnd_line();
	}
	tsnd_frame_end();
}

static int topspeed_state(void *ctx, int load)
{
	cpu_switch(0);
	if (!load) {
		m68k_get_context(s_ctx[0]);
		s_irq[0] = m68k_bus_irq_pending();
	}
	uint32_t csize = m68k_context_size();
	int misc[8] = { s_ioc_port, s_steer, s_dial, (int)s_rand, 0, 0, 0, 0 };
	int rc = 0;
	rc |= rbx_state_io(ctx, "m68k", s_ctx[0], csize);
	rc |= rbx_state_io(ctx, "m68kb", s_ctx[1], csize);
	rc |= rbx_state_io(ctx, "irq", s_irq, sizeof(s_irq));
	rc |= rbx_state_io(ctx, "shared", s_shared, sizeof(s_shared));
	rc |= rbx_state_io(ctx, "ram", s_ram, sizeof(s_ram));
	rc |= rbx_state_io(ctx, "pal", s_pal, sizeof(s_pal));
	rc |= rbx_state_io(ctx, "spr", s_spr, sizeof(s_spr));
	rc |= rbx_state_io(ctx, "sprmap", s_sprmap, sizeof(s_sprmap));
	rc |= rbx_state_io(ctx, "sn0", &s_sn[0]->r, sizeof(s_sn[0]->r));
	rc |= rbx_state_io(ctx, "sn1", &s_sn[1]->r, sizeof(s_sn[1]->r));
	rc |= rbx_state_io(ctx, "ioc", s_ioc_regs, sizeof(s_ioc_regs));
	rc |= rbx_state_io(ctx, "misc", misc, sizeof(misc));
	rc |= rbx_state_io(ctx, "frac", s_frac, sizeof(s_frac));
	rc |= rbx_state_io(ctx, "snd", &s_snd, sizeof(s_snd));
	rc |= rbx_state_io(ctx, "msm0", &s_msm[0], offsetof(msm5205_t, vck));
	rc |= rbx_state_io(ctx, "msm1", &s_msm[1], offsetof(msm5205_t, vck));
	rc |= tsnd_state(ctx, load);
	if (rc)
		return -1;
	if (load) {
		m68k_set_context(s_ctx[0]);
		m68k_bus_set_irq_pending(s_irq[0]);
		s_ioc_port = (uint8_t)misc[0];
		s_steer = misc[1];
		s_dial = misc[2];
		s_rand = (uint32_t)misc[3];
		rbx_ym_set_volume(0, (s_snd.vol[2] + s_snd.vol[3]) * 256 / 510);
	}
	return 0;
}

static int topspeed_save(void *ctx) { return topspeed_state(ctx, 0); }
static int topspeed_load(void *ctx) { return topspeed_state(ctx, 1); }
static int topspeed_supported(const char *machine) { return !strcmp(machine, "topspeed"); }

const rbx_driver rbx_drv_topspeed = { "Top Speed", topspeed_init, topspeed_exit, topspeed_reset, topspeed_frame,
                                      topspeed_save, topspeed_load, topspeed_supported, topspeed_render };
