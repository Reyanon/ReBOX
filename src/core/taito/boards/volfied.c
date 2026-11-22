/*  Volfied, ported from MAME src/mame/taito/volfied.cpp (BSD-3-Clause, copyright-holders: Bryan McPhail,
    Nicola Salmoria). 68000 + C-Chip (machine/taitocchip.c) + a 2 x 512x256 16-bit pixel layer with a write mask +
    PC090OJ sprites (video/pc090oj.c); sound board: Z80 + YM2203 + PC060HA (machine/taito_sound.c, map 8800).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m68k_bus.h"
#include "../../cpu/m68k/m68k.h"
#include "../machine/taito_sound.h"
#include "../machine/taitocchip.h"
#include "../video/pc090oj.h"

#include <stdlib.h>
#include <string.h>

#define LINES 256
#define SCREEN_W 320
#define VIS_Y0 8
#define VIS_Y1 247
#define VIS_H (VIS_Y1 - VIS_Y0 + 1)
#define ROW_WORDS 321   // pixel layer columns read per line (1 pixel offset)

static uint8_t *s_rom;
static uint32_t s_rom_size;
static uint8_t (*s_ram_hp)[0x4000];
#define s_ram (*s_ram_hp)
static uint16_t *s_vram;            // 0x40000 words: two screens of 256 x 512
static uint16_t (*s_pal_hp)[0x2000];
#define s_pal (*s_pal_hp)
static uint16_t s_video_ctrl, s_video_mask;
static pc090oj_t *s_oj;
static struct {
	pc090oj_regs oj;
	uint16_t pal[0x2000];
	uint16_t pix[VIS_H * ROW_WORDS];
} *s_snap;
static uint8_t (*s_pri_hp)[SCREEN_W * VIS_H];
#define s_pri (*s_pri_hp)
static int s_port_cc[4], s_port_dsw[2];
static uint64_t s_cpu_frac, s_cc_frac;

// ---- C-Chip ports ----
static uint8_t cc_pa(void) { return (uint8_t)rbx_port_read(s_port_cc[0]); }
static uint8_t cc_pb(void) { return (uint8_t)rbx_port_read(s_port_cc[1]); }
static uint8_t cc_pc(void) { return (uint8_t)rbx_port_read(s_port_cc[2]); }
static uint8_t cc_ad(void) { return (uint8_t)rbx_port_read(s_port_cc[3]); }
static void cc_counters(uint8_t v) { (void)v; }   // coin lockout / counters
static uint8_t ym_dswa(void) { return (uint8_t)rbx_port_read(s_port_dsw[0]); }
static uint8_t ym_dswb(void) { return (uint8_t)rbx_port_read(s_port_dsw[1]); }

// ---- main CPU devices ----
static uint16_t main_read16(uint32_t a)
{
	if (a >= 0xf00000 && a <= 0xf007ff) return cchip_mem68_r((a - 0xf00000) >> 1);
	if (a >= 0xf00800 && a <= 0xf00fff) return cchip_asic_r((a - 0xf00800) >> 1);
	if (a == 0xd00000) return 0x60;   // video_ctrl_r (collision detection?)
	if (a == 0xe00002) return tc0140syt_master_comm_r(tsnd_syt());
	return 0;
}

static uint8_t main_read8(uint32_t a)
{
	if (a == 0xe00003) return tc0140syt_master_comm_r(tsnd_syt());
	if (a >= 0xf00000 && a <= 0xf00fff && !(a & 1)) return 0;
	return (uint8_t)(main_read16(a & ~1u) >> ((a & 1) ? 0 : 8));
}

static void main_write16m(uint32_t a, uint16_t v, uint16_t mask)
{
	if (a >= 0x400000 && a <= 0x47ffff) {   // video_ram_w: mem_mask &= video_mask
		uint16_t m = mask & s_video_mask;
		uint16_t *p = &s_vram[(a - 0x400000) >> 1];
		*p = (uint16_t)((*p & ~m) | (v & m));
		return;
	}
	if (a >= 0xf00000 && a <= 0xf00fff) {
		if (mask & 0xff) {
			if (a <= 0xf007ff)
				cchip_mem68_w((a - 0xf00000) >> 1, (uint8_t)v);
			else
				cchip_asic68_w((a - 0xf00800) >> 1, (uint8_t)v);
		}
		return;
	}
	if (a == 0x600000) { s_video_mask = (uint16_t)((s_video_mask & ~mask) | (v & mask)); return; }
	if (a == 0x700000) { s_oj->sprite_ctrl = (uint16_t)((s_oj->sprite_ctrl & ~mask) | (v & mask)); return; }
	if (a == 0xd00000) { s_video_ctrl = (uint16_t)((s_video_ctrl & ~mask) | (v & mask)); return; }
	if (a == 0xe00000 && (mask & 0xff)) { tc0140syt_master_port_w(tsnd_syt(), (uint8_t)v); return; }
	if (a == 0xe00002 && (mask & 0xff)) { tc0140syt_master_comm_w(tsnd_syt(), (uint8_t)v); return; }
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
static void volfied_render(void)
{
	const uint16_t *p = s_snap->pal;
	for (int i = 0; i < 0x2000; i++) {   // xBGR_555
		int r = p[i] & 31, g = (p[i] >> 5) & 31, b = (p[i] >> 10) & 31;
		r = (r << 3) | (r >> 2); g = (g << 3) | (g >> 2); b = (b << 3) | (b >> 2);
		rbx_video.palette[i] = (uint32_t)(r << 16 | g << 8 | b);
	}
	uint16_t *bm = rbx_video.bitmap;
	int pitch = rbx_video.pitch;
	// refresh_pixel_layer
	for (int y = 0; y < VIS_H; y++) {
		const uint16_t *src = s_snap->pix + y * ROW_WORDS;
		uint16_t *dst = bm + y * pitch;
		for (int x = 1; x < SCREEN_W + 1; x++) {   // 1 pixel offset to align with the sprites
			uint16_t d = src[x];
			int color = (d << 2) & 0x700;
			if (d & 0x8000) {
				color |= 0x800 | ((d >> 9) & 0xf);
				if (d & 0x2000)
					color &= ~0xf;   // hack (MAME)
			} else
				color |= d & 0xf;
			dst[x - 1] = (uint16_t)color;
		}
	}
	memset(s_pri, 0, sizeof(s_pri));
	// colpri_cb: palette bank 0x100 | bits 2-5 of the sprite control, sprites over everything
	pc090oj_draw(s_oj, &s_snap->oj, bm, pitch, s_pri, SCREEN_W, VIS_Y0, VIS_H, 1,
	             0x100 | ((s_snap->oj.sprite_ctrl & 0x3c) << 2), 0);
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	pc090oj_snapshot(s_oj, &s_snap->oj);
	memcpy(s_snap->pal, s_pal, sizeof(s_pal));
	const uint16_t *v = s_vram + ((s_video_ctrl & 1) ? 0x20000 : 0) + VIS_Y0 * 512;
	for (int y = 0; y < VIS_H; y++)
		memcpy(s_snap->pix + y * ROW_WORDS, v + y * 512, ROW_WORDS * 2);
	rbx_video_frame_done();
}

// ---- init / frame ----
static void volfied_reset(void)
{
	// (the palette / video RAM too: a game that has not written it yet showed the last game's, as at start it is 0)
	memset(s_pal, 0, sizeof(s_pal));
	memset(s_ram, 0, sizeof(s_ram));
	s_video_ctrl = s_video_mask = 0;
	pc090oj_reset(s_oj);
	s_oj->sprite_ctrl = 0;
	cchip_reset();
	m68k_bus_irq_reset();
	m68k_pulse_reset();
	tsnd_reset();
	s_cpu_frac = s_cc_frac = 0;
}

static void volfied_exit(void);

static int volfied_init(const rbx_game *g)
{
	if (!s_pal_hp && !(s_pal_hp = calloc(1, sizeof(*s_pal_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	if (!s_pri_hp && !(s_pri_hp = calloc(1, sizeof(*s_pri_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	if (!s_ram_hp && !(s_ram_hp = calloc(1, sizeof(*s_ram_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	uint32_t oj_size;
	s_rom = rbx_region("maincpu", &s_rom_size);
	uint8_t *oj_gfx = rbx_region("pc090oj", &oj_size);
	if (!s_rom || !oj_gfx) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	m68k_bus_swap16(s_rom, s_rom_size);
	s_vram = (uint16_t *)calloc(0x40000, sizeof(uint16_t));
	s_oj = (pc090oj_t *)calloc(1, sizeof(pc090oj_t));
	s_snap = calloc(1, sizeof(*s_snap));
	if (!s_vram || !s_oj || !s_snap || pc090oj_init(s_oj, oj_gfx, oj_size, 0x2000)) {
		rbx_fail("%s: out of memory (graphics)", g->set);
		volfied_exit();
		return -1;
	}
	s_oj->direct = 1;
	static const char *const tags[4] = { "F00007", "F00009", "F0000B", "F0000D" };
	for (int i = 0; i < 4; i++)
		s_port_cc[i] = rbx_port_find(tags[i]);
	s_port_dsw[0] = rbx_port_find("DSWA");
	s_port_dsw[1] = rbx_port_find("DSWB");
	cchip_config cc;
	memset(&cc, 0, sizeof(cc));
	cc.clock = 10000000;   // 20 MHz / 2
	cc.in_pa = cc_pa;
	cc.in_pb = cc_pb;
	cc.in_pc = cc_pc;
	cc.in_ad = cc_ad;
	cc.out_pb = cc_counters;
	if (cchip_init(&cc)) {
		rbx_fail("%s: C-Chip ROMs missing", g->set);
		volfied_exit();
		return -1;
	}
	// sound: YM2203 4 MHz, SSG 0.15 x 3, FM 0.60; DIP switches on its ports
	tsnd_config sc;
	memset(&sc, 0, sizeof(sc));
	sc.map = TSND_MAP_8800;
	sc.z80_clock = 4000000;
	sc.frame_rate_mhz = 60000;
	sc.lines = LINES;
	sc.ym.type = RBX_YM2203;
	sc.ym.clock = 4000000;
	sc.ym.gain[0] = sc.ym.gain[1] = sc.ym.gain[2] = 38;
	sc.ym.gain[3] = 154;
	sc.ym.port_a_read = ym_dswa;
	sc.ym.port_b_read = ym_dswb;
	if (tsnd_init(&sc)) {
		rbx_fail("%s: sound init failed", g->set);
		volfied_exit();
		return -1;
	}
	if (rbx_video_setup(SCREEN_W, VIS_H, 0x2000)) {
		volfied_exit();
		return -1;
	}
	m68k_init();
	m68k_set_cpu_type(M68K_CPU_TYPE_68000);
	m68k_bus_reset_map();
	m68k_bus_map(0, 0x0fffff, s_rom, s_rom_size, 1, 0);
	m68k_bus_map(0x100000, 0x103fff, s_ram, sizeof(s_ram), 1, 1);
	m68k_bus_map(0x200000, 0x203fff, s_oj->ram, sizeof(s_oj->ram), 1, 1);
	m68k_bus_map(0x400000, 0x47ffff, s_vram, 0x80000, 1, 0);   // writes through video_ram_w
	m68k_bus_map(0x500000, 0x503fff, s_pal, sizeof(s_pal), 1, 1);
	m68k_bus.read8 = main_read8;
	m68k_bus.read16 = main_read16;
	m68k_bus.write8 = main_write8;
	m68k_bus.write16 = main_write16;
	volfied_reset();
	return 0;
}

static void volfied_exit(void)
{
	if (s_oj)
		pc090oj_exit(s_oj);
	free(s_oj);
	free(s_snap);
	free(s_vram);
	s_oj = NULL;
	s_snap = NULL;
	s_vram = NULL;
	tsnd_exit();
	free(s_pal_hp);
	s_pal_hp = NULL;
	free(s_pri_hp);
	s_pri_hp = NULL;
	free(s_ram_hp);
	s_ram_hp = NULL;
}

static void volfied_frame(void)
{
	for (int line = 0; line < LINES; line++) {
		if (line == VIS_Y1 + 1) {
			vblank_capture();
			m68k_bus_hold_irq(4);
			cchip_ext_interrupt(1);   // pulse (cleared by a zero-time timer in MAME)
			cchip_ext_interrupt(0);
		}
		m68k_execute(tsnd_line_cycles(&s_cpu_frac, 8000000));
		cchip_run_clocks(&s_cc_frac, 60000, LINES);
		tsnd_line();
	}
	tsnd_frame_end();
}

static int volfied_state(void *ctx, int load)
{
	uint8_t cpu[1024];
	uint32_t csize = m68k_context_size();
	if (csize > sizeof(cpu))
		return -1;
	if (!load)
		m68k_get_context(cpu);
	uint32_t irq = m68k_bus_irq_pending();
	uint64_t fr[2] = { s_cpu_frac, s_cc_frac };
	uint16_t vc[2] = { s_video_ctrl, s_video_mask };
	int rc = 0;
	rc |= rbx_state_io(ctx, "m68k", cpu, csize);
	rc |= rbx_state_io(ctx, "irq", &irq, 4);
	rc |= rbx_state_io(ctx, "ram", s_ram, sizeof(s_ram));
	rc |= rbx_state_io(ctx, "vram", s_vram, 0x80000);
	rc |= rbx_state_io(ctx, "pal", s_pal, sizeof(s_pal));
	rc |= rbx_state_io(ctx, "vctrl", vc, sizeof(vc));
	rc |= rbx_state_io(ctx, "pc090oj", s_oj->ram, sizeof(s_oj->ram));
	rc |= rbx_state_io(ctx, "ojctrl", &s_oj->sprite_ctrl, sizeof(s_oj->sprite_ctrl));
	rc |= rbx_state_io(ctx, "frac", fr, sizeof(fr));
	rc |= cchip_state(ctx, load) | tsnd_state(ctx, load);
	if (rc)
		return -1;
	if (load) {
		m68k_set_context(cpu);
		m68k_bus_set_irq_pending(irq);
		s_cpu_frac = fr[0];
		s_cc_frac = fr[1];
		s_video_ctrl = vc[0];
		s_video_mask = vc[1];
	}
	return 0;
}

static int volfied_save(void *ctx) { return volfied_state(ctx, 0); }
static int volfied_load(void *ctx) { return volfied_state(ctx, 1); }
static int volfied_supported(const char *machine) { return !strcmp(machine, "volfied"); }

const rbx_driver rbx_drv_volfied = { "Volfied", volfied_init, volfied_exit, volfied_reset, volfied_frame, volfied_save,
                                     volfied_load, volfied_supported, volfied_render };
