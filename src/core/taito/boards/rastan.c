/*  Rastan / Rastan Saga, ported from MAME src/mame/taito/rastan.cpp (GPL-2.0+, copyright-holders: Jarek Burczynski).
    68000 + PC080SN tilemaps (video/pc080sn.c) + PC090OJ sprites (video/pc090oj.c); sound board: Z80 + YM2151 +
    PC060HA (machine/taito_sound.c, map 9000) with an MSM5205 fed through an LS157 nibble selector.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m68k_bus.h"
#include "../../cpu/m68k/m68k.h"
#include "../machine/taito_sound.h"
#include "../../sound/msm5205u.h"
#include "../video/pc080sn.h"
#include "../video/pc090oj.h"

#include <stdlib.h>
#include <string.h>

#define LINES 256
#define SCREEN_W 320
#define VIS_Y0 8
#define VIS_Y1 247
#define VIS_H (VIS_Y1 - VIS_Y0 + 1)

static uint8_t *s_rom;
static uint32_t s_rom_size;
static uint8_t s_ram[0x4000];
static uint8_t s_pal[0x1000];
static pc080sn_t *s_sn;
static pc090oj_t *s_oj;
static struct {   // drawing snapshot
	pc080sn_regs sn;
	pc090oj_regs oj;
	uint8_t pal[0x1000];
} *s_snap;
static uint8_t s_pri[SCREEN_W * VIS_H];
static int s_port[6];   // P1, P2, SPECIAL, SYSTEM, DSWA, DSWB
static uint64_t s_cpu_frac;
// ADPCM: MSM5205 + LS157
static msm5205u_t s_msm;
static const uint8_t *s_adpcm;
static uint32_t s_adpcm_size;
static struct { uint16_t pos; uint8_t ff, a, b, pad[3]; } s_ad;

// ---- sound ----
static void msm_vck(msm5205u_t *m, void *user)
{
	(void)user;
	s_ad.ff = !s_ad.ff;   // LS157 select
	if (s_ad.ff) {
		uint8_t d = s_ad.pos < s_adpcm_size ? s_adpcm[s_ad.pos] : 0;
		s_ad.a = d & 0x0f;
		s_ad.b = d >> 4;
		s_ad.pos = (uint16_t)(s_ad.pos + 1);
	}
	msm5205u_data_w(m, s_ad.ff ? s_ad.b : s_ad.a);
}

static void snd_ext_write(uint16_t a, uint8_t v)
{
	switch (a & 0xf000) {
	case 0xb000: s_ad.pos = (uint16_t)((s_ad.pos & 0x00ff) | (v << 8)); break;   // msm5205u_address_w
	case 0xc000: msm5205u_reset_w(&s_msm, 0); s_ad.ff = 0; break;                     // msm5205u_start_w
	case 0xd000: msm5205u_reset_w(&s_msm, 1); s_ad.pos &= 0xff00; break;            // msm5205u_stop_w
	}
}

static void snd_ext_mix(int32_t *buf, int n, uint32_t rate) { msm5205u_render(&s_msm, buf, n, rate, 154); }   // 0.60

// ---- main CPU devices ----
static uint16_t main_read16(uint32_t a)
{
	if (a >= 0x390000 && a <= 0x39000b) return (uint16_t)(rbx_port_read(s_port[(a - 0x390000) >> 1]) & 0xff);
	if (a == 0x3e0002) return tc0140syt_master_comm_r(tsnd_syt());
	return 0;
}

static uint8_t main_read8(uint32_t a)
{
	if (a == 0x3e0003) return tc0140syt_master_comm_r(tsnd_syt());
	return (uint8_t)(main_read16(a & ~1u) >> ((a & 1) ? 0 : 8));
}

static void main_write16m(uint32_t a, uint16_t v, uint16_t mask)
{
	if (a >= 0xc20000 && a <= 0xc20003) { pc080sn_yscroll_w(s_sn, (a >> 1) & 1, v, mask); return; }
	if (a >= 0xc40000 && a <= 0xc40003) { pc080sn_xscroll_w(s_sn, (a >> 1) & 1, v, mask); return; }
	if (a >= 0xc50000 && a <= 0xc50003) { pc080sn_ctrl_w(s_sn, (a >> 1) & 1, v, mask); return; }
	if (a == 0x380000) { s_oj->sprite_ctrl = (uint16_t)((s_oj->sprite_ctrl & ~mask) | (v & mask)); return; }   // spritectrl_w
	if (a == 0x3e0000 && (mask & 0xff)) { tc0140syt_master_port_w(tsnd_syt(), (uint8_t)v); return; }
	if (a == 0x3e0002 && (mask & 0xff)) { tc0140syt_master_comm_w(tsnd_syt(), (uint8_t)v); return; }
	// 0x3c0000 watchdog, 0x350008 unknown
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
static void rastan_render(void)
{
	const uint16_t *p = (const uint16_t *)s_snap->pal;
	for (int i = 0; i < 2048; i++) {   // xBGR_555
		int r = p[i] & 31, g = (p[i] >> 5) & 31, b = (p[i] >> 10) & 31;
		r = (r << 3) | (r >> 2); g = (g << 3) | (g >> 2); b = (b << 3) | (b >> 2);
		rbx_video.palette[i] = (uint32_t)(r << 16 | g << 8 | b);
	}
	uint16_t *bm = rbx_video.bitmap;
	int pitch = rbx_video.pitch;
	pc080sn_draw(s_sn, &s_snap->sn, 0, bm, pitch, NULL, SCREEN_W, VIS_Y0, VIS_H, 1, 1);
	pc080sn_draw(s_sn, &s_snap->sn, 1, bm, pitch, NULL, SCREEN_W, VIS_Y0, VIS_H, 0, 2);
	// colpri_cb: bits 5-7 of the sprite control are the palette bank, sprites over everything
	pc090oj_draw(s_oj, &s_snap->oj, bm, pitch, s_pri, SCREEN_W, VIS_Y0, VIS_H, 1, (s_snap->oj.sprite_ctrl & 0xe0) >> 1, 0);
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	s_snap->sn = s_sn->r;
	pc090oj_snapshot(s_oj, &s_snap->oj);
	memcpy(s_snap->pal, s_pal, sizeof(s_pal));
	rbx_video_frame_done();
}

// ---- init / frame ----
static void rastan_reset(void)
{
	// (the palette / video RAM too: a game that has not written it yet showed the last game's, as at start it is 0)
	memset(s_pal, 0, sizeof(s_pal));
	memset(s_ram, 0, sizeof(s_ram));
	pc080sn_reset(s_sn);
	pc090oj_reset(s_oj);
	s_oj->sprite_ctrl = 0;
	memset(&s_ad, 0, sizeof(s_ad));
	msm5205u_reset(&s_msm);
	m68k_bus_irq_reset();
	m68k_pulse_reset();
	tsnd_reset();
	s_cpu_frac = 0;
}

static void rastan_exit(void);

static int rastan_init(const rbx_game *g)
{
	uint32_t sn_size, oj_size;
	s_rom = rbx_region("maincpu", &s_rom_size);
	uint8_t *sn_gfx = rbx_region("pc080sn", &sn_size);
	uint8_t *oj_gfx = rbx_region("pc090oj", &oj_size);
	s_adpcm = rbx_region("adpcm", &s_adpcm_size);
	if (!s_rom || !sn_gfx || !oj_gfx) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	m68k_bus_swap16(s_rom, s_rom_size);
	s_sn = (pc080sn_t *)calloc(1, sizeof(pc080sn_t));
	s_oj = (pc090oj_t *)calloc(1, sizeof(pc090oj_t));
	s_snap = calloc(1, sizeof(*s_snap));
	if (!s_sn || !s_oj || !s_snap || pc080sn_init(s_sn, sn_gfx, sn_size, PC080SN_GFX_PACKED_MSB) ||
	    pc090oj_init(s_oj, oj_gfx, oj_size, 2048)) {
		rbx_fail("%s: out of memory (graphics)", g->set);
		rastan_exit();
		return -1;
	}
	s_oj->direct = 1;
	static const char *const tags[6] = { "P1", "P2", "SPECIAL", "SYSTEM", "DSWA", "DSWB" };
	for (int i = 0; i < 6; i++)
		s_port[i] = rbx_port_find(tags[i]);
	// sound: YM2151 routes 0.50 + 0.50 to mono, MSM5205 0.60 (8 kHz, 4-bit)
	tsnd_config sc;
	memset(&sc, 0, sizeof(sc));
	sc.map = TSND_MAP_9000;
	sc.z80_clock = 4000000;
	sc.frame_rate_mhz = 60000;
	sc.lines = LINES;
	sc.ym.type = RBX_YM2151;
	sc.ym.clock = 4000000;
	sc.ym.gain[0] = sc.ym.gain[1] = 128;
	sc.ym_port_bank = 1;
	sc.ext_write = snd_ext_write;
	sc.ext_mix = snd_ext_mix;
	msm5205u_init(&s_msm, 384000, MSM5205_S48_4B, msm_vck, NULL);
	if (tsnd_init(&sc)) {
		rbx_fail("%s: sound init failed", g->set);
		rastan_exit();
		return -1;
	}
	if (rbx_video_setup(SCREEN_W, VIS_H, 2048)) {
		rastan_exit();
		return -1;
	}
	m68k_init();
	m68k_set_cpu_type(M68K_CPU_TYPE_68000);
	m68k_bus_reset_map();
	m68k_bus_map(0, 0x05ffff, s_rom, s_rom_size, 1, 0);
	m68k_bus_map(0x10c000, 0x10ffff, s_ram, sizeof(s_ram), 1, 1);
	m68k_bus_map(0x200000, 0x200fff, s_pal, sizeof(s_pal), 1, 1);
	m68k_bus_map(0xc00000, 0xc0ffff, s_sn->r.ram, sizeof(s_sn->r.ram), 1, 1);
	m68k_bus_map(0xd00000, 0xd03fff, s_oj->ram, sizeof(s_oj->ram), 1, 1);
	m68k_bus.read8 = main_read8;
	m68k_bus.read16 = main_read16;
	m68k_bus.write8 = main_write8;
	m68k_bus.write16 = main_write16;
	rastan_reset();
	return 0;
}

static void rastan_exit(void)
{
	if (s_sn)
		pc080sn_exit(s_sn);
	if (s_oj)
		pc090oj_exit(s_oj);
	free(s_sn);
	free(s_oj);
	free(s_snap);
	s_sn = NULL;
	s_oj = NULL;
	s_snap = NULL;
	tsnd_exit();
}

static void rastan_frame(void)
{
	for (int line = 0; line < LINES; line++) {
		if (line == VIS_Y1 + 1) {
			vblank_capture();
			m68k_bus_hold_irq(5);
		}
		m68k_execute(tsnd_line_cycles(&s_cpu_frac, 8000000));
		tsnd_line();
	}
	tsnd_frame_end();
}

static int rastan_state(void *ctx, int load)
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
	rc |= rbx_state_io(ctx, "pal", s_pal, sizeof(s_pal));
	rc |= rbx_state_io(ctx, "pc080sn", &s_sn->r, sizeof(s_sn->r));
	rc |= rbx_state_io(ctx, "pc090oj", s_oj->ram, sizeof(s_oj->ram));
	rc |= rbx_state_io(ctx, "ojctrl", &s_oj->sprite_ctrl, sizeof(s_oj->sprite_ctrl));
	rc |= rbx_state_io(ctx, "msm", &s_msm, MSM5205_STATE_SIZE);
	rc |= rbx_state_io(ctx, "adpcm", &s_ad, sizeof(s_ad));
	rc |= rbx_state_io(ctx, "frac", &fr, sizeof(fr));
	rc |= tsnd_state(ctx, load);
	if (rc)
		return -1;
	if (load) {
		m68k_set_context(cpu);
		m68k_bus_set_irq_pending(irq);
		s_cpu_frac = fr;
	}
	return 0;
}

static int rastan_save(void *ctx) { return rastan_state(ctx, 0); }
static int rastan_load(void *ctx) { return rastan_state(ctx, 1); }
static int rastan_supported(const char *machine) { return !strcmp(machine, "rastan"); }

const rbx_driver rbx_drv_rastan = { "Rastan", rastan_init, rastan_exit, rastan_reset, rastan_frame, rastan_save, rastan_load,
                                    rastan_supported, rastan_render };
