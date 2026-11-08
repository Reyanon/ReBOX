/*  Asuka & Asuka family, ported from MAME src/mame/taito/asuka.cpp (BSD-3-Clause, copyright-holders: David Graves,
    Brian Troha; thanks to Richard Bush): Bonze Adventure / Jigoku Meguri (C-Chip, YM2610), Asuka & Asuka, Maze of Flott,
    Galmedes, Earth Joker (YM2151 + MSM5205), Cadash (YM2151; the Z180 link CPU is not emulated, stand-alone play only)
    and Kokontouzai Eto Monogatari (YM2151).
    68000 + TC0100SCN tilemaps (video/tc0100scn.c) + PC090OJ sprites (video/pc090oj.c) + TC0110PCR palette
    (src/mame/taito/tc0110pcr.cpp, BSD-3-Clause, Nicola Salmoria) + TC0220IOC inputs (src/mame/taito/taitoio.cpp,
    BSD-3-Clause); sound board: Z80 + YM + PC060HA / TC0140SYT (machine/taito_sound.c), MSM5205 behind an LS157.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m68k_bus.h"
#include "../../cpu/m68k/m68k.h"
#include "../machine/taito_sound.h"
#include "../machine/taitocchip.h"
#include "../../sound/msm5205.h"
#include "../video/tc0100scn.h"
#include "../video/pc090oj.h"

#include <stdlib.h>
#include <string.h>

#define LINES 256
#define SCREEN_W 320

// ---- machine configurations (MAME bonzeadv / asuka / mofflott / cadash / eto) ----
enum { K_BONZE, K_ASUKA, K_CADASH, K_ETO };

typedef struct {
	const char *name;
	int kind;
	uint32_t cpu_clock;
	int vis_y0, vis_y1;            // screen visible area
	int oj_buffer;                 // pc090oj set_usebuffer
	int variable_pri;              // variable_colpri_cb (else fixed_colpri_cb)
	int scn_x;                     // tc0100scn set_offsets(x, 0)
	int xbgr444;                   // cadash colour callback
	int msm;                       // MSM5205 on the sound board
	int ym_gain;                   // YM2151 route gain (1/256)
	// main CPU map
	uint32_t rom_end, ram_start, ram_end, pcr, ioc, ioc_mirror, sprctrl, comm, scn, scnctrl, oj, cchip, shared;
} machine_t;

static const machine_t s_machines[] = {
	{ .name = "bonzeadv", .kind = K_BONZE, .cpu_clock = 8000000, .vis_y0 = 24, .vis_y1 = 247, .scn_x = 0,
	  .rom_end = 0x0fffff, .ram_start = 0x10c000, .ram_end = 0x10ffff, .pcr = 0x200000, .sprctrl = 0x3a0000,
	  .comm = 0x3e0000, .scn = 0xc00000, .scnctrl = 0xc20000, .oj = 0xd00000, .cchip = 0x800000 },
	{ .name = "asuka", .kind = K_ASUKA, .cpu_clock = 8000000, .vis_y0 = 16, .vis_y1 = 255, .oj_buffer = 1,
	  .variable_pri = 1, .scn_x = 0, .msm = 1, .ym_gain = 64,
	  .rom_end = 0x0fffff, .ram_start = 0x100000, .ram_end = 0x103fff, .pcr = 0x200000, .ioc = 0x400000,
	  .sprctrl = 0x3a0000, .comm = 0x3e0000, .scn = 0xc00000, .scnctrl = 0xc20000, .oj = 0xd00000 },
	{ .name = "mofflott", .kind = K_ASUKA, .cpu_clock = 8000000, .vis_y0 = 16, .vis_y1 = 255, .oj_buffer = 0,
	  .variable_pri = 1, .scn_x = 1, .msm = 1, .ym_gain = 64,
	  .rom_end = 0x0fffff, .ram_start = 0x100000, .ram_end = 0x103fff, .pcr = 0x200000, .ioc = 0x400000,
	  .sprctrl = 0x3a0000, .comm = 0x3e0000, .scn = 0xc00000, .scnctrl = 0xc20000, .oj = 0xd00000 },
	{ .name = "cadash", .kind = K_CADASH, .cpu_clock = 16000000, .vis_y0 = 16, .vis_y1 = 255, .oj_buffer = 1,
	  .scn_x = 1, .xbgr444 = 1, .ym_gain = 128,
	  .rom_end = 0x07ffff, .ram_start = 0x100000, .ram_end = 0x107fff, .pcr = 0xa00000, .ioc = 0x900000,
	  .sprctrl = 0x080000, .comm = 0x0c0000, .scn = 0xc00000, .scnctrl = 0xc20000, .oj = 0xb00000, .shared = 0x800000 },
	{ .name = "eto", .kind = K_ETO, .cpu_clock = 8000000, .vis_y0 = 16, .vis_y1 = 255, .variable_pri = 1, .scn_x = 1,
	  .ym_gain = 128,
	  .rom_end = 0x0fffff, .ram_start = 0x200000, .ram_end = 0x203fff, .pcr = 0x100000, .ioc = 0x300000,
	  .ioc_mirror = 0x400000, .sprctrl = 0x4a0000, .comm = 0x4e0000, .scn = 0xd00000, .scnctrl = 0xd20000,
	  .oj = 0xc00000 },
};

// ---- board state ----
static const machine_t *s_m;
static int s_vis_h;
static uint8_t *s_rom;
static uint32_t s_rom_size;
static uint8_t s_ram[0x8000];
static uint16_t s_pcr[0x1000];       // TC0110PCR RAM
static uint16_t s_pcr_addr;
static uint8_t s_ioc_regs[8];
static uint8_t s_shared[0x800];      // cadash network RAM
static tc0100scn_t *s_scn, *s_snap_scn;
static pc090oj_t *s_oj;
static struct {
	pc090oj_regs oj;
	uint16_t pal[0x1000];
} *s_snap;
static uint8_t *s_pri;
static int s_port_ioc[8], s_port_dswa, s_port_dswb, s_port_cc[4];
static uint64_t s_cpu_frac, s_cc_frac;
// ADPCM: MSM5205 fed through an LS157 nibble selector
static msm5205_t s_msm;
static const uint8_t *s_adpcm;
static uint32_t s_adpcm_size;
static uint32_t s_msm_acc;
static struct { uint16_t pos; uint8_t ff, pad; } s_ad;

// ---- sound ----
static void msm_vck(void)   // MAME msm5205_vck
{
	s_ad.ff = !s_ad.ff;
	if (s_ad.ff) {
		uint8_t d = s_ad.pos < s_adpcm_size ? s_adpcm[s_ad.pos] : 0;
		msm5205_data_w(&s_msm, (uint8_t)(d >> 4));   // LS157 select = 1: B input (high nibble)
		s_ad.pad = (uint8_t)(d & 0x0f);
		s_ad.pos = (uint16_t)(s_ad.pos + 1);
	} else {
		msm5205_data_w(&s_msm, s_ad.pad);           // select = 0: A input (low nibble) of the latched byte
	}
}

static void snd_ext_write(uint16_t a, uint8_t v)
{
	switch (a & 0xf000) {
	case 0xb000: s_ad.pos = (uint16_t)((s_ad.pos & 0x00ff) | (v << 8)); break;   // msm5205_address_w
	case 0xc000: msm5205_reset_w(&s_msm, 0); s_ad.ff = 0; break;                 // msm5205_start_w
	case 0xd000: msm5205_reset_w(&s_msm, 1); s_ad.pos &= 0xff00; break;          // msm5205_stop_w
	}
}

static void snd_ext_mix(int32_t *buf, int n, uint32_t rate)
{
	for (int i = 0; i < n; i++) {
		s_msm_acc += s_msm.clock;
		uint32_t k = s_msm_acc / rate;
		s_msm_acc -= k * rate;
		msm5205_advance(&s_msm, k);
		buf[i] += msm5205_output(&s_msm) / 2;   // route 0.5
	}
}

// ---- C-Chip ports (bonzeadv) ----
static uint8_t cc_pa(void) { return (uint8_t)rbx_port_read(s_port_cc[0]); }
static uint8_t cc_pb(void) { return (uint8_t)rbx_port_read(s_port_cc[1]); }
static uint8_t cc_pc(void) { return (uint8_t)rbx_port_read(s_port_cc[2]); }
static uint8_t cc_ad(void) { return (uint8_t)rbx_port_read(s_port_cc[3]); }
static void cc_counters(uint8_t v) { (void)v; }   // coin lockout / counters

// ---- TC0220IOC ----
static uint8_t ioc_read(int offset)
{
	switch (offset & 7) {
	case 0: case 1: case 2: case 3: case 7: return (uint8_t)rbx_port_read(s_port_ioc[offset & 7]);
	case 4: return s_ioc_regs[4];
	}
	return 0xff;
}

// ---- main CPU devices ----
static inline int in_win(uint32_t a, uint32_t base, uint32_t size) { return base && a >= base && a < base + size; }

static uint16_t main_read16(uint32_t a)
{
	const machine_t *m = s_m;
	if (in_win(a, m->pcr, 0x10)) return ((a >> 1) & 7) == 1 ? s_pcr[s_pcr_addr] : 0xff;
	if (in_win(a, m->ioc, 0x10)) return ioc_read((int)((a - m->ioc) >> 1));
	if (in_win(a, m->ioc_mirror, 0x10)) return ioc_read((int)((a - m->ioc_mirror) >> 1));
	if (in_win(a, m->scnctrl, 0x10)) return tc0100scn_ctrl_r(s_scn, (int)((a - m->scnctrl) >> 1));
	if (a == m->comm + 2) return tc0140syt_master_comm_r(tsnd_syt());
	if (in_win(a, m->cchip, 0x800)) return cchip_mem68_r((a - m->cchip) >> 1);
	if (in_win(a, m->cchip + 0x800, 0x800)) return cchip_asic_r((a - m->cchip - 0x800) >> 1);
	if (in_win(a, m->shared, 0x1000)) return s_shared[(a - m->shared) >> 1];
	if (m->kind == K_BONZE) {
		if (a == 0x390000) return (uint16_t)(rbx_port_read(s_port_dswa) & 0xff);
		if (a == 0x3b0000) return (uint16_t)(rbx_port_read(s_port_dswb) & 0xff);
	}
	return 0;
}

static uint8_t main_read8(uint32_t a)
{
	const machine_t *m = s_m;
	if (a == m->comm + 3) return tc0140syt_master_comm_r(tsnd_syt());
	if (!(a & 1) && (in_win(a, m->ioc, 0x10) || in_win(a, m->ioc_mirror, 0x10) || in_win(a, m->cchip, 0x1000)))
		return 0;   // umask16(0x00ff) devices
	return (uint8_t)(main_read16(a & ~1u) >> ((a & 1) ? 0 : 8));
}

static void main_write16m(uint32_t a, uint16_t v, uint16_t mask)
{
	const machine_t *m = s_m;
	if (in_win(a, m->pcr, 0x10)) {   // TC0110PCR word_w, shift 0
		int off = (int)((a >> 1) & 7);
		if (off == 0)
			s_pcr_addr = v & 0xfff;
		else if (off == 1)
			s_pcr[s_pcr_addr] = v;
		return;
	}
	if (in_win(a, m->ioc, 0x10)) {
		if (mask & 0xff)
			s_ioc_regs[((a - m->ioc) >> 1) & 7] = (uint8_t)v;   // 0 watchdog, 4 coin lockout / counters
		return;
	}
	if (in_win(a, m->scnctrl, 0x10)) { tc0100scn_ctrl_w(s_scn, (int)((a - m->scnctrl) >> 1), v, mask); return; }
	if (in_win(a, m->sprctrl, 4)) { s_oj->sprite_ctrl = (uint16_t)((s_oj->sprite_ctrl & ~mask) | (v & mask)); return; }
	if (a == m->comm && (mask & 0xff)) { tc0140syt_master_port_w(tsnd_syt(), (uint8_t)v); return; }
	if (a == m->comm + 2 && (mask & 0xff)) { tc0140syt_master_comm_w(tsnd_syt(), (uint8_t)v); return; }
	if (in_win(a, m->cchip, 0x1000)) {
		if (mask & 0xff) {
			if (a < m->cchip + 0x800)
				cchip_mem68_w((a - m->cchip) >> 1, (uint8_t)v);
			else
				cchip_asic68_w((a - m->cchip - 0x800) >> 1, (uint8_t)v);
		}
		return;
	}
	if (in_win(a, m->shared, 0x1000)) {
		if (mask & 0xff)
			s_shared[(a - m->shared) >> 1] = (uint8_t)v;
		return;
	}
	// bonzeadv 0x3c0000 watchdog, asuka 0xc10000 (error in the init code)
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
static void asuka_render(void)
{
	const uint16_t *p = s_snap->pal;
	if (s_m->xbgr444) {
		for (int i = 0; i < 4096; i++) {
			int r = p[i] & 15, g = (p[i] >> 4) & 15, b = (p[i] >> 8) & 15;
			rbx_video.palette[i] = (uint32_t)((r * 17) << 16 | (g * 17) << 8 | (b * 17));
		}
	} else {
		for (int i = 0; i < 4096; i++) {
			int r = p[i] & 31, g = (p[i] >> 5) & 31, b = (p[i] >> 10) & 31;
			r = (r << 3) | (r >> 2); g = (g << 3) | (g >> 2); b = (b << 3) | (b >> 2);
			rbx_video.palette[i] = (uint32_t)(r << 16 | g << 8 | b);
		}
	}
	uint16_t *bm = rbx_video.bitmap;
	int pitch = rbx_video.pitch;
	const int y0 = s_m->vis_y0, h = s_vis_h;
	const tc0100scn_t *scn = s_snap_scn;
	memset(s_pri, 0, (size_t)pitch * h);
	int layer0 = tc0100scn_bottomlayer(scn);
	// the screen is blanked even when the bottom layer is disabled
	if (tc0100scn_draw_ex(scn, layer0, bm, s_pri, pitch, y0, h, 1, TC0100SCN_OPAQUE))
		for (int y = 0; y < h; y++)
			memset(bm + y * pitch, 0, SCREEN_W * 2);
	tc0100scn_draw(scn, layer0 ^ 1, bm, s_pri, pitch, y0, h, 2);
	tc0100scn_draw(scn, 2, bm, s_pri, pitch, y0, h, 4);
	const uint16_t ctrl = s_snap->oj.sprite_ctrl;
	uint32_t pri_mask = s_m->variable_pri && (ctrl & 1) ? 0xfc : 0xf0;
	pc090oj_draw(s_oj, &s_snap->oj, bm, pitch, s_pri, SCREEN_W, y0, h, 1, (ctrl & 0x3c) << 2, pri_mask);
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	tc0100scn_copy(s_snap_scn, s_scn);
	pc090oj_snapshot(s_oj, &s_snap->oj);
	memcpy(s_snap->pal, s_pcr, sizeof(s_pcr));
	rbx_video_frame_done();
}

// ---- init / frame ----
static void asuka_reset(void)
{
	// (the palette / video RAM too: a game that has not written it yet showed the last game's, as at start it is 0)
	memset(s_pcr, 0, sizeof(s_pcr));
	memset(s_ram, 0, sizeof(s_ram));
	memset(s_shared, 0, sizeof(s_shared));
	memset(s_ioc_regs, 0, sizeof(s_ioc_regs));
	s_pcr_addr = 0;
	tc0100scn_reset(s_scn);
	pc090oj_reset(s_oj);
	s_oj->sprite_ctrl = 0;
	memset(&s_ad, 0, sizeof(s_ad));
	if (s_m->msm)
		msm5205_reset(&s_msm);
	s_msm_acc = 0;
	if (s_m->kind == K_BONZE)
		cchip_reset();
	m68k_bus_irq_reset();
	m68k_pulse_reset();
	tsnd_reset();
	s_cpu_frac = s_cc_frac = 0;
}

static void asuka_exit(void);

static const machine_t *find_machine(const char *name)
{
	for (size_t i = 0; i < sizeof(s_machines) / sizeof(s_machines[0]); i++)
		if (!strcmp(s_machines[i].name, name))
			return &s_machines[i];
	return NULL;
}

static int asuka_init(const rbx_game *g)
{
	uint32_t scn_size, oj_size;
	s_m = find_machine(g->machine);
	if (!s_m) {
		rbx_fail("%s: machine %s is not emulated", g->set, g->machine);
		return -1;
	}
	s_vis_h = s_m->vis_y1 - s_m->vis_y0 + 1;
	s_rom = rbx_region("maincpu", &s_rom_size);
	uint8_t *scn_gfx = rbx_region("tc0100scn", &scn_size);
	uint8_t *oj_gfx = rbx_region("pc090oj", &oj_size);
	if (!s_rom || !scn_gfx || !oj_gfx) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	m68k_bus_swap16(s_rom, s_rom_size);
	if (!strcmp(g->set, "earthjkr") && s_rom_size > 0x7aaa)   // MAME init_earthjkr: bitrot fix (357c -> 317c)
		((uint16_t *)s_rom)[0x7aaa / 2] = 0x317c;
	s_scn = (tc0100scn_t *)calloc(1, sizeof(tc0100scn_t));
	s_snap_scn = (tc0100scn_t *)malloc(sizeof(tc0100scn_t));
	s_oj = (pc090oj_t *)calloc(1, sizeof(pc090oj_t));
	s_snap = calloc(1, sizeof(*s_snap));
	s_pri = (uint8_t *)calloc(SCREEN_W, (size_t)s_vis_h);
	if (!s_scn || !s_snap_scn || !s_oj || !s_snap || !s_pri || tc0100scn_init(s_scn, scn_gfx, scn_size, TC0100SCN_4BPP) ||
	    pc090oj_init(s_oj, oj_gfx, oj_size, 4096)) {
		rbx_fail("%s: out of memory (graphics)", g->set);
		asuka_exit();
		return -1;
	}
	s_scn->x_offset = s_m->scn_x;
	s_scn->screen_h = s_m->vis_y0 + s_m->vis_y1 + 1;
	*s_snap_scn = *s_scn;
	s_oj->x_offset = 0;   // set_offsets(0, 8)
	s_oj->y_offset = 8;
	s_oj->use_buffer = s_m->oj_buffer;
	s_oj->direct = 1;
	static const char *const io_tags[8] = { "DSWA", "DSWB", "IN0", "IN1", 0, 0, 0, "IN2" };
	for (int i = 0; i < 8; i++)
		s_port_ioc[i] = io_tags[i] ? rbx_port_find(io_tags[i]) : -1;
	s_port_dswa = rbx_port_find("DSWA");
	s_port_dswb = rbx_port_find("DSWB");
	if (s_m->kind == K_BONZE) {
		static const char *const tags[4] = { "800007", "800009", "80000B", "80000D" };
		for (int i = 0; i < 4; i++)
			s_port_cc[i] = rbx_port_find(tags[i]);
		cchip_config cc;
		memset(&cc, 0, sizeof(cc));
		cc.clock = 12000000;
		cc.in_pa = cc_pa;
		cc.in_pb = cc_pb;
		cc.in_pc = cc_pc;
		cc.in_ad = cc_ad;
		cc.out_pb = cc_counters;
		if (cchip_init(&cc)) {
			rbx_fail("%s: C-Chip ROMs missing", g->set);
			asuka_exit();
			return -1;
		}
	}
	// sound
	tsnd_config sc;
	memset(&sc, 0, sizeof(sc));
	sc.frame_rate_mhz = 60000;
	sc.lines = LINES;
	sc.z80_clock = 4000000;
	if (s_m->kind == K_BONZE) {   // YM2610 8 MHz, routes 0.75 / 1.0 / 1.0
		sc.map = TSND_MAP_E000;
		sc.ym.type = RBX_YM2610;
		sc.ym.clock = 8000000;
		sc.ym.gain[0] = 192;
		sc.ym.gain[1] = sc.ym.gain[2] = 256;
	} else {                      // YM2151 4 MHz, banks from its CT pins
		sc.map = TSND_MAP_9000;
		sc.ym.type = RBX_YM2151;
		sc.ym.clock = 4000000;
		sc.ym.gain[0] = sc.ym.gain[1] = s_m->ym_gain;
		sc.ym_port_bank = 1;
		if (s_m->msm) {
			s_adpcm = rbx_region("msm", &s_adpcm_size);
			msm5205_init(&s_msm, 384000, MSM5205_S48_4B, msm_vck);   // 8 kHz, 4-bit
			sc.ext_write = snd_ext_write;
			sc.ext_mix = snd_ext_mix;
		}
	}
	if (tsnd_init(&sc)) {
		rbx_fail("%s: sound init failed", g->set);
		asuka_exit();
		return -1;
	}
	if (rbx_video_setup(SCREEN_W, s_vis_h, 4096)) {
		asuka_exit();
		return -1;
	}
	m68k_init();
	m68k_set_cpu_type(M68K_CPU_TYPE_68000);
	m68k_bus_reset_map();
	const machine_t *m = s_m;
	m68k_bus_map(0, m->rom_end, s_rom, s_rom_size, 1, 0);
	m68k_bus_map(m->ram_start, m->ram_end, s_ram, m->ram_end - m->ram_start + 1, 1, 1);
	m68k_bus_map(m->scn, m->scn + 0xffff, s_scn->ram, 0x10000, 1, 1);
	m68k_bus_map(m->oj, m->oj + 0x3fff, s_oj->ram, sizeof(s_oj->ram), 1, 1);
	if (m->kind == K_ETO)   // c00000-c0ffff: tc0100scn RAM mirror (write only) under the sprite RAM
		m68k_bus_map(0xc04000, 0xc0ffff, (uint8_t *)s_scn->ram + 0x4000, 0xc000, 0, 1);
	m68k_bus.read8 = main_read8;
	m68k_bus.read16 = main_read16;
	m68k_bus.write8 = main_write8;
	m68k_bus.write16 = main_write16;
	asuka_reset();
	return 0;
}

static void asuka_exit(void)
{
	if (s_scn)
		tc0100scn_exit(s_scn);
	if (s_oj)
		pc090oj_exit(s_oj);
	free(s_scn);
	free(s_snap_scn);
	free(s_oj);
	free(s_snap);
	free(s_pri);
	s_scn = s_snap_scn = NULL;
	s_oj = NULL;
	s_snap = NULL;
	s_pri = NULL;
	tsnd_exit();
}

static void asuka_frame(void)
{
	const int vbl = (s_m->vis_y1 + 1) % LINES;
	for (int line = 0; line < LINES; line++) {
		int cycles = tsnd_line_cycles(&s_cpu_frac, s_m->cpu_clock);
		if (line == vbl) {
			vblank_capture();
			pc090oj_eof(s_oj);   // screen_vblank (after the screen update)
			if (s_m->kind == K_CADASH) {   // IRQ4, IRQ5 500 cycles later
				m68k_bus_hold_irq(4);
				int done = m68k_execute(500);
				m68k_bus_hold_irq(5);
				if (cycles - done > 0)
					m68k_execute(cycles - done);
				cycles = 0;
			} else if (s_m->kind == K_BONZE) {
				m68k_bus_hold_irq(4);
				cchip_ext_interrupt(1);   // pulse (cleared by a zero-time timer in MAME)
				cchip_ext_interrupt(0);
			} else {
				m68k_bus_hold_irq(5);
			}
		}
		if (cycles > 0)
			m68k_execute(cycles);
		if (s_m->kind == K_BONZE)
			cchip_run_clocks(&s_cc_frac, 60000, LINES);
		tsnd_line();
	}
	tsnd_frame_end();
}

static int asuka_state(void *ctx, int load)
{
	uint8_t cpu[1024];
	uint32_t csize = m68k_context_size();
	if (csize > sizeof(cpu))
		return -1;
	if (!load)
		m68k_get_context(cpu);
	uint32_t irq = m68k_bus_irq_pending();
	uint64_t fr[2] = { s_cpu_frac, s_cc_frac };
	uint32_t misc[2] = { s_pcr_addr, s_msm_acc };
	int rc = 0;
	rc |= rbx_state_io(ctx, "m68k", cpu, csize);
	rc |= rbx_state_io(ctx, "irq", &irq, 4);
	rc |= rbx_state_io(ctx, "ram", s_ram, sizeof(s_ram));
	rc |= rbx_state_io(ctx, "pcr", s_pcr, sizeof(s_pcr));
	rc |= rbx_state_io(ctx, "ioc", s_ioc_regs, sizeof(s_ioc_regs));
	rc |= rbx_state_io(ctx, "shared", s_shared, sizeof(s_shared));
	rc |= rbx_state_io(ctx, "scn", s_scn->ram, sizeof(s_scn->ram));
	rc |= rbx_state_io(ctx, "scnc", s_scn->ctrl, sizeof(s_scn->ctrl));
	rc |= rbx_state_io(ctx, "pc090oj", s_oj->ram, sizeof(s_oj->ram));
	rc |= rbx_state_io(ctx, "ojbuf", s_oj->buffered, sizeof(s_oj->buffered));
	rc |= rbx_state_io(ctx, "ojctrl", &s_oj->sprite_ctrl, sizeof(s_oj->sprite_ctrl));
	rc |= rbx_state_io(ctx, "misc", misc, sizeof(misc));
	rc |= rbx_state_io(ctx, "frac", fr, sizeof(fr));
	if (s_m->msm) {
		rc |= rbx_state_io(ctx, "msm", &s_msm, offsetof(msm5205_t, vck));
		rc |= rbx_state_io(ctx, "adpcm", &s_ad, sizeof(s_ad));
	}
	if (s_m->kind == K_BONZE)
		rc |= cchip_state(ctx, load);
	rc |= tsnd_state(ctx, load);
	if (rc)
		return -1;
	if (load) {
		m68k_set_context(cpu);
		m68k_bus_set_irq_pending(irq);
		s_cpu_frac = fr[0];
		s_cc_frac = fr[1];
		s_pcr_addr = (uint16_t)(misc[0] & 0xfff);
		s_msm_acc = misc[1];
	}
	return 0;
}

static int asuka_save(void *ctx) { return asuka_state(ctx, 0); }
static int asuka_load(void *ctx) { return asuka_state(ctx, 1); }
static int asuka_supported(const char *machine) { return find_machine(machine) != NULL; }

const rbx_driver rbx_drv_asuka = { "Asuka & Asuka", asuka_init, asuka_exit, asuka_reset, asuka_frame, asuka_save,
                                   asuka_load, asuka_supported, asuka_render };
