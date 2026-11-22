/*  The Ninja Warriors / Darius II (triple screen), ported from MAME src/mame/taito/ninjaw.cpp (BSD-3-Clause,
    copyright-holders: David Graves), with TC0040IOC (src/mame/taito/taitoio.cpp, BSD-3-Clause) and TC0110PCR
    (src/mame/taito/tc0110pcr.cpp, BSD-3-Clause, Nicola Salmoria).
    Two 68000s, three TC0100SCN (video/tc0100scn.c, double width tilemaps, one per screen with a multiscreen x
    offset; writes to the first one go to all three), three TC0110PCR palettes, one sprite plane over the three
    screens; sound board Z80 + YM2610 + TC0140SYT (machine/taito_sound.c) with the pan latch at e400.
    The three 288 x 224 screens are drawn side by side into one 864 x 224 bitmap; screen k uses palette entries
    4096 * k (its TC0110PCR), index 12288 is the black pen.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m68k_bus.h"
#include "../../cpu/m68k_multi.h"
#include "../../cpu/m68k/m68k.h"
#include "../machine/taito_sound.h"
#include "../video/tc0100scn.h"
#include "../../video/gfxlayout.h"

#include <stdlib.h>
#include <string.h>

#define LINES 256
#define SCREENS 3
#define SCREEN_W 288
#define W (SCREENS * SCREEN_W)
#define VIS_Y0 24                    // visarea 3*8 .. 31*8-1
#define VIS_H 224
#define VBLANK_LINE (VIS_Y0 + VIS_H)
#define CPU_CLOCK 8000000
#define BLACK_PEN (SCREENS * 4096)
// CPU A / B interleave per scanline: the sub CPU polls the TC0040IOC port register that the main CPU switches
// (MAME quantum 16 MHz / 1024); a whole line per CPU lets it miss the IN0 select and hang
#define CPU_SLICES 4

static int s_darius2;
static uint8_t *s_rom_a, *s_rom_b;
static uint32_t s_rom_a_size, s_rom_b_size;
static uint8_t s_ram_a[0x10000], s_ram_b[0x10000], s_share1[0x10000], s_spr[0x4000];
static uint16_t (*s_pcr_hp)[SCREENS][0x1000];
#define s_pcr (*s_pcr_hp)
static uint16_t s_pcr_addr[SCREENS];
static tc0100scn_t *s_scn[SCREENS], *s_snap_scn[SCREENS];
static rbx_gfx s_sprgfx;
static int s_port[5];   // DSWA, DSWB, IN0, IN1, IN2
static uint64_t s_cpu_frac;
static struct {
	uint16_t cpua_ctrl;
	uint8_t b_reset, b_needreset;
	uint8_t ioc_port, ioc_regs[8];
	uint8_t pan[4];
	uint8_t pad[3];
} s_m;
static struct {
	uint16_t pcr[SCREENS][0x1000];
	uint8_t spr[0x4000];
} *s_snap;
static uint8_t *s_pri;
static uint16_t (*s_pal_last_hp)[SCREENS][0x1000];   // palette RAM converted into rbx_video.palette
#define s_pal_last (*s_pal_last_hp)
static int s_pal_valid;

// ---- sound: pancontrol_w (e400-e403) ----
static void update_pan(void)
{
	int pd[4];
	for (int i = 0; i < 4; i++)
		pd[i] = (int)((float)s_m.pan[i] * (100.f / 255.0f));
	// 2610.1.l / 2610.1.r take FM L, 2610.2.l / 2610.2.r FM R; mono = mean of the left and right speaker
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

// ---- TC0040IOC ----
static uint8_t ioc_read(uint32_t off)
{
	if (off & 1)
		return 0;   // watchdog
	switch (s_m.ioc_port) {
	case 0: return (uint8_t)rbx_port_read(s_port[0]);
	case 1: return (uint8_t)rbx_port_read(s_port[1]);
	case 2: return (uint8_t)rbx_port_read(s_port[2]);
	case 3: return (uint8_t)rbx_port_read(s_port[3]);
	case 4: return s_m.ioc_regs[4];
	case 7: return (uint8_t)rbx_port_read(s_port[4]);
	}
	return 0xff;
}

static void ioc_write(uint32_t off, uint8_t v)
{
	if (off & 1)
		s_m.ioc_port = v;
	else if (s_m.ioc_port < 8)
		s_m.ioc_regs[s_m.ioc_port] = v;   // 4: coin counters / lockout
}

// ---- 68000 buses ----
static int scn_ctrl_chip(uint32_t a)
{
	if (a >= 0x2a0000 && a <= 0x2a000f) return 0;
	if (a >= 0x2e0000 && a <= 0x2e000f) return 1;
	if (a >= 0x320000 && a <= 0x32000f) return 2;
	return -1;
}

static int pcr_chip(uint32_t a)
{
	if (s_darius2 && m68k_multi_current() == 1)
		return -1;   // darius2 slave map has no palette
	if (a >= 0x340000 && a <= 0x340007) return 0;
	if (a >= 0x350000 && a <= 0x350007) return 1;
	if (a >= 0x360000 && a <= 0x360007) return 2;
	return -1;
}

static uint16_t bus_read16(uint32_t a)
{
	if (a >= 0x200000 && a <= 0x200003) return ioc_read((a - 0x200000) >> 1);
	int cpu = m68k_multi_current();
	if (cpu == 0) {
		if (a == 0x220002) return tc0140syt_master_comm_r(tsnd_syt());
		int c = scn_ctrl_chip(a);
		if (c >= 0) return tc0100scn_ctrl_r(s_scn[c], (a >> 1) & 7);
	}
	int p = pcr_chip(a);
	if (p >= 0) return ((a >> 1) & 3) == 1 ? s_pcr[p][s_pcr_addr[p]] : 0xff;
	return 0;
}

static uint8_t bus_read8(uint32_t a)
{
	if (a == 0x220003 && m68k_multi_current() == 0) return tc0140syt_master_comm_r(tsnd_syt());
	return (uint8_t)(bus_read16(a & ~1u) >> ((a & 1) ? 0 : 8));
}

static void parse_control(void)
{
	int reset = !(s_m.cpua_ctrl & 1);
	if (reset && !s_m.b_reset)
		s_m.b_needreset = 1;
	s_m.b_reset = (uint8_t)reset;
}

static void bus_write16m(uint32_t a, uint16_t v, uint16_t mask)
{
	if (a >= 0x280000 && a <= 0x293fff) {   // tc0100scn_triple_screen_w
		uint32_t o = (a - 0x280000) >> 1;
		for (int i = 0; i < SCREENS; i++)
			s_scn[i]->ram[o] = (uint16_t)((s_scn[i]->ram[o] & ~mask) | (v & mask));
		return;
	}
	if (a >= 0x200000 && a <= 0x200003) {
		if (mask & 0xff)
			ioc_write((a - 0x200000) >> 1, (uint8_t)v);
		return;
	}
	int p = pcr_chip(a);
	if (p >= 0) {
		switch ((a >> 1) & 3) {
		case 0: s_pcr_addr[p] = v & 0xfff; break;
		case 1: s_pcr[p][s_pcr_addr[p]] = v; break;
		}
		return;
	}
	if (m68k_multi_current() != 0)
		return;
	if (a == 0x210000) {   // cpua_ctrl_w
		uint16_t d = v & mask;
		if ((d & 0xff00) && ((d & 0xff) == 0))
			d = d >> 8;
		s_m.cpua_ctrl = d;
		parse_control();
		return;
	}
	if (a == 0x220000 && (mask & 0xff)) { tc0140syt_master_port_w(tsnd_syt(), (uint8_t)v); return; }
	if (a == 0x220002 && (mask & 0xff)) { tc0140syt_master_comm_w(tsnd_syt(), (uint8_t)v); return; }
	int c = scn_ctrl_chip(a);
	if (c >= 0) { tc0100scn_ctrl_w(s_scn[c], (a >> 1) & 7, v, mask); return; }
}

static void bus_write16(uint32_t a, uint16_t v) { bus_write16m(a, v, 0xffff); }
static void bus_write8(uint32_t a, uint8_t v)
{
	if (a & 1)
		bus_write16m(a & ~1u, v, 0x00ff);
	else
		bus_write16m(a, (uint16_t)(v << 8), 0xff00);
}

// ---- video ----
// the sprites of the band of rows from y0 (rows h) of chip's screen; bm and pri at the band's first row
static void draw_sprites(int chip, uint16_t *bm, uint8_t *pri, int pitch, int y0, int h)
{
	static const uint32_t primask[2] = { 0xf0f0, 0xf0f0 | 0xcccc };   // over / under the middle layer
	const uint16_t *spr = (const uint16_t *)s_snap->spr;
	const int x_offs = SCREEN_W * chip, y_offs = 8;
	for (int offs = 0; offs < 0x4000 / 2; offs += 4) {
		uint32_t tilenum = spr[offs + 2] & 0x7fff;
		if (!tilenum)
			continue;
		int x = (spr[offs] - 32) & 0x3ff;
		int y = spr[offs + 1] & 0x1ff;
		uint16_t d = spr[offs + 3];
		int flipx = d & 1, flipy = (d >> 1) & 1, priority = (d >> 2) & 1, color = (d & 0x7f00) >> 8;
		x -= x_offs;
		y += y_offs;
		if (x > 0x3c0) x -= 0x400;
		if (y > 0x180) y -= 0x200;
		rbx_gfx_prio_transpen(&s_sprgfx, tilenum, (uint16_t)((color << 4) + 4096 * chip), flipx, flipy, x, y - VIS_Y0 - y0, bm,
		                      pri, pitch, SCREEN_W, h, primask[priority]);
	}
}

// the frame is drawn in pieces (a band of one screen's rows each: the pixels of one piece are no other's), by the
// render thread and the threads that help it (ninjaw_help)
#define BANDS 4
#define BAND_H (VIS_H / BANDS)
#define PIECES (SCREENS * BANDS)
static int s_layer0;
static int s_job_next, s_job_left, s_job_active, s_helpers;

static void draw_piece(int p)
{
	const int k = p / BANDS, y0 = p % BANDS * BAND_H;
	const tc0100scn_t *scn = s_snap_scn[k];
	const int pitch = rbx_video.pitch;
	uint16_t *bm = rbx_video.bitmap + y0 * pitch + k * SCREEN_W;
	uint8_t *pri = s_pri + y0 * pitch + k * SCREEN_W;
	for (int y = 0; y < BAND_H; y++)
		memset(pri + y * pitch, 0, SCREEN_W);
	// MAME: bottom layer priority 1; the sprite masks (0xf0f0, 0xfcfc) never test bit 0, so 0 draws the same
	if (tc0100scn_draw_ex(scn, s_layer0, bm, pri, pitch, VIS_Y0 + y0, BAND_H, 0, TC0100SCN_OPAQUE))
		for (int y = 0; y < BAND_H; y++)
			for (int x = 0; x < SCREEN_W; x++)
				bm[y * pitch + x] = BLACK_PEN;
	tc0100scn_draw(scn, s_layer0 ^ 1, bm, pri, pitch, VIS_Y0 + y0, BAND_H, 2);
	tc0100scn_draw(scn, 2, bm, pri, pitch, VIS_Y0 + y0, BAND_H, 4);
	draw_sprites(k, bm, pri, pitch, y0, BAND_H);
}

// pieces until none is left (one: a single piece); the number drawn
static int draw_pieces(int one)
{
	int n = 0;
	for (;;) {
		const int p = __atomic_fetch_add(&s_job_next, 1, __ATOMIC_ACQ_REL);
		if (p >= PIECES)
			return n;
		draw_piece(p);
		__atomic_sub_fetch(&s_job_left, 1, __ATOMIC_ACQ_REL);
		n++;
		if (one)
			return n;
	}
}

static int ninjaw_help(int helper)
{
	if (!__atomic_load_n(&s_job_active, __ATOMIC_SEQ_CST))
		return 0;
	__atomic_add_fetch(&s_helpers, 1, __ATOMIC_SEQ_CST);
	int did = 0;
	if (__atomic_load_n(&s_job_active, __ATOMIC_SEQ_CST))
		did = draw_pieces(helper & 0x100) > 0;
	__atomic_sub_fetch(&s_helpers, 1, __ATOMIC_SEQ_CST);
	return did;
}

static void ninjaw_render(void)
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
	s_layer0 = tc0100scn_bottomlayer(s_snap_scn[0]);
	s_job_next = 0;
	s_job_left = PIECES;
	__atomic_store_n(&s_job_active, 1, __ATOMIC_SEQ_CST);
	draw_pieces(0);
	while (__atomic_load_n(&s_job_left, __ATOMIC_ACQUIRE)) {
	}
	__atomic_store_n(&s_job_active, 0, __ATOMIC_SEQ_CST);
	while (__atomic_load_n(&s_helpers, __ATOMIC_SEQ_CST)) {   // (none may still take a piece of the next frame)
	}
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	for (int i = 0; i < SCREENS; i++)
		tc0100scn_copy(s_snap_scn[i], s_scn[i]);
	memcpy(s_snap->pcr, s_pcr, sizeof(s_pcr));
	memcpy(s_snap->spr, s_spr, sizeof(s_spr));
	rbx_video_frame_done();
}

// ---- init / frame ----
static void ninjaw_reset(void)
{
	// (the palette / video RAM too: a game that has not written it yet showed the last game's, as at start it is 0)
	memset(s_pcr, 0, sizeof(s_pcr));
	memset(s_ram_a, 0, sizeof(s_ram_a));
	memset(s_ram_b, 0, sizeof(s_ram_b));
	memset(s_share1, 0, sizeof(s_share1));
	memset(s_spr, 0, sizeof(s_spr));
	memset(s_pcr_addr, 0, sizeof(s_pcr_addr));
	for (int i = 0; i < SCREENS; i++)
		tc0100scn_reset(s_scn[i]);
	memset(&s_m, 0, sizeof(s_m));
	s_m.cpua_ctrl = 0xff;
	tsnd_reset();
	update_pan();
	m68k_multi_reset();
	s_cpu_frac = 0;
}

static void ninjaw_exit(void);

static int ninjaw_init(const rbx_game *g)
{
	if (!s_pal_last_hp && !(s_pal_last_hp = calloc(1, sizeof(*s_pal_last_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	if (!s_pcr_hp && !(s_pcr_hp = calloc(1, sizeof(*s_pcr_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	uint32_t scn_size, spr_size;
	s_darius2 = !strcmp(g->machine, "darius2");
	s_rom_a = rbx_region("maincpu", &s_rom_a_size);
	s_rom_b = rbx_region("sub", &s_rom_b_size);
	uint8_t *scn_gfx = rbx_region("tc0100scn_1", &scn_size);
	uint8_t *spr_gfx = rbx_region("sprites", &spr_size);
	if (!s_rom_a || !s_rom_b || !scn_gfx || !spr_gfx) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	m68k_bus_swap16(s_rom_a, s_rom_a_size);
	m68k_bus_swap16(s_rom_b, s_rom_b_size);
	// sprite tilelayout (Raine obj decoding)
	static const uint32_t planes[4] = { 0, 4, 8, 12 };
	static const uint32_t xo[16] = { 3, 2, 1, 0, 19, 18, 17, 16, 259, 258, 257, 256, 275, 274, 273, 272 };
	static const uint32_t yo[16] = { 0, 32, 64, 96, 128, 160, 192, 224, 512, 544, 576, 608, 640, 672, 704, 736 };
	static const rbx_gfx_layout layout = { 16, 16, 4, planes, xo, yo, 1024 };
	s_snap = calloc(1, sizeof(*s_snap));
	s_pri = (uint8_t *)malloc((size_t)W * VIS_H);
	if (!s_snap || !s_pri || rbx_gfx_decode(&s_sprgfx, &layout, spr_gfx, spr_size))
		goto mem_fail;
	for (int i = 0; i < SCREENS; i++) {
		s_scn[i] = (tc0100scn_t *)calloc(1, sizeof(tc0100scn_t));
		s_snap_scn[i] = (tc0100scn_t *)calloc(1, sizeof(tc0100scn_t));
		if (!s_scn[i] || !s_snap_scn[i])
			goto mem_fail;
	}
	// the three SCR ROM sets are identical: decode once, chips 2 and 3 share the tiles
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
		c->x_offset = 22;                    // set_offsets(22, 0)
		c->y_offset = 0;
		c->multiscrn_xoffs = 2 * i;          // 0, 2, 4
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
	// sound: YM2610 SSG 0.75 (subwoofer), FM through the pan filters
	tsnd_config sc;
	memset(&sc, 0, sizeof(sc));
	sc.map = TSND_MAP_E000;
	sc.z80_clock = 4000000;
	sc.frame_rate_mhz = 60000;
	sc.lines = LINES;
	sc.ym.type = RBX_YM2610;
	sc.ym.clock = 8000000;
	sc.ym.gain[0] = 192;
	sc.bank_mask = 7;
	sc.ext_write = snd_ext_write;
	if (tsnd_init(&sc)) {
		rbx_fail("%s: sound init failed", g->set);
		ninjaw_exit();
		return -1;
	}
	if (rbx_video_setup(W, VIS_H, 16384)) {
		ninjaw_exit();
		return -1;
	}
	s_pal_valid = 0;
	m68k_init();
	m68k_set_cpu_type(M68K_CPU_TYPE_68000);
	if (m68k_multi_init(2)) {
		ninjaw_exit();
		return -1;
	}
	m68k_bus_reset_map();   // CPU A
	m68k_bus_map(0, 0x0bffff, s_rom_a, s_rom_a_size, 1, 0);
	m68k_bus_map(0x0c0000, 0x0cffff, s_ram_a, sizeof(s_ram_a), 1, 1);
	m68k_bus_map(0x240000, 0x24ffff, s_share1, sizeof(s_share1), 1, 1);
	m68k_bus_map(0x260000, 0x263fff, s_spr, sizeof(s_spr), 1, 1);
	m68k_bus_map(0x280000, 0x293fff, s_scn[0]->ram, TC0100SCN_RAM_BYTES, 1, 0);
	m68k_bus_map(0x2c0000, 0x2d3fff, s_scn[1]->ram, TC0100SCN_RAM_BYTES, 1, 1);
	m68k_bus_map(0x300000, 0x313fff, s_scn[2]->ram, TC0100SCN_RAM_BYTES, 1, 1);
	m68k_bus.read8 = bus_read8;
	m68k_bus.read16 = bus_read16;
	m68k_bus.write8 = bus_write8;
	m68k_bus.write16 = bus_write16;
	m68k_multi_capture(0);
	m68k_bus_reset_map();   // CPU B
	m68k_bus_map(0, 0x05ffff, s_rom_b, s_rom_b_size, 1, 0);
	m68k_bus_map(0x080000, 0x08ffff, s_ram_b, sizeof(s_ram_b), 1, 1);
	m68k_bus_map(0x240000, 0x24ffff, s_share1, sizeof(s_share1), 1, 1);
	m68k_bus_map(0x260000, 0x263fff, s_spr, sizeof(s_spr), 1, 1);
	m68k_bus_map(0x280000, 0x293fff, s_scn[0]->ram, TC0100SCN_RAM_BYTES, 1, 0);
	m68k_multi_capture(1);
	m68k_multi_finish();
	ninjaw_reset();
	return 0;
mem_fail:
	rbx_fail("%s: out of memory (graphics)", g->set);
	ninjaw_exit();
	return -1;
}

static void ninjaw_exit(void)
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
	m68k_multi_exit();
	free(s_pal_last_hp);
	s_pal_last_hp = NULL;
	free(s_pcr_hp);
	s_pcr_hp = NULL;
}

static void ninjaw_frame(void)
{
	for (int line = 0; line < LINES; line++) {
		if (line == VBLANK_LINE) {
			vblank_capture();
			m68k_multi_hold_irq(0, 4);
			m68k_multi_hold_irq(1, 4);
		}
		int cyc = tsnd_line_cycles(&s_cpu_frac, CPU_CLOCK);
		for (int sl = 0; sl < CPU_SLICES; sl++) {
			int c = cyc / CPU_SLICES + (sl == 0 ? cyc % CPU_SLICES : 0);
			m68k_multi_switch(0);
			m68k_execute(c);
			if (!s_m.b_reset) {
				if (s_m.b_needreset) {
					m68k_multi_reset_cpu(1);
					s_m.b_needreset = 0;
				}
				m68k_multi_switch(1);
				m68k_execute(c);
			}
		}
		tsnd_line();
	}
	m68k_multi_switch(0);
	tsnd_frame_end();
}

static int ninjaw_state(void *ctx, int load)
{
	uint64_t fr = s_cpu_frac;
	int rc = m68k_multi_state(ctx, load);
	rc |= rbx_state_io(ctx, "ram_a", s_ram_a, sizeof(s_ram_a));
	rc |= rbx_state_io(ctx, "ram_b", s_ram_b, sizeof(s_ram_b));
	rc |= rbx_state_io(ctx, "share1", s_share1, sizeof(s_share1));
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
		s_cpu_frac = fr;
		update_pan();
	}
	return 0;
}

static int ninjaw_save(void *ctx) { return ninjaw_state(ctx, 0); }
static int ninjaw_load(void *ctx) { return ninjaw_state(ctx, 1); }
static int ninjaw_supported(const char *machine) { return !strcmp(machine, "ninjaw") || !strcmp(machine, "darius2"); }

const rbx_driver rbx_drv_ninjaw = { "Ninja Warriors", ninjaw_init, ninjaw_exit, ninjaw_reset, ninjaw_frame, ninjaw_save,
                                    ninjaw_load, ninjaw_supported, ninjaw_render, ninjaw_help };
