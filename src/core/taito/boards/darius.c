/*  Darius (three screens), ported from MAME src/mame/taito/darius.cpp (GPL-2.0+, copyright-holders: David Graves,
    Jarek Burczynski; sound panning by Hiromitsu Shioya).
    Two 68000s (CPU B held in reset by CPU A's control register), PC080SN tilemaps (one wide 1024 pixel tilemap pair
    covering the three screens), a text tilemap, 16x16 sprites; sound board: Z80 + 2 x YM2203 + PC060HA with a
    mixer / pan latch per output, second Z80 feeding an MSM5205 (NMI per sample).
    The three 288 pixel screens are drawn side by side into one 864 x 224 bitmap: MAME draws screen k with an x
    offset of 288 * k on every layer, so the wide bitmap is the same picture.
    Sound is mixed to mono (left + right of the MAME mixer).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m68k_bus.h"
#include "../../cpu/m68k_multi.h"
#include "../../cpu/m68k/m68k.h"
#include "../../cpu/z80/z80.h"
#include "../machine/taitosnd.h"
#include "../../sound/rbx_ym.h"
#include "../../sound/msm5205.h"
#include "../video/pc080sn.h"
#include "../../video/gfxlayout.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#define LINES 256
#define SCREENS 3
#define SCREEN_W 288                 // 36 * 8
#define W (SCREENS * SCREEN_W)
#define VIS_Y0 8                     // visarea 1*8 .. 29*8-1
#define VIS_H 224
#define VBLANK_LINE (VIS_Y0 + VIS_H)
#define CPU_CLOCK 8000000
#define Z80_CLOCK 4000000
#define YM_CLOCK 4000000
#define MSM_CLOCK 384000

// ---- state ----
static uint8_t *s_rom_a, *s_rom_b, *s_rom_snd, *s_rom_adpcm;
static uint32_t s_rom_a_size, s_rom_b_size, s_rom_snd_size, s_rom_adpcm_size;
static uint8_t s_ram_a[0x10000], s_ram_b[0x10000], s_ram_e10[0x1000];
static uint8_t s_pal[0x1000];
static uint8_t s_spr[0x1000];      // e00000-e00fff (sprite RAM from 0x100)
static uint8_t s_share2[0x2000];   // e01000-e02fff
static uint8_t (*s_fg_hp)[0x8000];       // e08000-e0ffff
#define s_fg (*s_fg_hp)
static pc080sn_t *s_sn;
static rbx_gfx s_sprgfx, s_txtgfx;
static int s_port[5];              // P1, P2, SYSTEM, DSW
static uint64_t s_cpu_frac, s_z80_frac, s_z80b_frac, s_ym_frac, s_msm_frac;

static struct {
	uint16_t cpua_ctrl, coin_word;
	uint8_t adpcm_command, nmi_enable, bank, ym_irq;
	uint8_t b_reset, b_needreset, pad[2];
	uint8_t vol[8], pan[5], pad2[3];
} s_m;
static int s_def_vol[16];

// sound
static z80 s_z80, s_z80b;
static tc0140syt_t s_ciu;
static uint8_t s_zram[0x1000];
static msm5205_t s_msm;
static int s_msm_gain;             // 1/256
static int32_t *s_mix;
static int s_mix_cap, s_mix_n;
static uint32_t s_ym_line, s_ym_done;
static int s_line_target;
static unsigned long s_line_start;

static struct {
	pc080sn_regs sn;
	uint8_t pal[0x1000], spr[0x1000], fg[0x8000];
} *s_snap;
static uint8_t *s_pri;

// ---- sound mixer (darius_state::update_fm0 ... update_da) ----
// filter_volume gains of the left and right outputs (integer percent as MAME computes them), summed to mono
static int mono_gain(int pan, int vol, int route256)
{
	int left = (pan * vol) >> 8, right = ((0xff - pan) * vol) >> 8;
	return (left + right) * route256 / 100;
}

static void update_fm(int chip)
{
	rbx_ym_set_gain(chip, 3, mono_gain(s_m.pan[chip], s_m.vol[6 + chip], 154));   // FM route 0.60
}

static void update_psg(int chip, int port)
{
	rbx_ym_set_gain(chip, port, mono_gain(s_m.pan[2 + chip], s_m.vol[chip * 3 + port], 20));   // SSG routes 0.08
}

static void update_da(void)
{
	s_msm_gain = (s_def_vol[s_m.pan[4] & 0x0f] + s_def_vol[(s_m.pan[4] >> 4) & 0x0f]) * 256 / 100;
}

static void update_mixer(void)
{
	for (int c = 0; c < 2; c++) {
		update_fm(c);
		for (int p = 0; p < 3; p++)
			update_psg(c, p);
	}
	update_da();
}

static void port_a0(uint8_t d) { s_m.vol[0] = (uint8_t)s_def_vol[(d >> 4) & 15]; s_m.vol[6] = (uint8_t)s_def_vol[d & 15]; update_fm(0); update_psg(0, 0); }
static void port_a1(uint8_t d) { s_m.vol[3] = (uint8_t)s_def_vol[(d >> 4) & 15]; s_m.vol[7] = (uint8_t)s_def_vol[d & 15]; update_fm(1); update_psg(1, 0); }
static void port_b0(uint8_t d) { s_m.vol[1] = (uint8_t)s_def_vol[(d >> 4) & 15]; s_m.vol[2] = (uint8_t)s_def_vol[d & 15]; update_psg(0, 1); update_psg(0, 2); }
static void port_b1(uint8_t d) { s_m.vol[4] = (uint8_t)s_def_vol[(d >> 4) & 15]; s_m.vol[5] = (uint8_t)s_def_vol[d & 15]; update_psg(1, 1); update_psg(1, 2); }
static void ym_irq(int state) { s_m.ym_irq = (uint8_t)(state != 0); }

// run both YM2203s up to the sound Z80's position in the current line
static void snd_sync(void)
{
	uint32_t done = (uint32_t)(s_z80.cyc - s_line_start);
	uint32_t target = s_line_target ? (uint32_t)((uint64_t)s_ym_line * done / (uint32_t)s_line_target) : 0;
	if (target > s_ym_line)
		target = s_ym_line;
	if (target <= s_ym_done)
		return;
	int n = rbx_ym_advance(0, target - s_ym_done, s_mix + s_mix_n, s_mix_cap - s_mix_n);
	rbx_ym_advance(1, target - s_ym_done, s_mix + s_mix_n, s_mix_cap - s_mix_n);
	int32_t da = (msm5205_output(&s_msm) * s_msm_gain) >> 8;
	for (int i = 0; i < n; i++)
		s_mix[s_mix_n + i] += da;
	s_mix_n += n;
	s_ym_done = target;
}

static uint8_t snd_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x4000)
		return s_rom_snd[a];
	if (a < 0x8000) {
		uint32_t o = (uint32_t)s_m.bank * 0x4000 + (a - 0x4000);
		return o < s_rom_snd_size ? s_rom_snd[o] : 0xff;
	}
	if (a < 0x9000) return s_zram[a & 0xfff];
	if (a == 0x9000 || a == 0x9001) { snd_sync(); return rbx_ym_read(0, a & 1); }
	if (a == 0xa000 || a == 0xa001) { snd_sync(); return rbx_ym_read(1, a & 1); }
	if (a == 0xb001) return tc0140syt_slave_comm_r(&s_ciu);
	return 0xff;
}

static void snd_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0x8000 && a < 0x9000) { s_zram[a & 0xfff] = v; return; }
	switch (a) {
	case 0x9000: case 0x9001: snd_sync(); rbx_ym_write(0, a & 1, v); return;
	case 0xa000: case 0xa001: snd_sync(); rbx_ym_write(1, a & 1, v); return;
	case 0xb000: tc0140syt_slave_port_w(&s_ciu, v); return;
	case 0xb001: tc0140syt_slave_comm_w(&s_ciu, v); return;
	case 0xc000: snd_sync(); s_m.pan[0] = v; update_fm(0); return;
	case 0xc400: snd_sync(); s_m.pan[1] = v; update_fm(1); return;
	case 0xc800: snd_sync(); s_m.pan[2] = v; for (int p = 0; p < 3; p++) update_psg(0, p); return;
	case 0xcc00: snd_sync(); s_m.pan[3] = v; for (int p = 0; p < 3; p++) update_psg(1, p); return;
	case 0xd000: snd_sync(); s_m.pan[4] = v; update_da(); return;
	case 0xd400: s_m.adpcm_command = v; return;
	case 0xdc00: s_m.bank = v & 3; return;
	}
}

static uint8_t z80_in_none(z80 *z, uint8_t p) { (void)z; (void)p; return 0xff; }
static void z80_out_none(z80 *z, uint8_t p, uint8_t v) { (void)z; (void)p; (void)v; }

// ADPCM Z80: ROM only (no RAM), I/O ports 0-3
static uint8_t adpcm_read(void *u, uint16_t a) { (void)u; return a < s_rom_adpcm_size ? s_rom_adpcm[a] : 0xff; }
static void adpcm_write(void *u, uint16_t a, uint8_t v) { (void)u; (void)a; (void)v; }
static uint8_t adpcm_in(z80 *z, uint8_t p)
{
	(void)z;
	return (p & 0xff) == 0 ? s_m.adpcm_command : 0;   // 2, 3: readport2 / readport3 = 0
}
static void adpcm_out(z80 *z, uint8_t p, uint8_t v)
{
	(void)z;
	switch (p & 0xff) {
	case 0: s_m.nmi_enable = 0; break;
	case 1: s_m.nmi_enable = 1; break;
	case 2: msm5205_data_w(&s_msm, v); msm5205_reset_w(&s_msm, !(v & 0x20)); break;
	}
}

static void msm_vck(void)
{
	if (s_m.nmi_enable)
		z80_gen_nmi(&s_z80b);
}

static void z80_setup(z80 *z, int which)
{
	if (which == 0) {
		z->read_byte = snd_read;
		z->write_byte = snd_write;
		z->port_in = z80_in_none;
		z->port_out = z80_out_none;
	} else {
		z->read_byte = adpcm_read;
		z->write_byte = adpcm_write;
		z->port_in = adpcm_in;
		z->port_out = adpcm_out;
	}
	z->userdata = NULL;
}

static int s_snd_reset_line;
static void ciu_nmi(int state) { if (state) z80_gen_nmi(&s_z80); }
static void ciu_reset(int state)
{
	if (state && !s_snd_reset_line) {
		unsigned long cyc = s_z80.cyc;
		z80_init(&s_z80);
		z80_setup(&s_z80, 0);
		s_z80.cyc = cyc;
	}
	s_snd_reset_line = state;
}

static void sound_line(void)
{
	// MSM5205 sample clock (VCK -> NMI of the ADPCM Z80)
	msm5205_advance(&s_msm, (uint32_t)rbx_line_cycles(&s_msm_frac, MSM_CLOCK, 60000, LINES));
	// ADPCM Z80
	unsigned long end = s_z80b.cyc + (unsigned long)rbx_line_cycles(&s_z80b_frac, Z80_CLOCK, 60000, LINES);
	while ((long)(s_z80b.cyc - end) < 0) {
		if (s_z80b.halted && !s_z80b.nmi_pending && !s_z80b.iff_delay && !(s_z80b.int_pending && s_z80b.iff1)) {
			// HALT until the next NMI (it only comes from the MSM5205 between lines): the NOP cycles of
			// z80_step, 4 T-states and one R increment each
			unsigned long n = (end - s_z80b.cyc + 3) / 4;
			s_z80b.cyc += 4 * n;
			s_z80b.r = (uint8_t)((s_z80b.r & 0x80) | ((s_z80b.r + n) & 0x7f));
			break;
		}
		z80_step(&s_z80b);
	}
	// sound Z80 + YM2203s
	s_ym_line = (uint32_t)rbx_line_cycles(&s_ym_frac, YM_CLOCK, 60000, LINES);
	s_ym_done = 0;
	int cycles = rbx_line_cycles(&s_z80_frac, Z80_CLOCK, 60000, LINES);
	s_line_target = cycles;
	s_line_start = s_z80.cyc;
	if (s_snd_reset_line) {
		s_z80.cyc += (unsigned long)cycles;
	} else {
		end = s_z80.cyc + (unsigned long)cycles;
		while ((long)(s_z80.cyc - end) < 0) {
			s_z80.int_pending = s_m.ym_irq;
			z80_step(&s_z80);
		}
	}
	snd_sync();
}

// ---- 68000 buses ----
static uint16_t a_read16(uint32_t a)
{
	switch (a) {
	case 0xc00002: return tc0140syt_master_comm_r(&s_ciu);
	case 0xc00008: return (uint16_t)rbx_port_read(s_port[0]);
	case 0xc0000a: return (uint16_t)rbx_port_read(s_port[1]);
	case 0xc0000c: return (uint16_t)rbx_port_read(s_port[2]);
	case 0xc0000e: return s_m.coin_word;   // bits 3 and 4 coin lockouts
	case 0xc00010: return (uint16_t)rbx_port_read(s_port[3]);
	}
	return 0;
}

static uint8_t a_read8(uint32_t a)
{
	if (a == 0xc00003) return tc0140syt_master_comm_r(&s_ciu);
	return (uint8_t)(a_read16(a & ~1u) >> ((a & 1) ? 0 : 8));
}

static void parse_control(void)
{
	int reset = !(s_m.cpua_ctrl & 1);
	if (reset && !s_m.b_reset)
		s_m.b_needreset = 1;   // the reset is applied before CPU B runs again
	s_m.b_reset = (uint8_t)reset;
}

static void a_write16m(uint32_t a, uint16_t v, uint16_t mask)
{
	if (a == 0x0a0000) {   // cpua_ctrl_w
		uint16_t d = v & mask;
		if ((d & 0xff00) && ((d & 0xff) == 0))
			d = d >> 8;
		s_m.cpua_ctrl = d;
		parse_control();
		return;
	}
	if (a == 0xc00000 && (mask & 0xff)) { tc0140syt_master_port_w(&s_ciu, (uint8_t)v); return; }
	if (a == 0xc00002 && (mask & 0xff)) { tc0140syt_master_comm_w(&s_ciu, (uint8_t)v); return; }
	if (a == 0xc00060) { s_m.coin_word = (uint16_t)((s_m.coin_word & ~mask) | (v & mask)); return; }
	if (a >= 0xd20000 && a <= 0xd20003) { pc080sn_yscroll_w(s_sn, (a >> 1) & 1, v, mask); return; }
	if (a >= 0xd40000 && a <= 0xd40003) { pc080sn_xscroll_w(s_sn, (a >> 1) & 1, v, mask); return; }
	if (a >= 0xd50000 && a <= 0xd50003) { pc080sn_ctrl_w(s_sn, (a >> 1) & 1, v, mask); return; }
	// 0x0b0000 watchdog, 0xc00050 unknown
}

static void a_write16(uint32_t a, uint16_t v) { a_write16m(a, v, 0xffff); }
static void a_write8(uint32_t a, uint8_t v)
{
	if (a & 1)
		a_write16m(a & ~1u, v, 0x00ff);
	else
		a_write16m(a, (uint16_t)(v << 8), 0xff00);
}

static uint16_t b_read16(uint32_t a) { (void)a; return 0; }
static uint8_t b_read8(uint32_t a) { (void)a; return 0; }
static void b_write16(uint32_t a, uint16_t v) { (void)a; (void)v; }
static void b_write8(uint32_t a, uint8_t v) { (void)a; (void)v; }

// ---- video ----
static void draw_text(uint16_t *bm, int pitch)
{
	// scrolly -8: screen line VIS_Y0 + r shows tilemap line r, so tiles are aligned to the bitmap (one blit each)
	const uint16_t *fg = (const uint16_t *)s_snap->fg;
	for (int ty = 0; ty < VIS_H / 8; ty++)
		for (int col = 0; col < W / 8; col++) {
			int idx = ty * 128 + col;
			uint16_t code = fg[idx + 0x2000] & 0x7ff, attr = fg[idx];
			if (code >= s_txtgfx.count)
				code = (uint16_t)(rbx_wrap(code, s_txtgfx.count));
			if (!s_txtgfx.used[code])
				continue;
			int fx = (attr & 0x4000) ? 7 : 0, fy = (attr & 0x8000) ? 7 : 0;
			uint16_t base = (uint16_t)((attr & 0x7f) << 4);
			for (int y = 0; y < 8; y++) {
				const uint8_t *src = s_txtgfx.pix + code * 64 + ((y ^ fy) << 3);
				uint16_t *d = bm + (ty * 8 + y) * pitch + col * 8;
				for (int x = 0; x < 8; x++) {
					uint8_t p = src[x ^ fx];
					if (p)
						d[x] = (uint16_t)(base + p);
				}
			}
		}
}

static void draw_sprites(uint16_t *bm, int pitch)
{
	static const uint32_t primask[2] = { 0xcccc, 0 };   // GFX_PMASK_2: under the middle layer / over
	const uint16_t *spr = (const uint16_t *)(s_snap->spr + 0x100);
	for (int offs = 0; offs < 0xf00 / 2; offs += 4) {
		uint32_t code = spr[offs + 2] & 0x1fff;
		if (!code)
			continue;
		int sy = (256 - spr[offs]) & 0x1ff;
		int sx = spr[offs + 1] & 0x3ff;
		int flipx = (spr[offs + 2] >> 14) & 1, flipy = (spr[offs + 2] >> 15) & 1;
		int priority = (spr[offs + 3] & 0x80) >> 7;
		int color = spr[offs + 3] & 0x7f;
		int curx = sx, cury = sy - 8;
		if (curx > 900) curx -= 1024;
		if (cury > 400) cury -= 512;
		rbx_gfx_prio_transpen(&s_sprgfx, code, (uint16_t)(color << 4), flipx, flipy, curx, cury - VIS_Y0, bm, s_pri,
		                      pitch, W, VIS_H, primask[priority]);
	}
}

static void darius_render(void)
{
	const uint16_t *p = (const uint16_t *)s_snap->pal;
	for (int i = 0; i < 2048; i++) {   // xBGR_555
		int r = p[i] & 31, g = (p[i] >> 5) & 31, b = (p[i] >> 10) & 31;
		r = (r << 3) | (r >> 2); g = (g << 3) | (g >> 2); b = (b << 3) | (b >> 2);
		rbx_video.palette[i] = (uint32_t)(r << 16 | g << 8 | b);
	}
	uint16_t *bm = rbx_video.bitmap;
	int pitch = rbx_video.pitch;
	memset(s_pri, 0, (size_t)pitch * VIS_H);
	// MAME draws the bottom layer with priority 1, but the sprite masks (0xcccc, 0) never test bit 0 of the
	// priority value: leaving it 0 gives the same picture without writing the priority buffer
	pc080sn_draw(s_sn, &s_snap->sn, 0, bm, pitch, NULL, W, VIS_Y0, VIS_H, 1, 1);
	pc080sn_draw(s_sn, &s_snap->sn, 1, bm, pitch, s_pri, W, VIS_Y0, VIS_H, 0, 2);
	draw_sprites(bm, pitch);
	draw_text(bm, pitch);
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	s_snap->sn = s_sn->r;
	memcpy(s_snap->pal, s_pal, sizeof(s_pal));
	memcpy(s_snap->spr, s_spr, sizeof(s_spr));
	memcpy(s_snap->fg, s_fg, sizeof(s_fg));
	rbx_video_frame_done();
}

// ---- init / frame ----
static void darius_reset(void)
{
	// (the palette / video RAM too: a game that has not written it yet showed the last game's, as at start it is 0)
	memset(s_pal, 0, sizeof(s_pal));
	memset(s_ram_a, 0, sizeof(s_ram_a));
	memset(s_ram_b, 0, sizeof(s_ram_b));
	memset(s_ram_e10, 0, sizeof(s_ram_e10));
	memset(s_spr, 0, sizeof(s_spr));
	memset(s_share2, 0, sizeof(s_share2));
	memset(s_fg, 0, sizeof(s_fg));
	memset(s_zram, 0, sizeof(s_zram));
	pc080sn_reset(s_sn);
	memset(&s_m, 0, sizeof(s_m));
	s_m.cpua_ctrl = 0xff;
	for (int i = 0; i < 5; i++)
		s_m.pan[i] = 0x80;   // center, volumes minimum (0)
	tc0140syt_reset(&s_ciu);
	z80_init(&s_z80);
	z80_setup(&s_z80, 0);
	z80_init(&s_z80b);
	z80_setup(&s_z80b, 1);
	s_snd_reset_line = 0;
	rbx_ym_reset(0);
	rbx_ym_reset(1);
	msm5205_reset(&s_msm);
	update_mixer();
	m68k_multi_reset();
	s_cpu_frac = s_z80_frac = s_z80b_frac = s_ym_frac = s_msm_frac = 0;
	s_mix_n = 0;
}

static void darius_exit(void);

static int darius_init(const rbx_game *g)
{
	if (!s_fg_hp && !(s_fg_hp = calloc(1, sizeof(*s_fg_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	uint32_t sn_size, spr_size, txt_size;
	s_rom_a = rbx_region("maincpu", &s_rom_a_size);
	s_rom_b = rbx_region("cpub", &s_rom_b_size);
	s_rom_snd = rbx_region("audiocpu", &s_rom_snd_size);
	s_rom_adpcm = rbx_region("adpcm", &s_rom_adpcm_size);
	uint8_t *sn_gfx = rbx_region("pc080sn", &sn_size);
	uint8_t *spr_gfx = rbx_region("sprites", &spr_size);
	uint8_t *txt_gfx = rbx_region("text", &txt_size);
	if (!s_rom_a || !s_rom_b || !s_rom_snd || !s_rom_adpcm || !sn_gfx || !spr_gfx || !txt_gfx) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	m68k_bus_swap16(s_rom_a, s_rom_a_size);
	m68k_bus_swap16(s_rom_b, s_rom_b_size);
	for (int i = 0; i < 16; i++)
		s_def_vol[i] = (int)(100.0f / (float)pow(10.0f, (32.0f - (i * (32.0f / (float)(0xf)))) / 20.0f));
	static const uint32_t spr_planes[4] = { 0, 8, 16, 24 };
	static const uint32_t spr_x[16] = { 0, 1, 2, 3, 4, 5, 6, 7, 256, 257, 258, 259, 260, 261, 262, 263 };
	static const uint32_t spr_y[16] = { 0, 32, 64, 96, 128, 160, 192, 224, 512, 544, 576, 608, 640, 672, 704, 736 };
	static const rbx_gfx_layout spr_layout = { 16, 16, 4, spr_planes, spr_x, spr_y, 1024 };
	static const uint32_t txt_planes[2] = { 0, 8 };
	static const uint32_t txt_x[8] = { 0, 1, 2, 3, 4, 5, 6, 7 };
	static const uint32_t txt_y[8] = { 0, 16, 32, 48, 64, 80, 96, 112 };
	static const rbx_gfx_layout txt_layout = { 8, 8, 2, txt_planes, txt_x, txt_y, 128 };
	s_sn = (pc080sn_t *)calloc(1, sizeof(pc080sn_t));
	s_snap = calloc(1, sizeof(*s_snap));
	s_pri = (uint8_t *)malloc((size_t)W * VIS_H);
	if (!s_sn || !s_snap || !s_pri || pc080sn_init(s_sn, sn_gfx, sn_size, PC080SN_GFX_PACKED_MSB) ||
	    rbx_gfx_decode(&s_sprgfx, &spr_layout, spr_gfx, spr_size) || rbx_gfx_decode(&s_txtgfx, &txt_layout, txt_gfx, txt_size)) {
		rbx_fail("%s: out of memory (graphics)", g->set);
		darius_exit();
		return -1;
	}
	// set_offsets(-16, 8), dblwidth; screens drawn with tilemap_draw_offset, whose base scroll delta is
	// -16 - x_offset (= 0) rather than device_start's -16 + x_offset: x_offset 16 gives the same placement here
	s_sn->dblwidth = 1;
	s_sn->x_offset = 16;
	s_sn->y_offset = 8;
	s_sn->y_invert = 0;
	s_sn->screen_w = SCREEN_W;
	s_sn->screen_h = 256;
	for (int i = 0; i < 4; i++)
		s_port[i] = rbx_port_find((const char *const[]){ "P1", "P2", "SYSTEM", "DSW" }[i]);
	// sound
	rbx_ym_config yc;
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM2203;
	yc.clock = YM_CLOCK;
	yc.irq = ym_irq;
	yc.port_a_write = port_a0;
	yc.port_b_write = port_b0;
	if (rbx_ym_init(0, &yc)) goto snd_fail;
	yc.irq = NULL;
	yc.port_a_write = port_a1;
	yc.port_b_write = port_b1;
	if (rbx_ym_init(1, &yc)) goto snd_fail;
	// (the two YM2203s make their samples on the worker: a third of the emulation thread's time in play)
	if (rbx_ym_set_async(0, 1) || rbx_ym_set_async(1, 1)) goto snd_fail;
	msm5205_init(&s_msm, MSM_CLOCK, MSM5205_S48_4B, msm_vck);
	s_ciu.nmi = ciu_nmi;
	s_ciu.reset = ciu_reset;
	s_ciu.sync = NULL;
	s_mix_cap = (int)(rbx_ym_rate(0) / 50) + 64;
	s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	if (!s_mix || rbx_audio_setup(rbx_ym_rate(0)))
		goto snd_fail;
	if (rbx_video_setup(W, VIS_H, 2048)) {
		darius_exit();
		return -1;
	}
	// 68000s
	m68k_init();
	m68k_set_cpu_type(M68K_CPU_TYPE_68000);
	if (m68k_multi_init(2)) {
		darius_exit();
		return -1;
	}
	m68k_bus_reset_map();   // CPU A
	m68k_bus_map(0, 0x05ffff, s_rom_a, s_rom_a_size, 1, 0);
	m68k_bus_map(0x080000, 0x08ffff, s_ram_a, sizeof(s_ram_a), 1, 1);
	m68k_bus_map(0xd00000, 0xd0ffff, s_sn->r.ram, sizeof(s_sn->r.ram), 1, 1);
	m68k_bus_map(0xd80000, 0xd80fff, s_pal, sizeof(s_pal), 1, 1);
	m68k_bus_map(0xe00000, 0xe00fff, s_spr, sizeof(s_spr), 1, 1);
	m68k_bus_map(0xe01000, 0xe02fff, s_share2, sizeof(s_share2), 1, 1);
	m68k_bus_map(0xe08000, 0xe0ffff, s_fg, sizeof(s_fg), 1, 1);
	m68k_bus_map(0xe10000, 0xe10fff, s_ram_e10, sizeof(s_ram_e10), 1, 1);
	m68k_bus.read8 = a_read8;
	m68k_bus.read16 = a_read16;
	m68k_bus.write8 = a_write8;
	m68k_bus.write16 = a_write16;
	m68k_multi_capture(0);
	m68k_bus_reset_map();   // CPU B
	m68k_bus_map(0, 0x03ffff, s_rom_b, s_rom_b_size, 1, 0);
	m68k_bus_map(0x040000, 0x04ffff, s_ram_b, sizeof(s_ram_b), 1, 1);
	m68k_bus_map(0xd80000, 0xd80fff, s_pal, sizeof(s_pal), 1, 1);
	m68k_bus_map(0xe00000, 0xe00fff, s_spr, sizeof(s_spr), 1, 1);
	m68k_bus_map(0xe01000, 0xe02fff, s_share2, sizeof(s_share2), 1, 1);
	m68k_bus_map(0xe08000, 0xe0ffff, s_fg, sizeof(s_fg), 1, 1);
	m68k_bus.read8 = b_read8;
	m68k_bus.read16 = b_read16;
	m68k_bus.write8 = b_write8;
	m68k_bus.write16 = b_write16;
	m68k_multi_capture(1);
	m68k_multi_finish();
	darius_reset();
	return 0;
snd_fail:
	rbx_fail("%s: sound init failed", g->set);
	darius_exit();
	return -1;
}

static void darius_exit(void)
{
	if (s_sn)
		pc080sn_exit(s_sn);
	free(s_sn);
	free(s_snap);
	free(s_pri);
	free(s_mix);
	s_sn = NULL;
	s_snap = NULL;
	s_pri = NULL;
	s_mix = NULL;
	rbx_gfx_free(&s_sprgfx);
	rbx_gfx_free(&s_txtgfx);
	rbx_ym_exit(0);
	rbx_ym_exit(1);
	m68k_multi_exit();
	free(s_fg_hp);
	s_fg_hp = NULL;
}

static void darius_frame(void)
{
	for (int line = 0; line < LINES; line++) {
		if (line == VBLANK_LINE) {
			vblank_capture();
			m68k_multi_hold_irq(0, 4);
			m68k_multi_hold_irq(1, 4);
		}
		int cyc = rbx_line_cycles(&s_cpu_frac, CPU_CLOCK, 60000, LINES);
		m68k_multi_switch(0);
		m68k_execute(cyc);
		if (!s_m.b_reset) {
			if (s_m.b_needreset) {
				m68k_multi_reset_cpu(1);
				s_m.b_needreset = 0;
			}
			m68k_multi_switch(1);
			m68k_execute(cyc);
		}
		sound_line();
	}
	m68k_multi_switch(0);
	rbx_ym_async_take(0, s_mix, s_mix_n);   // (the YM2203s' samples, made on the worker)
	rbx_ym_async_take(1, s_mix, s_mix_n);
	for (int i = 0; i < s_mix_n; i++)
		rbx_audio_put(s_mix[i]);
	memset(s_mix, 0, sizeof(int32_t) * (size_t)s_mix_n);
	s_mix_n = 0;
}

static int darius_state(void *ctx, int load)
{
	z80 z[2] = { s_z80, s_z80b };
	uint64_t fr[5] = { s_cpu_frac, s_z80_frac, s_z80b_frac, s_ym_frac, s_msm_frac };
	int misc = s_snd_reset_line;
	int rc = m68k_multi_state(ctx, load);
	rc |= rbx_state_io(ctx, "ram_a", s_ram_a, sizeof(s_ram_a));
	rc |= rbx_state_io(ctx, "ram_b", s_ram_b, sizeof(s_ram_b));
	rc |= rbx_state_io(ctx, "ram_e10", s_ram_e10, sizeof(s_ram_e10));
	rc |= rbx_state_io(ctx, "pal", s_pal, sizeof(s_pal));
	rc |= rbx_state_io(ctx, "spr", s_spr, sizeof(s_spr));
	rc |= rbx_state_io(ctx, "share2", s_share2, sizeof(s_share2));
	rc |= rbx_state_io(ctx, "fg", s_fg, sizeof(s_fg));
	rc |= rbx_state_io(ctx, "pc080sn", &s_sn->r, sizeof(s_sn->r));
	rc |= rbx_state_io(ctx, "misc", &s_m, sizeof(s_m));
	rc |= rbx_state_io(ctx, "z80", z, sizeof(z));
	rc |= rbx_state_io(ctx, "zram", s_zram, sizeof(s_zram));
	rc |= rbx_state_io(ctx, "ciu", &s_ciu, offsetof(tc0140syt_t, nmi));
	rc |= rbx_state_io(ctx, "msm", &s_msm, offsetof(msm5205_t, vck));
	rc |= rbx_state_io(ctx, "frac", fr, sizeof(fr));
	rc |= rbx_state_io(ctx, "sreset", &misc, sizeof(misc));
	rc |= rbx_ym_state(0, ctx, load);
	rc |= rbx_ym_state(1, ctx, load);
	if (rc)
		return -1;
	if (load) {
		s_z80 = z[0];
		s_z80b = z[1];
		z80_setup(&s_z80, 0);
		z80_setup(&s_z80b, 1);
		s_msm.vck = msm_vck;
		s_cpu_frac = fr[0]; s_z80_frac = fr[1]; s_z80b_frac = fr[2]; s_ym_frac = fr[3]; s_msm_frac = fr[4];
		s_snd_reset_line = misc;
		update_mixer();
	}
	return 0;
}

static int darius_save(void *ctx) { return darius_state(ctx, 0); }
static int darius_load(void *ctx) { return darius_state(ctx, 1); }
static int darius_supported(const char *machine) { return !strcmp(machine, "darius"); }

const rbx_driver rbx_drv_darius = { "Darius", darius_init, darius_exit, darius_reset, darius_frame, darius_save, darius_load,
                                    darius_supported, darius_render };
