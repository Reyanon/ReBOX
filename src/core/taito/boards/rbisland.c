/*  Rainbow Islands, ported from MAME src/mame/taito/rbisland.cpp (GPL-2.0+, copyright-holders: Mike Coates).
    68000 + C-Chip (machine/taitocchip.c, the real uPD78C11 program) + PC080SN tilemaps + PC090OJ sprites;
    sound board: Z80 + YM2151 + PC060HA (machine/taito_sound.c, map 9000, banks from the YM CT pins).
    Jumping (bootleg, jumping_state): no C-Chip, its own sprite RAM / layout and two YM2203 behind a sound latch.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m68k_bus.h"
#include "../../cpu/m68k/m68k.h"
#include "../../cpu/z80/z80.h"
#include "../machine/taito_sound.h"
#include "../machine/taitocchip.h"
#include "../../sound/rbx_ym.h"
#include "../video/pc080sn.h"
#include "../video/pc090oj.h"

#include <stdlib.h>
#include <string.h>

#define LINES 256
#define SCREEN_W 320
#define VIS_Y0 16
#define VIS_Y1 239
#define VIS_H (VIS_Y1 - VIS_Y0 + 1)

static uint8_t *s_rom;
static uint32_t s_rom_size;
static uint8_t s_ram[0x4000];
static uint8_t s_pal[0x4000];   // 200000-200fff palette, 201000-203fff RAM
static pc080sn_t *s_sn;
static pc090oj_t *s_oj;
static struct {
	pc080sn_regs sn;
	pc090oj_regs oj;
	uint8_t pal[0x1000];
} *s_snap;
static uint8_t s_pri[SCREEN_W * VIS_H];
static int s_port_dswa, s_port_dswb, s_port_cc[4];   // 800007, 800009, 80000B, 80000D
static uint64_t s_cpu_frac, s_cc_frac;
static int s_jumping;   // jumping_state machine

// ---- C-Chip ports ----
static uint8_t cc_pa(void) { return (uint8_t)rbx_port_read(s_port_cc[0]); }
static uint8_t cc_pb(void) { return (uint8_t)rbx_port_read(s_port_cc[1]); }
static uint8_t cc_pc(void) { return (uint8_t)rbx_port_read(s_port_cc[2]); }
static uint8_t cc_ad(void) { return (uint8_t)rbx_port_read(s_port_cc[3]); }
static void cc_counters(uint8_t v) { (void)v; }   // coin lockout / counters

// ---- main CPU devices ----
static uint16_t main_read16(uint32_t a)
{
	if (a >= 0x800000 && a <= 0x8007ff) return cchip_mem68_r((a - 0x800000) >> 1);
	if (a >= 0x800800 && a <= 0x800fff) return cchip_asic_r((a - 0x800800) >> 1);
	if (a >= 0x390000 && a <= 0x390003) return (uint16_t)(rbx_port_read(s_port_dswa) & 0xff);
	if (a >= 0x3b0000 && a <= 0x3b0003) return (uint16_t)(rbx_port_read(s_port_dswb) & 0xff);
	if (a == 0x3e0002) return tc0140syt_master_comm_r(tsnd_syt());
	return 0;
}

static uint8_t main_read8(uint32_t a)
{
	if (a == 0x3e0003) return tc0140syt_master_comm_r(tsnd_syt());
	if (a >= 0x800000 && a <= 0x800fff && !(a & 1)) return 0;
	return (uint8_t)(main_read16(a & ~1u) >> ((a & 1) ? 0 : 8));
}

static void main_write16m(uint32_t a, uint16_t v, uint16_t mask)
{
	if (a >= 0x800000 && a <= 0x800fff) {
		if (mask & 0xff) {
			if (a <= 0x8007ff)
				cchip_mem68_w((a - 0x800000) >> 1, (uint8_t)v);
			else
				cchip_asic68_w((a - 0x800800) >> 1, (uint8_t)v);
		}
		return;
	}
	if (a >= 0xc20000 && a <= 0xc20003) { pc080sn_yscroll_w(s_sn, (a >> 1) & 1, v, mask); return; }
	if (a >= 0xc40000 && a <= 0xc40003) { pc080sn_xscroll_w(s_sn, (a >> 1) & 1, v, mask); return; }
	if (a >= 0xc50000 && a <= 0xc50003) { pc080sn_ctrl_w(s_sn, (a >> 1) & 1, v, mask); return; }
	if (a == 0x3a0000) { s_oj->sprite_ctrl = (uint16_t)((s_oj->sprite_ctrl & ~mask) | (v & mask)); return; }
	if (a == 0x3e0000 && (mask & 0xff)) { tc0140syt_master_port_w(tsnd_syt(), (uint8_t)v); return; }
	if (a == 0x3e0002 && (mask & 0xff)) { tc0140syt_master_comm_w(tsnd_syt(), (uint8_t)v); return; }
	// 0x3c0000: written very often, watchdog?
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
static void jumping_render(void);
static void rbisland_render(void)
{
	if (s_jumping) {
		jumping_render();
		return;
	}
	const uint16_t *p = (const uint16_t *)s_snap->pal;
	for (int i = 0; i < 2048; i++) {   // xBGR_555
		int r = p[i] & 31, g = (p[i] >> 5) & 31, b = (p[i] >> 10) & 31;
		r = (r << 3) | (r >> 2); g = (g << 3) | (g >> 2); b = (b << 3) | (b >> 2);
		rbx_video.palette[i] = (uint32_t)(r << 16 | g << 8 | b);
	}
	uint16_t *bm = rbx_video.bitmap;
	int pitch = rbx_video.pitch;
	pc080sn_draw(s_sn, &s_snap->sn, 0, bm, pitch, s_pri, SCREEN_W, VIS_Y0, VIS_H, 1, 1);
	pc080sn_draw(s_sn, &s_snap->sn, 1, bm, pitch, s_pri, SCREEN_W, VIS_Y0, VIS_H, 0, 2);
	// colpri_cb: bits 5-7 of the sprite control are the palette bank, sprites under the top layer
	pc090oj_draw(s_oj, &s_snap->oj, bm, pitch, s_pri, SCREEN_W, VIS_Y0, VIS_H, 1, (s_snap->oj.sprite_ctrl & 0xe0) >> 1, 0xfc);
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	s_snap->sn = s_sn->r;
	pc090oj_snapshot(s_oj, &s_snap->oj);
	memcpy(s_snap->pal, s_pal, sizeof(s_snap->pal));
	rbx_video_frame_done();
}

// ---- Jumping (bootleg): own sprite RAM and layout, two YM2203 behind a sound latch ----
#define JSPR_WORDS 0x400
static uint32_t s_jclock;            // main CPU clock
static uint8_t *s_jspr;              // decoded sprites, 256 bytes each
static uint32_t s_jspr_n;
static uint16_t s_jsprram[JSPR_WORDS];
static uint8_t s_jram_d0[0x2000];    // original sprite RAM location (attract mode)
static uint8_t s_jctrl;              // spritectrl_w
static struct { uint16_t spr[JSPR_WORDS]; uint8_t ctrl; } s_jsnap;
static z80 s_jz80;
static uint8_t *s_jarom;
static uint32_t s_jarom_size;
static uint8_t s_jaram[0x1000];
static uint8_t s_latch, s_latch_pending;
static int s_port_j[2];              // 401001, 401003
static uint64_t s_jz80_frac, s_jym_frac;
static int32_t *s_jmix;
static int s_jmix_cap, s_jmix_n;

static uint8_t jz80_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x8000 || a >= 0xc000) return a < s_jarom_size ? s_jarom[a] : 0xff;
	if (a < 0x9000) return s_jaram[a & 0xfff];
	if (a == 0xb000 || a == 0xb001) return rbx_ym_read(0, a & 1);
	if (a == 0xb400 || a == 0xb401) return rbx_ym_read(1, a & 1);
	if (a == 0xb800) { s_latch_pending = 0; return s_latch; }   // generic_latch_8 read clears the pending flag
	return 0xff;
}
static void jz80_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0x8000 && a < 0x9000) { s_jaram[a & 0xfff] = v; return; }
	if (a == 0xb000 || a == 0xb001) { rbx_ym_write(0, a & 1, v); return; }
	if (a == 0xb400 || a == 0xb401) { rbx_ym_write(1, a & 1, v); return; }
}
static uint8_t jz80_in(z80 *z, uint8_t p) { (void)z; (void)p; return 0xff; }
static void jz80_out(z80 *z, uint8_t p, uint8_t v) { (void)z; (void)p; (void)v; }
static void jz80_start(void)
{
	z80_init(&s_jz80);
	s_jz80.read_byte = jz80_read;
	s_jz80.write_byte = jz80_write;
	s_jz80.port_in = jz80_in;
	s_jz80.port_out = jz80_out;
}

static uint16_t jmain_read16(uint32_t a)
{
	if (a == 0x400000) return (uint16_t)(rbx_port_read(s_port_dswa) & 0xff);
	if (a == 0x400002) return (uint16_t)(rbx_port_read(s_port_dswb) & 0xff);
	if (a == 0x401000) return (uint16_t)(rbx_port_read(s_port_j[0]) & 0xff);
	if (a == 0x401002) return (uint16_t)(rbx_port_read(s_port_j[1]) & 0xff);
	if (a >= 0x440000 && a <= 0x4407ff) return s_jsprram[(a - 0x440000) >> 1];
	return 0;
}
static uint8_t jmain_read8(uint32_t a) { return (uint8_t)(jmain_read16(a & ~1u) >> ((a & 1) ? 0 : 8)); }
static void jmain_write16m(uint32_t a, uint16_t v, uint16_t mask)
{
	if (a >= 0x440000 && a <= 0x4407ff) {
		uint16_t *p = &s_jsprram[(a - 0x440000) >> 1];
		*p = (uint16_t)((*p & ~mask) | (v & mask));
		return;
	}
	if (a >= 0x430000 && a <= 0x430003) { pc080sn_yscroll_w(s_sn, (a >> 1) & 1, v, mask); return; }
	if (a >= 0xc40000 && a <= 0xc40003) { pc080sn_xscroll_w(s_sn, (a >> 1) & 1, v, mask); return; }
	if (a == 0x3a0000 && (mask & 0xff)) { s_jctrl = (uint8_t)v; return; }
	if (a == 0x400006 && (mask & 0xff)) { s_latch = (uint8_t)v; s_latch_pending = 1; return; }   // soundlatch
	// 0x3c0000 watchdog?, 0x800000 original C-Chip location, 0xc20000 leftover y scroll
}
static void jmain_write16(uint32_t a, uint16_t v) { jmain_write16m(a, v, 0xffff); }
static void jmain_write8(uint32_t a, uint8_t v)
{
	if (a & 1)
		jmain_write16m(a & ~1u, v, 0x00ff);
	else
		jmain_write16m(a, (uint16_t)(v << 8), 0xff00);
}

static void jumping_render(void)
{
	const uint16_t *p = (const uint16_t *)s_snap->pal;
	for (int i = 0; i < 2048; i++) {   // xBGR_444
		int r = p[i] & 15, g = (p[i] >> 4) & 15, b = (p[i] >> 8) & 15;
		rbx_video.palette[i] = (uint32_t)((r * 17) << 16 | (g * 17) << 8 | (b * 17));
	}
	uint16_t *bm = rbx_video.bitmap;
	int pitch = rbx_video.pitch;
	pc080sn_draw(s_sn, &s_snap->sn, 0, bm, pitch, NULL, SCREEN_W, VIS_Y0, VIS_H, 1, 0);
	// 128 sprites, transparent pen 15
	const int colbank = (s_jsnap.ctrl & 0xe0) >> 1;
	for (int offs = JSPR_WORDS - 8; offs >= 0; offs -= 8) {
		const uint16_t *s = &s_jsnap.spr[offs];
		uint32_t tile = s[0];
		if (tile >= s_jspr_n)
			continue;
		int sy = ((s[1] - 0xfff1) ^ 0xffff) & 0x1ff;
		if (sy > 400) sy -= 512;
		int sx = (s[2] - 0x38) & 0x1ff;
		if (sx > 400) sx -= 512;
		const int flipx = (s[3] & 0x40) != 0, flipy = (s[3] & 0x80) != 0;
		const uint16_t base = (uint16_t)((((s[4] & 0x0f) | colbank) % 128) * 16);
		const int y = sy + 1 - VIS_Y0;
		if (sx <= -16 || sx >= SCREEN_W || y <= -16 || y >= VIS_H)
			continue;
		const uint8_t *gfx = s_jspr + tile * 256;
		for (int r = 0; r < 16; r++) {
			if (y + r < 0 || y + r >= VIS_H)
				continue;
			const uint8_t *src = gfx + ((flipy ? 15 - r : r) << 4);
			uint16_t *dst = bm + (y + r) * pitch;
			for (int c = 0; c < 16; c++) {
				const int x = sx + c;
				const int pen = src[flipx ? 15 - c : c];
				if (pen != 15 && x >= 0 && x < SCREEN_W)
					dst[x] = (uint16_t)(base + pen);
			}
		}
	}
	pc080sn_draw(s_sn, &s_snap->sn, 1, bm, pitch, NULL, SCREEN_W, VIS_Y0, VIS_H, 0, 0);
}

static void jumping_capture(void)
{
	rbx_video_wait_idle();
	s_snap->sn = s_sn->r;
	memcpy(s_jsnap.spr, s_jsprram, sizeof(s_jsprram));
	s_jsnap.ctrl = s_jctrl;
	memcpy(s_snap->pal, s_pal, sizeof(s_snap->pal));
	rbx_video_frame_done();
}

static void jumping_reset(void)
{
	memset(s_ram, 0, sizeof(s_ram));
	memset(s_jsprram, 0, sizeof(s_jsprram));
	memset(s_jaram, 0, sizeof(s_jaram));
	pc080sn_reset(s_sn);
	s_jctrl = 0;
	s_latch = s_latch_pending = 0;
	jz80_start();
	rbx_ym_reset(0);
	rbx_ym_reset(1);
	m68k_bus_irq_reset();
	m68k_pulse_reset();
	s_cpu_frac = s_jz80_frac = s_jym_frac = 0;
	s_jmix_n = 0;
}

static int jumping_init(const rbx_game *g)
{
	uint32_t sn_size, spr_size;
	uint8_t *sn_gfx = rbx_region("pc080sn", &sn_size);
	uint8_t *spr = rbx_region("sprites", &spr_size);
	s_jarom = rbx_region("audiocpu", &s_jarom_size);
	if (!sn_gfx || !spr || !s_jarom) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	s_jclock = !strcmp(g->machine, "jumpingi") ? 8000000 : 9216000;
	s_sn = (pc080sn_t *)calloc(1, sizeof(pc080sn_t));
	s_snap = calloc(1, sizeof(*s_snap));
	s_jspr_n = spr_size / 128;
	s_jspr = (uint8_t *)malloc((size_t)s_jspr_n * 256);
	if (!s_sn || !s_snap || !s_jspr || pc080sn_init(s_sn, sn_gfx, sn_size, PC080SN_GFX_PLANAR)) {
		rbx_fail("%s: out of memory (graphics)", g->set);
		return -1;
	}
	// jumping_spritelayout: planes at bits 0, 8, 16, 24 of a 32-bit row, right half 64 bytes on
	for (uint32_t t = 0; t < s_jspr_n; t++)
		for (int y = 0; y < 16; y++)
			for (int x = 0; x < 16; x++) {
				const uint8_t *row = spr + t * 128 + (x < 8 ? 0 : 64) + y * 4;
				int pix = 0;
				for (int pl = 0; pl < 4; pl++)
					pix |= ((row[pl] >> (7 - (x & 7))) & 1) << (3 - pl);
				s_jspr[t * 256 + y * 16 + x] = (uint8_t)(pix ^ 15);   // ROMREGION_INVERT
			}
	s_sn->y_invert = 1;           // set_yinvert(1)
	s_sn->trans_pen[1] = 15;      // set_trans_pen(1, 15)
	s_sn->fixed[1] = 1;           // set_scroll(1, 16, 0) every frame
	s_sn->fixed_x[1] = 16;
	s_sn->fixed_y[1] = 0;
	s_port_j[0] = rbx_port_find("401001");
	s_port_j[1] = rbx_port_find("401003");
	// sound: two YM2203 at 3 MHz, all outputs 0.30
	for (int c = 0; c < 2; c++) {
		rbx_ym_config yc;
		memset(&yc, 0, sizeof(yc));
		yc.type = RBX_YM2203;
		yc.clock = 3000000;
		yc.gain[0] = yc.gain[1] = yc.gain[2] = yc.gain[3] = 77;
		if (rbx_ym_init(c, &yc)) {
			rbx_fail("%s: sound init failed", g->set);
			return -1;
		}
	}
	s_jmix_cap = (int)(rbx_ym_rate(0) / 50) + 64;
	s_jmix = (int32_t *)calloc((size_t)s_jmix_cap, sizeof(int32_t));
	if (!s_jmix || rbx_audio_setup(rbx_ym_rate(0)) || rbx_video_setup(SCREEN_W, VIS_H, 2048))
		return -1;
	m68k_init();
	m68k_set_cpu_type(M68K_CPU_TYPE_68000);
	m68k_bus_reset_map();
	m68k_bus_map(0, 0x09ffff, s_rom, s_rom_size, 1, 0);
	m68k_bus_map(0x10c000, 0x10ffff, s_ram, sizeof(s_ram), 1, 1);
	m68k_bus_map(0x200000, 0x203fff, s_pal, sizeof(s_pal), 1, 1);
	m68k_bus_map(0xc00000, 0xc0ffff, s_sn->r.ram, sizeof(s_sn->r.ram), 1, 1);
	m68k_bus_map(0xd00000, 0xd01fff, s_jram_d0, sizeof(s_jram_d0), 1, 1);
	m68k_bus.read8 = jmain_read8;
	m68k_bus.read16 = jmain_read16;
	m68k_bus.write8 = jmain_write8;
	m68k_bus.write16 = jmain_write16;
	jumping_reset();
	return 0;
}

static void jumping_frame(void)
{
	for (int line = 0; line < LINES; line++) {
		if (line == VIS_Y1 + 1) {
			jumping_capture();
			m68k_bus_hold_irq(4);
		}
		m68k_execute(rbx_line_cycles(&s_cpu_frac, s_jclock, 60000, LINES));
		unsigned long end = s_jz80.cyc + (unsigned long)rbx_line_cycles(&s_jz80_frac, 6000000, 60000, LINES);
		while ((long)(s_jz80.cyc - end) < 0) {
			s_jz80.int_pending = s_latch_pending ? 1 : 0;   // data_pending -> IRQ 0
			z80_step(&s_jz80);
		}
		uint32_t clocks = (uint32_t)rbx_line_cycles(&s_jym_frac, 3000000, 60000, LINES);
		int n = rbx_ym_advance(0, clocks, s_jmix + s_jmix_n, s_jmix_cap - s_jmix_n);
		rbx_ym_advance(1, clocks, s_jmix + s_jmix_n, s_jmix_cap - s_jmix_n);
		s_jmix_n += n;
	}
	for (int i = 0; i < s_jmix_n; i++)
		rbx_audio_put(s_jmix[i]);
	memset(s_jmix, 0, sizeof(int32_t) * (size_t)s_jmix_n);
	s_jmix_n = 0;
}

static int jumping_state(void *ctx, int load)
{
	z80 z = s_jz80;
	uint8_t misc[4] = { s_jctrl, s_latch, s_latch_pending, 0 };
	uint64_t fr[2] = { s_jz80_frac, s_jym_frac };
	int rc = 0;
	rc |= rbx_state_io(ctx, "jspr", s_jsprram, sizeof(s_jsprram));
	rc |= rbx_state_io(ctx, "jd0", s_jram_d0, sizeof(s_jram_d0));
	rc |= rbx_state_io(ctx, "jz80", &z, sizeof(z));
	rc |= rbx_state_io(ctx, "jzram", s_jaram, sizeof(s_jaram));
	rc |= rbx_state_io(ctx, "jmisc", misc, sizeof(misc));
	rc |= rbx_state_io(ctx, "jfrac", fr, sizeof(fr));
	rc |= rbx_ym_state(0, ctx, load);
	rc |= rbx_ym_state(1, ctx, load);
	if (rc)
		return -1;
	if (load) {
		z.read_byte = jz80_read;
		z.write_byte = jz80_write;
		z.port_in = jz80_in;
		z.port_out = jz80_out;
		z.userdata = NULL;
		s_jz80 = z;
		s_jctrl = misc[0]; s_latch = misc[1]; s_latch_pending = misc[2];
		s_jz80_frac = fr[0]; s_jym_frac = fr[1];
	}
	return 0;
}

// ---- init / frame ----
static void rbisland_reset(void)
{
	// (the palette / video RAM too: a game that has not written it yet showed the last game's, as at start it is 0)
	memset(s_pal, 0, sizeof(s_pal));
	memset(s_jram_d0, 0, sizeof(s_jram_d0));
	if (s_jumping) {
		jumping_reset();
		return;
	}
	memset(s_ram, 0, sizeof(s_ram));
	pc080sn_reset(s_sn);
	pc090oj_reset(s_oj);
	s_oj->sprite_ctrl = 0;
	cchip_reset();
	m68k_bus_irq_reset();
	m68k_pulse_reset();
	tsnd_reset();
	s_cpu_frac = s_cc_frac = 0;
}

static void rbisland_exit(void);

static int rbisland_init(const rbx_game *g)
{
	uint32_t sn_size, oj_size;
	s_rom = rbx_region("maincpu", &s_rom_size);
	if (!s_rom) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	m68k_bus_swap16(s_rom, s_rom_size);
	s_port_dswa = rbx_port_find("DSWA");
	s_port_dswb = rbx_port_find("DSWB");
	s_jumping = strncmp(g->machine, "jumping", 7) == 0;
	if (s_jumping) {
		if (jumping_init(g)) {
			rbisland_exit();
			return -1;
		}
		return 0;
	}
	uint8_t *sn_gfx = rbx_region("pc080sn", &sn_size);
	uint8_t *oj_gfx = rbx_region("pc090oj", &oj_size);
	if (!sn_gfx || !oj_gfx) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	s_sn = (pc080sn_t *)calloc(1, sizeof(pc080sn_t));
	s_oj = (pc090oj_t *)calloc(1, sizeof(pc090oj_t));
	s_snap = calloc(1, sizeof(*s_snap));
	if (!s_sn || !s_oj || !s_snap || pc080sn_init(s_sn, sn_gfx, sn_size, PC080SN_GFX_PACKED_MSB) ||
	    pc090oj_init(s_oj, oj_gfx, oj_size, 2048)) {
		rbx_fail("%s: out of memory (graphics)", g->set);
		rbisland_exit();
		return -1;
	}
	s_oj->direct = 1;
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
		rbisland_exit();
		return -1;
	}
	// sound: YM2151 routes 0.50 + 0.50 to mono; ROM banks at 0xc000 of the audio region
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
	sc.bank_base = 0xc000;
	if (tsnd_init(&sc)) {
		rbx_fail("%s: sound init failed", g->set);
		rbisland_exit();
		return -1;
	}
	if (rbx_video_setup(SCREEN_W, VIS_H, 2048)) {
		rbisland_exit();
		return -1;
	}
	m68k_init();
	m68k_set_cpu_type(M68K_CPU_TYPE_68000);
	m68k_bus_reset_map();
	m68k_bus_map(0, 0x07ffff, s_rom, s_rom_size, 1, 0);
	m68k_bus_map(0x10c000, 0x10ffff, s_ram, sizeof(s_ram), 1, 1);
	m68k_bus_map(0x200000, 0x203fff, s_pal, sizeof(s_pal), 1, 1);
	m68k_bus_map(0xc00000, 0xc0ffff, s_sn->r.ram, sizeof(s_sn->r.ram), 1, 1);
	m68k_bus_map(0xd00000, 0xd03fff, s_oj->ram, sizeof(s_oj->ram), 1, 1);
	m68k_bus.read8 = main_read8;
	m68k_bus.read16 = main_read16;
	m68k_bus.write8 = main_write8;
	m68k_bus.write16 = main_write16;
	rbisland_reset();
	return 0;
}

static void rbisland_exit(void)
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
	if (s_jumping) {
		free(s_jspr);
		free(s_jmix);
		s_jspr = NULL;
		s_jmix = NULL;
		rbx_ym_exit(0);
		rbx_ym_exit(1);
	} else {
		tsnd_exit();
	}
}

static void rbisland_frame(void)
{
	if (s_jumping) {
		jumping_frame();
		return;
	}
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

static int rbisland_state(void *ctx, int load)
{
	uint8_t cpu[1024];
	uint32_t csize = m68k_context_size();
	if (csize > sizeof(cpu))
		return -1;
	if (!load)
		m68k_get_context(cpu);
	uint32_t irq = m68k_bus_irq_pending();
	uint64_t fr[2] = { s_cpu_frac, s_cc_frac };
	int rc = 0;
	rc |= rbx_state_io(ctx, "m68k", cpu, csize);
	rc |= rbx_state_io(ctx, "irq", &irq, 4);
	rc |= rbx_state_io(ctx, "ram", s_ram, sizeof(s_ram));
	rc |= rbx_state_io(ctx, "pal", s_pal, sizeof(s_pal));
	rc |= rbx_state_io(ctx, "pc080sn", &s_sn->r, sizeof(s_sn->r));
	if (!s_jumping) {
		rc |= rbx_state_io(ctx, "pc090oj", s_oj->ram, sizeof(s_oj->ram));
		rc |= rbx_state_io(ctx, "ojctrl", &s_oj->sprite_ctrl, sizeof(s_oj->sprite_ctrl));
	}
	rc |= rbx_state_io(ctx, "frac", fr, sizeof(fr));
	if (s_jumping)
		rc |= jumping_state(ctx, load);
	else
		rc |= cchip_state(ctx, load) | tsnd_state(ctx, load);
	if (rc)
		return -1;
	if (load) {
		m68k_set_context(cpu);
		m68k_bus_set_irq_pending(irq);
		s_cpu_frac = fr[0];
		s_cc_frac = fr[1];
	}
	return 0;
}

static int rbisland_save(void *ctx) { return rbisland_state(ctx, 0); }
static int rbisland_load(void *ctx) { return rbisland_state(ctx, 1); }
static int rbisland_supported(const char *machine)
{
	return !strcmp(machine, "rbisland") || !strcmp(machine, "jumping") || !strcmp(machine, "jumpingi");
}

const rbx_driver rbx_drv_rbisland = { "Rainbow Islands", rbisland_init, rbisland_exit, rbisland_reset, rbisland_frame,
                                      rbisland_save, rbisland_load, rbisland_supported, rbisland_render };
