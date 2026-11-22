/*  Goal! '92 (a bootleg of Seibu Cup Soccer): a 68000 with two 16x16 tilemaps, an 8x8 text layer and 16x16 sprites
    (buffered) in four priorities, a Z80 sound CPU with two YM2203s and an MSM5205 fed by the Z80 (banked ROM);
    2048 colors (xBGR_555 RAM).
    Ported from MAME 0.289 src/mame/seibu/goal92.cpp (BSD-3-Clause, copyright-holders: Pierpaolo Prazzoli, David
    Haywood).
    This file is part of ReBOX. GPL-2.0-or-later.

    The CPUs interleaved a scanline at a time.
*/
#include "../../rbx.h"
#include "../../cpu/m68k_bus.h"
#include "../../cpu/m68k/m68k.h"
#include "../../cpu/z80/z80.h"
#include "../../sound/rbx_ym.h"
#include "../../sound/msm5205.h"
#include "../../video/gfxlayout.h"
#include "../machine/smix.h"
#include "../video/seibu_draw.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define FRAME_MHZ 60000
#define LINES 256
#define VIS_Y0 8
#define VIS_W 320
#define VIS_H 240
#define CPU_CLOCK 12000000
#define SND_CLOCK 2500000
#define YM_CLOCK 1250000
#define MSM_CLOCK 400000
#define NPENS 2049
#define PEN_BLACK 2048

static uint8_t *s_rom, *s_zrom;
static uint32_t s_rom_size, s_zrom_size;
static rbx_gfx s_gspr, s_gtx, s_gtile;
static int s_port[5];
static z80 s_snd;
static msm5205_t s_msm;
static smix_t *s_mix;
// The YM2203s' samples are made on the rbx_aux worker (rbx_ym_set_async): a line notes where its slice is and how many
// samples each chip made; at the frame's end they are taken and stretched into the same slices (the same sums as
// adding them line by line)
#define YM_BUF 8192
typedef struct { int16_t off, slice, cnt[2]; } ym_line_t;
static ym_line_t *s_ym_line;   // [LINES]
static int32_t *s_ym_buf;      // [YM_BUF]

typedef struct {
	uint8_t ram[0x40000];        // 0x100000-0x13ffff (bg 0x800, fg 0x1000, text 0x2000, palette 0x3000)
	uint8_t spr[0x800];          // 0x140000
	uint16_t spritebuf[0x400];
	uint16_t scroll[4], fg_bank;
	uint8_t zram[0x800];
	uint8_t latch, latch_pending, sound_bank, msm5205next, adpcm_toggle;
	uint64_t main_frac, snd_frac, ym_frac, msm_frac;
	uint8_t state_end;
} board_t;
static board_t *s_bp;   // (calloc at the init: ADR-040)
#define b (*s_bp)

typedef struct {
	uint16_t bg[0x400], fg[0x400], tx[0x800], spr[0x400], scroll[4], fg_bank;
	uint32_t pal[NPENS];
} snap_t;
static snap_t *s_snapp;
#define s_snap (*s_snapp)

#define RAM16(off) ((const uint16_t *)(b.ram + (off)))

// ---- the sound CPU ----
static uint8_t snd_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x8000) return s_zrom[a];
	if (a < 0xc000) return s_zrom[0x8000 + (uint32_t)b.sound_bank * 0x4000 + (a - 0x8000)];
	if (a == 0xe800 || a == 0xe801) return rbx_ym_read(0, a & 1);
	if (a == 0xec00 || a == 0xec01) return rbx_ym_read(1, a & 1);
	if (a >= 0xf000 && a < 0xf800) return b.zram[a - 0xf000];
	if (a == 0xf800) {
		b.latch_pending = 0;
		return b.latch;
	}
	return 0xff;
}

static void snd_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a == 0xe000) {   // adpcm_control_w
		b.sound_bank = v & 1;
		msm5205_reset_w(&s_msm, (v >> 3) & 1);
	} else if (a == 0xe400)
		b.msm5205next = v;
	else if (a == 0xe800 || a == 0xe801)
		rbx_ym_write(0, a & 1, v);
	else if (a == 0xec00 || a == 0xec01)
		rbx_ym_write(1, a & 1, v);
	else if (a >= 0xf000 && a < 0xf800)
		b.zram[a - 0xf000] = v;
}

static uint8_t z_in(z80 *z, uint8_t port) { (void)z; (void)port; return 0xff; }
static void z_out(z80 *z, uint8_t port, uint8_t v) { (void)z; (void)port; (void)v; }

static void adpcm_int(void)
{
	msm5205_data_w(&s_msm, b.msm5205next);
	b.msm5205next >>= 4;
	b.adpcm_toggle ^= 1;
	if (b.adpcm_toggle)
		z80_gen_nmi(&s_snd);
}

// ---- the main CPU's devices ----
static uint16_t r16(uint32_t a)
{
	a &= 0xffffff;
	switch (a) {
	case 0x180000: return (uint16_t)rbx_port_read(s_port[0]);
	case 0x180002: return (uint16_t)rbx_port_read(s_port[1]);
	case 0x180004: return (uint16_t)rbx_port_read(s_port[2]);
	case 0x180006: return (uint16_t)rbx_port_read(s_port[3]);
	case 0x18000e: return (uint16_t)rbx_port_read(s_port[4]);
	case 0x18001c: return b.fg_bank;
	default: return 0;
	}
}

static uint8_t r8(uint32_t a)
{
	const uint16_t v = r16(a & ~1u);
	return (uint8_t)((a & 1) ? v : v >> 8);
}

static void w16m(uint32_t a, uint16_t v, uint16_t mask)
{
	a &= 0xffffff;
	if (a == 0x180008 && (mask & 0xff00)) {   // the sound latch (the high byte)
		b.latch = (uint8_t)(v >> 8);
		b.latch_pending = 1;
	} else if (a >= 0x180010 && a < 0x180018) {
		uint16_t *p = &b.scroll[(a - 0x180010) >> 1];
		*p = (uint16_t)((*p & ~mask) | (v & mask));
	} else if (a == 0x18001c)
		b.fg_bank = (uint16_t)((b.fg_bank & ~mask) | (v & mask));
}

static void w16(uint32_t a, uint16_t v) { w16m(a, v, 0xffff); }
static void w8(uint32_t a, uint8_t v)
{
	if (a & 1)
		w16m(a & ~1u, v, 0x00ff);
	else
		w16m(a, (uint16_t)(v << 8), 0xff00);
}

// ---- the drawing ----
static void tile_layer(const uint16_t *ram, int cols, int rows, int tsz, const rbx_gfx *g, uint32_t bank, uint16_t cbase,
                       int sx, int sy)
{
	const int mw = cols * tsz, mh = rows * tsz, tshift = tsz == 16 ? 4 : 3;   // (both powers of two)
	for (int r = 0; r < VIS_H; r++)
		sd_tile_row(rbx_video.bitmap + r * rbx_video.pitch, NULL, 0, ram, cols, tshift, mw - 1, (r + VIS_Y0 + sy) & (mh - 1),
		            sx, g, bank, 0, cbase, 15, VIS_W);
}

static void draw_sprites(int pri)
{
	const sd_target tg = { rbx_video.bitmap, rbx_video.pitch, 0, VIS_Y0, VIS_W, VIS_H, NULL };
	const uint16_t *s = s_snap.spr;
	for (int offs = 3; offs <= 0x400 - 5; offs += 4) {
		const uint16_t data = s[offs + 2];
		int y = s[offs];
		if (y & 0x8000)
			break;
		if (!(data & 0x8000))
			continue;
		const int sprite = s[offs + 1];
		if ((sprite >> 14) != pri)
			continue;
		int x = s[offs + 3] & 0x1ff;
		y &= 0x1ff;
		x -= 320 / 4 - 16 - 1;
		y = 256 - (y + 7);
		sd_transpen(&tg, &s_gspr, (uint32_t)(sprite & 0x1fff), (uint32_t)((data & 0x3f) + 0x40) * 16, (data >> 14) & 1, 0, x,
		            y, 15);
	}
}

static void gl_render(void)
{
	memcpy(rbx_video.palette, s_snap.pal, sizeof(s_snap.pal));
	const uint16_t *sc = s_snap.scroll;
	const int bank = s_snap.fg_bank & 0xff;
	for (int r = 0; r < VIS_H; r++) {
		uint16_t *d = rbx_video.bitmap + r * rbx_video.pitch;
		for (int x = 0; x < VIS_W; x++)
			d[x] = PEN_BLACK;
	}
	tile_layer(s_snap.bg, 32, 32, 16, &s_gtile, 0, 0, sc[0] + 60, sc[1] + 8);
	draw_sprites(2);
	if (!bank)
		draw_sprites(1);
	if (bank)
		tile_layer(s_snap.fg, 32, 32, 16, &s_gtile, 0x1000, 16 * 16, sc[0] + 60, sc[1] + 8);
	else
		tile_layer(s_snap.fg, 32, 32, 16, &s_gtile, 0x2000, 32 * 16, sc[2] + 60, sc[3] + 8);
	if (bank)
		draw_sprites(1);
	draw_sprites(0);
	draw_sprites(3);
	tile_layer(s_snap.tx, 64, 32, 8, &s_gtx, 0xc000, 48 * 16, 0, 0);
}

static void capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap.bg, b.ram + 0x800, sizeof(s_snap.bg));
	memcpy(s_snap.fg, b.ram + 0x1000, sizeof(s_snap.fg));
	memcpy(s_snap.tx, b.ram + 0x2000, sizeof(s_snap.tx));
	memcpy(s_snap.spr, b.spritebuf, sizeof(s_snap.spr));
	memcpy(s_snap.scroll, b.scroll, sizeof(s_snap.scroll));
	s_snap.fg_bank = b.fg_bank;
	const uint16_t *pal = RAM16(0x3000);
	for (int i = 0; i < 2048; i++) {   // xBGR_555
		const uint16_t w = pal[i];
		const uint32_t r = w & 31, g = (w >> 5) & 31, bl = (w >> 10) & 31;
		s_snap.pal[i] = (r << 3 | r >> 2) << 16 | (g << 3 | g >> 2) << 8 | (bl << 3 | bl >> 2);
	}
	s_snap.pal[PEN_BLACK] = 0;
	rbx_video_frame_done();
}

// ---- init / frame ----
static void gl_reset(void)
{
	rbx_video_wait_idle();
	memset(&b, 0, offsetof(__typeof__(b), state_end));
	m68k_bus_irq_reset();
	m68k_pulse_reset();
	z80_init(&s_snd);
	s_snd.read_byte = snd_read;
	s_snd.write_byte = snd_write;
	s_snd.port_in = z_in;
	s_snd.port_out = z_out;
	rbx_ym_reset(0);
	rbx_ym_reset(1);
	msm5205_reset(&s_msm);
	s_mix->n = 0;
	s_mix->frac = 0;
}

static void gl_exit(void)
{
	rbx_video_wait_idle();
	rbx_ym_exit(0);
	rbx_ym_exit(1);
	rbx_gfx_free(&s_gspr);
	rbx_gfx_free(&s_gtx);
	rbx_gfx_free(&s_gtile);
	free(s_bp);
	s_bp = NULL;
	free(s_snapp);
	s_snapp = NULL;
	free(s_mix);
	s_mix = NULL;
	free(s_ym_line);
	s_ym_line = NULL;
	free(s_ym_buf);
	s_ym_buf = NULL;
}

static int gl_init(const rbx_game *g)
{
	uint32_t ss, ts;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_zrom = rbx_region("audiocpu", &s_zrom_size);
	const uint8_t *sp = rbx_region("sprites", &ss), *tl = rbx_region("tiles", &ts);
	if (!s_rom || !s_zrom || s_zrom_size < 0x10000 || !sp || !tl) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	s_bp = calloc(1, sizeof(board_t));
	s_snapp = calloc(1, sizeof(snap_t));
	s_mix = calloc(1, sizeof(*s_mix));
	s_ym_line = calloc(LINES, sizeof(*s_ym_line));
	s_ym_buf = calloc(YM_BUF, sizeof(*s_ym_buf));
	if (!s_bp || !s_snapp || !s_mix || !s_ym_line || !s_ym_buf) {
		rbx_fail("%s: out of memory", g->set);
		gl_exit();
		return -1;
	}
	m68k_bus_swap16(s_rom, s_rom_size);
	{   // layout_16x16x4, gfx_8x8x4_planar (RGN_FRAC(1,4) planes)
		static const uint32_t x16[16] = { 0, 1, 2, 3, 4, 5, 6, 7, 128, 129, 130, 131, 132, 133, 134, 135 };
		static const uint32_t y16[16] = { 0, 8, 16, 24, 32, 40, 48, 56, 64, 72, 80, 88, 96, 104, 112, 120 };
		static const uint32_t x8[8] = { 0, 1, 2, 3, 4, 5, 6, 7 }, y8[8] = { 0, 8, 16, 24, 32, 40, 48, 56 };
		const uint32_t qs = ss / 4 * 8, qt = ts / 4 * 8;
		const uint32_t ps[4] = { 3 * qs, 2 * qs, qs, 0 }, pt[4] = { 3 * qt, 2 * qt, qt, 0 };
		const rbx_gfx_layout ls = { 16, 16, 4, ps, x16, y16, 256 }, lt = { 16, 16, 4, pt, x16, y16, 256 };
		const rbx_gfx_layout lc = { 8, 8, 4, pt, x8, y8, 64 };
		if (rbx_gfx_decode(&s_gspr, &ls, sp, ss / 4) || rbx_gfx_decode(&s_gtile, &lt, tl, ts / 4) ||
		    rbx_gfx_decode(&s_gtx, &lc, tl, ts / 4)) {
			rbx_fail("%s: out of memory (graphics)", g->set);
			gl_exit();
			return -1;
		}
	}
	rbx_ym_config yc;
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM2203;
	yc.clock = YM_CLOCK;
	yc.gain[0] = yc.gain[1] = yc.gain[2] = yc.gain[3] = 64;   // (0.25)
	if (rbx_ym_init(0, &yc) || rbx_ym_set_async(0, 1) || rbx_ym_init(1, &yc) || rbx_ym_set_async(1, 1) ||
	    rbx_audio_setup(48000) || rbx_video_setup(VIS_W, VIS_H, NPENS)) {
		rbx_fail("%s: out of memory", g->set);
		gl_exit();
		return -1;
	}
	smix_init(s_mix, 48000, FRAME_MHZ, LINES);
	msm5205_init(&s_msm, MSM_CLOCK, MSM5205_S96_4B, adpcm_int);
	static const char *const names[5] = { "DSW1", "IN1", "IN2", "IN3", "DSW2" };
	for (int i = 0; i < 5; i++)
		s_port[i] = rbx_port_find(names[i]);
	m68k_bus_reset_map();
	m68k_bus.read8 = r8;
	m68k_bus.read16 = r16;
	m68k_bus.write8 = w8;
	m68k_bus.write16 = w16;
	m68k_bus_map(0x000000, 0x0fffff, s_rom, s_rom_size, 1, 0);
	m68k_bus_map(0x100000, 0x13ffff, b.ram, sizeof(b.ram), 1, 1);
	m68k_bus_map(0x140000, 0x1407ff, b.spr, sizeof(b.spr), 1, 1);
	m68k_init();
	m68k_set_cpu_type(M68K_CPU_TYPE_68000);
	m68k_set_mame_timing(1);
	gl_reset();
	return 0;
}

static void gl_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	for (int line = 0; line < LINES; line++) {
		if (line == VIS_Y0 + VIS_H) {   // vblank: the sprite buffer, IRQ 6
			memcpy(b.spritebuf, b.spr, sizeof(b.spritebuf));
			capture();
			m68k_bus_hold_irq(6);
		}
		m68k_execute(rbx_line_cycles(&b.main_frac, CPU_CLOCK, FRAME_MHZ, LINES));
		int irq = b.latch_pending;   // (the latch's data pending: IRQ 0)
		s_snd.int_data = 0xff;
		z80_run(&s_snd, s_snd.cyc + (unsigned long)rbx_line_cycles(&b.snd_frac, SND_CLOCK, FRAME_MHZ, LINES), &irq, 0);
		smix_begin(s_mix);
		const uint32_t yc = (uint32_t)rbx_line_cycles(&b.ym_frac, YM_CLOCK, FRAME_MHZ, LINES);
		ym_line_t *yl = &s_ym_line[line];
		yl->off = (int16_t)s_mix->n;
		yl->slice = (int16_t)s_mix->slice;
		for (int i = 0; i < 2; i++)   // (the samples on the worker: nothing written to the buffer)
			yl->cnt[i] = (int16_t)rbx_ym_advance(i, yc, s_mix->tmp, SMIX_TMP);
		msm5205_advance(&s_msm, (uint32_t)rbx_line_cycles(&b.msm_frac, MSM_CLOCK, FRAME_MHZ, LINES));
		smix_level(s_mix, msm5205_output(&s_msm) * 154 / 256);   // (0.60)
		smix_end(s_mix);
	}
	for (int i = 0; i < 2; i++) {   // the frame's YM samples into the lines' slices
		int total = 0;
		for (int line = 0; line < LINES; line++)
			total += s_ym_line[line].cnt[i];
		if (total > YM_BUF)
			total = YM_BUF;
		memset(s_ym_buf, 0, sizeof(int32_t) * (size_t)total);
		rbx_ym_async_take(i, s_ym_buf, total);
		int pos = 0;
		for (int line = 0; line < LINES; line++) {
			const ym_line_t *yl = &s_ym_line[line];
			const int cnt = pos + yl->cnt[i] <= total ? yl->cnt[i] : total - pos;
			smix_stretch(s_mix->out + yl->off, yl->slice, s_ym_buf + pos, cnt);
			pos += cnt;
		}
	}
	smix_flush(s_mix);
}

static int gl_state(void *ctx, int load)
{
	const uint32_t csize = m68k_context_size();
	uint8_t *cpu = (uint8_t *)malloc(csize);
	if (!cpu)
		return -1;
	if (load)
		rbx_video_wait_idle();
	else
		m68k_get_context(cpu);
	uint32_t irq = m68k_bus_irq_pending();
	int rc = rbx_state_io(ctx, "m68k", cpu, csize);
	rc |= rbx_state_io(ctx, "irq", &irq, sizeof(irq));
	rc |= rbx_state_io(ctx, "goal92", &b, (uint32_t)offsetof(__typeof__(b), state_end));
	z80 c = s_snd;
	rc |= rbx_state_io(ctx, "goal92_snd", &c, sizeof(c));
	rc |= rbx_state_io(ctx, "goal92_msm", &s_msm, (uint32_t)offsetof(msm5205_t, vck));
	rc |= rbx_ym_state(0, ctx, load);
	rc |= rbx_ym_state(1, ctx, load);
	if (!rc && load) {
		m68k_set_context(cpu);
		m68k_bus_set_irq_pending(irq);
		c.read_byte = snd_read;
		c.write_byte = snd_write;
		c.port_in = z_in;
		c.port_out = z_out;
		c.userdata = NULL;
		c.read_map = NULL;
		c.op_rom = NULL;
		s_snd = c;
	}
	free(cpu);
	return rc ? -1 : 0;
}

static int gl_save(void *ctx) { return gl_state(ctx, 0); }
static int gl_load(void *ctx) { return gl_state(ctx, 1); }
static int gl_supported(const char *m) { return !strcmp(m, "goal92"); }

const rbx_driver rbx_drv_goal92 = { "Seibu Goal! '92", gl_init, gl_exit, gl_reset, gl_frame, gl_save, gl_load,
                                    gl_supported, gl_render, NULL };
