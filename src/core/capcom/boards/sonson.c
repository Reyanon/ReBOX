/*  Capcom Son Son. Ported from MAME src/mame/capcom/sonson.cpp (BSD-3-Clause, copyright-holders: Mirko Buffoni).
    MC6809 at 1.5 MHz (IRQ held at vblank), MC6809 sound CPU at 1.5 MHz (IRQ 240 times a second, FIRQ from the main
    CPU's latch) with two AY-8910 at 1.5 MHz. Video: a 256x256 tilemap of 8x8 (rows 5-31 scrolled), 16x16 sprites
    (3 bpp, also drawn wrapped); 32 colours through lookup PROMs. The screen is MAME's (55.40 Hz, 256 lines).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m6809/m6809.h"
#include "../../sound/ay8910.h"
#include "../../video/gfxlayout.h"
#include "cap8.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define FRAME_MHZ 55400
#define LINES 256
#define VIS_X0 8
#define VIS_W 240
#define VIS_Y0 8
#define VIS_H 240
#define CPU_CLOCK 1500000
#define AY_CLOCK 1500000
#define NPENS (64 * 4 + 32 * 8)

static uint8_t *s_rom, *s_zrom;
static uint32_t s_rom_size, s_zrom_size;
static uint32_t s_pens[NPENS];
static rbx_gfx s_gchar, s_gspr;
static int s_port[5];
static m6809_t s_main, s_snd;
static ay8910_t s_ay[2];
static int32_t s_mix[2048];
static int s_mix_n;

static struct {
	uint8_t ram[0x1000], vram[0x400], cram[0x400], spr[0x60];
	uint8_t zram[0x800];
	uint8_t scrollx, latch, mainlatch;
	uint32_t snd_acc;
	int64_t main_t, snd_t;
	uint64_t main_frac, snd_frac, ay_frac;
	uint8_t state_end;
} t;

static struct {
	uint8_t vram[0x400], cram[0x400], spr[0x60];
	uint8_t scrollx, flip;
} *s_snap;

static uint8_t snd_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x800) return t.zram[a];
	if (a == 0xa000) return t.latch;
	if (a >= 0xe000) return s_zrom[a];
	return 0;
}
static void snd_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a < 0x800) t.zram[a] = v;
	else if (a == 0x2000 || a == 0x2001) ay8910_write(&s_ay[0], a & 1, v);
	else if (a == 0x4000 || a == 0x4001) ay8910_write(&s_ay[1], a & 1, v);
}

static uint8_t main_read(void *u, uint16_t a)
{
	(void)u;
	if (a >= 0x4000) return s_rom[a];
	if (a < 0x1000) return t.ram[a];
	if (a < 0x1400) return t.vram[a - 0x1000];
	if (a < 0x1800) return t.cram[a - 0x1400];
	if (a >= 0x2020 && a < 0x2080) return t.spr[a - 0x2020];
	if (a >= 0x3002 && a <= 0x3006) return (uint8_t)rbx_port_read(s_port[a - 0x3002]);
	return 0;
}
static void main_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0x4000) return;
	if (a < 0x1000) t.ram[a] = v;
	else if (a < 0x1400) t.vram[a - 0x1000] = v;
	else if (a < 0x1800) t.cram[a - 0x1400] = v;
	else if (a >= 0x2020 && a < 0x2080) t.spr[a - 0x2020] = v;
	else if (a == 0x3000) t.scrollx = v;
	else if (a == 0x3010) t.latch = v;
	else if (a >= 0x3018 && a < 0x3020) {   // mainlatch (LS259)
		const int bit = a & 7, old = t.mainlatch;
		t.mainlatch = (uint8_t)((t.mainlatch & ~(1 << bit)) | ((v & 1) << bit));
		if (bit == 1 && !(old & 2) && (t.mainlatch & 2))   // (sh_irqtrigger_w: the sound FIRQ, held)
			m6809_hold_firq(&s_snd);
	}
}

static void ss_render(void)
{
	memcpy(rbx_video.palette, s_pens, sizeof(s_pens));
	const int flip = s_snap->flip;
	for (int y = VIS_Y0; y < VIS_Y0 + VIS_H; y++) {
		uint16_t *d = rbx_video.bitmap + (y - VIS_Y0) * rbx_video.pitch;
		const uint32_t ty = (uint32_t)(flip ? 255 - y : y) & 255;
		const uint32_t sx = (ty >> 3) >= 5 ? s_snap->scrollx : 0;   // (scroll rows 5-31)
		uint32_t last = 0xffffffffu, cbase = 0;
		const uint8_t *row = NULL;
		for (int x = VIS_X0; x < VIS_X0 + VIS_W; x++) {
			const uint32_t tx = (uint32_t)(flip ? 255 - x + (int)sx : x + (int)sx) & 255;
			if ((tx >> 3) != last) {   // (a character looked up once for its 8 pixels)
				last = tx >> 3;
				const uint32_t idx = (ty >> 3) * 32 + (tx >> 3);
				const uint8_t attr = s_snap->cram[idx];
				const uint32_t code = rbx_wrap((uint32_t)(s_snap->vram[idx] + 256 * (attr & 3)), s_gchar.count);
				row = s_gchar.pix + code * 64 + (ty & 7) * 8;
				cbase = (uint32_t)(attr >> 2) * 4;
			}
			d[x - VIS_X0] = (uint16_t)(cbase + row[tx & 7]);
		}
	}
	for (int o = 0x60 - 4; o >= 0; o -= 4) {
		const uint8_t *s = &s_snap->spr[o];
		const uint32_t code = (uint32_t)(s[2] + ((s[1] & 0x20) << 3));
		const uint16_t base = (uint16_t)(64 * 4 + (s[1] & 0x1f) * 8);
		int fx = !(s[1] & 0x40), fy = !(s[1] & 0x80), sx = s[3], sy = s[0];
		if (flip) {
			sx = 240 - sx;
			sy = 240 - sy;
			fx = !fx;
			fy = !fy;
		}
		cap8_transpen(&s_gspr, code, base, fx, fy, sx, sy, 0, VIS_Y0, VIS_Y0 + VIS_H, VIS_Y0, VIS_X0, VIS_W);
		cap8_transpen(&s_gspr, code, base, fx, fy, sx - 256, sy, 0, VIS_Y0, VIS_Y0 + VIS_H, VIS_Y0, VIS_X0, VIS_W);
		cap8_transpen(&s_gspr, code, base, fx, fy, sx, sy - 256, 0, VIS_Y0, VIS_Y0 + VIS_H, VIS_Y0, VIS_X0, VIS_W);
	}
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap->vram, t.vram, sizeof(t.vram));
	memcpy(s_snap->cram, t.cram, sizeof(t.cram));
	memcpy(s_snap->spr, t.spr, sizeof(t.spr));
	s_snap->scrollx = t.scrollx;
	s_snap->flip = !(t.mainlatch & 1);   // (Q0 inverted)
	rbx_video_frame_done();
}

static void ss_reset(void)
{
	memset(&t, 0, offsetof(__typeof__(t), state_end));
	m6809_init(&s_main);
	s_main.read = main_read;
	s_main.write = main_write;
	for (int p = 0; p < 0x10; p++)
		s_main.rpage[p] = s_main.wpage[p] = t.ram + p * 256;
	for (int p = 0x40; p < 0x100; p++)
		s_main.rpage[p] = s_rom + p * 256;
	m6809_reset(&s_main);
	m6809_init(&s_snd);
	s_snd.read = snd_read;
	s_snd.write = snd_write;
	for (int p = 0; p < 8; p++)
		s_snd.rpage[p] = s_snd.wpage[p] = t.zram + p * 256;
	for (int p = 0xe0; p < 0x100; p++)
		s_snd.rpage[p] = s_zrom + p * 256;
	m6809_reset(&s_snd);
	for (int i = 0; i < 2; i++)
		ay8910_reset(&s_ay[i]);
	s_mix_n = 0;
}

static void ss_exit(void);

static int ss_init(const rbx_game *g)
{
	uint32_t s1, s2, sp;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_zrom = rbx_region("audiocpu", &s_zrom_size);
	uint8_t *chars = rbx_region("chars", &s1), *spr = rbx_region("sprites", &s2);
	const uint8_t *prom = rbx_region("proms", &sp);
	if (!s_rom || s_rom_size < 0x10000 || !s_zrom || s_zrom_size < 0x10000 || !chars || !spr || !prom || sp < 0x240) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	{
		uint32_t rgb[32];
		for (int i = 0; i < 32; i++)
			rgb[i] = (uint32_t)(cap8_res4(prom[i + 0x20]) << 16 | cap8_res4(prom[i] >> 4) << 8 | cap8_res4(prom[i]));
		const uint8_t *cp = prom + 0x40;
		for (int i = 0; i < 0x100; i++)
			s_pens[i] = rgb[cp[i] & 0x0f];
		for (int i = 0x100; i < 0x200; i++)
			s_pens[i] = rgb[(cp[i] & 0x0f) | 0x10];
	}
	{	// gfx_8x8x2_planar (RGN_FRAC(1,2)), spritelayout (RGN_FRAC(1,3))
		static const uint32_t x8[8] = { 0, 1, 2, 3, 4, 5, 6, 7 }, y8[8] = { 0, 8, 16, 24, 32, 40, 48, 56 };
		static const uint32_t xs[16] = { 135, 134, 133, 132, 131, 130, 129, 128, 7, 6, 5, 4, 3, 2, 1, 0 };
		static uint32_t ys[16];
		for (int i = 0; i < 16; i++)
			ys[i] = (uint32_t)i * 8;
		const uint32_t h = s1 / 2 * 8, f = s2 / 3 * 8;
		const uint32_t pc[2] = { h, 0 }, ps[3] = { 2 * f, f, 0 };
		const rbx_gfx_layout lc = { 8, 8, 2, pc, x8, y8, 64 }, ls = { 16, 16, 3, ps, xs, ys, 256 };
		if (rbx_gfx_decode(&s_gchar, &lc, chars, s1 / 2) || rbx_gfx_decode(&s_gspr, &ls, spr, s2 / 3)) {
			rbx_fail("%s: out of memory (graphics)", g->set);
			ss_exit();
			return -1;
		}
	}
	s_snap = calloc(1, sizeof(*s_snap));
	for (int i = 0; i < 2; i++)
		ay8910_init(&s_ay[i], AY8910_TYPE_AY8910, AY_CLOCK, AY8910_LEGACY_OUTPUT, 77, 4, 0);   // (0.30)
	if (!s_snap || rbx_audio_setup(ay8910_rate(&s_ay[0])) || rbx_video_setup(VIS_W, VIS_H, NPENS)) {
		rbx_fail("%s: out of memory", g->set);
		ss_exit();
		return -1;
	}
	static const char *const names[5] = { "P1", "P2", "SYSTEM", "DSW1", "DSW2" };
	for (int i = 0; i < 5; i++)
		s_port[i] = rbx_port_find(names[i]);
	ss_reset();
	return 0;
}

static void ss_exit(void)
{
	rbx_video_wait_idle();
	rbx_gfx_free(&s_gchar);
	rbx_gfx_free(&s_gspr);
	free(s_snap);
	s_snap = NULL;
}

static void ss_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	for (int line = 0; line < LINES; line++) {
		if (line == VIS_Y0 + VIS_H) {
			vblank_capture();
			m6809_hold_irq(&s_main);
		}
		t.snd_acc += 240 * 1000;   // (set_periodic_int, 240 Hz)
		if (t.snd_acc >= (uint32_t)FRAME_MHZ * LINES) {
			t.snd_acc -= (uint32_t)FRAME_MHZ * LINES;
			m6809_hold_irq(&s_snd);
		}
		m6809_run_until(&s_main, t.main_t += rbx_line_cycles(&t.main_frac, CPU_CLOCK, FRAME_MHZ, LINES));
		m6809_run_until(&s_snd, t.snd_t += rbx_line_cycles(&t.snd_frac, CPU_CLOCK, FRAME_MHZ, LINES));
		const uint32_t ac = (uint32_t)rbx_line_cycles(&t.ay_frac, AY_CLOCK, FRAME_MHZ, LINES);
		const int room = 2048 - s_mix_n;
		ay8910_render(&s_ay[0], ac, s_mix + s_mix_n, room);
		s_mix_n += ay8910_render(&s_ay[1], ac, s_mix + s_mix_n, room);
	}
	for (int i = 0; i < s_mix_n; i++)
		rbx_audio_put(s_mix[i]);
	memset(s_mix, 0, sizeof(int32_t) * (size_t)s_mix_n);
	s_mix_n = 0;
}

static int ss_state(void *ctx, int load)
{
	(void)load;
	int rc = 0;
	rc |= rbx_state_io(ctx, "ss", &t, (uint32_t)offsetof(__typeof__(t), state_end));
	rc |= rbx_state_io(ctx, "ss_main", &s_main, (uint32_t)offsetof(m6809_t, rpage));
	rc |= rbx_state_io(ctx, "ss_snd", &s_snd, (uint32_t)offsetof(m6809_t, rpage));
	rc |= rbx_state_io(ctx, "ss_ay0", &s_ay[0], ay8910_state_size());
	rc |= rbx_state_io(ctx, "ss_ay1", &s_ay[1], ay8910_state_size());
	return rc ? -1 : 0;
}

static int ss_save(void *ctx) { return ss_state(ctx, 0); }
static int ss_load(void *ctx) { return ss_state(ctx, 1); }
static int ss_supported(const char *machine) { return !strcmp(machine, "sonson"); }

const rbx_driver rbx_drv_sonson = { "Capcom Son Son", ss_init, ss_exit, ss_reset, ss_frame, ss_save, ss_load,
                                    ss_supported, ss_render, NULL };
