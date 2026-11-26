/*  Technos Renegade / Nekketsu Kouha Kunio-kun. Ported from MAME src/mame/technos/renegade.cpp (BSD-3-Clause,
    copyright-holders: Phil Stroffolino, Carlos A. Lozano, Rob Rosenbrock).
    M6502 at 1.5 MHz (NMI at line 265, IRQ at line 16 and every 16 lines from 40), a Taito 68705 MCU at 3 MHz (not on
    the bootlegs), MC6809 sound CPU at 1.5 MHz with a YM3526 (FIRQ) and an MSM5205 whose sample end raises the NMI.
    Video: a 1024x256 tilemap of 16x16 tiles (3 bpp, eight banks), a 256x256 text layer of 8x8, 16x16 sprites (two
    high for big ones); xBGR_444 palette of 256.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m6502/m6502.h"
#include "../../cpu/m6809/m6809.h"
#include "../../machine/taito68705.h"
#include "../../sound/msm5205.h"
#include "../../sound/rbx_ym.h"
#include "../../video/gfxlayout.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define FRAME_MHZ 57445          // set_raw(12 MHz / 2, 384, 0, 256, 272, 19, 257)
#define LINES 272
#define VIS_Y0 19
#define VIS_H 238
#define VIS_W 256
#define MAIN_CLOCK 1500000
#define SND_CLOCK 1500000        // (MC6809 at 6 MHz, / 4)
#define MCU_CLOCK 750000         // (3 MHz / 4)
#define YM_CLOCK 3000000

static int s_has_mcu;
static uint8_t *s_rom, *s_srom, *s_adpcm;
static uint32_t s_rom_size, s_srom_size, s_adpcm_size;
static rbx_gfx s_chars, s_tiles[8], s_spr[16];
static int s_port[4];
static m6502_t s_main;
static m6809_t s_snd;
static taito68705_t s_mcu;
static msm5205_t s_msm;
static int32_t *s_mix;
static int s_mix_n, s_mix_cap;
static int s_line;

static struct {
	uint8_t ram[0x1800], fgram[0x800], bgram[0x800], spr[0x200], pal[0x200];
	uint8_t sndram[0x1000];
	int32_t scrollx;
	uint8_t flip, bank;
	uint8_t latch, nmi, irq;
	int32_t ym_irq;
	uint32_t adpcm_pos, adpcm_end;
	uint8_t adpcm_playing;
	int64_t main_t, snd_t;
	uint64_t main_frac, snd_frac, mcu_frac, ym_frac;
	uint32_t msm_acc;
	uint8_t state_end;
} t;

static struct {
	uint8_t fgram[0x800], bgram[0x800], spr[0x200], pal[0x200];
	int32_t scrollx;
	uint8_t flip;
} *s_snap;

// ---- sound ----
static void ym_irq(int state)
{
	t.ym_irq = state;
	m6809_set_firq(&s_snd, state);
}

static void adpcm_int(void)   // (vck_callback, the rising edge)
{
	if (!t.adpcm_playing)
		return;
	if (t.adpcm_pos >= t.adpcm_end) {
		msm5205_reset_w(&s_msm, 1);
		t.adpcm_playing = 0;
		m6809_set_nmi(&s_snd, 1);
	} else {
		const uint8_t d = t.adpcm_pos / 2 < s_adpcm_size ? s_adpcm[t.adpcm_pos / 2] : 0;
		msm5205_data_w(&s_msm, t.adpcm_pos & 1 ? d & 0xf : d >> 4);
		t.adpcm_pos++;
	}
}

static uint8_t snd_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x1000) return t.sndram[a];
	if (a < 0x1800) {
		m6809_set_irq(&s_snd, 0);
		return t.latch;
	}
	if (a >= 0x2800 && a < 0x3000) return rbx_ym_read(0, a & 1);
	if (a >= 0x8000) return s_srom[rbx_wrap((a - 0x8000), s_srom_size)];
	return 0;
}
static void snd_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a < 0x1000) t.sndram[a] = v;
	else if (a < 0x1800) return;
	else if (a < 0x2000) {   // adpcm_start_w
		msm5205_reset_w(&s_msm, 0);
		t.adpcm_playing = 1;
		m6809_set_nmi(&s_snd, 0);
	} else if (a < 0x2800) {   // adpcm_addr_w
		switch (v & 0x1c) {
		case 0x18: t.adpcm_pos = 0 * 0x8000 * 2; break;
		case 0x14: t.adpcm_pos = 1 * 0x8000 * 2; break;
		case 0x0c: t.adpcm_pos = 2 * 0x8000 * 2; break;
		default: t.adpcm_pos = t.adpcm_end = 0; return;
		}
		t.adpcm_pos |= (uint32_t)(v & 3) * 0x2000 * 2;
		t.adpcm_end = t.adpcm_pos + 0x2000 * 2;
		msm5205_set_selector(&s_msm, (v & 0x20) ? MSM5205_S48_4B : MSM5205_S96_4B);
	} else if (a < 0x3000)
		rbx_ym_write(0, a & 1, v);
	else if (a < 0x3800) {   // adpcm_stop_w
		msm5205_reset_w(&s_msm, 1);
		t.adpcm_playing = 0;
		m6809_set_nmi(&s_snd, 1);
	}
}

static void sound_line(void)
{
	const uint32_t clocks = (uint32_t)rbx_line_cycles(&t.ym_frac, YM_CLOCK, FRAME_MHZ, LINES);
	const int n = rbx_ym_advance(0, clocks, s_mix + s_mix_n, s_mix_cap - s_mix_n);
	const uint32_t yr = rbx_ym_rate(0);
	for (int i = 0; i < n; i++) {
		t.msm_acc += s_msm.clock;
		const uint32_t k = t.msm_acc / yr;
		t.msm_acc -= k * yr;
		msm5205_advance(&s_msm, k);
		s_mix[s_mix_n + i] += msm5205_output(&s_msm);
	}
	s_mix_n += n;
}

// ---- main CPU ----
static void map_bank(void)
{
	for (int p = 0; p < 0x40; p++)
		s_main.rpage[0x40 + p] = s_rom + rbx_wrap(((uint32_t)t.bank * 0x4000 + (uint32_t)p * 256), s_rom_size);
}

static int vblank(void) { return s_line < VIS_Y0 || s_line >= VIS_Y0 + VIS_H; }

static uint8_t main_read(void *u, uint16_t a)
{
	(void)u;
	if (a >= 0x8000) return s_rom[a];
	if (a >= 0x4000) return s_rom[rbx_wrap(((uint32_t)t.bank * 0x4000 + (a - 0x4000)), s_rom_size)];
	if (a < 0x1800) return t.ram[a];
	if (a < 0x2000) return t.fgram[a - 0x1800];
	if (a < 0x2800) return t.spr[a & 0x1ff];
	if (a < 0x3000) return t.bgram[a - 0x2800];
	if (a < 0x3200) return t.pal[a - 0x3000];
	switch (a) {
	case 0x3800: return (uint8_t)rbx_port_read(s_port[0]);
	case 0x3801: return (uint8_t)rbx_port_read(s_port[1]);
	case 0x3802: {
		uint32_t st = 0;
		if (s_has_mcu)
			st = (taito68705_host_semaphore_r(&s_mcu) ? 0 : 1) | (taito68705_mcu_semaphore_r(&s_mcu) ? 0 : 2);
		rbx_port_custom(s_port[2], 0x70, st << 4 | (vblank() ? 0x40u : 0));
		return (uint8_t)rbx_port_read(s_port[2]);
	}
	case 0x3803: return (uint8_t)rbx_port_read(s_port[3]);
	case 0x3804: return s_has_mcu ? taito68705_data_r(&s_mcu) : 0;
	case 0x3805:   // mcu_reset_r
		if (s_has_mcu) {
			taito68705_reset_w(&s_mcu, 1);
			taito68705_reset_w(&s_mcu, 0);
		}
		return 0;
	}
	return 0;
}
static void main_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0x4000) return;
	if (a < 0x1800) t.ram[a] = v;
	else if (a < 0x2000) t.fgram[a - 0x1800] = v;
	else if (a < 0x2800) t.spr[a & 0x1ff] = v;
	else if (a < 0x3000) t.bgram[a - 0x2800] = v;
	else if (a < 0x3200) t.pal[a - 0x3000] = v;
	else switch (a) {
	case 0x3800: t.scrollx = (t.scrollx & 0xff00) | v; break;
	case 0x3801: t.scrollx = (t.scrollx & 0xff) | v << 8; break;
	case 0x3802:   // (soundlatch: data pending to the IRQ)
		t.latch = v;
		m6809_set_irq(&s_snd, 1);
		break;
	case 0x3803: t.flip = (~v & 1); break;
	case 0x3804: if (s_has_mcu) taito68705_data_w(&s_mcu, v); break;
	case 0x3805: t.bank = v & 1; map_bank(); break;
	case 0x3806: t.nmi = 0; m6502_set_nmi(&s_main, 0); break;
	case 0x3807: t.irq = 0; m6502_set_irq(&s_main, 0); break;
	}
}

// ---- video ----
static void draw_spr(const rbx_gfx *g, uint32_t code, int color, int fx, int fy, int sx, int sy)
{
	code %= g->count;
	if (!g->used[code])
		return;
	const uint8_t *src = g->pix + code * 256;
	for (int j = 0; j < 16; j++) {
		const int yy = sy + j;
		if (yy < VIS_Y0 || yy >= VIS_Y0 + VIS_H)
			continue;
		const uint8_t *s = src + (fy ? 15 - j : j) * 16;
		uint16_t *d = rbx_video.bitmap + (yy - VIS_Y0) * rbx_video.pitch;
		for (int i = 0; i < 16; i++) {
			const int xx = sx + i;
			if ((unsigned)xx >= VIS_W)
				continue;
			const uint8_t p = s[fx ? 15 - i : i];
			if (p)
				d[xx] = (uint16_t)(128 + color * 8 + p);
		}
	}
}

static void rg_render(void)
{
	for (int i = 0; i < 256; i++) {   // xBGR_444
		const uint32_t p = (uint32_t)(s_snap->pal[0x100 + i] << 8 | s_snap->pal[i]);
		rbx_video.palette[i] = ((p & 15) * 17) << 16 | (((p >> 4) & 15) * 17) << 8 | ((p >> 8) & 15) * 17;
	}
	const int flip = s_snap->flip, scroll = s_snap->scrollx;
	for (int y = VIS_Y0; y < VIS_Y0 + VIS_H; y++) {
		uint16_t *d = rbx_video.bitmap + (y - VIS_Y0) * rbx_video.pitch;
		const uint32_t ty = (uint32_t)(flip ? 265 - y : y - 10) & 255;
		for (int x = 0; x < VIS_W; x++) {   // bg: scrolldx(256, 0), scrolldy(10, 10)
			const uint32_t tx = (uint32_t)(flip ? 255 - x + scroll : x - 256 + scroll) & 1023;
			const uint32_t idx = (ty >> 4) * 64 + (tx >> 4);
			const uint8_t attr = s_snap->bgram[idx + 0x400];
			const rbx_gfx *g = &s_tiles[attr & 7];
			const uint32_t code = rbx_wrap(s_snap->bgram[idx], g->count);
			d[x] = (uint16_t)(192 + (attr >> 5) * 8 + g->pix[code * 256 + (ty & 15) * 16 + (tx & 15)]);
		}
	}
	for (int o = 0; o < 0x200; o += 4) {
		const uint8_t *s = &s_snap->spr[o];
		int sy = 234 - s[0];
		const int attr = s[1];
		int sx = s[3];
		const rbx_gfx *g = &s_spr[attr & 0xf];
		const int color = (attr >> 4) & 3;
		int xflip = (attr & 0x40) != 0;
		if (sx > 248) sx -= 256;
		if (sy < 0) sy += 256;
		if (flip) {
			sx = 240 - sx;
			sy = 260 - sy;
			xflip = !xflip;
		}
		if (attr & 0x80)
			draw_spr(g, s[2] | 1u, color, xflip, flip, sx, sy + (flip ? -16 : 16));
		else
			sy += flip ? -16 : 16;
		draw_spr(g, s[2], color, xflip, flip, sx, sy);
	}
	for (int y = VIS_Y0; y < VIS_Y0 + VIS_H; y++) {
		uint16_t *d = rbx_video.bitmap + (y - VIS_Y0) * rbx_video.pitch;
		const uint32_t ty = (uint32_t)(flip ? 265 - y : y - 10) & 255;
		for (int x = 0; x < VIS_W; x++) {
			const uint32_t tx = (uint32_t)(flip ? 255 - x : x) & 255;
			const uint32_t idx = (ty >> 3) * 32 + (tx >> 3);
			const uint8_t attr = s_snap->fgram[idx + 0x400];
			const uint32_t code = rbx_wrap(((attr & 3u) << 8 | s_snap->fgram[idx]), s_chars.count);
			const uint8_t p = s_chars.pix[code * 64 + (ty & 7) * 8 + (tx & 7)];
			if (p)
				d[x] = (uint16_t)((attr >> 6) * 8 + p);
		}
	}
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap->fgram, t.fgram, sizeof(t.fgram));
	memcpy(s_snap->bgram, t.bgram, sizeof(t.bgram));
	memcpy(s_snap->spr, t.spr, sizeof(t.spr));
	memcpy(s_snap->pal, t.pal, sizeof(t.pal));
	s_snap->scrollx = t.scrollx;
	s_snap->flip = t.flip;
	rbx_video_frame_done();
}

// ---- init / frame ----
static void rg_reset(void)
{
	memset(&t, 0, offsetof(__typeof__(t), state_end));
	m6502_init(&s_main, M6502_NMOS);
	s_main.read = main_read;
	s_main.write = main_write;
	for (int p = 0; p < 0x18; p++)
		s_main.rpage[p] = s_main.wpage[p] = t.ram + p * 256;
	for (int p = 0x80; p < 0x100; p++)
		s_main.rpage[p] = s_rom + p * 256;
	map_bank();
	m6502_reset(&s_main);
	m6809_init(&s_snd);
	s_snd.read = snd_read;
	s_snd.write = snd_write;
	for (int p = 0; p < 0x10; p++)
		s_snd.rpage[p] = s_snd.wpage[p] = t.sndram + p * 256;
	for (int p = 0x80; p < 0x100; p++)
		s_snd.rpage[p] = s_srom + ((uint32_t)(p - 0x80) * 256) % s_srom_size;
	m6809_reset(&s_snd);
	m6809_set_nmi(&s_snd, 1);   // (machine_reset: the sound NMI asserted)
	if (s_has_mcu)
		taito68705_reset(&s_mcu);
	rbx_ym_reset(0);
	msm5205_reset(&s_msm);
	msm5205_reset_w(&s_msm, 1);
	msm5205_set_selector(&s_msm, MSM5205_S48_4B);
	s_mix_n = 0;
	s_line = 0;
}

static void rg_exit(void);

static int rg_init(const rbx_game *g)
{
	s_has_mcu = !strncmp(g->machine, "renegade", 8);
	uint32_t csize, tsize, ssize, msize = 0;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_srom = rbx_region("audiocpu", &s_srom_size);
	uint8_t *chars = rbx_region("chars", &csize), *tiles = rbx_region("tiles", &tsize), *spr = rbx_region("sprites", &ssize);
	s_adpcm = rbx_region("adpcm", &s_adpcm_size);
	uint8_t *mrom = s_has_mcu ? rbx_region("mcu:mcu", &msize) : NULL;
	if (!s_rom || s_rom_size < 0x10000 || !s_srom || !chars || csize < 0x8000 || !tiles || tsize < 0x30000 || !spr ||
	    ssize < 0x60000 || !s_adpcm || (s_has_mcu && (!mrom || msize < 0x800))) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	if (s_has_mcu) {
		memset(&s_mcu, 0, sizeof(s_mcu));
		if (taito68705_init(&s_mcu, TAITO68705_STD, mrom, 0x800)) {
			rbx_fail("%s: MCU", g->set);
			return -1;
		}
	}
	{	// charlayout, tileslayout1..4
		static const uint32_t pc[3] = { 2, 4, 6 }, xc[8] = { 1, 0, 65, 64, 129, 128, 193, 192 };
		static const uint32_t yc[8] = { 0, 8, 16, 24, 32, 40, 48, 56 };
		static const uint32_t xt[16] = { 3, 2, 1, 0, 16 * 8 + 3, 16 * 8 + 2, 16 * 8 + 1, 16 * 8 + 0,
		                                 32 * 8 + 3, 32 * 8 + 2, 32 * 8 + 1, 32 * 8 + 0, 48 * 8 + 3, 48 * 8 + 2, 48 * 8 + 1, 48 * 8 + 0 };
		static uint32_t yt[16];
		for (int i = 0; i < 16; i++)
			yt[i] = (uint32_t)i * 8;
		static const uint32_t pl[4][3] = { { 4, 0x8000 * 8 + 0, 0x8000 * 8 + 4 }, { 0, 0xc000 * 8 + 0, 0xc000 * 8 + 4 },
		                                   { 0x4000 * 8 + 4, 0x10000 * 8 + 0, 0x10000 * 8 + 4 },
		                                   { 0x4000 * 8 + 0, 0x14000 * 8 + 0, 0x14000 * 8 + 4 } };
		const rbx_gfx_layout lc = { 8, 8, 3, pc, xc, yc, 32 * 8 };
		int rc = rbx_gfx_decode(&s_chars, &lc, chars, 1024 * 32);
		for (int i = 0; i < 8 && !rc; i++) {
			const rbx_gfx_layout lt = { 16, 16, 3, pl[i & 3], xt, yt, 64 * 8 };
			rc = rbx_gfx_decode(&s_tiles[i], &lt, tiles + (i >> 2) * 0x18000, 256 * 64);
		}
		for (int i = 0; i < 16 && !rc; i++) {
			const rbx_gfx_layout lt = { 16, 16, 3, pl[i & 3], xt, yt, 64 * 8 };
			rc = rbx_gfx_decode(&s_spr[i], &lt, spr + (i >> 2) * 0x18000, 256 * 64);
		}
		if (rc) {
			rbx_fail("%s: out of memory (graphics)", g->set);
			rg_exit();
			return -1;
		}
	}
	s_snap = calloc(1, sizeof(*s_snap));
	rbx_ym_config yc;
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM3526;
	yc.clock = YM_CLOCK;
	yc.gain[0] = 256;
	yc.irq = ym_irq;
	if (!s_snap || rbx_ym_init(0, &yc) || rbx_ym_set_async(0, 1)) {
		rbx_fail("%s: out of memory", g->set);
		rg_exit();
		return -1;
	}
	s_mix_cap = (int)(rbx_ym_rate(0) / 50) + 256;
	s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	if (!s_mix || rbx_audio_setup(rbx_ym_rate(0)) || rbx_video_setup(VIS_W, VIS_H, 256)) {
		rbx_fail("%s: out of memory", g->set);
		rg_exit();
		return -1;
	}
	msm5205_init(&s_msm, 12000000 / 32, MSM5205_S48_4B, adpcm_int);
	static const char *const names[4] = { "IN0", "IN1", "DSW2", "DSW1" };
	for (int i = 0; i < 4; i++)
		s_port[i] = rbx_port_find(names[i]);
	rg_reset();
	return 0;
}

static void rg_exit(void)
{
	rbx_video_wait_idle();
	rbx_ym_exit(0);
	free(s_mix);
	s_mix = NULL;
	rbx_gfx_free(&s_chars);
	for (int i = 0; i < 8; i++)
		rbx_gfx_free(&s_tiles[i]);
	for (int i = 0; i < 16; i++)
		rbx_gfx_free(&s_spr[i]);
	free(s_snap);
	s_snap = NULL;
}

static void rg_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	for (int line = 0; line < LINES; line++) {
		s_line = line;
		if (line == VIS_Y0 + VIS_H)
			vblank_capture();
		if (line == 265) {
			t.nmi = 1;
			m6502_set_nmi(&s_main, 1);
		}
		if (line == 0x10 || (line > 0x20 && (line & 0xf) == 8)) {
			t.irq = 1;
			m6502_set_irq(&s_main, 1);
		}
		m6502_run_until(&s_main, t.main_t += rbx_line_cycles(&t.main_frac, MAIN_CLOCK, FRAME_MHZ, LINES));
		if (s_has_mcu)
			taito68705_execute(&s_mcu, rbx_line_cycles(&t.mcu_frac, MCU_CLOCK, FRAME_MHZ, LINES));
		m6809_run_until(&s_snd, t.snd_t += rbx_line_cycles(&t.snd_frac, SND_CLOCK, FRAME_MHZ, LINES));
		sound_line();
	}
	rbx_ym_async_take(0, s_mix, s_mix_n);
	for (int i = 0; i < s_mix_n; i++)
		rbx_audio_put(s_mix[i]);
	memset(s_mix, 0, sizeof(int32_t) * (size_t)s_mix_n);
	s_mix_n = 0;
}

static int rg_state(void *ctx, int load)
{
	int rc = 0;
	rc |= rbx_state_io(ctx, "rg", &t, (uint32_t)offsetof(__typeof__(t), state_end));
	rc |= rbx_state_io(ctx, "rg_main", &s_main, (uint32_t)offsetof(m6502_t, rpage));
	rc |= rbx_state_io(ctx, "rg_snd", &s_snd, (uint32_t)offsetof(m6809_t, rpage));
	if (s_has_mcu) {
		rc |= rbx_state_io(ctx, "rg_tmcu", &s_mcu.st, sizeof(s_mcu.st));
		rc |= rbx_state_io(ctx, "rg_mcu", &s_mcu.mcu.st, sizeof(s_mcu.mcu.st));
	}
	rc |= rbx_ym_state(0, ctx, load);
	rc |= rbx_state_io(ctx, "rg_msm", &s_msm, (uint32_t)offsetof(msm5205_t, vck));
	if (!rc && load)
		map_bank();
	return rc ? -1 : 0;
}

static int rg_save(void *ctx) { return rg_state(ctx, 0); }
static int rg_load(void *ctx) { return rg_state(ctx, 1); }
static int rg_supported(const char *machine) { return !strcmp(machine, "renegade") || !strcmp(machine, "kuniokunb"); }

const rbx_driver rbx_drv_renegade = { "Technos Renegade", rg_init, rg_exit, rg_reset, rg_frame, rg_save, rg_load,
                                      rg_supported, rg_render, NULL };
