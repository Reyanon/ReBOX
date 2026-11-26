/*  Technos Mat Mania / Exciting Hour / Mania Challenge. Ported from MAME src/mame/technos/matmania.cpp
    (BSD-3-Clause, copyright-holders: Brad Oliver).
    M6502 at 1.5 MHz (IRQ held at vblank). Mat Mania: M6502 sound CPU at 1 MHz with two AY-8910 and an 8-bit DAC, 16
    NMIs a frame when enabled. Mania Challenge: MC6809E at 1.5 MHz with a YM3526 and the DAC, a Taito 68705 MCU.
    Video: two 256x512 pages of 16x16 (one shown, scrolled vertically), a 256x256 text layer, 16x16 sprites; 3 bpp,
    64 PROM colours and 16 RAM colours.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m6502/m6502.h"
#include "../../cpu/m6809/m6809.h"
#include "../../machine/taito68705.h"
#include "../../sound/ay8910.h"
#include "../../sound/rbx_ym.h"
#include "../../video/gfxlayout.h"
#include "tec8.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define FRAME_MHZ 57445          // set_raw(12 MHz / 2, 384, 0, 256, 272, 8, 248)
#define LINES 272
#define VIS_Y0 8
#define VIS_H 240
#define VIS_W 256
#define MAIN_CLOCK 1500000
#define SND_CLOCK_MM 1000000
#define SND_CLOCK_MC 1500000
#define AY_CLOCK 1500000
#define YM_CLOCK 3000000
#define MCU_CLOCK 750000
#define SND_RATE 46875           // (the AY-8910's native rate / 4)

static int s_maniach;
static uint8_t *s_rom, *s_zrom;
static uint32_t s_rom_size, s_zrom_size;
static uint32_t s_ppal[64];
static rbx_gfx s_gchar, s_gtile, s_gspr;
static int s_port[4];
static m6502_t s_main, s_snd65;
static m6809_t s_snd09;
static taito68705_t s_mcu;
static ay8910_t s_ay[2];
static int32_t *s_mix;
static int s_mix_n, s_mix_cap;
static int s_line;

static struct {
	uint8_t ram[0x800], vram1[0x800], vram0[0x800], pal[0x30];
	uint8_t zram[0x1000];
	uint8_t pagesel, scroll, latch, nmi_en, dac;
	int32_t ym_irq;
	int64_t main_t, snd_t;
	uint64_t main_frac, snd_frac, mcu_frac, out_frac;
	uint8_t state_end;
} t;

static struct {
	uint8_t ram[0x800], vram1[0x800], vram0[0x800], pal[0x30];
	uint8_t pagesel, scroll;
} *s_snap;

// ---- sound ----
static uint8_t latch_read(void)
{
	if (s_maniach) m6809_set_irq(&s_snd09, 0); else m6502_set_irq(&s_snd65, 0);
	return t.latch;
}

static void ym_irq(int state)
{
	t.ym_irq = state;
	m6809_set_firq(&s_snd09, state);
}

static uint8_t snd65_read(void *u, uint16_t a)   // matmania sound_map
{
	(void)u;
	if (a < 0x200) return t.zram[a];
	if (a == 0x2007) return latch_read();
	if (a >= 0x8000) return s_zrom[a];
	return 0;
}
static void snd65_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a < 0x200) t.zram[a] = v;
	else if (a == 0x2000 || a == 0x2001) ay8910_write(&s_ay[0], ~a & 1, v);
	else if (a == 0x2002 || a == 0x2003) ay8910_write(&s_ay[1], ~a & 1, v);
	else if (a == 0x2004) t.dac = v;
	else if (a == 0x2005) t.nmi_en = v & 1;
}

static uint8_t snd09_read(void *u, uint16_t a)   // maniach sound_map
{
	(void)u;
	if (a < 0x1000) return t.zram[a];
	if (a == 0x2004) return latch_read();
	if (a >= 0x4000) return s_zrom[a];
	return 0;
}
static void snd09_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a < 0x1000) t.zram[a] = v;
	else if (a == 0x2000 || a == 0x2001) rbx_ym_write(0, a & 1, v);
	else if (a == 0x2002) t.dac = v;
}

static void sound_line(void)
{
	const int32_t dac = ((int32_t)t.dac - 128) * 102;   // (DAC_8BIT_R2R, 0.4)
	if (s_maniach) {
		const uint32_t clocks = (uint32_t)rbx_line_cycles(&t.out_frac, YM_CLOCK, FRAME_MHZ, LINES);
		const int n = rbx_ym_advance(0, clocks, s_mix + s_mix_n, s_mix_cap - s_mix_n);
		for (int i = 0; i < n; i++)
			s_mix[s_mix_n + i] += dac;
		s_mix_n += n;
	} else {
		const uint32_t ac = (uint32_t)rbx_line_cycles(&t.out_frac, AY_CLOCK, FRAME_MHZ, LINES);
		const int room = s_mix_cap - s_mix_n;
		ay8910_render(&s_ay[0], ac, s_mix + s_mix_n, room);
		const int n = ay8910_render(&s_ay[1], ac, s_mix + s_mix_n, room);
		for (int i = 0; i < n; i++)
			s_mix[s_mix_n + i] += dac;
		s_mix_n += n;
	}
}

// ---- main CPU ----
static uint8_t main_read(void *u, uint16_t a)
{
	(void)u;
	if (a >= 0x4000) return s_rom[a];
	if (a < 0x780) return t.ram[a];
	if (a < 0x7e0) return s_maniach ? t.ram[a] : 0;
	if (a >= 0x1000 && a < 0x1800) return t.vram1[a - 0x1000];
	if (a >= 0x2000 && a < 0x2800) return t.vram0[a - 0x2000];
	switch (a) {
	case 0x3000: return (uint8_t)rbx_port_read(s_port[0]);
	case 0x3010: return (uint8_t)rbx_port_read(s_port[1]);
	case 0x3020: return (uint8_t)rbx_port_read(s_port[2]);
	case 0x3030:
		rbx_port_custom(s_port[3], 0x80, (s_line < VIS_Y0 || s_line >= VIS_Y0 + VIS_H) ? 0x80u : 0);
		return (uint8_t)rbx_port_read(s_port[3]);
	case 0x3040: return s_maniach ? taito68705_data_r(&s_mcu) : 0;
	case 0x3041:
		if (!s_maniach)
			return 0;
		return (uint8_t)((taito68705_mcu_semaphore_r(&s_mcu) ? 0 : 1) | (taito68705_host_semaphore_r(&s_mcu) ? 0 : 2));
	}
	return 0;
}
static void main_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0x4000) return;
	if (a < 0x7e0) t.ram[a] = v;
	else if (a >= 0x1000 && a < 0x1800) t.vram1[a - 0x1000] = v;
	else if (a >= 0x2000 && a < 0x2800) t.vram0[a - 0x2000] = v;
	else if (a >= 0x3050 && a < 0x3080) t.pal[a - 0x3050] = v;
	else switch (a) {
	case 0x3000: t.pagesel = v; break;
	case 0x3010:
		t.latch = v;
		if (s_maniach) m6809_set_irq(&s_snd09, 1); else m6502_set_irq(&s_snd65, 1);
		break;
	case 0x3020: t.scroll = v; break;
	case 0x3040: if (s_maniach) taito68705_data_w(&s_mcu, v); break;
	}
}

// ---- video ----
static inline int res4(int v) { return 0x0e * (v & 1) + 0x1f * ((v >> 1) & 1) + 0x43 * ((v >> 2) & 1) + 0x8f * ((v >> 3) & 1); }

static void mm_render(void)
{
	memcpy(rbx_video.palette, s_ppal, sizeof(s_ppal));
	for (int i = 0; i < 16; i++)   // (paletteram_w)
		rbx_video.palette[64 + i] = (uint32_t)(res4(s_snap->pal[i]) << 16 | res4(s_snap->pal[i | 0x10]) << 8 | res4(s_snap->pal[i | 0x20]));
	// the page: 16 x 32 tiles of 16x16 (column offs / 32 from the right, the lower half's tiles flipped vertically)
	const uint8_t *vr = (s_snap->pagesel & 1) ? s_snap->vram0 + 0x400 : s_snap->vram0;
	const uint8_t *cr = vr + 0x200;
	for (int y = VIS_Y0; y < VIS_Y0 + VIS_H; y++) {
		uint16_t *d = rbx_video.bitmap + (y - VIS_Y0) * rbx_video.pitch;
		const int ty = y + s_snap->scroll;   // (copyscrollbitmap, -scroll; at most 502)
		const int row = ty >> 4;
		for (int x = 0; x < VIS_W; x++) {
			const int offs = (15 - (x >> 4)) * 32 + row;
			const uint32_t code = rbx_wrap((uint32_t)(vr[offs] + (s_maniach ? (cr[offs] & 3) << 8 : (cr[offs] & 8) << 5)), s_gtile.count);
			const int py = row >= 16 ? 15 - (ty & 15) : ty & 15;
			d[x] = (uint16_t)(32 + ((cr[offs] & 0x30) >> 4) * 8 + s_gtile.pix[code * 256 + py * 16 + (x & 15)]);
		}
	}
	for (int o = 0x780; o < 0x7e0; o += 4) {
		const uint8_t *s = &s_snap->ram[o];
		if (!(s[0] & 1))
			continue;
		tec8_transpen(&s_gspr, (uint32_t)(s[1] + ((s[0] & 0xf0) << 4)), (uint16_t)(64 + ((s[0] & 8) >> 3) * 8), s[0] & 4,
		              s[0] & 2, 239 - s[3], (240 - s[2]) & 0xff, VIS_Y0, VIS_Y0 + VIS_H, VIS_Y0, VIS_W);
	}
	for (int y = VIS_Y0; y < VIS_Y0 + VIS_H; y++) {
		uint16_t *d = rbx_video.bitmap + (y - VIS_Y0) * rbx_video.pitch;
		for (int x = 0; x < VIS_W; x++) {
			const int offs = (31 - (x >> 3)) * 32 + (y >> 3);
			const uint8_t c = s_snap->vram1[0x400 + offs];
			const uint32_t code = rbx_wrap((uint32_t)(s_snap->vram1[offs] + 256 * (c & 7)), s_gchar.count);
			const uint8_t p = s_gchar.pix[code * 64 + (y & 7) * 8 + (x & 7)];
			if (p)
				d[x] = (uint16_t)(((c & 0x30) >> 4) * 8 + p);
		}
	}
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap->ram, t.ram, sizeof(t.ram));
	memcpy(s_snap->vram1, t.vram1, sizeof(t.vram1));
	memcpy(s_snap->vram0, t.vram0, sizeof(t.vram0));
	memcpy(s_snap->pal, t.pal, sizeof(t.pal));
	s_snap->pagesel = t.pagesel;
	s_snap->scroll = t.scroll;
	rbx_video_frame_done();
}

// ---- init / frame ----
static void mm_reset(void)
{
	memset(&t, 0, offsetof(__typeof__(t), state_end));
	m6502_init(&s_main, M6502_NMOS);
	s_main.read = main_read;
	s_main.write = main_write;
	s_main.irq_hold = 1;
	for (int p = 0; p < 7; p++)
		s_main.rpage[p] = s_main.wpage[p] = t.ram + p * 256;
	for (int p = 0x10; p < 0x18; p++)
		s_main.rpage[p] = s_main.wpage[p] = t.vram1 + (p - 0x10) * 256;
	for (int p = 0x20; p < 0x28; p++)
		s_main.rpage[p] = s_main.wpage[p] = t.vram0 + (p - 0x20) * 256;
	for (int p = 0x40; p < 0x100; p++)
		s_main.rpage[p] = s_rom + p * 256;
	m6502_reset(&s_main);
	if (s_maniach) {
		m6809_init(&s_snd09);
		s_snd09.read = snd09_read;
		s_snd09.write = snd09_write;
		for (int p = 0; p < 0x10; p++)
			s_snd09.rpage[p] = s_snd09.wpage[p] = t.zram + p * 256;
		for (int p = 0x40; p < 0x100; p++)
			s_snd09.rpage[p] = s_zrom + p * 256;
		m6809_reset(&s_snd09);
		taito68705_reset(&s_mcu);
		rbx_ym_reset(0);
	} else {
		m6502_init(&s_snd65, M6502_NMOS);
		s_snd65.read = snd65_read;
		s_snd65.write = snd65_write;
		for (int p = 0; p < 2; p++)
			s_snd65.rpage[p] = s_snd65.wpage[p] = t.zram + p * 256;
		for (int p = 0x80; p < 0x100; p++)
			s_snd65.rpage[p] = s_zrom + p * 256;
		m6502_reset(&s_snd65);
		ay8910_reset(&s_ay[0]);
		ay8910_reset(&s_ay[1]);
	}
	s_mix_n = 0;
	s_line = 0;
}

static void mm_exit(void);

static int mm_init(const rbx_game *g)
{
	s_maniach = !strcmp(g->machine, "maniach");
	uint32_t csize, tsize, ssize, psize, msize = 0;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_zrom = rbx_region("audiocpu", &s_zrom_size);
	uint8_t *chars = rbx_region("chars", &csize), *tiles = rbx_region("tiles", &tsize), *spr = rbx_region("sprites", &ssize);
	const uint8_t *prom = rbx_region("proms", &psize);
	uint8_t *mrom = s_maniach ? rbx_region("mcu:mcu", &msize) : NULL;
	const uint32_t ntiles = s_maniach ? 1024 : 512;
	if (!s_rom || s_rom_size < 0x10000 || !s_zrom || s_zrom_size < 0x10000 || !chars || csize < 3 * 1024 * 8 || !tiles ||
	    tsize < 3 * ntiles * 32 || !spr || ssize < 3 * 3584 * 32 || !prom || psize < 0x80 || (s_maniach && (!mrom || msize < 0x800))) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	if (s_maniach) {
		memset(&s_mcu, 0, sizeof(s_mcu));
		if (taito68705_init(&s_mcu, TAITO68705_STD, mrom, 0x800)) {
			rbx_fail("%s: MCU", g->set);
			return -1;
		}
	}
	for (int i = 0; i < 64; i++)
		s_ppal[i] = (uint32_t)(res4(prom[i]) << 16 | res4(prom[i] >> 4) << 8 | res4(prom[64 + i]));
	{	// charlayout, tilelayout / maniach_tilelayout, matmania / maniach_spritelayout (planes apart)
		static const uint32_t x8[8] = { 0, 1, 2, 3, 4, 5, 6, 7 }, y8[8] = { 0, 8, 16, 24, 32, 40, 48, 56 };
		static const uint32_t xs[16] = { 128, 129, 130, 131, 132, 133, 134, 135, 0, 1, 2, 3, 4, 5, 6, 7 };
		static uint32_t ys[16];
		for (int i = 0; i < 16; i++)
			ys[i] = (uint32_t)i * 8;
		const uint32_t pc[3] = { 2 * 1024 * 64, 1024 * 64, 0 }, pt[3] = { 2 * ntiles * 256, ntiles * 256, 0 };
		const uint32_t ps_mm[3] = { 2 * 3584 * 256, 3584 * 256, 0 }, ps_mc[3] = { 0, 3584 * 256, 2 * 3584 * 256 };
		const rbx_gfx_layout lc = { 8, 8, 3, pc, x8, y8, 64 };
		const rbx_gfx_layout lt = { 16, 16, 3, pt, xs, ys, 256 };
		const rbx_gfx_layout ls = { 16, 16, 3, s_maniach ? ps_mc : ps_mm, xs, ys, 256 };
		if (rbx_gfx_decode(&s_gchar, &lc, chars, 1024 * 8) || rbx_gfx_decode(&s_gtile, &lt, tiles, ntiles * 32) ||
		    rbx_gfx_decode(&s_gspr, &ls, spr, 3584 * 32)) {
			rbx_fail("%s: out of memory (graphics)", g->set);
			mm_exit();
			return -1;
		}
	}
	s_snap = calloc(1, sizeof(*s_snap));
	uint32_t rate;
	int rc = !s_snap;
	if (s_maniach) {
		rbx_ym_config yc;
		memset(&yc, 0, sizeof(yc));
		yc.type = RBX_YM3526;
		yc.clock = YM_CLOCK;
		yc.gain[0] = 256;
		yc.irq = ym_irq;
		rc = rc || rbx_ym_init(0, &yc);
		rate = rc ? 48000 : rbx_ym_rate(0);
	} else {
		for (int i = 0; i < 2; i++)
			ay8910_init(&s_ay[i], AY8910_TYPE_AY8910, AY_CLOCK, AY8910_LEGACY_OUTPUT, 77, 4, 0);   // (0.3)
		rate = ay8910_rate(&s_ay[0]);
	}
	s_mix_cap = (int)(rate / 50) + 256;
	s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	if (rc || !s_mix || rbx_audio_setup(rate) || rbx_video_setup(VIS_W, VIS_H, 80)) {
		rbx_fail("%s: out of memory", g->set);
		mm_exit();
		return -1;
	}
	static const char *const names[4] = { "IN0", "IN1", "DSW2", "DSW1" };
	for (int i = 0; i < 4; i++)
		s_port[i] = rbx_port_find(names[i]);
	mm_reset();
	return 0;
}

static void mm_exit(void)
{
	rbx_video_wait_idle();
	if (s_maniach)
		rbx_ym_exit(0);
	free(s_mix);
	s_mix = NULL;
	rbx_gfx_free(&s_gchar);
	rbx_gfx_free(&s_gtile);
	rbx_gfx_free(&s_gspr);
	free(s_snap);
	s_snap = NULL;
}

static void mm_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	for (int line = 0; line < LINES; line++) {
		s_line = line;
		if (line == VIS_Y0 + VIS_H) {
			vblank_capture();
			m6502_set_irq(&s_main, 1);   // (irq0_line_hold at vblank)
		}
		if (!s_maniach && line % 16 == 8 && line < 256 && t.nmi_en) {
			m6502_set_nmi(&s_snd65, 1);
			m6502_set_nmi(&s_snd65, 0);
		}
		m6502_run_until(&s_main, t.main_t += rbx_line_cycles(&t.main_frac, MAIN_CLOCK, FRAME_MHZ, LINES));
		if (s_maniach) {
			taito68705_execute(&s_mcu, rbx_line_cycles(&t.mcu_frac, MCU_CLOCK, FRAME_MHZ, LINES));
			m6809_run_until(&s_snd09, t.snd_t += rbx_line_cycles(&t.snd_frac, SND_CLOCK_MC, FRAME_MHZ, LINES));
		} else
			m6502_run_until(&s_snd65, t.snd_t += rbx_line_cycles(&t.snd_frac, SND_CLOCK_MM, FRAME_MHZ, LINES));
		sound_line();
	}
	for (int i = 0; i < s_mix_n; i++)
		rbx_audio_put(s_mix[i]);
	memset(s_mix, 0, sizeof(int32_t) * (size_t)s_mix_n);
	s_mix_n = 0;
}

static int mm_state(void *ctx, int load)
{
	int rc = 0;
	rc |= rbx_state_io(ctx, "mm", &t, (uint32_t)offsetof(__typeof__(t), state_end));
	rc |= rbx_state_io(ctx, "mm_main", &s_main, (uint32_t)offsetof(m6502_t, rpage));
	if (s_maniach) {
		rc |= rbx_state_io(ctx, "mm_snd", &s_snd09, (uint32_t)offsetof(m6809_t, rpage));
		rc |= rbx_state_io(ctx, "mm_tmcu", &s_mcu.st, sizeof(s_mcu.st));
		rc |= rbx_state_io(ctx, "mm_mcu", &s_mcu.mcu.st, sizeof(s_mcu.mcu.st));
		rc |= rbx_ym_state(0, ctx, load);
	} else {
		rc |= rbx_state_io(ctx, "mm_snd", &s_snd65, (uint32_t)offsetof(m6502_t, rpage));
		rc |= rbx_state_io(ctx, "mm_ay0", &s_ay[0], ay8910_state_size());
		rc |= rbx_state_io(ctx, "mm_ay1", &s_ay[1], ay8910_state_size());
	}
	return rc ? -1 : 0;
}

static int mm_save(void *ctx) { return mm_state(ctx, 0); }
static int mm_load(void *ctx) { return mm_state(ctx, 1); }
static int mm_supported(const char *machine) { return !strcmp(machine, "matmania") || !strcmp(machine, "maniach"); }

const rbx_driver rbx_drv_matmania = { "Technos Mat Mania", mm_init, mm_exit, mm_reset, mm_frame, mm_save, mm_load,
                                      mm_supported, mm_render, NULL };
