/*  Irem Red Alert, WW III, Panther and Demoneye-X. Ported from MAME src/mame/irem/redalert.cpp, redalert_v.cpp,
    redalert_a.cpp (BSD-3-Clause, copyright-holders: Mike Balfour) and the palette's
    src/emu/video/resnet.cpp (BSD-3-Clause, copyright-holders: Couriersud). A 6502 at 781 kHz, a 1 bpp bitmap with colour
    blocks under RAM characters; the sound: a second 6502 with an AY-3-8910, run line by line with the main CPU (it shares
    a handshake with it). Not done: the analog effects, Red Alert's voice board (8085 and HC55516) and Demoneye-X's sound
    board (6802, PIA, two AYs): silent.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m6502/m6502.h"
#include "../../sound/ay8910.h"
#include "../video/irem_draw.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define CPU_CLOCK 781250u      // (12.5 MHz / 16)
#define SND_CLOCK 1041667u     // (12.5 MHz / 12)
#define AY_CLOCK 2083333u      // (12.5 MHz / 6)
#define AY_DIV 6
#define SND_RATE (AY_CLOCK / 8 / AY_DIV)
#define LINES 256              // (set_refresh_hz(60), set_size(256, 256), set_vblank_time(0))
#define LINE_HZ (60u * LINES)
#define VY0 8
#define VW 256
#define VH 240
#define PEN_BM 0x200
#define PEN_BACK (PEN_BM + 8)
#define NPENS (PEN_BACK + 1)
#define IRQ_US_X100 86900      // (the sound IRQ: 555 astable 120k, 2.7k, 0.01u, each 869 us)
#define SND_IRQ_LEN 19         // (its discharge time, 18.7 us)

enum { K_REDALERT, K_WW3, K_PANTHER, K_DEMONEYE };

static uint8_t *s_rom, *s_srom;
static uint32_t s_rom_size, s_srom_size;
static m6502_t s_cpu, s_snd;
static ay8910_t s_ay;
static int s_kind, s_port_dsw, s_port_key1, s_port_key2, s_port_vol, s_port_coin;

static struct {
	uint8_t ram[0x2000], bitmap[0x2000], cram[0x400], chars[0x2000], sram[0x400];
	uint8_t video_control, bitmap_color, layer[4], yoffs, coin;
	uint8_t latch, hs, l1, l2;
	uint32_t cpu_frac, snd_frac, ay_frac, irq_acc;
	int64_t main_end, snd_end;
	uint32_t frame;
	uint8_t state_end;
} t;

// ---- the drawing's snapshot ----
static struct { uint8_t bitmap[0x2000], cram[0x400], chars[0x2000], video_control, layer[4], yoffs; } s_vs;

static void capture(void)
{
	memcpy(s_vs.bitmap, t.bitmap, sizeof(s_vs.bitmap));
	memcpy(s_vs.cram, t.cram, sizeof(s_vs.cram));
	memcpy(s_vs.chars, t.chars, sizeof(s_vs.chars));
	s_vs.video_control = t.video_control;
	memcpy(s_vs.layer, t.layer, 4);
	s_vs.yoffs = t.yoffs;
}

static void ra_render(void)
{
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch, demon = s_kind == K_DEMONEYE;
	const int straight = demon ? !(s_vs.video_control & 4) : ((s_vs.video_control ^ (s_kind == K_WW3 || s_kind == K_PANTHER ? 4 : 0)) & 4) != 0;
	const uint8_t *cv = s_vs.chars;
	for (int offs = 0; offs < 0x2000; offs++) {
		const int y = offs & 0xff, x0 = ((~offs >> 8) << 3) & 0xff;
		const int by = (straight ? y : y ^ 0xff) - VY0;
		if (by < 0 || by >= VH)
			continue;
		int bd = s_vs.bitmap[offs];
		const int bc = s_vs.cram[offs >> 3];
		const int code = cv[(demon ? 0x1000 : 0) | (offs >> 3)];
		const int base = ((code & 0x7f) << 3) | (offs & 7);
		int d1, d2;
		if (code & 0x80) {   // (D7 of the code selects the set)
			d1 = cv[0x0400 | base];
			d2 = cv[0x0c00 | base];
		} else {
			d1 = demon ? cv[base] : 0;
			d2 = cv[0x0800 | base];
		}
		uint16_t *d = bm + by * pitch;
		for (int i = 0; i < 8; i++) {
			const int bit = bd & 0x80, a01 = ((d2 & 0x80) >> 6) | ((d1 & 0x80) >> 7);
			uint16_t pen;
			if (a01 == 0 || (bit && (code & 0xc0) == 0xc0))
				pen = (uint16_t)(bit ? PEN_BM + bc : PEN_BACK);
			else
				pen = (uint16_t)(((code & 0xfe) << 1) | a01);
			d[straight ? x0 + i : (x0 + i) ^ 0xff] = pen;
			bd <<= 1;
			d1 <<= 1;
			d2 <<= 1;
		}
	}
	if (demon && (s_vs.layer[1] & 1)) {   // (the boss' layer)
		const int x = s_vs.layer[0], y = s_vs.yoffs, control = s_vs.layer[1];
		const int width = (control & 2) ? 8 : 4;
		int base = 0x1400 + ((control & 0x10) ? 0x20 : 0) + (s_vs.layer[2] & 3) * 0x100;
		for (int xb = 0; xb < 8; xb++)
			for (int yb = 0; yb < 8; yb++) {
				if (yb < width && xb < width)
					for (int iy = 0; iy < 8; iy++) {
						int l0 = cv[(base + iy) & 0x1fff], l1 = cv[(base + 0x400 + iy) & 0x1fff], l2 = cv[(base + 0x800 + iy) & 0x1fff];
						for (int ix = 0; ix < 8; ix++) {
							const int ccc = ((l0 & 0x80) >> 5) | ((l1 & 0x80) >> 6) | ((l2 & 0x80) >> 7);
							if (ccc) {
								const int yd = y + 8 * width - (yb * 8 + iy) - VY0, xd = x + 8 * width - (xb * 8 + 7 - ix);
								if (yd >= 0 && yd < VH && xd >= 0 && xd < VW)
									bm[yd * pitch + xd] = (uint16_t)(PEN_BM + ccc);
							}
							l0 <<= 1;
							l1 <<= 1;
							l2 <<= 1;
						}
					}
				base += 8;
			}
	}
}

// ---- the main 6502's bus ----
static uint8_t rd(void *c, uint16_t a)
{
	(void)c;
	if (a >= 0xc000 && a < 0xd000) {
		switch (a & 0x70) {
		case 0x00: return (uint8_t)rbx_port_read(s_port_dsw);
		case 0x10: return (uint8_t)rbx_port_read(s_port_key1);
		case 0x20: return (uint8_t)rbx_port_read(s_port_key2);
		case 0x70:
			m6502_set_irq(&s_cpu, 0);
			return s_kind == K_PANTHER ? (uint8_t)rbx_port_read(s_port_vol) : 0;
		}
	}
	return 0;
}

static void wr(void *c, uint16_t a, uint8_t v)
{
	(void)c;
	if (a >= 0x2000 && a < 0x4000) {   // (redalert_bitmap_videoram_w)
		t.bitmap[a - 0x2000] = v;
		t.cram[(a - 0x2000) >> 3] = t.bitmap_color & 7;
	} else if (a >= 0xc000 && a < 0xd000) {
		switch (a & 0x70) {
		case 0x30:   // (audio_command_w)
			t.latch = v;
			t.hs = 1;
			if (s_kind != K_DEMONEYE && !(v & 0x80)) {
				m6502_set_nmi(&s_snd, 1);
				m6502_set_nmi(&s_snd, 0);
			}
			break;
		case 0x40: t.video_control = v; break;
		case 0x50: t.bitmap_color = v; break;
		case 0x60:
			if (s_kind == K_DEMONEYE)
				t.layer[a & 3] = v;
			break;
		case 0x70:
			if (s_kind == K_DEMONEYE)
				t.yoffs = v;
			m6502_set_irq(&s_cpu, 0);
			break;
		}
	}
}

// ---- the sound 6502's bus ----
static uint8_t latch_r(void *u) { (void)u; return t.latch; }

static void ay_w(uint8_t v)   // (BC2 is high: bits 0 and 1 are BC1 and BDIR)
{
	switch (v & 3) {
	case 1:
		t.hs = 0;
		t.l1 = ay8910_data_r(&s_ay);
		break;
	case 2: ay8910_write(&s_ay, 1, t.l2); break;
	case 3: ay8910_write(&s_ay, 0, t.l2); break;
	}
}

static uint8_t srd(void *c, uint16_t a)
{
	(void)c;
	a &= 0x7fff;
	if (a < 0x1000)
		return t.sram[a & 0x3ff];
	if (a >= 0x7000)
		return s_srom[0x7000 + (a & 0x7ff)];
	if ((a & 1) && (s_kind == K_PANTHER ? a >= 0x2000 && a < 0x3000 : a < 0x2000))
		return t.l1;
	return 0;
}

static void swr(void *c, uint16_t a, uint8_t v)
{
	(void)c;
	a &= 0x7fff;
	if (a < 0x1000)
		t.sram[a & 0x3ff] = v;
	else if (s_kind == K_PANTHER ? a >= 0x2000 && a < 0x3000 : a < 0x2000) {
		if (a & 1)
			t.l2 = v;
		else
			ay_w(v);
	}
}

// ---- init / frame ----
static void ra_reset(void);

static void palette(const uint8_t *prom)
{
	static const int res_bm[1] = { 100 }, res_rg[3] = { 390, 220, 180 }, res_b[2] = { 220, 100 }, res_br[1] = { 1100 }, res_bg[1] = { 570 };
	double wb[1], wrg[3], wc[2], wr[1], wg[1];
	const double s1 = irem_res_raw(res_bm, 1, 470, wb), s2 = irem_res_raw(res_rg, 3, 470, wrg), s3 = irem_res_raw(res_b, 2, 470, wc);
	const double m = s1 > s2 ? (s1 > s3 ? s1 : s3) : (s2 > s3 ? s2 : s3), scale = 255.0 / m;
	irem_res_raw(res_br, 1, 470, wr);
	irem_res_raw(res_bg, 1, 470, wg);
	uint32_t *pal = rbx_video.palette;
	// (MAME indexes bitmap_weight with the colour bit, and only its first entry is set: the second is whatever the stack
	// holds, 0 in its snapshots; hence the bitmap's colours inverted)
	const int on = (int)(wb[0] * scale);
	#define W(b) ((b) ? 0 : on)
	for (int i = 0; i < 0x200; i++) {
		const int v = prom[i];
		if (s_kind == K_PANTHER) {
			pal[i] = (uint32_t)(W((~v >> 2) & 1) << 16 | W((~v >> 1) & 1) << 8 | W(~v & 1));
			continue;
		}
		#define B(n) ((v >> (n)) & 1)
		const int r = (int)(scale * (wrg[0] * B(2) + wrg[1] * B(6) + wrg[2] * B(4)) + 0.5);
		const int g = (int)(scale * (wrg[0] * B(1) + wrg[1] * B(3) + wrg[2] * B(5)) + 0.5);
		const int b = (int)(scale * (wc[0] * B(0) + wc[1] * B(7)) + 0.5);
		#undef B
		pal[i] = (uint32_t)(r << 16 | g << 8 | b);
	}
	for (int i = 0; i < 8; i++)
		pal[PEN_BM + i] = (uint32_t)(W((i >> 2) & 1) << 16 | W((i >> 1) & 1) << 8 | W(i & 1));
	#undef W
	const int br = (int)(wr[0] * scale), bg = (int)(wg[0] * scale);
	pal[PEN_BACK] = s_kind == K_REDALERT || s_kind == K_WW3 ? (uint32_t)(br << 16 | bg << 8 | bg) : 0;
}

static int ra_init(const rbx_game *g)
{
	uint32_t ps;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_srom = rbx_region("soundboard:audiocpu", &s_srom_size);
	const uint8_t *prom = rbx_region("proms", &ps);
	s_kind = !strcmp(g->machine, "ww3") ? K_WW3 : !strcmp(g->machine, "panther") ? K_PANTHER
	       : !strcmp(g->machine, "demoneye") ? K_DEMONEYE : K_REDALERT;
	if (!s_rom || s_rom_size < 0x10000 || !s_srom || s_srom_size < 0x8000 || !prom || ps < 0x200) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	memset(&t, 0, sizeof(t));
	s_port_dsw = rbx_port_find("DSW");
	s_port_key1 = rbx_port_find("KEY1");
	s_port_key2 = rbx_port_find("KEY2");
	s_port_vol = rbx_port_find("VOLUM");
	s_port_coin = rbx_port_find("COIN");
	ay8910_init(&s_ay, AY8910_TYPE_AY8910, AY_CLOCK, AY8910_LEGACY_OUTPUT, 128, AY_DIV, 0);
	s_ay.port_a_read = latch_r;
	if (rbx_audio_setup(SND_RATE) || rbx_video_setup(VW, VH, NPENS)) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	palette(prom);
	ra_reset();
	return 0;
}

static void ra_exit(void) { }

static void ra_reset(void)
{
	memset(&t, 0, offsetof(__typeof__(t), state_end));
	memset(&s_vs, 0, sizeof(s_vs));
	ay8910_reset(&s_ay);
	m6502_init(&s_cpu, M6502_NMOS);
	s_cpu.read = rd;
	s_cpu.write = wr;
	for (int p = 0; p < 0x20; p++)
		s_cpu.rpage[p] = s_cpu.wpage[p] = t.ram + p * 256;
	for (int p = 0; p < 0x20; p++)
		s_cpu.rpage[0x20 + p] = t.bitmap + p * 256;
	const int cn = s_kind == K_DEMONEYE ? 0x20 : 0x10;
	for (int p = 0; p < cn; p++)
		s_cpu.rpage[0x40 + p] = s_cpu.wpage[0x40 + p] = t.chars + p * 256;
	for (int p = 0x40 + cn; p < 0xc0; p++)
		s_cpu.rpage[p] = s_rom + p * 256;
	for (int p = 0; p < 0x10; p++)
		s_cpu.rpage[0xf0 + p] = s_rom + 0x8000 + p * 256;
	m6502_reset(&s_cpu);
	m6502_init(&s_snd, M6502_NMOS);
	s_snd.read = srd;
	s_snd.write = swr;
	m6502_reset(&s_snd);
}

static void ra_frame(void)
{
	const int coin = (int)rbx_port_read(s_port_coin) & 7;
	if (coin != t.coin) {   // (coin_inserted: asserted when let go)
		t.coin = (uint8_t)coin;
		m6502_set_nmi(&s_cpu, !coin);
	}
	for (int line = 0; line < LINES; line++) {
		if (line == VY0 + VH) {   // (the update; redalert_vblank_interrupt)
			rbx_video_wait_idle();
			capture();
			rbx_video_frame_done();
			m6502_set_irq(&s_cpu, 1);
		}
		t.cpu_frac += CPU_CLOCK;
		const uint32_t c = t.cpu_frac / LINE_HZ;
		t.cpu_frac -= c * LINE_HZ;
		t.main_end += c;
		m6502_run_until(&s_cpu, t.main_end);
		if (s_kind != K_DEMONEYE) {
			rbx_port_custom(s_port_key2, 1, t.hs);
			t.snd_frac += SND_CLOCK;
			const uint32_t sc = t.snd_frac / LINE_HZ;
			t.snd_frac -= sc * LINE_HZ;
			t.irq_acc += 100000000u / LINE_HZ;
			if (t.irq_acc >= IRQ_US_X100) {
				t.irq_acc -= IRQ_US_X100;
				m6502_set_irq(&s_snd, 1);
				m6502_run_until(&s_snd, t.snd_end + SND_IRQ_LEN);
				m6502_set_irq(&s_snd, 0);
			}
			t.snd_end += sc;
			m6502_run_until(&s_snd, t.snd_end);
		}
		t.ay_frac += AY_CLOCK;
		const uint32_t ac = t.ay_frac / LINE_HZ;
		t.ay_frac -= ac * LINE_HZ;
		int32_t mix[64];
		const int max = ay8910_out_max(&s_ay, ac);
		memset(mix, 0, (size_t)max * sizeof(mix[0]));
		const int n = ay8910_render(&s_ay, ac, mix, max);
		for (int i = 0; i < n; i++)
			rbx_audio_put(mix[i]);
	}
	t.frame++;
}

static int ra_state(void *ctx, int load)
{
	int rc = 0;
	(void)load;
	rc |= rbx_state_io(ctx, "ra", &t, (uint32_t)offsetof(__typeof__(t), state_end));
	rc |= rbx_state_io(ctx, "ra_cpu", &s_cpu, M6502_STATE_SIZE);
	rc |= rbx_state_io(ctx, "ra_snd", &s_snd, M6502_STATE_SIZE);
	rc |= rbx_state_io(ctx, "ra_ay", &s_ay, ay8910_state_size());
	return rc ? -1 : 0;
}

static int ra_save(void *ctx) { return ra_state(ctx, 0); }
static int ra_load(void *ctx) { return ra_state(ctx, 1); }
static int ra_supported(const char *machine)
{
	return !strcmp(machine, "redalert") || !strcmp(machine, "ww3") || !strcmp(machine, "panther") || !strcmp(machine, "demoneye");
}

const rbx_driver rbx_drv_redalert = { "Irem Red Alert", ra_init, ra_exit, ra_reset, ra_frame, ra_save, ra_load, ra_supported,
                                      ra_render, NULL };
