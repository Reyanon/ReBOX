/*  Atari Gauntlet hardware: Gauntlet, Gauntlet II, Vindicators Part II. Ported from MAME src/mame/atari/gauntlet.cpp
    (BSD-3-Clause, copyright-holders: Aaron Giles).
    68010 at 7.16 MHz; a 64x64 playfield of 8x8 tiles, the Atari motion objects (atarimo.c), a 64x31 alphanumerics
    layer; the slapstic (104 / 106 / 107 / 118) on 0x38000-0x3ffff; a 2804 parallel EEPROM.
    Sound: 6502 at 1.79 MHz with a YM2151, a POKEY and a TMS5220C; the two latches between the CPUs.
    MAME draws the screen in pieces (the scroll writes and its motion object timer every 64 lines update it up to the
    beam): drawn here the same way, as it goes, into the frame; the frame is handed over at the vblank.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m68k_bus.h"
#include "../../cpu/m68k/m68k.h"
#include "../../cpu/m6502/m6502.h"
#include "../../sound/rbx_ym.h"
#include "../../sound/pokey.h"
#include "../../sound/tms5220.h"
#include "../../video/gfxlayout.h"
#include "../video/atarimo.h"
#include "../machine/slapstic.h"
#include "../../rbx_prof.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAIN_CLOCK 7159090         // 14.318181 MHz / 2
#define SND_CLOCK 1789772          // / 8
#define YM_CLOCK 3579545           // / 4
#define LINE_CYC 456               // set_raw(14.318181 MHz / 2, 456, 0, 336, 262, 0, 240)
#define SND_LINE_CYC 114
#define YM_LINE_CLK 228
#define LINES 262
#define VIS_W 336
#define VIS_H 240
#define FRAME_MHZ 59923
#define SLICES 4

// ---- the boards ----
static int s_vindctr2;
static int s_chip;
static uint8_t *s_rom, *s_srom;
static uint32_t s_rom_size, s_srom_size;
static rbx_gfx s_gfx_pf, s_gfx_an;
static amo_t s_mo;
static slapstic_t s_slap;

// main memory (host-order words)
static uint16_t s_ram[0x1000];       // 800000-801fff
static uint16_t s_pf[0x1000];        // 900000-901fff
static uint16_t s_mob[0x1000];       // 902000-903fff
static uint16_t s_spare[0x800];      // 904000-904fff
static uint16_t s_alpha[0x800];      // 905000-905fff (alpha, yscroll at 0x905f6e, the SLIP RAM at 0x905f80)
static uint16_t s_pal[0x400];        // 910000-9107ff
static uint8_t s_eeprom[0x200];
static int s_eep_unlocked;
static char s_nv_path[512];
static uint16_t s_xscroll;
static uint8_t s_pf_tile_bank, s_pf_color_bank;
static uint8_t s_outlatch;
static int s_port[6];
static int s_line, s_last_drawn;
static int s_irq_vbl;

// the latches
static uint8_t s_soundlatch, s_mainlatch;
static int s_snd_pending, s_main_pending;

// sound
static m6502_t s_snd;
static uint8_t s_sram[0x1000];
static int s_snd_reset;             // 1 = held in reset
static uint8_t s_soundctl;          // LS259 16T/U outputs
static int s_soundctl_clear;
static uint8_t s_mixer;
static pokey_t s_pokey;
static tms5220_t s_tms;
static uint32_t s_tms_clock;
static int64_t s_snd_cyc0;          // 6502 cycles at the frame's start
static int64_t s_pk_done;           // POKEY clocks rendered this frame
static uint64_t s_tms_acc;          // TMS clocks x SND_CLOCK not rendered yet
static int64_t s_tms_last;          // 6502 cycles of this frame converted
static int32_t *s_ym_l, *s_ym_r;
static int s_ym_n, s_ym_cap;
static int16_t *s_pk_buf, *s_tms_buf;
static int s_pk_n, s_pk_cap, s_tms_n, s_tms_cap;
static uint32_t s_tms_frac;          // the TMS's sub-sample clocks (of 80)
static int64_t s_mdebt;
static int s_pq, s_sndacc;   // perfect quantum: main clocks left; the 6502's clocks carried (x 4)

// video
static uint16_t *s_frame;            // VIS_W x VIS_H being drawn
static uint16_t *s_mobm;             // the MO bitmap
static struct { uint16_t frame[VIS_W * VIS_H]; uint32_t pal[0x400]; } *s_snap;

static void tms_sync(void);
static void pokey_sync(void);
// the sampled profile's marks (ADR-155)
enum { GT_PA_68K = RBX_PA_DRV, GT_PA_6502, GT_PA_DRAW, GT_PA_YM, GT_PA_TMS, GT_PA_WAITPK, GT_PA_END };
static const char *const s_gt_pa_names[] = { "68010", "6502", "draw", "ym2151", "tms5220", "pokey wait" };

// the POKEY on the second worker (its clock-exact model is the frame's largest cost): the 6502's writes and the samples
// to make are queued in order and done there (the same calls in the same order: the same samples). A read, the mix
// and a reset or state wait for it first
enum { PK_RENDER = 0x01000000u, PK_WRITE = 0x02000000u, PK_KIND = 0xff000000u };
#define PKQ 4096
static uint32_t s_pkq[PKQ][2];
static volatile uint32_t s_pkq_h, s_pkq_t;
static void pk_job(void)
{
	for (;;) {
		const uint32_t t = s_pkq_t, h = __atomic_load_n(&s_pkq_h, __ATOMIC_ACQUIRE);
		if (t == h)
			return;
		for (uint32_t i = t; i != h; i++) {
			const uint32_t a = s_pkq[i % PKQ][0], b = s_pkq[i % PKQ][1];
			if ((a & PK_KIND) == PK_WRITE)
				pokey_write(&s_pokey, (uint8_t)(a & 0x0f), (uint8_t)b);
			else {
				const int n = (int)(a & 0xffff), off = (int)(b & 0xffff), g = (int)(b >> 16);
				int16_t *o = s_pk_buf + off;
				pokey_render(&s_pokey, o, n);
				for (int k = 0; k < n; k++)   // (0.32 x g / 3)
					o[k] = (int16_t)(o[k] * g * 32 / 300);
			}
		}
		__atomic_store_n(&s_pkq_t, h, __ATOMIC_RELEASE);
	}
}
static void pk_push(uint32_t a, uint32_t b)
{
	const uint32_t h = s_pkq_h;
	if (h - __atomic_load_n(&s_pkq_t, __ATOMIC_ACQUIRE) >= PKQ)
		rbx_aux2_sync();
	s_pkq[h % PKQ][0] = a;
	s_pkq[h % PKQ][1] = b;
	__atomic_store_n(&s_pkq_h, h + 1, __ATOMIC_RELEASE);
	rbx_aux2_kick();
}

// ---- the latches and the sound CPU's lines ----
static void main_irq6(void) { m68k_bus_set_line(6, s_main_pending); }

static void sound_reset_w(int state)
{
	if (!state) {
		s_snd_reset = 1;
		// soundctl clear_w(0): every output low
		s_soundctl = 0;
		s_soundctl_clear = 1;
		s_main_pending = 0;   // mainlatch acknowledge
		main_irq6();
		pokey_sync();
		tms_sync();
		s_mixer = 0;
		rbx_ym_set_gain(0, 0, 0);
		rbx_ym_set_gain(0, 1, 0);
		tms5220_wsq_w(&s_tms, 0);
		tms5220_rsq_w(&s_tms, 0);
	} else {
		s_soundctl_clear = 0;
		if (s_snd_reset) {
			s_snd_reset = 0;
			m6502_reset(&s_snd);
			m6502_set_irq(&s_snd, 0);
			m6502_set_nmi(&s_snd, s_snd_pending);
		}
	}
}

static void outlatch_w(int bit, int d)
{
	if (((s_outlatch >> bit) & 1) == d)
		return;
	s_outlatch = (uint8_t)((s_outlatch & ~(1u << bit)) | (unsigned)d << bit);
	if (bit == 7)
		sound_reset_w(d);
}

static void speech_squeak_w(int state)
{
	const uint32_t data = 5 | (state ? 2 : 0);
	const uint32_t clk = 14318181u / 2 / (16 - data);
	if (clk != s_tms_clock) {
		tms_sync();
		s_tms_clock = clk;
		tms5220_set_clock(&s_tms, clk);
	}
}

static void soundctl_w(int bit, int d)
{
	if (s_soundctl_clear) {   // (held clear: the bit pulses through, the others stay low)
		if (!d)
			return;
	} else if (((s_soundctl >> bit) & 1) == d)
		return;
	const uint8_t old = s_soundctl;
	s_soundctl = (uint8_t)((s_soundctl & ~(1u << bit)) | (unsigned)d << bit);
	switch (bit) {
	case 0:   // YM2151 reset, active low
		if (d && !(old & 1))
			rbx_ym_reset(0);
		break;
	case 1: tms_sync(); tms5220_wsq_w(&s_tms, d); break;
	case 2: tms_sync(); tms5220_rsq_w(&s_tms, d); break;
	case 3: speech_squeak_w(d); break;
	default: break;   // coin counters
	}
}

static void mixer_w(uint8_t data)
{
	pokey_sync();
	tms_sync();
	s_mixer = data;
	const int g = (int)((data & 7) * 256 * 48 / 700);   // 0.48 x (data & 7) / 7
	rbx_ym_set_gain(0, 0, g);
	rbx_ym_set_gain(0, 1, g);
}

static uint8_t switch_6502_r(void)
{
	uint8_t temp = 0x30;
	if (s_snd_pending) temp ^= 0x80;
	if (s_main_pending) temp ^= 0x40;
	tms_sync();
	if (!tms5220_readyq_r(&s_tms)) temp ^= 0x20;
	if (!(rbx_port_read(s_port[4]) & 0x08)) temp ^= 0x10;
	return temp;
}

static uint8_t snd_r(void *ctx, uint16_t a)
{
	(void)ctx;
	if (a >= 0x4000)
		return s_srom[a];
	if (!(a & 0x1000))
		return s_sram[a & 0xfff];
	switch (a & 0x0830) {
	case 0x0010:   // the sound command
		s_snd_pending = 0;
		m6502_set_nmi(&s_snd, 0);
		return s_soundlatch;
	case 0x0020: return (uint8_t)rbx_port_read(s_port[5]);
	case 0x0030: return switch_6502_r();
	case 0x0800: pokey_sync(); rbx_aux2_sync(); return pokey_read(&s_pokey, a & 0x0f);
	case 0x0810: return s_soundctl & 1 ? rbx_ym_read(0, a & 1) : 0;
	case 0x0830: m6502_set_irq(&s_snd, 0); return 0xff;
	default: return 0xff;
	}
}

static void snd_w(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	if (a >= 0x4000)
		return;
	if (!(a & 0x1000)) {
		s_sram[a & 0xfff] = v;
		return;
	}
	switch (a & 0x0830) {
	case 0x0000:   // the sound response
		s_mainlatch = v;
		s_main_pending = 1;
		main_irq6();
		break;
	case 0x0020: mixer_w(v); break;
	case 0x0030: soundctl_w(a & 7, v >> 7); break;
	case 0x0800: pokey_sync(); pk_push(PK_WRITE | (a & 0x0f), v); break;
	case 0x0810:
		if (s_soundctl & 1)
			rbx_ym_write(0, a & 1, v);
		break;
	case 0x0820: tms_sync(); tms5220_data_w(&s_tms, v); break;
	case 0x0830: m6502_set_irq(&s_snd, 0); break;
	}
}

// the POKEY and the TMS5220 brought up to the 6502's time, at their gains of the mixer
static void pokey_sync(void)
{
	const int64_t now = s_snd.cycles - s_snd_cyc0;
	if (now <= s_pk_done)
		return;
	const int64_t want = rbx_div64_small((uint64_t)now * pokey_rate(&s_pokey), SND_CLOCK);
	const int64_t have = rbx_div64_small((uint64_t)s_pk_done * pokey_rate(&s_pokey), SND_CLOCK);
	s_pk_done = now;
	int n = (int)(want - have);
	if (n <= 0)
		return;
	if (s_pk_n + n > s_pk_cap)
		n = s_pk_cap - s_pk_n;
	if (n <= 0)
		return;
	pk_push(PK_RENDER | (uint32_t)n, (uint32_t)s_pk_n | (uint32_t)((s_mixer >> 3) & 3) << 16);   // (pk_job)
	s_pk_n += n;
}

static void tms_sync(void)
{
	// the 6502's cycles since the last sync in TMS clocks
	const int64_t now = s_snd.cycles - s_snd_cyc0;
	if (now <= s_tms_last)
		return;
	s_tms_acc += (uint64_t)(now - s_tms_last) * s_tms_clock;
	s_tms_last = now;
	const uint32_t clocks = (uint32_t)(s_tms_acc / SND_CLOCK);
	s_tms_acc -= (uint64_t)clocks * SND_CLOCK;
	s_tms_frac += clocks;
	int n = (int)(s_tms_frac / 80);
	s_tms_frac -= (uint32_t)n * 80;
	if (s_tms_n + n > s_tms_cap)
		n = s_tms_cap - s_tms_n;
	if (n <= 0)
		return;
	tms5220_render(&s_tms, s_tms_buf + s_tms_n, n);
	const int g = (s_mixer >> 5) & 7;   // 0.80 x g / 7
	for (int i = 0; i < n; i++)
		s_tms_buf[s_tms_n + i] = (int16_t)(s_tms_buf[s_tms_n + i] * g * 80 / 700);
	s_tms_n += n;
}

// ---- video ----
static inline uint32_t irgb_4444(uint16_t raw)   // IRGB_4444 (standard_irgb_decoder<4,4,4,4, 12,8,4,0>)
{
	const uint32_t i = ((raw >> 12) & 15) * 0x11;
	const uint32_t r = (i * (((raw >> 8) & 15) * 0x11)) >> 8;
	const uint32_t g = (i * (((raw >> 4) & 15) * 0x11)) >> 8;
	const uint32_t b = (i * ((raw & 15) * 0x11)) >> 8;
	return r << 16 | g << 8 | b;
}

static void draw_lines(int y0, int y1)   // screen_update on [y0, y1]
{
	const int scrollx = s_xscroll, scrolly = s_alpha[0xf6e / 2] >> 7;
	// the playfield (64x64 tiles, scanned by columns)
	const uint32_t pfcount = s_gfx_pf.count;
	for (int y = y0; y <= y1; y++) {
		uint16_t *d = s_frame + y * VIS_W;
		const int ty = (y + scrolly) & 511;
		const int row = ty >> 3, py = ty & 7;
		int x = 0;
		int tx = scrollx & 511;
		while (x < VIS_W) {
			const int col = tx >> 3;
			const uint16_t data = s_pf[col * 64 + row];
			const uint32_t code = rbx_wrap((((uint32_t)s_pf_tile_bank * 0x1000) + (data & 0xfff)) ^ 0x800, pfcount);
			const uint16_t color = (uint16_t)(256 + (0x10 + s_pf_color_bank * 8 + ((data >> 12) & 7)) * 16);
			const uint8_t *src = s_gfx_pf.pix + code * 64 + py * 8;
			const int flip = data >> 15;
			for (int px = tx & 7; px < 8 && x < VIS_W; px++, x++, tx++)
				d[x] = (uint16_t)(color + src[flip ? 7 - px : px]);
			tx &= 511;
		}
	}
	// the motion objects, merged
	amo_draw(&s_mo, s_mob, &s_alpha[0xf80 / 2], s_mobm, VIS_W, 0, VIS_W - 1, y0, y1);
	for (int y = y0; y <= y1; y++) {
		const uint16_t *mo = s_mobm + y * VIS_W;
		uint16_t *pf = s_frame + y * VIS_W;
		for (int x = 0; x < VIS_W; x++)
			if (mo[x] != 0xffff) {
				if ((mo[x] & 0x0f) == 1) {   // MO pen 1 clears PF color bit 0x80
					if (!s_vindctr2 || (mo[x] & 0xf0) != 0)
						pf[x] ^= 0x80;
				} else
					pf[x] = mo[x];
			}
	}
	// the alphanumerics (64x31, by rows; pen 0 transparent unless the tile is opaque)
	for (int y = y0; y <= y1; y++) {
		uint16_t *d = s_frame + y * VIS_W;
		const int row = y >> 3, py = y & 7;
		if (row >= 31)
			continue;
		for (int col = 0; col < VIS_W / 8; col++) {
			const uint16_t data = s_alpha[row * 64 + col];
			const uint32_t code = rbx_wrap(data & 0x3ff, s_gfx_an.count);
			const int opaque = data & 0x8000;
			if (!opaque && !s_gfx_an.used[code])
				continue;
			const uint16_t color = (uint16_t)((((data >> 10) & 0x0f) | ((data >> 9) & 0x20)) * 4);
			const uint8_t *src = s_gfx_an.pix + code * 64 + py * 8;
			uint16_t *dd = d + col * 8;
			for (int k = 0; k < 8; k++)
				if (src[k] || opaque)
					dd[k] = (uint16_t)(color + src[k]);
		}
	}
}

static void update_partial(int vpos)   // screen().update_partial(vpos)
{
	if (vpos > VIS_H - 1)
		vpos = VIS_H - 1;
	if (vpos <= s_last_drawn)
		return;
	const unsigned pa = rbx_prof_enter(GT_PA_DRAW);
	draw_lines(s_last_drawn + 1, vpos);
	rbx_prof_leave(pa);
	s_last_drawn = vpos;
}

static void xscroll_w(uint16_t v, uint16_t mask)
{
	const uint16_t old = s_xscroll;
	const uint16_t n = (uint16_t)((s_xscroll & ~mask) | (v & mask));
	if (n != old) {
		update_partial(s_line);
		s_xscroll = n;
		amo_set_xscroll(&s_mo, n & 0x1ff);
	}
}

static void yscroll_w(uint16_t v, uint16_t mask)
{
	uint16_t *ys = &s_alpha[0xf6e / 2];
	const uint16_t old = *ys;
	const uint16_t n = (uint16_t)((old & ~mask) | (v & mask));
	if (n != old) {
		update_partial(s_line);
		*ys = n;
		s_pf_tile_bank = n & 3;
		amo_set_yscroll(&s_mo, (n >> 7) & 0x1ff);
	}
}

// ---- main bus ----
static uint32_t s_slices;   // (m68k_execute calls: with the cycles left, which instruction this is)
static void slap_tap(uint32_t a)
{
	const uint32_t pc = m68k_get_reg(NULL, M68K_REG_PC) & 0xffffff;
	const uint32_t inst = s_slices * 4096u + (uint32_t)m68k_cycles_remaining();
	slapstic_tap(&s_slap, a & 0xffffff, pc, (pc & ~0x280000u) >= 0x38000 && (pc & ~0x280000u) < 0x40000, inst);
}

static uint16_t slap_r(uint32_t a)
{
	const uint16_t v = *(const uint16_t *)(s_rom + 0x38000 + s_slap.current_bank * 0x2000 + (a & 0x1ffe));
	slap_tap(a);
	return v;
}

static uint16_t main_read16(uint32_t a)
{
	a &= 0xffffff;
	if (!(a & 0x800000)) {
		const uint32_t o = a & ~0x280000u;
		if (o >= 0x38000 && o < 0x40000)
			return slap_r(a);
		return o < 0x80000 ? *(const uint16_t *)(s_rom + o) : 0xffff;
	}
	if (a & 0x400000)
		return 0xffff;
	if (!(a & 0x100000)) {   // MBUS
		const uint32_t o = a & 0x3fff;
		if (o < 0x2000) return s_ram[o >> 1];
		if (o < 0x3000) return (uint16_t)(0xff00 | s_eeprom[(o >> 1) & 0x1ff]);
		switch (o & 0x10e) {
		case 0x000: return (uint16_t)rbx_port_read(s_port[0]);
		case 0x002: return (uint16_t)rbx_port_read(s_port[1]);
		case 0x004: return (uint16_t)rbx_port_read(s_port[2]);
		case 0x006: return (uint16_t)rbx_port_read(s_port[3]);
		case 0x008: {
			uint16_t v = (uint16_t)(rbx_port_read(s_port[4]) & ~0x70u);
			if (s_main_pending) v |= 0x10;
			if (s_snd_pending) v |= 0x20;
			if (s_line < VIS_H) v |= 0x40;   // VBLANK, active low
			return v;
		}
		case 0x00e:
			if (!(o & 0x100)) {   // the sound response (read clears its pending)
				s_main_pending = 0;
				main_irq6();
				return (uint16_t)(0xff00 | s_mainlatch);
			}
			return 0xffff;
		default: return 0xffff;
		}
	}
	// VBUS
	switch (a & 0x30000) {
	case 0x00000: {
		const uint32_t o = a & 0x7fff;
		if (o < 0x2000) return s_pf[o >> 1];
		if (o < 0x4000) return s_mob[(o - 0x2000) >> 1];
		if (o < 0x5000) return s_spare[(o - 0x4000) >> 1];
		if (o < 0x6000) return s_alpha[(o - 0x5000) >> 1];
		return 0xffff;
	}
	case 0x10000: return s_pal[(a & 0x7ff) >> 1];
	default: return 0xffff;
	}
}

static uint8_t main_read8(uint32_t a)
{
	const uint16_t v = main_read16(a & ~1u);
	return (uint8_t)(a & 1 ? v : v >> 8);
}

static void main_write16m(uint32_t a, uint16_t v, uint16_t mask)
{
	a &= 0xffffff;
	if (!(a & 0x800000)) {
		const uint32_t o = a & ~0x280000u;
		if (o >= 0x38000 && o < 0x40000)
			slap_tap(a);
		return;
	}
	if (a & 0x400000)
		return;
	if (!(a & 0x100000)) {   // MBUS
		const uint32_t o = a & 0x3fff;
		if (o < 0x2000) {
			s_ram[o >> 1] = (uint16_t)((s_ram[o >> 1] & ~mask) | (v & mask));
			return;
		}
		if (o < 0x3000) {
			if ((mask & 0x00ff) && s_eep_unlocked) {
				s_eeprom[(o >> 1) & 0x1ff] = (uint8_t)v;
				s_eep_unlocked = 0;   // (lock_after_write)
			}
			return;
		}
		switch (o & 0x170) {
		case 0x100: break;   // watchdog
		case 0x120:
			if (mask & 0x00ff)
				outlatch_w((int)((o >> 1) & 7), v & 1);
			break;
		case 0x140: s_irq_vbl = 0; m68k_bus_set_line(4, 0); break;
		case 0x150: s_eep_unlocked = 1; break;
		case 0x170:
			if (mask & 0x00ff) {   // the sound command: NMI on the 6502
				s_soundlatch = (uint8_t)v;
				s_snd_pending = 1;
				m6502_set_nmi(&s_snd, 1);
				s_pq = 716;   // (100 us)
				m68k_end_timeslice();
			}
			break;
		}
		return;
	}
	// VBUS
	switch (a & 0x30000) {
	case 0x00000: {
		const uint32_t o = a & 0x7fff;
		uint16_t *p;
		if (o < 0x2000) p = &s_pf[o >> 1];
		else if (o < 0x4000) p = &s_mob[(o - 0x2000) >> 1];
		else if (o < 0x5000) p = &s_spare[(o - 0x4000) >> 1];
		else if (o < 0x6000) {
			if (o == 0x5f6e) {
				yscroll_w(v, mask);
				return;
			}
			p = &s_alpha[(o - 0x5000) >> 1];
		} else
			return;
		*p = (uint16_t)((*p & ~mask) | (v & mask));
		return;
	}
	case 0x10000: {
		uint16_t *p = &s_pal[(a & 0x7ff) >> 1];
		*p = (uint16_t)((*p & ~mask) | (v & mask));
		return;
	}
	case 0x30000: xscroll_w(v, mask); return;
	default: return;
	}
}

static void main_write16(uint32_t a, uint16_t v) { main_write16m(a, v, 0xffff); }
static void main_write8(uint32_t a, uint8_t v)
{
	main_write16m(a & ~1u, (uint16_t)(v << 8 | v), a & 1 ? 0x00ff : 0xff00);
}

static void map_main(void)
{
	m68k_bus_reset_map();
	static const uint32_t mir[4] = { 0, 0x80000, 0x200000, 0x280000 };
	for (int i = 0; i < 4; i++) {   // the ROM but the slapstic's 0x38000-0x3ffff
		m68k_bus_map(mir[i], mir[i] + 0x37fff, s_rom, 0x38000, 1, 0);
		m68k_bus_map(mir[i] + 0x40000, mir[i] + 0x7ffff, s_rom + 0x40000, 0x40000, 1, 0);
	}
	m68k_bus_map(0x800000, 0x801fff, s_ram, sizeof(s_ram), 1, 1);
	m68k_bus_map(0x900000, 0x901fff, s_pf, sizeof(s_pf), 1, 1);
	m68k_bus_map(0x902000, 0x903fff, s_mob, sizeof(s_mob), 1, 1);
	m68k_bus_map(0x904000, 0x904fff, s_spare, sizeof(s_spare), 1, 1);
	m68k_bus_map(0x905000, 0x905fff, s_alpha, sizeof(s_alpha), 1, 0);   // (writes: the yscroll)
	m68k_bus_map(0x910000, 0x9107ff, s_pal, sizeof(s_pal), 1, 1);
	m68k_bus.read8 = main_read8;
	m68k_bus.read16 = main_read16;
	m68k_bus.write8 = main_write8;
	m68k_bus.write16 = main_write16;
}

// ---- frame ----
static void capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap->frame, s_frame, sizeof(s_snap->frame));
	for (int i = 0; i < 0x400; i++)
		s_snap->pal[i] = irgb_4444(s_pal[i]);
	rbx_video_frame_done();
}

static void gt_render(void)
{
	for (int y = 0; y < VIS_H; y++)
		memcpy(rbx_video.bitmap + y * rbx_video.pitch, s_snap->frame + y * VIS_W, VIS_W * 2);
	memcpy(rbx_video.palette, s_snap->pal, sizeof(s_snap->pal));
}

static void snd_run(int cyc)
{
	if (s_snd_reset)
		s_snd.cycles += cyc;
	else
		m6502_run_until(&s_snd, s_snd.cycles + cyc);
}

static void gt_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	s_snd_cyc0 = s_snd.cycles;
	s_pk_done = s_tms_last = 0;
	s_pk_n = s_tms_n = 0;
	s_last_drawn = -1;
	for (int line = 0; line < LINES; line++) {
		s_line = line;
		if (line == 64 || line == 128 || line == 192)   // the MO force_update timer
			update_partial(line - 1);
		if (line == VIS_H) {   // VIDEO_UPDATE_BEFORE_VBLANK, then the VBLANK IRQ
			update_partial(VIS_H - 1);
			capture();
			s_irq_vbl = 1;
			m68k_bus_set_line(4, 1);
		}
		if ((line & 31) == 0 && (line & 32) && !s_snd_reset)   // the sound IRQ on 32V
			m6502_set_irq(&s_snd, 1);
		// the CPUs in slices; after a sound command (MAME's perfect_quantum of 100 us) in steps of a few clocks
		for (int left = LINE_CYC; left > 0;) {
			int step = s_pq > 0 ? 16 : LINE_CYC / SLICES;
			if (step > left)
				step = left;
			left -= step;
			s_mdebt += step;
			if (s_mdebt > 0) {
				s_slices++;
				rbx_prof_set(GT_PA_68K);
				s_mdebt -= m68k_execute((int)s_mdebt);
			}
			if (s_pq > 0)
				s_pq = s_pq > step ? s_pq - step : 0;
			s_sndacc += step;   // (the 6502 at a quarter of the 68010's clock)
			const int sc = s_sndacc >> 2;
			s_sndacc &= 3;
			if (s_snd_reset)
				s_snd.cycles += sc;
			else {
				rbx_prof_set(GT_PA_6502);
				m6502_run_until(&s_snd, s_snd.cycles + sc);
			}
		}
		// (the POKEY's samples queued every line: the worker keeps up, and a read waits only for the last bit)
		if ((line & 7) == 7 && !s_snd_reset)
			pokey_sync();
		rbx_prof_set(GT_PA_YM);
		s_ym_n += rbx_ym_advance_lr(0, YM_LINE_CLK, s_ym_l + s_ym_n, s_ym_r + s_ym_n, s_ym_cap - s_ym_n);
	}
	pokey_sync();
	rbx_prof_set(GT_PA_TMS);
	tms_sync();
	rbx_prof_set(GT_PA_WAITPK);
	rbx_aux2_sync();   // (the POKEY's samples done)
	rbx_prof_set(RBX_PA_DRIVER);
	rbx_ym_async_take_lr(0, s_ym_l, s_ym_r, s_ym_n);
	// (sample i of the others: i * n / s_ym_n, stepped without a divide)
	int pi = 0, pr = 0, ti = 0, tr = 0;
	for (int i = 0; i < s_ym_n; i++) {
		int32_t o = 0;
		if (s_pk_n)
			o += s_pk_buf[pi];
		if (s_tms_n)
			o += s_tms_buf[ti];
		rbx_audio_put_stereo(s_ym_r[i] + o, s_ym_l[i] + o);   // (YM output 1 to the left)
		for (pr += s_pk_n; pr >= s_ym_n; pr -= s_ym_n)
			pi++;
		for (tr += s_tms_n; tr >= s_ym_n; tr -= s_ym_n)
			ti++;
	}
	memset(s_ym_l, 0, sizeof(int32_t) * (size_t)s_ym_n);
	memset(s_ym_r, 0, sizeof(int32_t) * (size_t)s_ym_n);
	s_ym_n = 0;
}

// ---- init / reset ----
static void gt_reset(void)
{
	memset(s_ram, 0, sizeof(s_ram));
	memset(s_pf, 0, sizeof(s_pf));
	memset(s_mob, 0, sizeof(s_mob));
	memset(s_spare, 0, sizeof(s_spare));
	memset(s_alpha, 0, sizeof(s_alpha));
	memset(s_pal, 0, sizeof(s_pal));
	memset(s_sram, 0, sizeof(s_sram));
	s_xscroll = 0;
	s_pf_tile_bank = 0;
	s_pf_color_bank = s_vindctr2 ? 0 : 1;
	s_mo.bank = 0;
	amo_set_xscroll(&s_mo, 0);
	amo_set_yscroll(&s_mo, 0);
	s_eep_unlocked = 0;
	s_soundlatch = s_mainlatch = 0;
	s_snd_pending = s_main_pending = 0;
	s_irq_vbl = 0;
	slapstic_reset(&s_slap);
	m68k_bus_irq_reset();
	// the LS259s at reset: every output low (the sound CPU held in reset, its mixer at 0)
	s_outlatch = 0;
	s_snd_reset = 1;
	m6502_init(&s_snd, M6502_NMOS);
	for (int p = 0; p < 0x40; p++)
		if (!(p & 0x10)) s_snd.rpage[p] = s_snd.wpage[p] = s_sram + (p & 0x0f) * 256;
	for (int p = 0x40; p < 0x100; p++)
		s_snd.rpage[p] = s_srom + p * 256;
	s_snd.read = snd_r;
	s_snd.write = snd_w;
	s_snd_cyc0 = s_snd.cycles = 0;
	rbx_ym_reset(0);
	rbx_aux2_sync();
	s_pkq_h = s_pkq_t = 0;
	pokey_reset(&s_pokey);
	s_tms_clock = 14318181u / 2 / 11;
	tms5220_set_clock(&s_tms, s_tms_clock);
	tms5220_reset(&s_tms);
	s_soundctl = 0;
	s_soundctl_clear = 1;
	s_mixer = 0;
	rbx_ym_set_gain(0, 0, 0);
	rbx_ym_set_gain(0, 1, 0);
	s_tms_acc = 0;
	s_tms_frac = 0;
	s_pk_done = s_tms_last = 0;
	s_pk_n = s_tms_n = s_ym_n = 0;
	s_mdebt = 0;
	s_pq = s_sndacc = 0;
	s_line = 0;
	s_last_drawn = -1;
	m68k_pulse_reset();
}

static void gt_exit(void);

static int gt_init(const rbx_game *g)
{
	const char *m = g->machine;
	s_vindctr2 = !strncmp(m, "vindctr2", 8);
	s_chip = !strncmp(m, "gaunt2p", 7) ? 107 : !strncmp(m, "gauntlet2", 9) ? 106 : s_vindctr2 ? 118 : 104;
	uint32_t cs, ss;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_srom = rbx_region("audiocpu", &s_srom_size);
	uint8_t *chars = rbx_region("chars", &cs), *spr = rbx_region("spr_tiles", &ss);
	if (!s_rom || s_rom_size < 0x80000 || !s_srom || s_srom_size < 0x10000 || !chars || !spr) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	if (s_vindctr2 && ss >= 0x90000) {   // init_vindctr2: the address lines of the chip at 2J scrambled
		uint8_t *data = (uint8_t *)malloc(0x8000);
		if (!data) {
			rbx_fail("%s: out of memory", g->set);
			return -1;
		}
		memcpy(data, spr + 0x88000, 0x8000);
		for (int i = 0; i < 0x8000; i++) {
			const int srcoffs = (i & 0x4000) | ((i << 11) & 0x3800) | ((i >> 3) & 0x07ff);
			spr[0x88000 + i] = data[srcoffs];
		}
		free(data);
	}
	m68k_bus_swap16(s_rom, s_rom_size);
	{   // gfx_8x8x4_planar (RGN_FRAC(1,4) planes, the highest first) and anlayout
		const uint32_t q = ss / 4 * 8;
		const uint32_t pp[4] = { 3 * q, 2 * q, q, 0 };
		static const uint32_t px[8] = { 0, 1, 2, 3, 4, 5, 6, 7 }, py[8] = { 0, 8, 16, 24, 32, 40, 48, 56 };
		static const uint32_t ap[2] = { 0, 4 }, ax[8] = { 0, 1, 2, 3, 8, 9, 10, 11 };
		static const uint32_t ay[8] = { 0, 16, 32, 48, 64, 80, 96, 112 };
		const rbx_gfx_layout lp = { 8, 8, 4, pp, px, py, 64 };
		const rbx_gfx_layout la = { 8, 8, 2, ap, ax, ay, 128 };
		if (rbx_gfx_decode(&s_gfx_pf, &lp, spr, ss / 4) || rbx_gfx_decode(&s_gfx_an, &la, chars, cs)) {
			rbx_fail("%s: out of memory (graphics)", g->set);
			gt_exit();
			return -1;
		}
	}
	static const amo_config moc = {
		1, 1, 1, 0, 0, 0,   // banks, linked, split, reverse, swapxy, nextneighbor
		8, 1, 0, 0x100, 0,  // slipheight, slipoffset, maxperline, palettebase, transpen
		{ 0, 0, 0, 0x03ff }, { 0x7fff, 0, 0, 0 }, { 0, 0x000f, 0, 0 }, { 0, 0xff80, 0, 0 }, { 0, 0, 0xff80, 0 },
		{ 0, 0, 0x0038, 0 }, { 0, 0, 0x0007, 0 }, { 0, 0, 0x0040, 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, 0
	};
	if (amo_init(&s_mo, &moc, 8, 8)) {
		rbx_fail("%s: out of memory", g->set);
		gt_exit();
		return -1;
	}
	s_mo.gfx[0] = &s_gfx_pf;
	s_mo.granularity[0] = 16;
	for (int i = 0; i < s_mo.codesize; i++)   // video_start: the code XOR
		s_mo.codelookup[i] ^= 0x800;
	if (slapstic_init(&s_slap, s_chip, 0x38000, 0x3ffff, 0x280000, 16)) {
		rbx_fail("%s: slapstic", g->set);
		gt_exit();
		return -1;
	}
	s_frame = (uint16_t *)calloc(VIS_W * VIS_H, sizeof(uint16_t));
	s_mobm = (uint16_t *)calloc(VIS_W * VIS_H, sizeof(uint16_t));
	s_snap = calloc(1, sizeof(*s_snap));
	rbx_ym_config yc;
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM2151;
	yc.clock = YM_CLOCK;
	yc.gain[0] = yc.gain[1] = 123;   // 0.48 (route 1 to the left, 0 to the right: swapped at the mix)
	int rc = !s_frame || !s_mobm || !s_snap || rbx_ym_init(0, &yc) || rbx_ym_set_async(0, 2);
	if (!rc) {
		s_ym_cap = (int)(rbx_ym_rate(0) / 50) + 256;
		s_ym_l = (int32_t *)calloc((size_t)s_ym_cap, sizeof(int32_t));
		s_ym_r = (int32_t *)calloc((size_t)s_ym_cap, sizeof(int32_t));
		pokey_init(&s_pokey, SND_CLOCK);
		tms5220_init(&s_tms, TMS5220_VARIANT_5220C, 14318181u / 2 / 11);
		s_pk_cap = (int)(pokey_rate(&s_pokey) / 50) + 64;
		s_tms_cap = 20000 / 50 + 64;
		s_pk_buf = (int16_t *)calloc((size_t)s_pk_cap, sizeof(int16_t));
		s_pkq_h = s_pkq_t = 0;
		rbx_aux2_set_alone(1);   // (CPU3 on the Vita: the YM is on the first worker, the render thread on CPU2)
		rbx_aux2_set_job(pk_job);
		rbx_aux2_set_spin(300);   // (samples queued every 8 lines: it waits for the next by spinning, not asleep)
		rbx_prof_name(RBX_PA_DRV, s_gt_pa_names, GT_PA_END - RBX_PA_DRV);
		s_tms_buf = (int16_t *)calloc((size_t)s_tms_cap, sizeof(int16_t));
		rc = !s_ym_l || !s_ym_r || !s_pk_buf || !s_tms_buf || rbx_audio_setup_stereo(rbx_ym_rate(0)) ||
		     rbx_video_setup(VIS_W, VIS_H, 0x400);
	}
	if (rc) {
		rbx_fail("%s: out of memory", g->set);
		gt_exit();
		return -1;
	}
	static const char *const names[6] = { "803000", "803002", "803004", "803006", "803008", "COIN" };
	for (int i = 0; i < 6; i++)
		s_port[i] = rbx_port_find(names[i]);
	// the EEPROM: blank (0xff), then the saved file
	memset(s_eeprom, 0xff, sizeof(s_eeprom));
	s_nv_path[0] = 0;
	{
		extern const char *rbx_nvram_dir(void);
		const char *dir = rbx_nvram_dir();
		if (dir) {
			snprintf(s_nv_path, sizeof(s_nv_path), "%s%s.nv", dir, g->set);
			FILE *f = fopen(s_nv_path, "rb");
			if (f) {
				if (fread(s_eeprom, 1, sizeof(s_eeprom), f) != sizeof(s_eeprom))
					memset(s_eeprom, 0xff, sizeof(s_eeprom));
				fclose(f);
			}
		}
	}
	m68k_init();
	m68k_set_cpu_type(M68K_CPU_TYPE_68010);
	m68k_set_mame_timing(1);
	map_main();
	gt_reset();
	return 0;
}

static void gt_exit(void)
{
	rbx_aux2_set_job(NULL);   // (syncs first)
	if (s_nv_path[0]) {
		FILE *f = fopen(s_nv_path, "wb");
		if (f) {
			fwrite(s_eeprom, 1, sizeof(s_eeprom), f);
			fclose(f);
		}
		s_nv_path[0] = 0;
	}
	m68k_set_mame_timing(0);
	rbx_ym_exit(0);
	amo_exit(&s_mo);
	rbx_gfx_free(&s_gfx_pf);
	rbx_gfx_free(&s_gfx_an);
	free(s_frame);
	free(s_mobm);
	free(s_snap);
	free(s_ym_l);
	free(s_ym_r);
	free(s_pk_buf);
	free(s_tms_buf);
	s_frame = s_mobm = NULL;
	s_snap = NULL;
	s_ym_l = s_ym_r = NULL;
	s_pk_buf = s_tms_buf = NULL;
}

// ---- save state ----
static int gt_state(void *ctx, int load)
{
	const uint32_t csize = m68k_context_size();
	uint8_t *cpu = (uint8_t *)malloc(csize);
	if (!cpu)
		return -1;
	if (!load)
		m68k_get_context(cpu);
	uint32_t irq[2] = { m68k_bus_irq_pending(), m68k_bus_cur->irq_asserted };
	int32_t regs[20] = { s_xscroll, s_pf_tile_bank, s_outlatch, s_eep_unlocked, s_soundlatch, s_mainlatch,
	                     s_snd_pending, s_main_pending, s_irq_vbl, s_snd_reset, s_soundctl, s_soundctl_clear, s_mixer,
	                     (int32_t)s_tms_clock, (int32_t)s_tms_frac, (int32_t)s_mdebt, s_mo.xscroll, s_mo.yscroll,
	                     s_mo.bank, s_pq * 4 + s_sndacc };
	uint8_t slap[3] = { s_slap.state, s_slap.current_bank, s_slap.loaded_bank };
	uint64_t acc = s_tms_acc;
	int rc = 0;
	rc |= rbx_state_io(ctx, "m68k", cpu, csize);
	rc |= rbx_state_io(ctx, "irq", irq, sizeof(irq));
	rc |= rbx_state_io(ctx, "ram", s_ram, sizeof(s_ram));
	rc |= rbx_state_io(ctx, "pf", s_pf, sizeof(s_pf));
	rc |= rbx_state_io(ctx, "mob", s_mob, sizeof(s_mob));
	rc |= rbx_state_io(ctx, "spare", s_spare, sizeof(s_spare));
	rc |= rbx_state_io(ctx, "alpha", s_alpha, sizeof(s_alpha));
	rc |= rbx_state_io(ctx, "pal", s_pal, sizeof(s_pal));
	rc |= rbx_state_io(ctx, "eeprom", s_eeprom, sizeof(s_eeprom));
	rc |= rbx_state_io(ctx, "regs", regs, sizeof(regs));
	rc |= rbx_state_io(ctx, "slap", slap, sizeof(slap));
	rc |= rbx_state_io(ctx, "m6502", &s_snd, M6502_STATE_SIZE);
	rc |= rbx_state_io(ctx, "sram", s_sram, sizeof(s_sram));
	rc |= rbx_state_io(ctx, "tmsacc", &acc, sizeof(acc));
	rc |= rbx_ym_state(0, ctx, load);
	rbx_aux2_sync();
	rc |= pokey_state(&s_pokey, ctx, load, "pokey");
	rc |= tms5220_state(&s_tms, ctx, load, "tms");
	if (!rc && load) {
		m68k_set_context(cpu);
		m68k_bus_set_irq_pending(irq[0]);
		m68k_bus_cur->irq_asserted = irq[1];
		m68k_bus_update_irq();
		s_xscroll = (uint16_t)regs[0];
		s_pf_tile_bank = (uint8_t)regs[1];
		s_outlatch = (uint8_t)regs[2];
		s_eep_unlocked = regs[3];
		s_soundlatch = (uint8_t)regs[4];
		s_mainlatch = (uint8_t)regs[5];
		s_snd_pending = regs[6];
		s_main_pending = regs[7];
		s_irq_vbl = regs[8];
		s_snd_reset = regs[9];
		s_soundctl = (uint8_t)regs[10];
		s_soundctl_clear = regs[11];
		s_mixer = (uint8_t)regs[12];
		s_tms_clock = (uint32_t)regs[13];
		s_tms_frac = (uint32_t)regs[14];
		s_mdebt = regs[15];
		s_mo.xscroll = regs[16];
		s_mo.yscroll = regs[17];
		s_mo.bank = regs[18];
		s_pq = regs[19] >> 2;
		s_sndacc = regs[19] & 3;
		s_slap.state = slap[0];
		s_slap.current_bank = slap[1];
		s_slap.loaded_bank = slap[2];
		s_tms_acc = acc;
		const int g = (int)((s_mixer & 7) * 256 * 48 / 700);
		rbx_ym_set_gain(0, 0, g);
		rbx_ym_set_gain(0, 1, g);
	}
	free(cpu);
	return rc ? -1 : 0;
}

static int gt_save(void *ctx) { return gt_state(ctx, 0); }
static int gt_load(void *ctx) { return gt_state(ctx, 1); }
static int gt_supported(const char *machine)
{
	return !strncmp(machine, "gauntlet:", 9) || !strncmp(machine, "gaunt2p:", 8) || !strncmp(machine, "gauntlet2:", 10) ||
	       !strncmp(machine, "vindctr2:", 9);
}

const rbx_driver rbx_drv_gauntlet = { "Atari Gauntlet", gt_init, gt_exit, gt_reset, gt_frame, gt_save, gt_load,
                                      gt_supported, gt_render, NULL };
