/*  Atari System 1: Marble Madness, Peter Pack Rat, Indiana Jones and the Temple of Doom, Road Runner, Road Blasters,
    Relief Pitcher (prototype). Ported from MAME src/mame/atari/atarisy1.cpp, atarisy1.h, atarisy1_v.cpp
    (BSD-3-Clause, copyright-holders: Aaron Giles) and the port side of src/devices/machine/6522via.cpp
    (BSD-3-Clause, copyright-holders: Peter Trauner, Mathis Rosenhauer).
    68010 at 7.16 MHz; a 64x64 playfield and the motion objects whose graphics banks and colors come from the
    cartridge's mapping PROMs; a 64x32 alphanumerics layer; the slapstic (103, 105, 107-110) on 0x80000-0x87fff;
    an ADC0809 (joysticks, pedal) with its IRQ 2, the trackballs of Marble Madness, the scanline IRQ 3 of the later
    boards (timers in the motion object list). Sound: 6502 at 1.79 MHz with a YM2151 and a POKEY, and on the speech
    boards a TMS5220C behind a 6522 VIA.
    The screen is drawn in pieces up to the beam, as MAME's partial updates (recorded by the emulation, drawn on the
    render thread). The YM2151 makes its samples on the rbx_aux worker.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m68k_bus.h"
#include "../../cpu/m68k/m68k.h"
#include "../../cpu/m6502/m6502.h"
#include "../../machine/adc0808.h"
#include "../../sound/rbx_ym.h"
#include "../../sound/pokey.h"
#include "../../sound/tms5220.h"
#include "../../video/gfxlayout.h"
#include "../video/atarimo.h"
#include "../machine/slapstic.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define SND_CLOCK 1789772
#define YM_CLOCK 3579545
#define LINE_CYC 456
#define SND_LINE_CYC 114
#define YM_LINE_CLK 228
#define LINES 262
#define VIS_W 336
#define VIS_H 240
#define FRAME_MHZ 59923
#define SLICES 4
#define NGFX 24

enum { ADC_NONE, ADC_DIGITAL, ADC_ROADRUNN, ADC_ROADBLST, ADC_RELIEF };

// ---- configuration ----
static int s_chip, s_tb_type, s_r, s_speech, s_adc_mode;
static uint8_t *s_rom, *s_srom;
static uint32_t s_rom_size, s_srom_size;
static rbx_gfx s_gfx[NGFX];         // [0] the alphanumerics, then the banks decoded for the PROMs
static uint8_t s_color_shift[NGFX];
static int s_ngfx;
static uint16_t s_pflookup[256];
static amo_t s_mo;
static slapstic_t s_slap;
static int s_port[7];               // IN0..IN3, F60000, 1820

// ---- main memory ----
static uint16_t s_ram[0x1000];      // 400000-401fff
static uint16_t *s_cart;            // 900000-9fffff
static uint16_t s_pf[0x1000];       // a00000-a01fff
static uint16_t s_mob[0x800];       // a02000-a02fff
static uint16_t s_alpha[0x800];     // a03000-a03fff
static uint16_t s_pal[0x400];       // b00000-b007ff
static uint8_t s_eeprom[0x200];
static int s_eep_unlocked;
static char s_nv_path[512];

// ---- video / machine state (saved in regs) ----
static uint16_t s_xscroll, s_yscroll, s_prio_pens;
static int s_scrolly;               // the playfield's scroll as the tilemap has it (adjusted for the beam)
static int s_yreset;                // reset_yscroll_callback pending (-1 none)
static uint8_t s_bankselect, s_tile_bank;
static int s_next_timer, s_timer_armed, s_timer_wrap, s_int3_state, s_int3off;
static int s_irq_vbl, s_ajs_in0;
static uint8_t s_cur[2][2];
static int s_adcval[8];
static adc0808_t s_adc;
static int s_line, s_last_drawn;
static int64_t s_mdebt;
static int s_pq, s_sndacc;   // perfect quantum: main clocks left; the 6502's clocks carried (x 4)

// the latches
static uint8_t s_soundlatch, s_mainlatch;
static int s_snd_pending, s_main_pending;

// ---- sound ----
static m6502_t s_snd;
static uint8_t s_sram[0x1000];
static int s_snd_reset;
static uint8_t s_outlatch;          // the sound LS259 (q0: YM2151 reset)
static int s_ym_irq;
static pokey_t s_pokey;
static tms5220_t s_tms;
static uint32_t s_tms_clock;
static struct { uint8_t out_a, ddr_a, out_b, ddr_b, reg[16]; } s_via;
static int64_t s_snd_cyc0, s_pk_done, s_tms_last;
static uint64_t s_tms_acc;
static uint32_t s_tms_frac;
static int32_t *s_ym_l, *s_ym_r;
static int s_ym_n, s_ym_cap;
static int16_t *s_pk_buf, *s_tms_buf;
static int s_pk_n, s_pk_cap, s_tms_n, s_tms_cap;

// ---- video buffers ----
static uint16_t *s_mobm;             // (the render thread's MO bitmap)

static void update_partial(int vpos);
static void pokey_sync(void);
static void tms_sync(void);

// ---- interrupts ----
static void irq2_update(void) { m68k_bus_set_line(2, s_ajs_in0 && s_adc.eoc); }
static void main_irq6(void) { m68k_bus_set_line(6, s_main_pending); }

// ---- the scanline timers (atarisy1r_state::update_timers) ----
static void update_timers(int scanline)
{
	if (!s_r)
		return;
	const int offset = s_mo.bank * 64 * 4;
	int link = 0, best = scanline, found = 0;
	uint8_t visit[64];
	memset(visit, 0, sizeof(visit));
	while (!visit[link]) {
		if (s_mob[(offset + link + 0x40) & 0x7ff] == 0xffff) {   // timers: 0xffff in entry 2
			const int data = s_mob[(offset + link) & 0x7ff];
			const int vsize = (data & 15) + 1;
			const int ypos = (256 - (data >> 5) - vsize * 8 - 1) & 0x1ff;
			found = 1;
			if (best <= scanline) {
				if ((ypos <= scanline && ypos < best) || ypos > scanline)
					best = ypos;
			} else if (ypos < best)
				best = ypos;
		}
		visit[link] = 1;
		link = s_mob[(offset + link + 0xc0) & 0x7ff] & 0x3f;
	}
	if (!found)
		best = -1;
	if (best != s_next_timer) {
		s_next_timer = best;
		s_timer_armed = best != -1;
		s_timer_wrap = best <= s_line;   // (time_until_pos: a line not ahead of the beam is the next frame's)
	}
}

static void int3_callback(int scanline)
{
	s_int3_state = 1;
	m68k_bus_set_line(3, 1);
	s_int3off = scanline + 1;   // (one scan period later)
	s_next_timer = -1;
	s_timer_armed = 0;
	update_timers(scanline);
}

// ---- the sound side ----
static void ym_irq(int state)
{
	s_ym_irq = state;
	m6502_set_irq(&s_snd, state);
}

static void outlatch_clear(void)
{
	s_outlatch = 0;   // (q0 low: the YM2151 held in reset)
}

static void outlatch_w(int bit, int d)
{
	if (!(s_bankselect & 0x80)) {   // held clear
		if (!d)
			return;
	} else if (((s_outlatch >> bit) & 1) == d)
		return;
	const uint8_t old = s_outlatch;
	s_outlatch = (uint8_t)((s_outlatch & ~(1u << bit)) | (unsigned)d << bit);
	if (bit == 0 && d && !(old & 1))
		rbx_ym_reset(0);
}

static void via_pb_out(uint8_t data)   // via_pb_w
{
	tms_sync();
	tms5220_wsq_w(&s_tms, data & 1);
	tms5220_rsq_w(&s_tms, (data & 2) >> 1);
	const uint32_t d = 5 | ((data >> 3) & 2);
	const uint32_t clk = 14318181u / 2 / (16 - d);
	if (clk != s_tms_clock) {
		s_tms_clock = clk;
		tms5220_set_clock(&s_tms, clk);
	}
}
static void via_pa_out(uint8_t data) { tms_sync(); tms5220_data_w(&s_tms, data); }

static void via_reset(void)
{
	s_via.out_a = s_via.ddr_a = s_via.out_b = s_via.ddr_b = 0;
	memset(s_via.reg, 0, sizeof(s_via.reg));
	if (s_speech) {
		via_pa_out(0xff);
		via_pb_out(0xff);
	}
}

static uint8_t via_r(int r)
{
	switch (r) {
	case 0: {   // PB
		uint8_t pb = (uint8_t)~s_via.ddr_b;
		if (s_via.ddr_b != 0xff) {
			tms_sync();
			pb &= (uint8_t)(tms5220_readyq_r(&s_tms) << 2 | tms5220_intq_r(&s_tms) << 3);
		}
		return (uint8_t)(pb | (s_via.out_b & s_via.ddr_b));
	}
	case 1: case 15:   // PA
		tms_sync();
		return (uint8_t)((~s_via.ddr_a & tms5220_status_r(&s_tms)) | (s_via.out_a & s_via.ddr_a));
	case 2: return s_via.ddr_b;
	case 3: return s_via.ddr_a;
	default: return s_via.reg[r];
	}
}

static void via_w(int r, uint8_t v)
{
	switch (r) {
	case 0:
		s_via.out_b = v;
		if (s_via.ddr_b)
			via_pb_out((uint8_t)((s_via.out_b & s_via.ddr_b) | ~s_via.ddr_b));
		break;
	case 1: case 15:
		s_via.out_a = v;
		if (s_via.ddr_a)
			via_pa_out((uint8_t)((s_via.out_a & s_via.ddr_a) | ~s_via.ddr_a));
		break;
	case 2:
		if (v != s_via.ddr_b) {
			s_via.ddr_b = v;
			via_pb_out((uint8_t)((s_via.out_b & s_via.ddr_b) | ~s_via.ddr_b));
		}
		break;
	case 3:
		if (v != s_via.ddr_a) {
			s_via.ddr_a = v;
			via_pa_out((uint8_t)((s_via.out_a & s_via.ddr_a) | ~s_via.ddr_a));
		}
		break;
	default: s_via.reg[r] = v; break;   // (the timers and the shift register are not wired here)
	}
}

static uint8_t snd_r(void *ctx, uint16_t a)
{
	(void)ctx;
	if (a >= 0x4000)
		return s_srom[a];
	if (!(a & 0x1000))
		return s_sram[a & 0xfff];
	if (!(a & 0x0800))
		return s_speech ? via_r(a & 0x0f) : 0;
	switch (a & 0x0070) {
	case 0x00: return s_outlatch & 1 ? rbx_ym_read(0, a & 1) : 0;
	case 0x10:
		s_snd_pending = 0;
		m6502_set_nmi(&s_snd, 0);
		return s_soundlatch;
	case 0x20: {   // switch_6502_r
		uint8_t t = (uint8_t)(rbx_port_read(s_port[5]) & ~0x18u);
		if (s_snd_pending) t |= 0x08;
		if (s_main_pending) t |= 0x10;
		if (!(rbx_port_read(s_port[4]) & 0x40)) t ^= 0x80;
		return t;
	}
	case 0x70: pokey_sync(); return pokey_read(&s_pokey, a & 0x0f);
	default: return 0;
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
	if (!(a & 0x0800)) {
		if (s_speech)
			via_w(a & 0x0f, v);
		return;
	}
	switch (a & 0x0070) {
	case 0x00:
		if (s_outlatch & 1)
			rbx_ym_write(0, a & 1, v);
		break;
	case 0x10:
		s_mainlatch = v;
		s_main_pending = 1;
		main_irq6();
		break;
	case 0x20: outlatch_w(a & 7, v & 1); break;
	case 0x70: pokey_sync(); pokey_write(&s_pokey, a & 0x0f, v); break;
	}
}

static void pokey_sync(void)
{
	const int64_t now = s_snd.cycles - s_snd_cyc0;
	if (now <= s_pk_done)
		return;
	const int64_t want = rbx_div64_small((uint64_t)now * pokey_rate(&s_pokey), SND_CLOCK);
	const int64_t have = rbx_div64_small((uint64_t)s_pk_done * pokey_rate(&s_pokey), SND_CLOCK);
	s_pk_done = now;
	int n = (int)(want - have);
	if (s_pk_n + n > s_pk_cap)
		n = s_pk_cap - s_pk_n;
	if (n <= 0)
		return;
	pokey_render(&s_pokey, s_pk_buf + s_pk_n, n);
	for (int i = 0; i < n; i++)   // 0.24
		s_pk_buf[s_pk_n + i] = (int16_t)(s_pk_buf[s_pk_n + i] * 24 / 100);
	s_pk_n += n;
}

static void tms_sync(void)
{
	if (!s_speech)
		return;
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
	for (int i = 0; i < n; i++)   // 0.6
		s_tms_buf[s_tms_n + i] = (int16_t)(s_tms_buf[s_tms_n + i] * 6 / 10);
	s_tms_n += n;
}

// ---- bank select ----
static void bankselect_w(uint8_t data)
{
	const uint8_t diff = s_bankselect ^ data;
	if (diff & 0x80) {
		if (!(data & 0x80)) {   // the sound CPU held in reset, its latch cleared, the VIA reset
			outlatch_clear();
			s_snd_reset = 1;
			s_main_pending = 0;
			main_irq6();
			s_bankselect = data;   // (the VIA's outputs at its reset)
			via_reset();
		} else if (s_snd_reset) {
			s_snd_reset = 0;
			m6502_reset(&s_snd);
			m6502_set_irq(&s_snd, s_ym_irq);
			m6502_set_nmi(&s_snd, s_snd_pending);
		}
	}
	if (diff & 0x3c)
		update_partial(s_line);
	s_mo.bank = (data >> 3) & 7;
	s_bankselect = data;
	update_timers(s_line);
	if (diff & 0x04)
		s_tile_bank = (data >> 2) & 1;
}

// ---- the ADC and the trackballs ----
static void adc_start(int offset)
{
	s_adc.addr = (uint8_t)(offset & 7);
	adc0808_start(&s_adc, offset & 7);
	s_ajs_in0 = !((offset >> 3) & 1);
	irq2_update();
}

static int adc_input(int ch)
{
	switch (s_adc_mode) {
	case ADC_DIGITAL: return ch < 4 ? ((rbx_port_read(s_port[0]) >> (7 - ch)) & 1 ? 0xf0 : 0x00) : 0;
	case ADC_ROADRUNN: return ch == 6 ? s_adcval[0] : ch == 7 ? s_adcval[1] : 0;
	case ADC_ROADBLST: return ch == 2 ? s_adcval[1] : 0;
	case ADC_RELIEF: return ch >= 4 ? s_adcval[ch - 4] : 0;
	default: return 0xff;
	}
}

static void adc_advance(int clocks)
{
	if (adc0808_advance(&s_adc, clocks)) {
		s_adc.sar = (uint8_t)adc_input(s_adc.addr);
		irq2_update();
	}
}

static uint16_t trakball_r(int offset)
{
	int result = 0xff;
	if (s_tb_type == 1) {   // Marble Madness: rotated 45 degrees
		const int player = (offset >> 1) & 1, which = offset & 1;
		if (which == 0) {
			const uint8_t posx = (uint8_t)s_adcval[player * 2], posy = (uint8_t)s_adcval[player * 2 + 1];
			s_cur[player][0] = (uint8_t)(posx + posy);
			s_cur[player][1] = (uint8_t)(posx - posy);
		}
		result = s_cur[player][which];
	} else if (s_tb_type == 2)   // Road Blasters' wheel
		result = s_adcval[0];
	return (uint16_t)result;
}

// the analog controls: the trackballs count, the sticks move to the d-pad and back to the centre
static void analog_frame(void)
{
	if (s_tb_type == 1) {
		for (int p = 0; p < 2; p++) {
			s_adcval[p * 2] = (s_adcval[p * 2] - rbx_analog_dpad(p + 1, RBX_IPT_TRACKBALL_X, 4)) & 0xff;   // (reversed)
			s_adcval[p * 2 + 1] = (s_adcval[p * 2 + 1] + rbx_analog_dpad(p + 1, RBX_IPT_TRACKBALL_Y, 4)) & 0xff;
		}
		return;
	}
	for (int i = 0; i < 4; i++) {
		int def = 0x80, lo = 0x10, hi = 0xf0, d = 0;
		const int p = 1 + (i >> 1);
		if (s_adc_mode == ADC_ROADBLST) {
			if (i == 0) {   // the wheel (0x7f, reversed)
				def = 0x40, lo = 0, hi = 0x7f;
				d = -rbx_analog_dpad(1, RBX_IPT_AD_STICK_X, 4);
			} else if (i == 1) {   // the pedal: button 1 or the stick
				const int ped = rbx_analog(1, RBX_IPT_PEDAL);
				s_adcval[1] = (rbx_input(1) & (1u << 4)) || ped > 0 ? 0xff : 0;
				continue;
			} else
				continue;
		} else if (s_adc_mode == ADC_ROADRUNN || s_adc_mode == ADC_RELIEF) {
			if (s_adc_mode == ADC_ROADRUNN && i > 1)
				continue;
			d = (i & 1) ? -rbx_analog_dpad(p, RBX_IPT_AD_STICK_X, 8) : rbx_analog_dpad(p, RBX_IPT_AD_STICK_Y, 8);
		} else
			continue;
		int v = s_adcval[i];
		if (d)
			v += d;
		else if (v < def)
			v = v + 8 > def ? def : v + 8;
		else if (v > def)
			v = v - 8 < def ? def : v - 8;
		s_adcval[i] = v < lo ? lo : v > hi ? hi : v;
	}
}

// ---- video ----
static inline uint32_t irgb_4444(uint16_t raw)
{
	const uint32_t i = ((raw >> 12) & 15) * 0x11;
	const uint32_t r = (i * (((raw >> 8) & 15) * 0x11)) >> 8;
	const uint32_t g = (i * (((raw >> 4) & 15) * 0x11)) >> 8;
	const uint32_t b = (i * ((raw & 15) * 0x11)) >> 8;
	return r << 16 | g << 8 | b;
}

// the playfield's tile lookup by its PROM entry (bank << 7 | the tile's bits 8-14): the graphics, the code's high
// byte, the color (filled at the init from s_pflookup)
static struct { const uint8_t *pix; uint32_t count, codehi; uint16_t color; } s_pft[256];

static void pft_init(void)
{
	for (int i = 0; i < 256; i++) {
		const uint16_t lookup = s_pflookup[i];
		const int gi = (lookup >> 8) & 15;
		s_pft[i].pix = s_gfx[gi].pix;
		s_pft[i].count = s_gfx[gi].count;
		s_pft[i].codehi = (uint32_t)(lookup & 0xff) << 8;
		// (the banks' gfx elements have the color base 256)
		s_pft[i].color = (uint16_t)((gi ? 256 : 0) + (0x20 + (((lookup >> 12) & 15) << s_color_shift[gi])) * 8);
	}
}

// The screen's pieces (MAME's partial updates up to the beam) are recorded by the emulation (the registers and, when
// they changed, the RAM they draw from) and drawn on the render thread: the same pixels
typedef struct {
	int16_t y0, y1;
	uint16_t xscroll, prio;
	int32_t scrolly;
	uint8_t tile_bank;
	uint16_t pf, al, mo;   // their snapshots in the pools
} seg_t;
typedef struct {
	seg_t seg[VIS_H];
	int nseg;
	uint16_t *pf, *al, *mo;   // the snapshots: the playfield (0x1000 words), the alphanumerics (0x800), the MO bank (0x100)
	int npf, nal, nmo, cpf, cal, cmo;
	uint32_t pal[0x400];
} rec_t;
static rec_t *s_rec[2];      // [0] the frame being recorded, [1] the last one (the render thread's)
static int s_pf_dirty, s_al_dirty, s_mo_dirty, s_mo_snapbank;
static amo_t *s_mo_r;        // (the render thread's copy of the MO state: its scratch)

static void draw_lines(const rec_t *r, const seg_t *sg, uint16_t *bm, int pitch)
{
	const int y0 = sg->y0, y1 = sg->y1;
	const int scrollx = sg->xscroll & 511;
	const uint16_t *pfram = r->pf + (size_t)sg->pf * 0x1000, *alram = r->al + (size_t)sg->al * 0x800;
	const int prio_pens = sg->prio;
	for (int y = y0; y <= y1; y++) {   // the playfield (64x64, by rows): a tile's row at a time
		uint16_t *d = bm + y * pitch;
		const int ty = (y + sg->scrolly) & 511;
		const int py = ty & 7;
		const uint16_t *prow = pfram + (ty >> 3) * 64;
		const int tb = sg->tile_bank << 7;
		int x = 0, tx = scrollx;
		while (x < VIS_W) {
			const uint16_t data = prow[tx >> 3];
			const int li = ((data >> 8) & 0x7f) | tb;
			const uint32_t code = rbx_wrap(s_pft[li].codehi | (data & 0xff), s_pft[li].count);
			const uint16_t color = s_pft[li].color;
			const uint8_t *src = s_pft[li].pix + code * 64 + py * 8;
			const int px0 = tx & 7;
			int n = 8 - px0;
			if (n > VIS_W - x)
				n = VIS_W - x;
			uint16_t *dd = d + x;
			if (n == 8) {
				if (data & 0x8000)
					for (int k = 0; k < 8; k++)
						dd[k] = (uint16_t)(color + src[7 - k]);
				else
					for (int k = 0; k < 8; k++)
						dd[k] = (uint16_t)(color + src[k]);
			} else if (data & 0x8000) {
				for (int k = 0; k < n; k++)
					dd[k] = (uint16_t)(color + src[7 - px0 - k]);
			} else {
				for (int k = 0; k < n; k++)
					dd[k] = (uint16_t)(color + src[px0 + k]);
			}
			x += n;
			tx = (tx + n) & 511;
		}
	}
	s_mo_r->bank = 0;   // (the snapshot is the bank's 0x100 words)
	amo_draw(s_mo_r, r->mo + (size_t)sg->mo * 0x100, NULL, s_mobm, VIS_W, 0, VIS_W - 1, y0, y1);
	for (int y = y0; y <= y1; y++) {   // merge the motion objects
		const uint16_t *mo = s_mobm + y * VIS_W;
		uint16_t *pf = bm + y * pitch;
		for (int x = 0; x < VIS_W; x++) {
			if (!(x & 3)) {   // (4 empty pixels at once)
				uint64_t q;
				memcpy(&q, mo + x, 8);
				if (q == ~0ull) {
					x += 3;
					continue;
				}
			}
			const uint16_t m = mo[x];
			if (m == 0xffff)
				continue;
			if (m & 0xf000) {
				if ((m & 0x0f) != 1)
					pf[x] = (uint16_t)(0x300 + ((pf[x] & 0x0f) << 4) + (m & 0x0f));
			} else if ((pf[x] & 0xf8) != 0 || !(prio_pens & (1 << (pf[x] & 0x07))))
				pf[x] = m;
		}
	}
	const rbx_gfx *ag = &s_gfx[0];
	for (int y = y0; y <= y1; y++) {   // the alphanumerics (64x32, by rows)
		uint16_t *d = bm + y * pitch;
		const int row = y >> 3, py = y & 7;
		for (int col = 0; col < VIS_W / 8; col++) {
			const uint16_t data = alram[row * 64 + col];
			const uint32_t code = rbx_wrap(data & 0x3ff, ag->count);
			const int opaque = data & 0x2000;
			if (!opaque && !ag->used[code])
				continue;
			const uint16_t color = (uint16_t)(((data >> 10) & 7) * 4);
			const uint8_t *src = ag->pix + code * 64 + py * 8;
			uint16_t *dd = d + col * 8;
			for (int k = 0; k < 8; k++)
				if (src[k] || opaque)
					dd[k] = (uint16_t)(color + src[k]);
		}
	}
}

// a pool's next snapshot (grown as needed: a few a frame, Road Blasters' MO bank about 60); NULL: out of memory
static uint16_t *snap_add(uint16_t **pool, int *n, int *cap, int words)
{
	if (*n == *cap) {
		const int nc = *cap ? *cap * 2 : 4;
		uint16_t *p = (uint16_t *)realloc(*pool, (size_t)nc * (size_t)words * 2);
		if (!p)
			return NULL;
		*pool = p;
		*cap = nc;
	}
	return *pool + (size_t)(*n)++ * (size_t)words;
}

static void update_partial(int vpos)   // screen().update_partial(vpos): the piece recorded
{
	if (vpos > VIS_H - 1)
		vpos = VIS_H - 1;
	if (vpos <= s_last_drawn)
		return;
	rec_t *r = s_rec[0];
	uint16_t *p;
	if ((s_pf_dirty || !r->npf) && (p = snap_add(&r->pf, &r->npf, &r->cpf, 0x1000)) != NULL) {
		memcpy(p, s_pf, sizeof(s_pf));
		s_pf_dirty = 0;
	}
	if ((s_al_dirty || !r->nal) && (p = snap_add(&r->al, &r->nal, &r->cal, 0x800)) != NULL) {
		memcpy(p, s_alpha, sizeof(s_alpha));
		s_al_dirty = 0;
	}
	if ((s_mo_dirty || !r->nmo || s_mo.bank != s_mo_snapbank) && (p = snap_add(&r->mo, &r->nmo, &r->cmo, 0x100)) != NULL) {
		memcpy(p, s_mob + (s_mo.bank & 7) * 0x100, 0x100 * 2);
		s_mo_dirty = 0;
		s_mo_snapbank = s_mo.bank;
	}
	if (!r->npf || !r->nal || !r->nmo)
		return;   // (out of memory: nothing drawn)
	seg_t *sg = &r->seg[r->nseg++];
	sg->y0 = (int16_t)(s_last_drawn + 1);
	sg->y1 = (int16_t)vpos;
	sg->xscroll = s_xscroll;
	sg->prio = s_prio_pens;
	sg->scrolly = s_scrolly;
	sg->tile_bank = s_tile_bank;
	sg->pf = (uint16_t)(r->npf - 1);
	sg->al = (uint16_t)(r->nal - 1);
	sg->mo = (uint16_t)(r->nmo - 1);
	s_last_drawn = vpos;
}

// the PROMs' graphics banks (decode_gfx, get_bank)
static uint8_t s_bank_gfx[3][8];
static int get_bank(uint8_t prom1, uint8_t prom2, int bpp, const uint8_t *tiles, uint32_t tsize)
{
	int bi;
	if (!(prom1 & 0x10)) bi = 1;
	else if (!(prom1 & 0x20)) bi = 2;
	else if (!(prom1 & 0x40)) bi = 3;
	else if (!(prom1 & 0x80)) bi = 4;
	else if (!(prom2 & 0x40)) bi = 5;
	else if (!(prom2 & 0x80)) bi = (prom2 & 0x08) ? 6 : 7;
	else return 0;
	if (s_bank_gfx[bpp - 4][bi])
		return s_bank_gfx[bpp - 4][bi];
	if (0x80000u * (uint32_t)(bi - 1) >= tsize || s_ngfx >= NGFX)
		return 0;
	static const uint32_t xo[8] = { 0, 1, 2, 3, 4, 5, 6, 7 }, yo[8] = { 0, 8, 16, 24, 32, 40, 48, 56 };
	uint32_t po[6];
	for (int p = 0; p < bpp; p++)
		po[p] = (uint32_t)(bpp - 1 - p) * 8 * 0x10000;
	const rbx_gfx_layout l = { 8, 8, bpp, po, xo, yo, 64 };
	const int gi = s_ngfx;
	if (rbx_gfx_decode(&s_gfx[gi], &l, tiles + 0x80000u * (uint32_t)(bi - 1), 4096 * 8))
		return 0;
	s_ngfx++;
	s_color_shift[gi] = (uint8_t)(bpp - 3);
	return s_bank_gfx[bpp - 4][bi] = (uint8_t)gi;
}

static void decode_gfx(const uint8_t *proms, const uint8_t *tiles, uint32_t tsize, uint16_t *pf, uint16_t *mo)
{
	memset(s_bank_gfx, 0, sizeof(s_bank_gfx));
	const uint8_t *prom1 = proms, *prom2 = proms + 0x200;
	for (int obj = 0; obj < 2; obj++)
		for (int i = 0; i < 256; i++, prom1++, prom2++) {
			int bpp = 4;
			if (*prom2 & 0x10) {
				bpp = 5;
				if (*prom2 & 0x20)
					bpp = 6;
			}
			int offset = *prom1 & 0x0f, color;
			int bank = get_bank(*prom1, *prom2, bpp, tiles, tsize);
			if (obj == 0) {
				color = (~*prom2 & 0x0f) >> (bpp - 4);
				if (bank == 0) {
					bank = 1;
					offset = color = 0;
				}
				pf[i] = (uint16_t)(offset | (bank << 8) | (color << 12));
			} else {
				color = (~*prom2 & 0x07) >> (bpp - 4);
				mo[i] = (uint16_t)(offset | (bank << 8) | (color << 12));
			}
		}
}

// ---- main bus ----
static uint32_t s_slices;   // (m68k_execute calls: with the cycles left, which instruction this is)
static void slap_tap(uint32_t a)
{
	const uint32_t pc = m68k_get_reg(NULL, M68K_REG_PC) & 0xffffff;
	const uint32_t inst = s_slices * 4096u + (uint32_t)m68k_cycles_remaining();
	slapstic_tap(&s_slap, a & 0xffffff, pc, pc >= 0x80000 && pc < 0x88000, inst);
}

static uint16_t slap_r(uint32_t a)
{
	const uint16_t v = *(const uint16_t *)(s_rom + 0x80000 + s_slap.current_bank * 0x2000 + (a & 0x1ffe));
	slap_tap(a);
	return v;
}

static uint16_t main_read16(uint32_t a)
{
	a &= 0xffffff;
	if (a < 0x80000)
		return *(const uint16_t *)(s_rom + a);
	if (a < 0x88000)
		return s_chip ? slap_r(a) : 0;
	switch (a >> 16) {
	case 0x2e: return a < 0x2e0002 ? (uint16_t)(s_int3_state ? 0x0080 : 0) : 0;
	case 0xf0: return a < 0xf00400 ? s_eeprom[(a >> 1) & 0x1ff] : 0;
	case 0xf2: return a < 0xf20008 ? trakball_r((a >> 1) & 3) : 0;
	case 0xf4:
		if (a < 0xf40020) {   // adc_r: the value, then a new conversion
			const uint8_t v = adc0808_data_r(&s_adc);
			adc_start((a >> 1) & 15);
			return v;
		}
		return 0;
	case 0xf6:
		if (a < 0xf60004) {
			uint16_t v = (uint16_t)(rbx_port_read(s_port[4]) & ~0x90u);
			if (s_line < VIS_H) v |= 0x10;   // VBLANK, active low
			if (s_snd_pending) v |= 0x80;
			return v;
		}
		return 0;
	case 0xfc:
		if (a == 0xfc0000) {
			s_main_pending = 0;
			main_irq6();
			return s_mainlatch;
		}
		return 0;
	default: return 0;
	}
}

static uint8_t main_read8(uint32_t a)
{
	if ((a & 0xffffff) == 0xfc0000)
		return 0;
	const uint16_t v = main_read16(a & ~1u);
	return (uint8_t)(a & 1 ? v : v >> 8);
}

static void spriteram_w(uint32_t offset, uint16_t data, uint16_t mask)
{
	const int oldword = s_mob[offset];
	const int newword = (oldword & ~mask) | (data & mask);
	if (oldword != newword && (int)(offset >> 8) == s_mo.bank) {
		if (((offset & 0xc0) == 0x00 && s_mob[offset | 0x40] == 0xffff) ||
		    ((offset & 0xc0) == 0x40 && (newword == 0xffff || oldword == 0xffff))) {
			s_mob[offset] = (uint16_t)newword;
			update_timers(s_line);
		} else
			update_partial(s_line + 2);
	}
	if (oldword != newword)
		s_mo_dirty = 1;
	s_mob[offset] = (uint16_t)newword;
}

static void main_write16m(uint32_t a, uint16_t v, uint16_t mask)
{
	a &= 0xffffff;
	if (a >= 0x80000 && a < 0x88000) {
		if (s_chip)
			slap_tap(a);
		return;
	}
	if (a >= 0xa00000 && a < 0xa04000) {
		if (a >= 0xa02000 && a < 0xa03000)
			spriteram_w((a - 0xa02000) >> 1, v, mask);
		else {   // the playfield, the alphanumerics: their next piece takes a new snapshot
			uint16_t *w = a < 0xa02000 ? &s_pf[(a - 0xa00000) >> 1] : &s_alpha[(a - 0xa03000) >> 1];
			const uint16_t n = (uint16_t)((*w & ~mask) | (v & mask));
			if (n != *w) {
				*w = n;
				if (a < 0xa02000)
					s_pf_dirty = 1;
				else
					s_al_dirty = 1;
			}
		}
		return;
	}
	switch (a >> 16) {
	case 0x80: {   // xscroll
		const uint16_t n = (uint16_t)((s_xscroll & ~mask) | (v & mask));
		if (n != s_xscroll)
			update_partial(s_line);
		s_xscroll = n;
		break;
	}
	case 0x82: {   // yscroll: latched into the scroll base, adjusted for the beam
		const uint16_t n = (uint16_t)((s_yscroll & ~mask) | (v & mask));
		update_partial(s_line);
		int adj = n;
		if (s_line <= VIS_H - 1)
			adj -= s_line + 1;
		s_scrolly = adj;
		s_yreset = n;
		s_yscroll = n;
		break;
	}
	case 0x84: {   // priority pens
		const uint16_t n = (uint16_t)((s_prio_pens & ~mask) | (v & mask));
		if (n != s_prio_pens)
			update_partial(s_line);
		s_prio_pens = n;
		break;
	}
	case 0x86: if (mask & 0x00ff) bankselect_w((uint8_t)v); break;
	case 0x8a: if (mask & 0x00ff) { s_irq_vbl = 0; m68k_bus_set_line(4, 0); } break;
	case 0x8c: if (mask & 0x00ff) s_eep_unlocked = 1; break;
	case 0xf0:
		if (a < 0xf00400 && (mask & 0x00ff) && s_eep_unlocked) {
			s_eeprom[(a >> 1) & 0x1ff] = (uint8_t)v;
			s_eep_unlocked = 0;
		}
		break;
	case 0xf4: if (a < 0xf40020 && (mask & 0x00ff)) adc_start((a >> 1) & 15); break;
	case 0xf8: case 0xfe:
		if (mask & 0x00ff) {   // the sound command
			s_soundlatch = (uint8_t)v;
			s_snd_pending = 1;
			m6502_set_nmi(&s_snd, 1);
			s_pq = 716;   // (100 us)
			m68k_end_timeslice();
		}
		break;
	}
}

static void main_write16(uint32_t a, uint16_t v) { main_write16m(a, v, 0xffff); }
static void main_write8(uint32_t a, uint8_t v) { main_write16m(a & ~1u, (uint16_t)(v << 8 | v), a & 1 ? 0x00ff : 0xff00); }

// ---- frame ----
static void capture(void)
{
	rbx_video_wait_idle();
	rec_t *t = s_rec[1];   // (the frame's pieces to the render thread, the last frame's pools to be refilled)
	s_rec[1] = s_rec[0];
	s_rec[0] = t;
	t->nseg = t->npf = t->nal = t->nmo = 0;
	for (int i = 0; i < 0x400; i++)
		s_rec[1]->pal[i] = irgb_4444(s_pal[i]);
	memcpy(s_mo_r, &s_mo, sizeof(s_mo));
	rbx_video_frame_done();
}

static void sy_render(void)
{
	const rec_t *r = s_rec[1];
	for (int i = 0; i < r->nseg; i++)
		draw_lines(r, &r->seg[i], rbx_video.bitmap, rbx_video.pitch);
	memcpy(rbx_video.palette, r->pal, sizeof(r->pal));
}

static void sy_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	analog_frame();
	s_snd_cyc0 = s_snd.cycles;
	s_pk_done = s_tms_last = 0;
	s_pk_n = s_tms_n = 0;
	s_last_drawn = -1;
	s_timer_wrap = 0;
	for (int line = 0; line < LINES; line++) {
		s_line = line;
		if (line == 0 && s_yreset >= 0) {   // reset_yscroll_callback
			s_scrolly = s_yreset;
			s_yreset = -1;
		}
		if (s_int3off == line) {
			s_int3off = -1;
			s_int3_state = 0;
			m68k_bus_set_line(3, 0);
		}
		if (s_timer_armed && !s_timer_wrap && s_next_timer == line)
			int3_callback(line);
		if (line == 64 || line == 128 || line == 192)   // the MO force_update timer
			update_partial(line - 1);
		if (line == VIS_H) {
			update_partial(VIS_H - 1);
			capture();
			s_irq_vbl = 1;
			m68k_bus_set_line(4, 1);
		}
		// the CPUs in slices; after a sound command (MAME's perfect_quantum of 100 us) in steps of a few clocks
		for (int left = LINE_CYC; left > 0;) {
			int step = s_pq > 0 ? 16 : LINE_CYC / SLICES;
			if (step > left)
				step = left;
			left -= step;
			s_mdebt += step;
			if (s_mdebt > 0) {
				s_slices++;
				s_mdebt -= m68k_execute((int)s_mdebt);
			}
			if (s_pq > 0)
				s_pq = s_pq > step ? s_pq - step : 0;
			s_sndacc += step;   // (the 6502 at a quarter of the 68010's clock)
			const int sc = s_sndacc >> 2;
			s_sndacc &= 3;
			if (s_snd_reset)
				s_snd.cycles += sc;
			else
				m6502_run_until(&s_snd, s_snd.cycles + sc);
				adc_advance(step / 8);   // (the ADC at 14.318181 MHz / 16)
		}
		s_ym_n += rbx_ym_advance_lr(0, YM_LINE_CLK, s_ym_l + s_ym_n, s_ym_r + s_ym_n, s_ym_cap - s_ym_n);
	}
	if (s_int3off >= LINES)
		s_int3off -= LINES;
	pokey_sync();
	tms_sync();
	rbx_ym_async_take_lr(0, s_ym_l, s_ym_r, s_ym_n);
	// (sample i of the others: i * n / s_ym_n, stepped without a divide)
	int pi = 0, pr = 0, ti = 0, tr = 0;
	for (int i = 0; i < s_ym_n; i++) {
		int32_t o = 0;
		if (s_pk_n)
			o += s_pk_buf[pi];
		if (s_tms_n)
			o += s_tms_buf[ti];
		rbx_audio_put_stereo(s_ym_l[i] + o, s_ym_r[i] + o);
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
static void sy_reset(void)
{
	memset(s_ram, 0, sizeof(s_ram));
	memset(s_cart, 0, 0x100000);
	memset(s_pf, 0, sizeof(s_pf));
	memset(s_mob, 0, sizeof(s_mob));
	memset(s_alpha, 0, sizeof(s_alpha));
	memset(s_pal, 0, sizeof(s_pal));
	memset(s_sram, 0, sizeof(s_sram));
	s_xscroll = s_yscroll = s_prio_pens = 0;
	s_scrolly = 0;
	s_yreset = -1;
	s_tile_bank = 0;
	s_mo.bank = 0;
	amo_set_yscroll(&s_mo, 256);
	s_next_timer = -1;
	s_timer_armed = s_timer_wrap = 0;
	s_int3_state = 0;
	s_int3off = -1;
	s_irq_vbl = 0;
	s_eep_unlocked = 0;
	s_soundlatch = s_mainlatch = 0;
	s_snd_pending = s_main_pending = 0;
	memset(s_cur, 0, sizeof(s_cur));
	for (int i = 0; i < 8; i++)
		s_adcval[i] = s_tb_type == 1 ? 0 : s_adc_mode == ADC_ROADBLST ? (i == 0 ? 0x40 : 0) : 0x80;
	adc0808_reset(&s_adc);
	s_ajs_in0 = 0;
	slapstic_reset(&s_slap);
	m68k_bus_irq_reset();
	m6502_init(&s_snd, M6502_NMOS);
	for (int p = 0; p < 0x40; p++)
		if (!(p & 0x10))
			s_snd.rpage[p] = s_snd.wpage[p] = s_sram + (p & 0x0f) * 256;
	for (int p = 0x40; p < 0x100; p++)
		s_snd.rpage[p] = s_srom + p * 256;
	s_snd.read = snd_r;
	s_snd.write = snd_w;
	s_snd_cyc0 = s_snd.cycles = 0;
	s_snd_reset = 1;
	s_ym_irq = 0;
	rbx_ym_reset(0);
	pokey_reset(&s_pokey);
	s_tms_clock = 14318181u / 2 / 11;
	tms5220_set_clock(&s_tms, s_tms_clock);
	tms5220_reset(&s_tms);
	s_tms_acc = 0;
	s_tms_frac = 0;
	s_pk_done = s_tms_last = 0;
	s_pk_n = s_tms_n = s_ym_n = 0;
	s_outlatch = 0;
	s_mdebt = 0;
	s_pq = s_sndacc = 0;
	s_line = 0;
	s_last_drawn = -1;
	s_rec[0]->nseg = s_rec[0]->npf = s_rec[0]->nal = s_rec[0]->nmo = 0;
	s_pf_dirty = s_al_dirty = s_mo_dirty = 1;
	s_bankselect = 0xff;   // (video_start), then machine_reset: bankselect_w(0)
	bankselect_w(0);
	irq2_update();
	m68k_pulse_reset();
}

static void sy_exit(void);

static int sy_init(const rbx_game *g)
{
	const char *m = g->machine;
	s_chip = !strncmp(m, "marble:", 7) ? 103 : !strncmp(m, "peterpak:", 9) ? 107 : !strncmp(m, "indytemp:", 9) ? 105
	       : !strncmp(m, "roadrunn:", 9) ? 108 : !strncmp(m, "roadb109:", 9) ? 109 : !strncmp(m, "roadb110:", 9) ? 110 : 0;
	s_r = !strncmp(m, "roadrunn:", 9) || !strncmp(m, "roadb1", 6) || !strncmp(m, "reliefs1:", 9);
	s_speech = !strncmp(m, "indytemp:", 9) || !strncmp(m, "roadrunn:", 9) || !strncmp(m, "roadb1", 6) ||
	           !strncmp(m, "reliefs1:", 9);
	s_tb_type = !strncmp(m, "marble:", 7) ? 1 : !strncmp(m, "roadb1", 6) ? 2 : 0;
	s_adc_mode = !strncmp(m, "peterpak:", 9) || !strncmp(m, "indytemp:", 9) ? ADC_DIGITAL
	           : !strncmp(m, "roadrunn:", 9) ? ADC_ROADRUNN : !strncmp(m, "roadb1", 6) ? ADC_ROADBLST
	           : !strncmp(m, "reliefs1:", 9) ? ADC_RELIEF : ADC_NONE;
	uint32_t as, ts, ps;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_srom = rbx_region("audiocpu", &s_srom_size);
	uint8_t *alpha = rbx_region("alpha", &as), *tiles = rbx_region("tiles", &ts), *proms = rbx_region("proms", &ps);
	if (!s_rom || s_rom_size < 0x88000 || !s_srom || s_srom_size < 0x10000 || !alpha || !tiles || !proms || ps < 0x400) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	m68k_bus_swap16(s_rom, s_rom_size);
	memset(s_gfx, 0, sizeof(s_gfx));
	memset(s_color_shift, 0, sizeof(s_color_shift));
	{
		static const uint32_t ap[2] = { 0, 4 }, ax[8] = { 0, 1, 2, 3, 8, 9, 10, 11 };
		static const uint32_t ay[8] = { 0, 16, 32, 48, 64, 80, 96, 112 };
		const rbx_gfx_layout la = { 8, 8, 2, ap, ax, ay, 128 };
		if (rbx_gfx_decode(&s_gfx[0], &la, alpha, as)) {
			rbx_fail("%s: out of memory (graphics)", g->set);
			return -1;
		}
		s_ngfx = 1;
	}
	uint16_t motable[256];
	decode_gfx(proms, tiles, ts, s_pflookup, motable);
	pft_init();
	static const amo_config moc = {
		8, 1, 1, 0, 0, 0,     // banks, linked, split, reverse, swapxy, nextneighbor
		0, 0, 0x38, 0x100, 0, // slipheight, slipoffset, maxperline, palettebase, transpen
		{ 0, 0, 0, 0x003f }, { 0, 0xffff, 0, 0 }, { 0, 0xff00, 0, 0 }, { 0, 0, 0x3fe0, 0 }, { 0x3fe0, 0, 0, 0 },
		{ 0 }, { 0x000f, 0, 0, 0 }, { 0x8000, 0, 0, 0 }, { 0 }, { 0, 0, 0x8000, 0 }, { 0 }, { 0 }, { 0, 0xffff, 0, 0 },
		0xffff
	};
	if (amo_init(&s_mo, &moc, 8, 8)) {
		rbx_fail("%s: out of memory", g->set);
		sy_exit();
		return -1;
	}
	for (int i = 0; i < s_mo.codesize; i++)   // video_start
		s_mo.codelookup[i] = (uint32_t)((i & 0xff) | ((motable[i >> 8] & 0xff) << 8));
	for (int i = 0; i < 256; i++) {
		s_mo.colorlookup[i] = (uint8_t)(((motable[i] >> 12) & 15) << 1);
		s_mo.gfxlookup[i] = (uint8_t)((motable[i] >> 8) & 15);
	}
	for (int i = 0; i < NGFX && i < 24; i++) {
		s_mo.gfx[i] = &s_gfx[i];
		s_mo.granularity[i] = i ? 8 : 4;
	}
	if (s_chip && slapstic_init(&s_slap, s_chip, 0x80000, 0x87fff, 0, 16)) {
		rbx_fail("%s: slapstic", g->set);
		sy_exit();
		return -1;
	}
	s_cart = (uint16_t *)calloc(0x80000, sizeof(uint16_t));
	s_mobm = (uint16_t *)calloc(VIS_W * VIS_H, sizeof(uint16_t));
	s_rec[0] = (rec_t *)calloc(1, sizeof(rec_t));
	s_rec[1] = (rec_t *)calloc(1, sizeof(rec_t));
	s_mo_r = (amo_t *)calloc(1, sizeof(amo_t));
	rbx_ym_config yc;
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM2151;
	yc.clock = YM_CLOCK;
	yc.gain[0] = yc.gain[1] = 123;   // 0.48
	yc.irq = ym_irq;
	// (the YM2151's samples made on the worker: the same samples, the frame's largest sound cost)
	int rc = !s_cart || !s_mobm || !s_rec[0] || !s_rec[1] || !s_mo_r || rbx_ym_init(0, &yc) || rbx_ym_set_async(0, 2);
	if (!rc) {
		s_ym_cap = (int)(rbx_ym_rate(0) / 50) + 256;
		s_ym_l = (int32_t *)calloc((size_t)s_ym_cap, sizeof(int32_t));
		s_ym_r = (int32_t *)calloc((size_t)s_ym_cap, sizeof(int32_t));
		pokey_init(&s_pokey, SND_CLOCK);
		tms5220_init(&s_tms, TMS5220_VARIANT_5220C, 14318181u / 2 / 11);
		s_pk_cap = (int)(pokey_rate(&s_pokey) / 50) + 64;
		s_tms_cap = 20000 / 50 + 64;
		s_pk_buf = (int16_t *)calloc((size_t)s_pk_cap, sizeof(int16_t));
		s_tms_buf = (int16_t *)calloc((size_t)s_tms_cap, sizeof(int16_t));
		rc = !s_ym_l || !s_ym_r || !s_pk_buf || !s_tms_buf || rbx_audio_setup_stereo(rbx_ym_rate(0)) ||
		     rbx_video_setup(VIS_W, VIS_H, 0x400);
	}
	if (rc) {
		rbx_fail("%s: out of memory", g->set);
		sy_exit();
		return -1;
	}
	static const char *const names[6] = { "IN0", "IN1", "IN2", "IN3", "F60000", "1820" };
	for (int i = 0; i < 6; i++)
		s_port[i] = rbx_port_find(names[i]);
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
	m68k_bus_reset_map();
	m68k_bus_map(0, 0x7ffff, s_rom, 0x80000, 1, 0);
	m68k_bus_map(0x400000, 0x401fff, s_ram, sizeof(s_ram), 1, 1);
	m68k_bus_map(0x900000, 0x9fffff, s_cart, 0x100000, 1, 1);
	// (writes through the handler: the playfield's and the alphanumerics' for the pieces' snapshots, the MO's for the
	// timers and the partial updates)
	m68k_bus_map(0xa00000, 0xa01fff, s_pf, sizeof(s_pf), 1, 0);
	m68k_bus_map(0xa02000, 0xa02fff, s_mob, sizeof(s_mob), 1, 0);
	m68k_bus_map(0xa03000, 0xa03fff, s_alpha, sizeof(s_alpha), 1, 0);
	m68k_bus_map(0xb00000, 0xb007ff, s_pal, sizeof(s_pal), 1, 1);
	m68k_bus.read8 = main_read8;
	m68k_bus.read16 = main_read16;
	m68k_bus.write8 = main_write8;
	m68k_bus.write16 = main_write16;
	sy_reset();
	return 0;
}

static void sy_exit(void)
{
	rbx_video_wait_idle();
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
	for (int i = 0; i < NGFX; i++)
		rbx_gfx_free(&s_gfx[i]);
	s_ngfx = 0;
	free(s_cart);
	free(s_mobm);
	for (int i = 0; i < 2; i++)
		if (s_rec[i]) {
			free(s_rec[i]->pf);
			free(s_rec[i]->al);
			free(s_rec[i]->mo);
			free(s_rec[i]);
			s_rec[i] = NULL;
		}
	free(s_mo_r);   // (its lookups are s_mo's)
	s_mo_r = NULL;
	free(s_ym_l);
	free(s_ym_r);
	free(s_pk_buf);
	free(s_tms_buf);
	s_cart = s_mobm = NULL;
	s_ym_l = s_ym_r = NULL;
	s_pk_buf = s_tms_buf = NULL;
}

// ---- save state ----
static int sy_state(void *ctx, int load)
{
	const uint32_t csize = m68k_context_size();
	uint8_t *cpu = (uint8_t *)malloc(csize);
	if (!cpu)
		return -1;
	if (!load)
		m68k_get_context(cpu);
	uint32_t irq[2] = { m68k_bus_irq_pending(), m68k_bus_cur->irq_asserted };
	int32_t regs[40] = { s_xscroll, s_yscroll, s_prio_pens, s_scrolly, s_yreset, s_bankselect, s_tile_bank,
	                     s_next_timer, s_timer_armed, s_timer_wrap, s_int3_state, s_int3off, s_irq_vbl, s_ajs_in0,
	                     s_eep_unlocked, s_soundlatch, s_mainlatch, s_snd_pending, s_main_pending, s_snd_reset,
	                     s_outlatch, s_ym_irq, (int32_t)s_tms_clock, (int32_t)s_tms_frac, (int32_t)s_mdebt, s_mo.bank,
	                     s_cur[0][0], s_cur[0][1], s_cur[1][0], s_cur[1][1], s_adcval[0], s_adcval[1], s_adcval[2],
	                     s_adcval[3], s_pq, s_sndacc, 0, 0, 0, 0 };
	uint8_t slap[3] = { s_slap.state, s_slap.current_bank, s_slap.loaded_bank };
	uint64_t acc = s_tms_acc;
	int rc = 0;
	rc |= rbx_state_io(ctx, "m68k", cpu, csize);
	rc |= rbx_state_io(ctx, "irq", irq, sizeof(irq));
	rc |= rbx_state_io(ctx, "ram", s_ram, sizeof(s_ram));
	rc |= rbx_state_io(ctx, "cart", s_cart, 0x100000);
	rc |= rbx_state_io(ctx, "pf", s_pf, sizeof(s_pf));
	rc |= rbx_state_io(ctx, "mob", s_mob, sizeof(s_mob));
	rc |= rbx_state_io(ctx, "alpha", s_alpha, sizeof(s_alpha));
	rc |= rbx_state_io(ctx, "pal", s_pal, sizeof(s_pal));
	rc |= rbx_state_io(ctx, "eeprom", s_eeprom, sizeof(s_eeprom));
	rc |= rbx_state_io(ctx, "regs", regs, sizeof(regs));
	rc |= rbx_state_io(ctx, "slap", slap, sizeof(slap));
	rc |= rbx_state_io(ctx, "adc", &s_adc, sizeof(s_adc));
	rc |= rbx_state_io(ctx, "via", &s_via, sizeof(s_via));
	rc |= rbx_state_io(ctx, "m6502", &s_snd, M6502_STATE_SIZE);
	rc |= rbx_state_io(ctx, "sram", s_sram, sizeof(s_sram));
	rc |= rbx_state_io(ctx, "tmsacc", &acc, sizeof(acc));
	rc |= rbx_ym_state(0, ctx, load);
	rc |= pokey_state(&s_pokey, ctx, load, "pokey");
	rc |= tms5220_state(&s_tms, ctx, load, "tms");
	if (!rc && load) {
		m68k_set_context(cpu);
		m68k_bus_set_irq_pending(irq[0]);
		m68k_bus_cur->irq_asserted = irq[1];
		m68k_bus_update_irq();
		s_xscroll = (uint16_t)regs[0];
		s_yscroll = (uint16_t)regs[1];
		s_prio_pens = (uint16_t)regs[2];
		s_scrolly = regs[3];
		s_yreset = regs[4];
		s_bankselect = (uint8_t)regs[5];
		s_tile_bank = (uint8_t)regs[6];
		s_next_timer = regs[7];
		s_timer_armed = regs[8];
		s_timer_wrap = regs[9];
		s_int3_state = regs[10];
		s_int3off = regs[11];
		s_irq_vbl = regs[12];
		s_ajs_in0 = regs[13];
		s_eep_unlocked = regs[14];
		s_soundlatch = (uint8_t)regs[15];
		s_mainlatch = (uint8_t)regs[16];
		s_snd_pending = regs[17];
		s_main_pending = regs[18];
		s_snd_reset = regs[19];
		s_outlatch = (uint8_t)regs[20];
		s_ym_irq = regs[21];
		s_tms_clock = (uint32_t)regs[22];
		s_tms_frac = (uint32_t)regs[23];
		s_mdebt = regs[24];
		s_mo.bank = regs[25];
		s_cur[0][0] = (uint8_t)regs[26];
		s_cur[0][1] = (uint8_t)regs[27];
		s_cur[1][0] = (uint8_t)regs[28];
		s_cur[1][1] = (uint8_t)regs[29];
		for (int i = 0; i < 4; i++)
			s_adcval[i] = regs[30 + i];
		s_pq = regs[34];
		s_sndacc = regs[35];
		s_slap.state = slap[0];
		s_slap.current_bank = slap[1];
		s_slap.loaded_bank = slap[2];
		s_tms_acc = acc;
	}
	free(cpu);
	return rc ? -1 : 0;
}

static int sy_save(void *ctx) { return sy_state(ctx, 0); }
static int sy_load(void *ctx) { return sy_state(ctx, 1); }
static int sy_supported(const char *machine)
{
	static const char *const ok[] = { "marble:", "peterpak:", "indytemp:", "roadrunn:", "roadb109:", "roadb110:",
	                                  "reliefs1:" };
	for (unsigned i = 0; i < sizeof(ok) / sizeof(ok[0]); i++)
		if (!strncmp(machine, ok[i], strlen(ok[i])))
			return 1;
	return 0;
}

const rbx_driver rbx_drv_atarisy1 = { "Atari System 1", sy_init, sy_exit, sy_reset, sy_frame, sy_save, sy_load,
                                      sy_supported, sy_render, NULL };
