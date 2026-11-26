/*  SNK Neo Geo MVS. Ported from MAME src/mame/snk/neogeo.cpp, neogeo_v.cpp, neogeo_spr.cpp (BSD-3-Clause,
    copyright-holders: Bryan McPhail, Ernesto Corvi, Andrew Prime, Zsolt Vasvari), the uPD4990A from
    src/devices/machine/upd1990a.cpp (BSD-3-Clause, copyright-holders: Curt Coder) and the cartridge devices of
    src/devices/bus/neogeo (BSD-3-Clause).
    A 68000 at 12 MHz with the MVS BIOS, a Z80 at 4 MHz (the cartridge's banks: NEO-ZMC), a YM2610; the LSPC: 381
    sprites of 16 pixel columns with zoom (96 a line), the 8x8 fixed layer, 2 palette banks of 4096 colours with a
    shadow, the display position IRQ. MAME draws each line at the start of the next one from the video RAM of that
    time: here the lines are drawn when the RAM or the state they show changes (on the emulation thread, up to the
    line before), the rest at the vblank from a snapshot (in bands). 384x264 at 6 MHz (59.19 Hz), 320x224 shown.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m68k_bus.h"
#include "../../cpu/m68k/m68k.h"
#include "../../cpu/z80/z80.h"
#include "../../sound/rbx_ym.h"
#include "../../rbx_core.h"
#include "../machine/neoprot.h"

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define CPU_CLOCK 12000000
#define Z80_CLOCK 4000000
#define YM_CLOCK 8000000
#define FRAME_MHZ 59186          // 6 MHz / (384 x 264)
#define LINES 264
#define LINE_CYC 768             // the 68000's cycles a line (2 a pixel)
#define VIS_Y0 16                // NEOGEO_VBEND
#define VIS_Y1 240               // NEOGEO_VBSTART
#define VIS_W 320
#define VIS_H 224
#define WATCHDOG_CYC 1622015     // (3244030 master clocks)

enum { Q_SHADOW = 0, Q_CART_VECTORS = 1, Q_CART_AUDIO = 5, Q_SRAM_UNLOCK = 6, Q_PAL_BANK = 7 };
enum { IRQ2_ENABLE = 4, IRQ2_LOAD_RELATIVE = 5, IRQ2_AUTOLOAD_VBLANK = 6, IRQ2_AUTOLOAD_REPEAT = 7 };

// the cartridges (the machine configurations of neogeo.cpp, their slot defaults in src/devices/bus/neogeo)
enum { C_ROM, C_FATFURY2, C_KOF98, C_MSLUGX, C_CMC42, C_CMC50, C_SMA, C_PCM2, C_K2K2, C_PVC };
enum { P_NONE, P_KOF99, P_GAROU, P_GAROUH, P_MSLUG3, P_MSLUG3A, P_KOF2000,   // (SMA)
       P_KOF2002, P_SAMSHO5, P_SAMSH5SP,                                     // (K2K2: the P ROM)
       P_MSLUG5, P_SVC, P_KOF2003, P_KOF2003H };                             // (PVC)
typedef struct {
	const char *machine;
	uint8_t type, prot, key, fixbank;   // fixbank: the fixed layer banks (1 Garou, 2 KoF 2000)
	int8_t pcm2;                        // NEO-PCM2: the address lines (> 0) or the swap table (< 0: -1 - value)
	uint8_t m1, sfix;                   // the CMC50 M1; the S data from the sprites
} cart_t;
static const cart_t s_carts[] = {
	{ "neobase", C_ROM, 0, 0, 0, 0, 0, 0 },
	{ "fatfur2", C_FATFURY2, 0, 0, 0, 0, 0, 0 },
	{ "kof98", C_KOF98, 0, 0, 0, 0, 0, 0 },
	{ "mslugx", C_MSLUGX, 0, 0, 0, 0, 0, 0 },
	{ "zupapa", C_CMC42, 0, NEO_ZUPAPA_GFX_KEY, 1, 0, 0, 1 },
	{ "mslug3h", C_CMC42, 0, NEO_MSLUG3_GFX_KEY, 1, 0, 0, 1 },
	{ "ganryu", C_CMC42, 0, NEO_GANRYU_GFX_KEY, 1, 0, 0, 1 },
	{ "s1945p", C_CMC42, 0, NEO_S1945P_GFX_KEY, 1, 0, 0, 1 },
	{ "preisle2", C_CMC42, 0, NEO_PREISLE2_GFX_KEY, 1, 0, 0, 1 },
	{ "bangbead", C_CMC42, 0, NEO_BANGBEAD_GFX_KEY, 1, 0, 0, 1 },
	{ "nitd", C_CMC42, 0, NEO_NITD_GFX_KEY, 1, 0, 0, 1 },
	{ "sengoku3", C_CMC42, 0, NEO_SENGOKU3_GFX_KEY, 1, 0, 0, 1 },
	{ "kof99k", C_CMC42, 0, NEO_KOF99_GFX_KEY, 1, 0, 0, 1 },
	{ "kof2001", C_CMC50, 0, NEO_KOF2001_GFX_KEY, 1, 0, 1, 1 },
	{ "kof2000n", C_CMC50, 0, NEO_KOF2000_GFX_KEY, 2, 0, 1, 1 },
	{ "kof99", C_SMA, P_KOF99, NEO_KOF99_GFX_KEY, 1, 0, 0, 1 },
	{ "garou", C_SMA, P_GAROU, NEO_GAROU_GFX_KEY, 1, 0, 0, 1 },
	{ "garouh", C_SMA, P_GAROUH, NEO_GAROU_GFX_KEY, 1, 0, 0, 1 },
	{ "mslug3", C_SMA, P_MSLUG3, NEO_MSLUG3_GFX_KEY, 1, 0, 0, 1 },
	{ "mslug3a", C_SMA, P_MSLUG3A, NEO_MSLUG3_GFX_KEY, 1, 0, 0, 1 },
	{ "kof2000", C_SMA, P_KOF2000, NEO_KOF2000_GFX_KEY, 2, 0, 1, 1 },
	{ "mslug4", C_PCM2, 0, NEO_MSLUG4_GFX_KEY, 1, 8, 1, 1 },
	{ "ms4plus", C_PCM2, 0, NEO_MSLUG4_GFX_KEY, 0, 8, 1, 0 },
	{ "rotd", C_PCM2, 0, NEO_ROTD_GFX_KEY, 1, 16, 1, 1 },
	{ "pnyaa", C_PCM2, 0, NEO_PNYAA_GFX_KEY, 1, 4, 1, 1 },
	{ "kof2002", C_K2K2, P_KOF2002, NEO_KOF2002_GFX_KEY, 0, -1 - 0, 1, 1 },
	{ "kf2k2pls", C_K2K2, P_KOF2002, NEO_KOF2002_GFX_KEY, 0, -1 - 0, 1, 0 },
	{ "matrim", C_K2K2, P_KOF2002, NEO_MATRIM_GFX_KEY, 2, -1 - 1, 1, 1 },
	{ "samsho5", C_K2K2, P_SAMSHO5, NEO_SAMSHO5_GFX_KEY, 1, -1 - 4, 1, 1 },
	{ "samsh5sp", C_K2K2, P_SAMSH5SP, NEO_SAMSHO5SP_GFX_KEY, 1, -1 - 6, 1, 1 },
	{ "mslug5", C_PVC, P_MSLUG5, NEO_MSLUG5_GFX_KEY, 1, -1 - 2, 1, 1 },
	{ "svc", C_PVC, P_SVC, NEO_SVC_GFX_KEY, 2, -1 - 3, 1, 1 },
	{ "kof2003", C_PVC, P_KOF2003, NEO_KOF2003_GFX_KEY, 2, -1 - 5, 1, 1 },
	{ "kof2003h", C_PVC, P_KOF2003H, NEO_KOF2003_GFX_KEY, 2, -1 - 5, 1, 1 },
	{ NULL, 0, 0, 0, 0, 0, 0, 0 },
};
static const cart_t *s_cart;
static uint16_t s_kof98_default[2];

// ---- ROMs ----
static uint8_t *s_rom, *s_bios, *s_zrom, *s_zbios, *s_spr, *s_fix, *s_fixbios, *s_zoomy, *s_adpcma, *s_adpcmb;
static uint32_t s_rom_size, s_bios_size, s_zrom_size, s_zbios_size, s_spr_size, s_fix_size, s_fixbios_size,
                s_zoomy_size, s_adpcma_size, s_adpcmb_size;
static uint32_t s_spr_mask;      // (the pixel address mask: the sprite data's size in pixels rounded up to a power of 2)
static uint8_t s_page0[0x1000];  // 0x000000 .. 0x000fff: the vectors (BIOS or cartridge), then the cartridge
static int s_port_dsw, s_port_system, s_port_audio, s_port_test, s_port_joy[2], s_port_start;

// ---- the uPD4990A (serial mode, clocked by a 32768 Hz crystal) ----
enum { RTC_HOLD = 0, RTC_SHIFT, RTC_TIME_SET, RTC_TIME_READ, RTC_TP_64HZ, RTC_TP_256HZ, RTC_TP_2048HZ, RTC_TP_4096HZ,
       RTC_TP_1S_INT, RTC_TP_10S_INT, RTC_TP_30S_INT, RTC_TP_60S_INT, RTC_INT_RESET, RTC_INT_RUN, RTC_INT_STOP, RTC_TEST };
typedef struct {
	uint8_t time_counter[6], shift_reg[7];
	uint8_t oe, cs, stb, data_in, data_out, c, clk, tp, c_unlatched, testmode, clock_on, pad;
	int32_t clock_left, tp_period, tp_left, dout_period, dout_left;   // (32768 Hz ticks; a period 0: stopped)
} rtc_t;

// ---- the state (saved) ----
static struct {
	uint16_t ram[0x8000];       // 0x100000 (64 KB, mirrored)
	uint16_t sram[0x8000];      // 0xd00000 (the backup RAM)
	uint16_t vram[0x8800];
	uint16_t pal[0x2000];
	uint16_t vram_offset, vram_read_buffer, vram_modulo, irq2_ctrl;
	uint8_t anim_speed, anim_disabled, anim_counter, anim_frame_counter;
	uint8_t latch;              // the HC259's outputs
	uint8_t irq3, irq_rast, irq_vbl;
	uint8_t snd_latch, snd_pending, snd_result, nmi_enable, nmi_out, card_bank;
	uint8_t zbank[4], pad[2];
	uint32_t disp_counter, bank_base;
	int64_t cyc;                // the 68000's cycles run
	int64_t frame0;             // (the cycle of this frame's line 0)
	int64_t irq2_at;            // the display position IRQ's cycle, -1: none
	int64_t wd_kick;            // (the watchdog's last kick)
	int64_t z80_cyc;            // the Z80's cycles run
	uint64_t ym_frac, rtc_frac;
	uint32_t frame;
	rtc_t rtc;
	// the cartridge
	uint16_t pvc_ram[0x1000];
	uint16_t sma_rng;
	uint8_t kof98_state, pad2;
	neo_mslugx_t mslugx;
	neo_proct0_t ct0;
	uint8_t state_end;
} n;

static z80 s_z80;
static uint8_t s_zram[0x800];
static const uint8_t *s_zmap[256];
static int s_ym_irq;
static int32_t *s_mix_l, *s_mix_r;
static int s_mix_n, s_mix_cap;

// ---- the uPD4990A ----
static int rtc_data_out(void) { return (n.rtc.oe || n.rtc.testmode) ? n.rtc.data_out : 1; }
static int rtc_serial(void) { return n.rtc.c_unlatched == 7; }

static uint8_t bcd_inc(uint8_t v, int max, int min)
{
	int d = (v >> 4) * 10 + (v & 15) + 1;
	if (d > max)
		d = min;
	return (uint8_t)((d / 10) << 4 | d % 10);
}
static void rtc_advance_seconds(void)   // (device_rtc_interface: the counters in BCD, with their carries)
{
	static const uint8_t mdays[13] = { 0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };
	uint8_t *t = n.rtc.time_counter;
	t[0] = bcd_inc(t[0], 59, 0);
	if (t[0])
		return;
	t[1] = bcd_inc(t[1], 59, 0);
	if (t[1])
		return;
	t[2] = bcd_inc(t[2], 23, 0);
	if (t[2])
		return;
	int month = t[4] >> 4, dow = t[4] & 15;
	dow = (dow + 1) % 7;
	const int year = (t[5] >> 4) * 10 + (t[5] & 15);
	int days = month >= 1 && month <= 12 ? mdays[month] : 31;
	if (month == 2 && !(year & 3))
		days = 29;
	const int day = (t[3] >> 4) * 10 + (t[3] & 15);
	if (day >= days) {
		t[3] = 0x01;
		if (++month > 12) {
			month = 1;
			t[5] = bcd_inc(t[5], 99, 0);
		}
	} else
		t[3] = bcd_inc(t[3], 31, 1);
	t[4] = (uint8_t)(month << 4 | dow);
}
static void rtc_tp_timer(int32_t period) { n.rtc.tp_period = period; n.rtc.tp_left = period; n.rtc.tp = !n.rtc.tp; }
static void rtc_dout_timer(int32_t period) { n.rtc.dout_period = period; n.rtc.dout_left = period; n.rtc.data_out = !n.rtc.data_out; }
static void rtc_stb_w(int state)
{
	rtc_t *r = &n.rtc;
	if (!r->cs)
		return;
	if (!r->stb && state) {
		if (rtc_serial())
			r->c = r->shift_reg[6];
		else {
			r->c = r->c_unlatched;
			if (r->c == 7)
				r->c = RTC_TEST;
		}
		if (r->c == RTC_HOLD || (r->c >= RTC_TP_64HZ && r->c < RTC_TEST)) {
			r->clock_on = 1;
			r->testmode = 0;
		}
		const int max_shift = rtc_serial() ? 6 : 5;
		switch (r->c) {
		case RTC_HOLD:
			rtc_dout_timer(16384);
			rtc_tp_timer(128);
			break;
		case RTC_SHIFT:
			if (!r->testmode)
				r->clock_on = 1;
			r->dout_period = 0;
			r->data_out = r->shift_reg[0] & 1;
			break;
		case RTC_TIME_SET:
			r->clock_on = 0;
			r->dout_period = 0;
			r->data_out = r->shift_reg[0] & 1;
			for (int i = 0; i < max_shift; i++)
				r->time_counter[i] = r->shift_reg[i];
			r->clock_left = 32768;
			r->clock_on = 1;   // (MAME re-arms the clock timer at once)
			break;
		case RTC_TIME_READ:
			if (!r->testmode)
				r->clock_on = 1;
			for (int i = 0; i < max_shift; i++)
				r->shift_reg[i] = r->time_counter[i];
			rtc_dout_timer(16384);
			break;
		case RTC_TP_64HZ: rtc_tp_timer(256); break;
		case RTC_TP_256HZ: rtc_tp_timer(64); break;
		case RTC_TP_2048HZ: rtc_tp_timer(8); break;
		case RTC_TP_4096HZ: rtc_tp_timer(4); break;
		case RTC_TP_1S_INT: rtc_tp_timer(16384); break;
		case RTC_TP_10S_INT: rtc_tp_timer(16384 * 10); break;
		case RTC_TP_30S_INT: rtc_tp_timer(16384 * 30); break;
		case RTC_TP_60S_INT: rtc_tp_timer(16384 * 60); break;
		case RTC_TEST:
			r->clock_on = 0;
			r->dout_period = 0;
			r->testmode = 1;
			break;
		default:
			break;
		}
	}
	r->stb = (uint8_t)state;
}
static void rtc_clk_w(int state)
{
	rtc_t *r = &n.rtc;
	if (!r->cs)
		return;
	if (!r->clk && state) {
		int in = r->data_in;
		if (rtc_serial()) {
			in = r->shift_reg[6] & 1;
			r->shift_reg[6] = (uint8_t)((r->shift_reg[6] >> 1) | (r->data_in << 3));
		}
		if (r->c == RTC_SHIFT) {
			const int max_shift = rtc_serial() ? 6 : 5;
			for (int i = 0; i < max_shift; i++) {
				r->shift_reg[i] >>= 1;
				if (i == max_shift - 1)
					r->shift_reg[i] |= (uint8_t)(in << 7);
				else
					r->shift_reg[i] |= (uint8_t)((r->shift_reg[i + 1] << 7) & 0x80);
			}
			r->data_out = r->shift_reg[0] & 1;
		}
	}
	r->clk = (uint8_t)state;
}
static void rtc_advance(int ticks)
{
	rtc_t *r = &n.rtc;
	if (r->clock_on) {
		r->clock_left -= ticks;
		while (r->clock_left <= 0) {
			r->clock_left += 32768;
			rtc_advance_seconds();
		}
	}
	if (r->tp_period) {
		r->tp_left -= ticks;
		while (r->tp_left <= 0) {
			r->tp_left += r->tp_period;
			r->tp = !r->tp;
		}
	}
	if (r->dout_period) {
		r->dout_left -= ticks;
		while (r->dout_left <= 0) {
			r->dout_left += r->dout_period;
			r->data_out = !r->data_out;
		}
	}
}
static void rtc_reset(void)
{
	memset(&n.rtc, 0, sizeof(n.rtc));
	// machine_start: chip select, output enable, serial mode
	n.rtc.cs = n.rtc.oe = 1;
	n.rtc.c_unlatched = 7;
	n.rtc.clock_on = 1;
	n.rtc.clock_left = 32768;
	// (a fixed date for the same runs: 2000-01-01 00:00:00, a Saturday)
	n.rtc.time_counter[3] = 0x01;
	n.rtc.time_counter[4] = 0x16;
	n.rtc.time_counter[5] = 0x00;
}

// ---- the 68000's interrupts (levels 1 vblank, 2 display position, 3 at power on: asserted until acknowledged) ----
static void update_irq(void)
{
	m68k_bus_cur->irq_asserted = (n.irq_vbl ? 1u << 1 : 0) | (n.irq_rast ? 1u << 2 : 0) | (n.irq3 ? 1u << 3 : 0);
	m68k_bus_update_irq();
}
static inline int64_t now_cyc(void) { return n.cyc + m68k_cycles_run(); }
static void irq2_arm(int64_t from) { n.irq2_at = from + ((int64_t)n.disp_counter + 1) * 2; }

// ---- drawing ----
typedef struct {
	uint8_t shadow, bank, fix_cart, anim_disabled, anim_counter, pad[3];
} vstate_t;
static void vstate_now(vstate_t *v)
{
	v->shadow = (n.latch >> Q_SHADOW) & 1;
	v->bank = (n.latch >> Q_PAL_BANK) & 1;
	v->fix_cart = (n.latch >> Q_CART_AUDIO) & 1;
	v->anim_disabled = n.anim_disabled;
	v->anim_counter = n.anim_counter;
	memset(v->pad, 0, sizeof(v->pad));
}

// the y and rows of each sprite (a chained one: its block's), as parse_sprites keeps them
typedef struct { int16_t y[384]; uint8_t rows[384]; } sprpos_t;
static void sprite_positions(const uint16_t *vram, sprpos_t *p)
{
	int y = 0, rows = 0;
	for (int s = 0; s < 381; s++) {
		const uint16_t yc = vram[0x8200 | s];
		if (!((yc >> 6) & 1)) {
			y = 0x200 - (yc >> 7);
			rows = yc & 0x3f;
		}
		p->y[s] = (int16_t)y;
		p->rows[s] = (uint8_t)rows;
	}
}
static inline int on_line(int line, int y, int rows) { return rows == 0 || rows >= 0x20 || ((line - y) & 0x1ff) < rows * 0x10; }

static const uint16_t zoom_x_tables[16] = { 0x0080, 0x0880, 0x0888, 0x2888, 0x288a, 0x2a8a, 0x2aaa, 0xaaaa, 0xaaea, 0xbaea, 0xbaeb, 0xbbeb, 0xbbef, 0xfbef, 0xfbff, 0xffff };

// a sprite row's pixel (packed: two a byte, the even one low; 0 outside the data)
static inline int spr_pix(uint32_t a) { const uint8_t v = s_spr[a >> 1]; return (a & 1) ? v >> 4 : v & 15; }

// draw_sprites then draw_fixed_layer for one screen line into dst (indices: the palette's 0x4000 entries)
static void draw_line(int line, const uint16_t *vram, const vstate_t *vs, const sprpos_t *pos, uint16_t *dst)
{
	const uint16_t base = (uint16_t)((vs->bank ? 0x1000 : 0) + (vs->shadow ? 0x2000 : 0));
	uint16_t buf[0x200 + 16];
	for (int x = 0; x < VIS_W; x++)
		buf[x] = (uint16_t)(base + 0xfff);
	// parse_sprites: the list of this line (up to 96, then 0s)
	uint16_t list[97];
	int count = 0;
	for (int s = 0; s < 381 && count < 96; s++) {
		if (!pos->rows[s] || !on_line(line, pos->y[s], pos->rows[s]))
			continue;
		list[count++] = (uint16_t)s;
	}
	for (int i = count; i < 97; i++)
		list[i] = 0;
	int max_index;
	for (max_index = 95; max_index >= 0; max_index--)
		if (list[max_index] != 0)
			break;
	if (max_index != 95)
		max_index++;
	int y = 0, x = 0, rows = 0, zoom_y = 0, zoom_x = 0;
	for (int si = 0; si <= max_index; si++) {
		const uint16_t sn = list[si] & 0x1ff;
		const uint16_t y_control = vram[0x8200 | sn], zoom_control = vram[0x8000 | sn];
		if ((y_control >> 6) & 1) {   // chained: the next column
			x = (x + zoom_x + 1) & 0x1ff;
			zoom_x = (zoom_control >> 8) & 15;
		} else {
			y = 0x200 - (y_control >> 7);
			x = vram[0x8400 | sn] >> 7;
			zoom_y = zoom_control & 0xff;
			zoom_x = (zoom_control >> 8) & 15;
			rows = y_control & 0x3f;
		}
		if (x >= 0x140 && x <= 0x1f0)
			continue;
		if (!on_line(line, y, rows))
			continue;
		const int sprite_line = (line - y) & 0x1ff;
		int zoom_line = sprite_line & 0xff;
		int invert = (sprite_line >> 8) & 1;
		if (invert)
			zoom_line ^= 0xff;
		if (rows > 0x20) {
			zoom_line = zoom_line % ((zoom_y + 1) << 1);
			if (zoom_line > zoom_y) {
				zoom_line = ((zoom_y + 1) << 1) - 1 - zoom_line;
				invert = !invert;
			}
		}
		const uint8_t yt = s_zoomy[(zoom_y << 8) | zoom_line];
		int sprite_y = yt & 15, tile = yt >> 4;
		if (invert) {
			sprite_y ^= 15;
			tile ^= 0x1f;
		}
		const uint32_t ao = ((uint32_t)sn << 6) | (uint32_t)(tile << 1);
		const uint16_t attr = vram[ao + 1];
		uint32_t code = (((uint32_t)attr << 12) & 0xf0000) | vram[ao];
		if (!vs->anim_disabled) {
			if ((attr >> 3) & 1)
				code = (code & ~7u) | (vs->anim_counter & 7);
			else if ((attr >> 2) & 1)
				code = (code & ~3u) | (vs->anim_counter & 3);
		}
		if ((attr >> 1) & 1)
			sprite_y ^= 15;
		uint16_t zt = zoom_x_tables[zoom_x];
		uint32_t gb = ((code << 8) | (uint32_t)(sprite_y << 4)) & s_spr_mask;
		const int valid = (gb >> 1) + 8 <= s_spr_size;   // (beyond the data: MAME's buffer is 0 there)
		const uint16_t pens = (uint16_t)(base + ((attr >> 8) << 4));
		int x_inc = 1;
		if (attr & 1) {
			gb += 15;
			x_inc = -1;
		}
		if (x <= 0x1f0) {
			uint16_t *p = buf + x;
			for (int i = 0; i < 16; i++) {
				if ((zt >> 15) & 1) {
					if (valid) {
						const int g = spr_pix(gb);
						if (g)
							*p = (uint16_t)(pens + g);
					}
					p++;
				}
				zt <<= 1;
				if (!zt)
					break;
				gb += (uint32_t)x_inc;
			}
		} else {   // (wraps around to the left)
			int xx = x;
			uint16_t *p = buf;
			for (int i = 0; i < 16; i++) {
				if ((zt >> 15) & 1) {
					if (xx >= 0x200) {
						if (valid) {
							const int g = spr_pix(gb);
							if (g)
								*p = (uint16_t)(pens + g);
						}
						p++;
					}
					xx++;
				}
				zt <<= 1;
				if (!zt)
					break;
				gb += (uint32_t)x_inc;
			}
		}
	}
	// the fixed layer
	const uint8_t *fb = vs->fix_cart && s_fix ? s_fix : s_fixbios;
	const uint32_t amask = (vs->fix_cart && s_fix ? s_fix_size : s_fixbios_size) - 1;
	const uint16_t *vd = &vram[0x7000 | (line >> 3)];
	uint16_t *p = buf;
	const int banked = vs->fix_cart && s_fix && amask > 0x1ffff ? s_cart->fixbank : 0;
	int garouoffsets[34];
	if (banked == 1) {   // (Garou, Metal Slug 3: the line banks from the RAM at 0x7500)
		int garoubank = 0, k = 0, y = 0;
		while (y < 32) {
			if (vram[0x7500 + k] == 0x0200 && (vram[0x7580 + k] & 0xff00) == 0xff00) {
				garoubank = vram[0x7580 + k] & 3;
				garouoffsets[y++] = garoubank;
			}
			garouoffsets[y++] = garoubank;
			k += 2;
		}
	}
	for (int tx = 0; tx < 40; tx++, vd += 0x20) {
		const uint16_t cp = *vd;
		uint32_t code = cp & 0x0fff;
		if (banked == 1)
			code += 0x1000 * (uint32_t)(garouoffsets[((line >> 3) - 2) & 31] ^ 3);
		else if (banked == 2)   // (KoF 2000)
			code += 0x1000 * (uint32_t)(((vram[0x7500 + (((line >> 3) - 1) & 31) + 32 * (tx / 6)] >> (5 - (tx % 6)) * 2) & 3) ^ 3);
		const uint32_t go = ((code << 5) | (uint32_t)(line & 7)) & amask;
		const uint16_t pens = (uint16_t)(base + ((cp >> 12) << 4));
		static const uint8_t po[4] = { 0x10, 0x18, 0x00, 0x08 };
		for (int i = 0; i < 4; i++) {
			const uint8_t d = fb[(go + po[i]) & amask];
			if (d & 15)
				p[0] = (uint16_t)(pens + (d & 15));
			if (d >> 4)
				p[1] = (uint16_t)(pens + (d >> 4));
			p += 2;
		}
	}
	memcpy(dst, buf, VIS_W * sizeof(uint16_t));
}

// ---- the frame drawn at the vblank, as MAME's partial updates: line L is drawn at the start of line L + 1 with the
// state of that time. The video RAM and the drawing state are copied at the start of the first shown line; the
// changes made while the lines are shown are logged with their line (a change during line C shows from line C on).
// The drawing side replays the log, band by band.
#define LOG_MAX 0x20000
#define LOG_VSTATE 0xffff   // (an entry of the drawing state: value = shadow | bank << 1 | fix << 2 | anim disabled << 3)
typedef struct { uint16_t line, addr, value; } vlog_t;
typedef struct {
	uint16_t vram[0x8800];
	vstate_t vs;
	vlog_t log[LOG_MAX];
	int nlog, overflow;
	uint16_t pal[0x2000];
	int pal_dirty;
} frame_t;
static frame_t *s_frames[2];
static frame_t *s_cur;    // (this frame's: written by the emulation from the first shown line)
static frame_t *s_snap;   // (the one being drawn)
static int s_logging;

static int cur_line(void) { return (int)((now_cyc() - n.frame0) / LINE_CYC); }
static uint16_t vstate_pack(const vstate_t *v) { return (uint16_t)(v->shadow | v->bank << 1 | v->fix_cart << 2 | v->anim_disabled << 3); }
static void vstate_unpack(vstate_t *v, uint16_t p)
{
	v->shadow = p & 1;
	v->bank = (p >> 1) & 1;
	v->fix_cart = (p >> 2) & 1;
	v->anim_disabled = (p >> 3) & 1;
}
static void log_put(uint16_t addr, uint16_t value)
{
	if (!s_logging)
		return;
	if (s_cur->nlog >= LOG_MAX) {
		s_cur->overflow = 1;
		return;
	}
	int line = cur_line();
	if (line < VIS_Y0)
		line = VIS_Y0;
	vlog_t *e = &s_cur->log[s_cur->nlog++];
	e->line = (uint16_t)line;
	e->addr = addr;
	e->value = value;
}
static void log_vstate(void)
{
	vstate_t v;
	vstate_now(&v);
	log_put(LOG_VSTATE, vstate_pack(&v));
}
static void frame_start_lines(void)   // the start of the first shown line
{
	memcpy(s_cur->vram, n.vram, sizeof(n.vram));
	vstate_now(&s_cur->vs);
	s_cur->nlog = 0;
	s_cur->overflow = 0;
	s_logging = 1;
}

#define BANDS 4
static uint16_t (*s_pal_conv_hp)[0x2000];
#define s_pal_conv (*s_pal_conv_hp)
static int s_pal_valid;
static uint8_t s_lookup[32][4];   // create_rgb_lookups: normal, dark, shadow, dark shadow
static int s_band_next, s_band_left, s_band_active, s_band_helpers;
static uint16_t (*s_band_vram_hp)[BANDS][0x8800];
#define s_band_vram (*s_band_vram_hp)
static sprpos_t s_band_pos[BANDS];

static void run_band(int band)
{
	const int y0 = VIS_Y0 + VIS_H * band / BANDS, y1 = VIS_Y0 + VIS_H * (band + 1) / BANDS;
	uint16_t *vram = s_band_vram[band];
	sprpos_t *pos = &s_band_pos[band];
	memcpy(vram, s_snap->vram, sizeof(s_band_vram[0]));
	vstate_t vs = s_snap->vs;
	int k = 0, pos_dirty = 1;
	const vlog_t *log = s_snap->log;
	const int nlog = s_snap->nlog;
	for (int l = y0; l < y1; l++) {
		for (; k < nlog && log[k].line <= l; k++) {
			if (log[k].addr == LOG_VSTATE)
				vstate_unpack(&vs, log[k].value);
			else {
				vram[log[k].addr] = log[k].value;
				if (log[k].addr >= 0x8200 && log[k].addr < 0x8400)
					pos_dirty = 1;
			}
		}
		if (pos_dirty) {
			sprite_positions(vram, pos);
			pos_dirty = 0;
		}
		draw_line(l, vram, &vs, pos, rbx_video.bitmap + (l - VIS_Y0) * rbx_video.pitch);
	}
}

static void ng_render(void)
{
	if (s_snap->pal_dirty || !s_pal_valid) {
		for (int i = 0; i < 0x2000; i++) {
			const uint16_t d = s_snap->pal[i];
			if (s_pal_valid && s_pal_conv[i] == d)
				continue;
			s_pal_conv[i] = d;
			const int dark = d >> 15;
			const int r = ((d >> 14) & 1) | ((d >> 7) & 0x1e), g = ((d >> 13) & 1) | ((d >> 3) & 0x1e),
			          b = ((d >> 12) & 1) | ((d << 1) & 0x1e);
			rbx_video.palette[i] = (uint32_t)(s_lookup[r][dark] << 16 | s_lookup[g][dark] << 8 | s_lookup[b][dark]);
			rbx_video.palette[i + 0x2000] =
				(uint32_t)(s_lookup[r][dark + 2] << 16 | s_lookup[g][dark + 2] << 8 | s_lookup[b][dark + 2]);
		}
		s_pal_valid = 1;
		s_snap->pal_dirty = 0;
	}
	__atomic_store_n(&s_band_next, 0, __ATOMIC_SEQ_CST);
	__atomic_store_n(&s_band_left, BANDS, __ATOMIC_SEQ_CST);
	__atomic_store_n(&s_band_active, 1, __ATOMIC_SEQ_CST);
	for (;;) {
		const int bnd = __atomic_fetch_add(&s_band_next, 1, __ATOMIC_ACQ_REL);
		if (bnd >= BANDS)
			break;
		run_band(bnd);
		__atomic_sub_fetch(&s_band_left, 1, __ATOMIC_ACQ_REL);
	}
	while (__atomic_load_n(&s_band_left, __ATOMIC_ACQUIRE) > 0)
		;
	__atomic_store_n(&s_band_active, 0, __ATOMIC_SEQ_CST);
	while (__atomic_load_n(&s_band_helpers, __ATOMIC_ACQUIRE) > 0)
		;
}

static int ng_render_help(int helper)
{
	const int one = helper & 0x100;
	helper &= 0xff;
	if (helper < 1 || helper > 2 || !__atomic_load_n(&s_band_active, __ATOMIC_SEQ_CST))
		return 0;
	__atomic_add_fetch(&s_band_helpers, 1, __ATOMIC_SEQ_CST);
	int did = 0;
	if (__atomic_load_n(&s_band_active, __ATOMIC_SEQ_CST))
		for (;;) {
			const int bnd = __atomic_fetch_add(&s_band_next, 1, __ATOMIC_ACQ_REL);
			if (bnd >= BANDS)
				break;
			run_band(bnd);
			__atomic_sub_fetch(&s_band_left, 1, __ATOMIC_ACQ_REL);
			did = 1;
			if (one)
				break;
		}
	__atomic_sub_fetch(&s_band_helpers, 1, __ATOMIC_SEQ_CST);
	return did;
}

// the start of line 240: the frame (its copy and log) goes to the drawing side, the other buffer takes the next
static void vblank_draw(void)
{
	rbx_video_wait_idle();
	if (!s_logging)   // (no first line this frame: after a reset or a state load)
		frame_start_lines();
	s_logging = 0;
	if (memcmp(s_cur->pal, n.pal, sizeof(n.pal))) {
		memcpy(s_cur->pal, n.pal, sizeof(n.pal));
		s_cur->pal_dirty = 1;
	}
	frame_t *other = s_cur == s_frames[0] ? s_frames[1] : s_frames[0];
	s_snap = s_cur;
	rbx_video_frame_done();
	memcpy(other->pal, s_cur->pal, sizeof(other->pal));   // (the next frame's dirty flag against this one)
	other->pal_dirty = 0;
	s_cur = other;
}

// ---- the video registers ----
static void set_vram_offset(uint16_t data)
{
	n.vram_offset = (data & 0x8000) ? (data & 0x87ff) : data;
	n.vram_read_buffer = n.vram[n.vram_offset];
}
static uint16_t video_control(void)
{
	const int vpos = (int)((now_cyc() - n.frame0) / LINE_CYC) % LINES;
	int v = vpos + 0x100;
	if (v >= 0x200)
		v -= LINES;
	return (uint16_t)((v << 7) | (n.anim_counter & 7));
}
static uint16_t unmapped_r(void);
static void video_w(int offset, uint16_t data, uint16_t mask)
{
	if (mask == 0x00ff)
		return;
	if (mask == 0xff00)
		data = (uint16_t)((data & 0xff00) | (data >> 8));
	switch (offset) {
	case 0: set_vram_offset(data); break;
	case 1:
		n.vram[n.vram_offset] = data;
		log_put(n.vram_offset, data);
		set_vram_offset((uint16_t)((n.vram_offset & 0x8000) | ((n.vram_offset + n.vram_modulo) & 0x7fff)));
		break;
	case 2: n.vram_modulo = data; break;
	case 3:   // set_video_control
		n.anim_speed = (uint8_t)(data >> 8);
		if (n.anim_disabled != ((data >> 3) & 1)) {
			n.anim_disabled = (data >> 3) & 1;
			log_vstate();
		}
		n.irq2_ctrl = data & 0x00f0;
		break;
	case 4: n.disp_counter = (n.disp_counter & 0x0000ffff) | ((uint32_t)data << 16); break;
	case 5:
		n.disp_counter = (n.disp_counter & 0xffff0000) | data;
		if ((n.irq2_ctrl >> IRQ2_LOAD_RELATIVE) & 1)
			irq2_arm(now_cyc());
		break;
	case 6:   // acknowledge
		if (data & 1) n.irq3 = 0;
		if (data & 2) n.irq_rast = 0;
		if (data & 4) n.irq_vbl = 0;
		update_irq();
		break;
	default: break;
	}
}

// ---- the 68000's memory ----
static void map_bank(void)
{
	const uint32_t base = n.bank_base < s_rom_size ? n.bank_base : 0;
	const uint32_t size = (s_rom_size - base < 0x100000 ? s_rom_size - base : 0x100000) & ~0xfffu;
	// (whole pages only: a block smaller than the window would leave a range on the bus looked up before the
	// handlers, and one more at each bank change)
	if (size)
		m68k_bus_map(0x200000, 0x2fffff, s_rom + base, size, 1, 0);
	else
		for (uint32_t p = 0x200; p < 0x300; p++)
			m68k_bus_cur->rd[p] = NULL;
	// (the cartridge handlers: their pages read through main_r16)
	if (s_cart->type == C_FATFURY2)
		for (uint32_t p = 0x200; p < 0x300; p++)
			m68k_bus_cur->rd[p] = NULL;
	else if (s_cart->type == C_MSLUGX || s_cart->type == C_SMA || s_cart->type == C_PVC)
		m68k_bus_cur->rd[0x2fe] = m68k_bus_cur->rd[0x2ff] = NULL;
}
static uint16_t bank_rom16(uint32_t a)   // the banked ROM behind a cartridge handler
{
	const uint32_t base = n.bank_base < s_rom_size ? n.bank_base : 0;
	uint32_t size = (s_rom_size - base < 0x100000 ? s_rom_size - base : 0x100000) & ~0xfffu;
	if (!size)
		size = s_rom_size - base;
	const uint32_t o = rbx_wrap((a - 0x200000), size);
	return *(const uint16_t *)(s_rom + base + (o & ~1u));
}
static void set_vectors(void)   // the system latch Q1: the vectors from the cartridge or the BIOS
{
	memcpy(s_page0, s_rom, sizeof(s_page0));
	if (!((n.latch >> Q_CART_VECTORS) & 1))
		memcpy(s_page0, s_bios, 0x80);
	if (s_cart->type == C_KOF98) {   // (the overlay at 0x100: kof98_prot_device::protection_r)
		uint16_t *w = (uint16_t *)(s_page0 + 0x100);
		if (n.kof98_state == 1) {
			w[0] = 0x00c2;
			w[1] = 0x00fd;
		} else if (n.kof98_state == 2) {
			w[0] = 0x4e45;
			w[1] = 0x4f2d;
		} else {
			w[0] = s_kof98_default[0];
			w[1] = s_kof98_default[1];
		}
	}
}
static void set_bank(uint32_t base)
{
	n.bank_base = base;
	map_bank();
}
static void write_banksel(uint16_t data)
{
	if (s_rom_size <= 0x100000 && (data & 7))
		return;
	int bank = data & 7;
	if ((uint32_t)(bank + 1) * 0x100000 >= s_rom_size)
		bank = 0;
	n.bank_base = (uint32_t)(bank + 1) * 0x100000;
	map_bank();
}
static void snd_nmi_update(void)
{
	const int out = n.snd_pending && n.nmi_enable;
	if (out && !n.nmi_out)
		z80_gen_nmi(&s_z80);
	n.nmi_out = (uint8_t)out;
}
static void z80_sync(int64_t cyc68);
static void audio_command_w(uint8_t data)
{
	z80_sync(now_cyc());
	n.snd_latch = data;
	n.snd_pending = 1;
	snd_nmi_update();
	m68k_end_timeslice();   // (MAME: a perfect quantum of 50 us: the Z80 answers before the 68000 goes on)
}
static void latch_w(int q, int v)   // the HC259 (hc259 write_a3)
{
	const uint8_t old = n.latch;
	n.latch = (uint8_t)((n.latch & ~(1u << q)) | ((unsigned)(v & 1) << q));
	if (old == n.latch)
		return;
	if (q == Q_SHADOW || q == Q_PAL_BANK || q == Q_CART_AUDIO)
		log_vstate();
	if (q == Q_CART_VECTORS)
		set_vectors();
	if (q == Q_CART_AUDIO) {   // the Z80's 0x0000 .. 0x7fff: the cartridge or the SM1 BIOS
		const uint8_t *mainz = ((n.latch >> Q_CART_AUDIO) & 1) || !s_zbios ? s_zrom : s_zbios;
		for (int p = 0; p < 0x80; p++)
			s_zmap[p] = mainz + p * 256;
	}
}
static void io_control_w(int offset, uint8_t data)   // mvs_state / ngarcade_base_state
{
	switch (offset & 0x78) {
	case 0x08: n.card_bank = data & 7; break;
	case 0x28:   // the uPD4990A
		n.rtc.data_in = data & 1;
		rtc_clk_w((data >> 1) & 1);
		rtc_stb_w((data >> 2) & 1);
		break;
	default: break;
	}
}

static uint16_t inputs_p(int player)   // the edge connector's JOY1 / JOY2 (neogeo_joy_ac)
{
	const int p = s_port_joy[player - 1];
	if (p >= 0)
		return (uint16_t)(rbx_port_read(p) & 0xff);
	return (uint16_t)(~rbx_player_input(player) & 0xff);   // (the bits of rbx_core.h are the Neo Geo's: up .. D)
}
static uint16_t main_r16(uint32_t a);
static int s_unmapped_recurse;
static uint16_t unmapped_r(void)   // the last word on the bus: the prefetch at the PC
{
	if (s_unmapped_recurse)
		return 0xffff;
	s_unmapped_recurse = 1;
	const uint32_t pc = m68k_get_reg(NULL, M68K_REG_PC) & 0xfffffe;
	const uint8_t *p = m68k_bus_cur->rd[pc >> M68K_PAGE_BITS];
	const uint16_t v = p ? *(const uint16_t *)(p + (pc & ((1u << M68K_PAGE_BITS) - 1))) : main_r16(pc);
	s_unmapped_recurse = 0;
	return v;
}
static uint8_t bus_r8(uint32_t a) { return (uint8_t)m68k_read_memory_8(a); }
static uint16_t bus_r16(uint32_t a) { return (uint16_t)m68k_read_memory_16(a); }
static uint16_t cart_r16(uint32_t a)   // 0x200000 .. 0x2fffff where the cartridge has handlers
{
	switch (s_cart->type) {
	case C_FATFURY2:
		return neo_fatfury2_r(&n.ct0, (a - 0x200000) >> 1);
	case C_MSLUGX:
		if (a >= 0x2fffe0 && a < 0x2ffff0)
			return neo_mslugx_r(&n.mslugx, bus_r8, bus_r16);
		break;
	case C_SMA:
		if (a == 0x2fe446)
			return 0x9a37;
		switch (s_cart->prot) {
		case P_KOF99: if (a == 0x2ffff8 || a == 0x2ffffa) return neo_sma_random(&n.sma_rng); break;
		case P_GAROU: case P_GAROUH: if (a == 0x2fffcc || a == 0x2ffff0) return neo_sma_random(&n.sma_rng); break;
		case P_KOF2000: if (a == 0x2fffd8 || a == 0x2fffda) return neo_sma_random(&n.sma_rng); break;
		}
		break;
	case C_PVC:
		if (a >= 0x2fe000)
			return n.pvc_ram[(a - 0x2fe000) >> 1];
		break;
	}
	return bank_rom16(a);
}
static uint16_t main_r16(uint32_t a)
{
	a &= 0xfffffe;
	switch (a >> 16) {
	case 0x20: case 0x21: case 0x22: case 0x23: case 0x24: case 0x25: case 0x26: case 0x27:
	case 0x28: case 0x29: case 0x2a: case 0x2b: case 0x2c: case 0x2d: case 0x2e: case 0x2f:
		return cart_r16(a);
	case 0x30: case 0x31:
		if (a & 0x80)
			return s_port_test >= 0 ? (uint16_t)rbx_port_read(s_port_test) : 0xff7f;
		return (uint16_t)(inputs_p(1) << 8 | (s_port_dsw >= 0 ? (rbx_port_read(s_port_dsw) & 0xff) : 0xff));
	case 0x32: case 0x33: {   // AUDIO_COIN: the coins, the uPD4990A's TP and data, the Z80's reply
		const uint32_t custom = (uint32_t)n.rtc.tp << 6 | (uint32_t)rtc_data_out() << 7 | (uint32_t)n.snd_result << 8;
		if (s_port_audio < 0)
			return (uint16_t)(0x1f | custom);
		rbx_port_custom(s_port_audio, 0xffe0, custom);
		return (uint16_t)rbx_port_read(s_port_audio);
	}
	case 0x34: case 0x35:
		return (uint16_t)(inputs_p(2) << 8 | 0xff);
	case 0x38: case 0x39: {   // SYSTEM: start (bits 8, 10, active low), the memory cards absent, MVS
		uint32_t ss;
		if (s_port_start >= 0)
			ss = (rbx_port_read(s_port_start) | ~0x05u) & 0x0f;
		else {
			const uint32_t p1 = rbx_player_input(1), p2 = rbx_player_input(2);
			ss = 0x0f & ~((p1 & RBX_IN_START) ? 1u : 0) & ~((p2 & RBX_IN_START) ? 4u : 0);
		}
		return (uint16_t)(0x00ff | ss << 8 | 0x7000 | 0x8000);
	}
	case 0x3c: case 0x3d:
		switch ((a >> 1) & 3) {
		case 0: case 1: return n.vram_read_buffer;
		case 2: return n.vram_modulo;
		default: return video_control();
		}
	}
	if (a >= 0x400000 && a < 0x800000)
		return n.pal[((n.latch >> Q_PAL_BANK) & 1) * 0x1000 + ((a >> 1) & 0xfff)];
	if (a >= 0x800000 && a < 0xc00000)   // (the memory card: none)
		return 0xffff;
	return unmapped_r();
}
static uint8_t main_r8(uint32_t a)
{
	if ((a >> 16) == 0x3c || (a >> 16) == 0x3d) {   // (the video registers: the low byte alone is not mapped)
		if (a & 1)
			return (uint8_t)unmapped_r();
	}
	const uint16_t w = main_r16(a & ~1u);
	return (uint8_t)((a & 1) ? w : w >> 8);
}
static void main_w16m(uint32_t a, uint16_t v, uint16_t mask)
{
	a &= 0xfffffe;
	if (a >= 0x200000 && a < 0x300000) {
		switch (s_cart->type) {
		case C_FATFURY2:
			neo_fatfury2_w(&n.ct0, (a - 0x200000) >> 1, v);
			return;
		case C_KOF98:
			if (a == 0x20aaaa) {   // kof98_prot_device::protection_w
				if (v == 0x0090)
					n.kof98_state = 1;
				else if (v == 0x00f0)
					n.kof98_state = 2;
				set_vectors();
				return;
			}
			break;
		case C_MSLUGX:
			if (a >= 0x2fffe0 && a < 0x2ffff0) {
				neo_mslugx_w(&n.mslugx, (int)((a - 0x2fffe0) >> 1), v);
				return;
			}
			break;
		case C_SMA: {   // write_bankprot at the cartridge address
			static const uint32_t at[7] = { 0, 0x2ffff0, 0x2fffc0, 0x2fffc0, 0x2fffe4, 0x2fffe4, 0x2fffec };
			if (a == at[s_cart->prot]) {
				uint32_t b = 0;
				switch (s_cart->prot) {
				case P_KOF99: b = neo_sma_kof99_bank_base(v); break;
				case P_GAROU: b = neo_sma_garou_bank_base(v); break;
				case P_GAROUH: b = neo_sma_garouh_bank_base(v); break;
				case P_MSLUG3: b = neo_sma_mslug3_bank_base(v); break;
				case P_MSLUG3A: b = neo_sma_mslug3a_bank_base(v); break;
				case P_KOF2000: b = neo_sma_kof2000_bank_base(v); break;
				}
				set_bank(b);
				return;
			}
			break;
		}
		case C_PVC:
			if (a >= 0x2fe000) {   // write_bankprot_pvc
				const int o = (int)((a - 0x2fe000) >> 1);
				neo_pvc_w(n.pvc_ram, o, v, mask);
				if (o >= 0xff8)
					set_bank(neo_pvc_bank_base(n.pvc_ram));
				return;
			}
			break;
		}
		if (a >= 0x2ffff0)
			write_banksel(v);
		return;
	}
	switch (a >> 16) {
	case 0x30: case 0x31:
		if (mask & 0xff)
			n.wd_kick = now_cyc();
		return;
	case 0x32: case 0x33:
		if (mask & 0xff00)
			audio_command_w((uint8_t)(v >> 8));
		return;
	case 0x38: case 0x39:
		if ((mask & 0xff) && (a & 0xff00) == 0)
			io_control_w((int)((a & 0xff) >> 1), (uint8_t)v);
		return;
	case 0x3a: case 0x3b:
		if (mask & 0xff) {
			const int o = (int)((a & 0x1f) >> 1);
			latch_w(o & 7, (o >> 3) & 1);
		}
		return;
	case 0x3c: case 0x3d:
		video_w((int)((a >> 1) & 7), v, mask);
		return;
	}
	if (a >= 0x400000 && a < 0x800000) {
		uint16_t *p = &n.pal[((n.latch >> Q_PAL_BANK) & 1) * 0x1000 + ((a >> 1) & 0xfff)];
		*p = (uint16_t)((v & mask) | (*p & ~mask));
		return;
	}
	if (a >= 0xd00000 && a < 0xe00000) {   // the backup RAM: written when unlocked
		if ((n.latch >> Q_SRAM_UNLOCK) & 1) {
			uint16_t *p = &n.sram[(a >> 1) & 0x7fff];
			*p = (uint16_t)((v & mask) | (*p & ~mask));
		}
		return;
	}
}
static void main_w16(uint32_t a, uint16_t v) { main_w16m(a, v, 0xffff); }
static void main_w8(uint32_t a, uint8_t v)
{
	main_w16m(a & ~1u, (a & 1) ? v : (uint16_t)(v << 8), (a & 1) ? 0x00ff : 0xff00);
}

// ---- the Z80 ----
static void z_banks(void)   // NEO-ZMC: f000 (2 KB), e000 (4 KB), c000 (8 KB), 8000 (16 KB) from the cartridge's 0x10000
{
	if (s_zrom_size <= 0x10000) {
		for (int p = 0x80; p < 0xf8; p++)
			s_zmap[p] = p * 256 + 256 <= (int)s_zrom_size ? s_zrom + p * 256 : NULL;
		return;
	}
	const uint32_t mask = (s_zrom_size - 0x10000 - 1) & 0x3ffff;
	static const int start[4] = { 0xf0, 0xe0, 0xc0, 0x80 }, pages[4] = { 8, 16, 32, 64 };
	for (int r = 0; r < 4; r++) {
		const uint32_t ba = 0x10000 + (((uint32_t)n.zbank[r] << (11 + r)) & mask);
		for (int p = 0; p < pages[r]; p++)
			s_zmap[start[r] + p] = ba + (uint32_t)p * 256 + 256 <= s_zrom_size ? s_zrom + ba + p * 256 : NULL;
	}
}
static uint8_t z_read(void *u, uint16_t a)
{
	(void)u;
	const uint8_t *p = s_zmap[a >> 8];
	if (p)
		return p[a & 0xff];
	if (a >= 0xf800)
		return s_zram[a - 0xf800];
	return 0xff;
}
static void z_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0xf800)
		s_zram[a - 0xf800] = v;
}
static uint8_t z_in(z80 *z, uint8_t p)
{
	const uint16_t port = z->mem_ptr;   // (the 16-bit port address: the core keeps it in mem_ptr during port_in)
	switch (p & 0x0f) {
	case 0x00:   // the sound latch (read: acknowledged)
		n.snd_pending = 0;
		snd_nmi_update();
		return n.snd_latch;
	case 0x04: case 0x05: case 0x06: case 0x07:
		return rbx_ym_read(0, p & 3);
	case 0x08: case 0x09: case 0x0a: case 0x0b:   // audio_cpu_bank_select_r
		n.zbank[p & 3] = (uint8_t)(port >> 8);
		z_banks();
		return 0;
	}
	return 0xff;
}
static void z_out(z80 *z, uint8_t p, uint8_t v)
{
	(void)z;
	switch (p & 0x0f) {
	case 0x00: n.snd_latch = 0; break;   // (clear_w)
	case 0x04: case 0x05: case 0x06: case 0x07: rbx_ym_write(0, p & 3, v); break;
	case 0x08:   // out ($08) enables the NMI, out ($18) disables it
		n.nmi_enable = !((p >> 4) & 1);
		snd_nmi_update();
		break;
	case 0x0c: n.snd_result = v; break;
	}
}
static void ym_irq(int state) { s_ym_irq = state; }
static void z_start(void)
{
	z80_init(&s_z80);
	s_z80.read_byte = z_read;
	s_z80.write_byte = z_write;
	s_z80.port_in = z_in;
	s_z80.port_out = z_out;
	s_z80.read_map = s_zmap;
}
static void z80_sync(int64_t cyc68)   // the Z80 up to the 68000's cycle cyc68 (4 MHz: a third)
{
	const int64_t target = cyc68 / 3;
	if (target <= n.z80_cyc)
		return;
	const unsigned long c0 = s_z80.cyc;
	z80_run(&s_z80, c0 + (unsigned long)(target - n.z80_cyc), &s_ym_irq, 0);
	n.z80_cyc += (int64_t)(unsigned long)(s_z80.cyc - c0);
}

// ---- the frame ----
static void soft_reset(void);
static void run68_until(int64_t t)
{
	while (n.cyc < t) {
		int64_t lim = t;
		if (n.irq2_at >= 0 && n.irq2_at < lim)
			lim = n.irq2_at > n.cyc ? n.irq2_at : n.cyc + 1;
		const int used = m68k_execute((int)(lim - n.cyc));
		n.cyc += used;
		if (n.irq2_at >= 0 && n.cyc >= n.irq2_at) {   // display_position_interrupt_callback
			const int64_t at = n.irq2_at;
			n.irq2_at = -1;
			if ((n.irq2_ctrl >> IRQ2_ENABLE) & 1) {
				n.irq_rast = 1;
				update_irq();
			}
			if ((n.irq2_ctrl >> IRQ2_AUTOLOAD_REPEAT) & 1)
				irq2_arm(at);
		}
		z80_sync(n.cyc);
		if (n.cyc - n.wd_kick >= WATCHDOG_CYC) {
			soft_reset();
			return;
		}
	}
}

static void sound_line(void)
{
	const uint32_t clocks = (uint32_t)rbx_line_cycles(&n.ym_frac, YM_CLOCK, FRAME_MHZ, LINES);
	const int k = rbx_ym_advance_lr(0, clocks, s_mix_l + s_mix_n, s_mix_r + s_mix_n, s_mix_cap - s_mix_n);
	s_mix_n += k;
	rtc_advance(rbx_line_cycles(&n.rtc_frac, 32768, FRAME_MHZ, LINES));
}

static void ng_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	n.frame0 = n.cyc;
	for (int line = 0; line < LINES; line++) {
		const int64_t t0 = n.frame0 + (int64_t)line * LINE_CYC;
		if (line == VIS_Y0)
			frame_start_lines();
		else if (line == VIS_Y1) {
			vblank_draw();
			run68_until(t0 + 29);   // (NEOGEO_VBLANK_IRQ_HTIM: 58 master clocks)
			n.irq_vbl = 1;
			update_irq();
			run68_until(t0 + 573);   // the display counter reloaded (1146 master clocks)
			if ((n.irq2_ctrl >> IRQ2_AUTOLOAD_VBLANK) & 1)
				irq2_arm(t0 + 573);
		} else if (line == 0x100) {   // the auto animation (NEOGEO_VSSTART)
			if (n.anim_frame_counter == 0) {
				n.anim_frame_counter = n.anim_speed;
				n.anim_counter++;
			} else
				n.anim_frame_counter--;
		}
		run68_until(t0 + LINE_CYC);
		sound_line();
	}
	// (the cycles past the frame's end start the next one)
	rbx_ym_async_take_lr(0, s_mix_l, s_mix_r, s_mix_n);
	for (int i = 0; i < s_mix_n; i++)
		rbx_audio_put_stereo(s_mix_l[i], s_mix_r[i]);
	memset(s_mix_l, 0, sizeof(int32_t) * (size_t)s_mix_n);
	memset(s_mix_r, 0, sizeof(int32_t) * (size_t)s_mix_n);
	s_mix_n = 0;
	n.frame++;
}

// machine_reset (and the watchdog's): the CPUs, the latch, the sound; the RAMs kept
static void soft_reset(void)
{
	n.latch = 0;
	set_vectors();
	latch_w(Q_CART_AUDIO, 0);
	{
		const uint8_t *mainz = s_zbios ? s_zbios : s_zrom;
		for (int p = 0; p < 0x80; p++)
			s_zmap[p] = mainz + p * 256;
	}
	n.zbank[0] = 0x1e; n.zbank[1] = 0x0e; n.zbank[2] = 0x06; n.zbank[3] = 0x02;   // (init_audio)
	z_banks();
	z_start();
	n.nmi_enable = 0;
	n.snd_pending = 0;
	n.nmi_out = 0;
	s_ym_irq = 0;
	rbx_ym_reset(0);
	n.bank_base = s_rom_size > 0x100000 ? 0x100000 : 0;
	map_bank();
	n.irq_vbl = n.irq_rast = 0;
	n.irq2_at = -1;
	n.wd_kick = n.cyc;
	m68k_bus_irq_reset();
	m68k_pulse_reset();
	update_irq();
}

static void ng_reset(void)
{
	memset(&n, 0, offsetof(__typeof__(n), state_end));
	memset(s_zram, 0, sizeof(s_zram));
	rtc_reset();
	n.irq3 = 1;   // (an IRQ3 at power on, not on a reset)
	n.sma_rng = 0x2345;
	neo_proct0_reset(&n.ct0);
	s_logging = 0;
	s_mix_n = 0;
	s_pal_valid = 0;
	soft_reset();
}

// ---- init ----
static void decode_sprites(void)   // neosprite_optimized_device's layout, packed: 16 pixels a row, 8 bytes
{
	for (uint32_t t = 0; t + 0x80 <= s_spr_size; t += 0x80) {
		uint8_t *src = s_spr + t, out[0x80];
		for (int y = 0; y < 16; y++) {
			for (int half = 0; half < 2; half++) {
				const int o = (half ? 0x00 : 0x40) | (y << 2);
				for (int x = 0; x < 8; x += 2) {
					uint8_t pv[2];
					for (int k = 0; k < 2; k++) {
						const int xx = x + k;
						pv[k] = (uint8_t)(((src[o | 3] >> xx) & 1) << 3 | ((src[o | 1] >> xx) & 1) << 2 |
						                  ((src[o | 2] >> xx) & 1) << 1 | ((src[o | 0] >> xx) & 1));
					}
					out[y * 8 + half * 4 + (x >> 1)] = (uint8_t)(pv[0] | pv[1] << 4);
				}
			}
		}
		memcpy(src, out, sizeof(out));
	}
	uint32_t mask = 0xffffffff;
	for (uint32_t bit = 0x80000000u; bit; bit >>= 1) {   // get_region_mask (in pixels: twice the bytes)
		if ((s_spr_size * 2 - 1) & bit)
			break;
		mask >>= 1;
	}
	s_spr_mask = mask;
}

static void rgb_lookups(void)   // create_rgb_lookups (compute_resistor_weights, 5 resistors, 4 pulldowns)
{
	static const double res[5] = { 3900, 2200, 1000, 470, 220 };
	static const double pd[4] = { 0, 8200, 150, 0 };
	double w[4][5];
	double scale = 0;
	for (int net = 0; net < 4; net++) {
		double pdv = pd[net];
		if (net == 3)
			pdv = (double)(int)(1.0 / ((1.0 / 8200) + (1.0 / 150)));   // (passed as an int)
		double sum = 0;
		for (int i = 0; i < 5; i++) {
			double r0 = pdv == 0 ? 1.0 / 1e12 : 1.0 / pdv, r1 = 1.0 / 1e12;
			for (int j = 0; j < 5; j++) {
				if (j == i)
					r1 += 1.0 / res[j];
				else
					r0 += 1.0 / res[j];
			}
			r0 = 1.0 / r0;
			r1 = 1.0 / r1;
			double v = 255.0 * r0 / (r1 + r0);
			if (v < 0) v = 0;
			if (v > 255) v = 255;
			w[net][i] = v;
			sum += v;
		}
		if (net == 0)
			scale = 255.0 / sum;
	}
	for (int i = 0; i < 32; i++)
		for (int net = 0; net < 4; net++) {
			double s = 0;
			for (int b = 0; b < 5; b++)
				if ((i >> b) & 1)
					s += w[net][b] * scale;
			s_lookup[i][net] = (uint8_t)(int)(s + 0.5);
		}
}

static void ng_exit(void)
{
	rbx_ym_exit(0);
	free(s_mix_l);
	free(s_mix_r);
	s_mix_l = s_mix_r = NULL;
	free(s_frames[0]);
	free(s_frames[1]);
	s_frames[0] = s_frames[1] = s_cur = s_snap = NULL;
	free(s_band_vram_hp);
	s_band_vram_hp = NULL;
	free(s_pal_conv_hp);
	s_pal_conv_hp = NULL;
}

static const cart_t *cart_of(const char *machine)
{
	for (const cart_t *c = s_carts; c->machine; c++)
		if (!strcmp(c->machine, machine))
			return c;
	return NULL;
}
static int ng_supported(const char *machine)
{
	return cart_of(machine) != NULL;
}

// decrypt_all of the cartridge (the P ROM in the MAME host-order layout, before the sprites are packed)
static int cart_decrypt(void)
{
	const cart_t *c = s_cart;
	int rc = 0;
	switch (c->type) {
	case C_KOF98: rc = neo_kof98_decrypt_68k(s_rom, s_rom_size, s_kof98_default); break;
	case C_SMA:
		if (s_rom_size < 0x900000)
			return -1;
		switch (c->prot) {
		case P_KOF99: neo_sma_kof99_decrypt_68k(s_rom); break;
		case P_GAROU: neo_sma_garou_decrypt_68k(s_rom); break;
		case P_GAROUH: neo_sma_garouh_decrypt_68k(s_rom); break;
		case P_MSLUG3: neo_sma_mslug3_decrypt_68k(s_rom); break;
		case P_MSLUG3A: neo_sma_mslug3a_decrypt_68k(s_rom); break;
		case P_KOF2000: neo_sma_kof2000_decrypt_68k(s_rom); break;
		}
		break;
	case C_K2K2:
		rc = c->prot == P_SAMSHO5 ? neo_samsho5_decrypt_68k(s_rom, s_rom_size)
		   : c->prot == P_SAMSH5SP ? neo_samsh5sp_decrypt_68k(s_rom, s_rom_size) : neo_kof2002_decrypt_68k(s_rom, s_rom_size);
		break;
	case C_PVC:
		switch (c->prot) {
		case P_MSLUG5: rc = neo_mslug5_decrypt_68k(s_rom, s_rom_size); break;
		case P_SVC: rc = neo_svc_px_decrypt(s_rom, s_rom_size); break;
		case P_KOF2003: rc = neo_kof2003_decrypt_68k(s_rom, s_rom_size); break;
		case P_KOF2003H: rc = neo_kof2003h_decrypt_68k(s_rom, s_rom_size); break;
		}
		break;
	}
	if (rc)
		return rc;
	if (c->type == C_PVC && c->pcm2 < 0)   // (the PVC carts swap the V ROMs first)
		rc |= neo_pcm2_swap(s_adpcma, s_adpcma_size, -1 - c->pcm2);
	if (c->m1) {
		uint32_t csz;
		uint8_t *crypt = rbx_region("cslot1:audiocrypt", &csz);
		if (!crypt || neo_cmc50_m1_decrypt(crypt, csz, s_zrom, s_zrom_size))
			return -1;
	}
	const int cmc50 = c->type == C_CMC50 || c->type == C_PCM2 || c->type == C_K2K2 || c->type == C_PVC ||
	                  (c->type == C_SMA && c->prot == P_KOF2000);
	if (c->type == C_CMC42 || c->type == C_SMA || cmc50)
		rc |= cmc50 ? neo_cmc50_gfx_decrypt(s_spr, s_spr_size, c->key) : neo_cmc42_gfx_decrypt(s_spr, s_spr_size, c->key);
	if (c->sfix && s_fix)
		neo_sfix_decrypt(s_spr, s_spr_size, s_fix, s_fix_size);
	if (c->type == C_PCM2)
		neo_pcm2_decrypt(s_adpcma, s_adpcma_size, c->pcm2);
	else if (c->type == C_K2K2 && c->pcm2 < 0)
		rc |= neo_pcm2_swap(s_adpcma, s_adpcma_size, -1 - c->pcm2);
	return rc;
}

static int ng_init(const rbx_game *g)
{
	if (!s_band_vram_hp && !(s_band_vram_hp = calloc(1, sizeof(*s_band_vram_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	if (!s_pal_conv_hp && !(s_pal_conv_hp = calloc(1, sizeof(*s_pal_conv_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	s_cart = cart_of(g->machine);
	if (!s_cart) {
		rbx_fail("%s: machine %s not emulated", g->set, g->machine);
		return -1;
	}
	s_rom = rbx_region("cslot1:maincpu", &s_rom_size);
	s_bios = rbx_region("mainbios", &s_bios_size);
	s_zrom = rbx_region("cslot1:audiocpu", &s_zrom_size);
	s_zbios = rbx_region("audiobios", &s_zbios_size);
	s_spr = rbx_region("cslot1:sprites", &s_spr_size);
	s_fix = rbx_region("cslot1:fixed", &s_fix_size);
	s_fixbios = rbx_region("fixedbios", &s_fixbios_size);
	s_zoomy = rbx_region("spritegen:zoomy", &s_zoomy_size);
	s_adpcma = rbx_region("cslot1:ymsnd:adpcma", &s_adpcma_size);
	s_adpcmb = rbx_region("cslot1:ymsnd:adpcmb", &s_adpcmb_size);
	if (!s_adpcmb) {
		s_adpcmb = s_adpcma;
		s_adpcmb_size = s_adpcma_size;
	}
	if (!s_rom || !s_bios || !s_zrom || !s_spr || !s_fixbios || !s_zoomy || s_zoomy_size < 0x10000 || s_rom_size < 0x1000) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	m68k_bus_swap16(s_rom, s_rom_size);
	m68k_bus_swap16(s_bios, s_bios_size);
	if (cart_decrypt()) {
		rbx_fail("%s: decryption failed", g->set);
		return -1;
	}
	decode_sprites();
	rgb_lookups();
	memset(&n, 0, sizeof(n));
	m68k_init();
	m68k_set_cpu_type(M68K_CPU_TYPE_68000);
	m68k_bus_reset_map();
	m68k_bus.read8 = main_r8;
	m68k_bus.read16 = main_r16;
	m68k_bus.write8 = main_w8;
	m68k_bus.write16 = main_w16;
	m68k_bus_map(0x000000, 0x000fff, s_page0, sizeof(s_page0), 1, 0);
	m68k_bus_map(0x001000, 0x0fffff, s_rom + 0x1000, (s_rom_size < 0x100000 ? s_rom_size : 0x100000) - 0x1000, 1, 0);
	for (uint32_t m = 0x100000; m < 0x200000; m += 0x10000)
		m68k_bus_map(m, m + 0xffff, n.ram, 0x10000, 1, 1);
	for (uint32_t m = 0xc00000; m < 0xd00000; m += 0x20000)
		m68k_bus_map(m, m + 0x1ffff, s_bios, s_bios_size < 0x20000 ? s_bios_size : 0x20000, 1, 0);
	for (uint32_t m = 0xd00000; m < 0xe00000; m += 0x10000)
		m68k_bus_map(m, m + 0xffff, n.sram, 0x10000, 1, 0);
	s_port_dsw = rbx_port_find("DSW");
	s_port_system = rbx_port_find("SYSTEM");
	s_port_audio = rbx_port_find("AUDIO_COIN");
	s_port_test = rbx_port_find("TEST");
	s_port_joy[0] = rbx_port_find("JOY1");
	s_port_joy[1] = rbx_port_find("JOY2");
	s_port_start = rbx_port_find("START");
	s_frames[0] = (frame_t *)calloc(1, sizeof(frame_t));
	s_frames[1] = (frame_t *)calloc(1, sizeof(frame_t));
	s_cur = s_frames[0];
	s_snap = s_frames[1];
	rbx_ym_config yc;
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM2610;
	yc.clock = YM_CLOCK;
	yc.adpcma = s_adpcma;
	yc.adpcma_size = s_adpcma_size;
	yc.adpcmb = s_adpcmb;
	yc.adpcmb_size = s_adpcmb_size;
	yc.gain[0] = 215;   // neogeo_stereo: the SSG 0.84 to both sides, the FM / ADPCM 0.98 to theirs
	yc.gain[1] = yc.gain[2] = 251;
	yc.irq = ym_irq;
	// (the YM2610's samples made on the worker: rbx_ym_async_take_lr at the frame's end)
	if (!s_frames[0] || !s_frames[1] || rbx_ym_init(0, &yc) || rbx_ym_set_async(0, 2)) {
		rbx_fail("%s: out of memory", g->set);
		ng_exit();
		return -1;
	}
	s_mix_cap = (int)(rbx_ym_rate(0) / 50) + 256;
	s_mix_l = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	s_mix_r = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	if (!s_mix_l || !s_mix_r || rbx_audio_setup_stereo(rbx_ym_rate(0)) || rbx_video_setup(VIS_W, VIS_H, 0x4000)) {
		rbx_fail("%s: out of memory", g->set);
		ng_exit();
		return -1;
	}
	ng_reset();
	return 0;
}

static int ng_state(void *ctx, int load)
{
	const uint32_t csize = m68k_context_size();
	uint8_t *cpu = (uint8_t *)malloc(csize);
	if (!cpu)
		return -1;
	if (!load)
		m68k_get_context(cpu);
	z80 z = s_z80;
	int32_t ymirq = s_ym_irq;
	int rc = 0;
	rc |= rbx_state_io(ctx, "m68k", cpu, csize);
	rc |= rbx_state_io(ctx, "neogeo", &n, (uint32_t)offsetof(__typeof__(n), state_end));
	rc |= rbx_state_io(ctx, "z80", &z, sizeof(z));
	rc |= rbx_state_io(ctx, "zram", s_zram, sizeof(s_zram));
	rc |= rbx_state_io(ctx, "ymirq", &ymirq, sizeof(ymirq));
	rc |= rbx_ym_state(0, ctx, load);
	if (!rc && load) {
		m68k_set_context(cpu);
		z.read_byte = z_read;
		z.write_byte = z_write;
		z.port_in = z_in;
		z.port_out = z_out;
		z.userdata = NULL;
		z.read_map = s_zmap;
		s_z80 = z;
		s_ym_irq = ymirq;
		set_vectors();
		map_bank();
		{
			const uint8_t *mainz = ((n.latch >> Q_CART_AUDIO) & 1) || !s_zbios ? s_zrom : s_zbios;
			for (int p = 0; p < 0x80; p++)
				s_zmap[p] = mainz + p * 256;
		}
		z_banks();
		update_irq();
		s_pal_valid = 0;
	}
	free(cpu);
	return rc ? -1 : 0;
}

static int ng_save(void *ctx) { return ng_state(ctx, 0); }
static int ng_load(void *ctx) { return ng_state(ctx, 1); }

const rbx_driver rbx_drv_neogeo = { "SNK Neo Geo", ng_init, ng_exit, ng_reset, ng_frame, ng_save, ng_load,
                                    ng_supported, ng_render, ng_render_help };
