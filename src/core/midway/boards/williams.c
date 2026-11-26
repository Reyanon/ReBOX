/*  Williams 6809 hardware, ported to C from MAME src/mame/williams/williams.cpp, williams.h, williams_m.cpp,
    williams_v.cpp, williamsblitter.cpp / .h (BSD-3-Clause, copyright-holders: Aaron Giles) and the Joust 2 background
    sound board from src/mame/shared/s11c_bg.cpp (BSD-3-Clause, copyright-holders: Barry Rodewald, Jonathan
    Gevaryahu), its CVSD filters as src/devices/sound/flt_biquad.cpp (BSD-3-Clause, copyright-holders: K.Wilkins,
    Couriersud, Derrick Renaud, Frank Palazzolo, Jonathan Gevaryahu), the watchdog's default of
    src/devices/machine/watchdog.cpp and the ticket dispenser of src/devices/machine/ticket.cpp (BSD-3-Clause,
    copyright-holders: Aaron Giles).
    Main CPU: MC6809E at 12 MHz / 3 / 4 (cpu/m6809). Sound: M6808 at 3.579545 MHz (internal / 4; 12 MHz / 3 on the
    second generation, cpu/m6800) with a 6821 PIA (machine/pia6821) and an MC1408 DAC; Sinistar and PlayBall add an
    HC-55516 (sound/hc55516); the Sinistar cockpit and Blaster a second sound board; Joust 2 the D-11298 background
    board (MC6809E at 2 MHz, YM2151 on rbx_ym, PIA, DAC, HC-55516 behind its two op-amp filters).
    Video: the 4 bpp bitmap of 0x0000-0xbfff in column order (byte x / 2 * 256 + y), 16 pens of 8-bit RRRGGGBB colors;
    the SC1 / SC2 blitter (VL2001 / VL2001A) halting the CPU for its accesses; Sinistar's and Blaster's window clip,
    Blaster's PROM remap, per-line color 0 and erase-behind; the second generation's 24x16 tile background with its
    1024-entry palette (the netlist table of calc_col), Mystic Marathon's comparator colors.
    Machines (MAME's configs): defender, defender_6802snd, nextcent, jin, mayday, williams_base, williams_b1, joust,
    splat, alienar, bubbles, sinistar_upright, sinistar_cockpit, playball, spdball, lottofun, blastkit, blaster,
    mysticm, tshoot, inferno, joust2 (every one of williams.cpp).
    Timing: 260 lines of 64 main CPU clocks (60.10 Hz; Mystic Marathon 256 lines); every CPU runs a line at a time
    (MAME's synchronize()d sound commands land at the line's end of the main CPU). Each line is captured into the
    frame's snapshot as the beam passes it (MAME's VIDEO_UPDATE_SCANLINE), drawn by render.
    Sound: each sound CPU's DAC / CVSD levels box-filtered on its own clock into samples (the same grid for all),
    summed at the line's end; a 5 Hz DC block.
    The watchdog (MAME's 3 s default once kicked) soft-resets the board. A first power on (blank CMOS) is set up as an
    operator would: High Score Reset held at power on, Advance pressed while the game waits for it (setup_frame).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../rbx_core.h"
#include "../../rbx_prof.h"
#include "../../cpu/m6809/m6809.h"
#include "../../cpu/m6800/m6800.h"
#include "../../machine/pia6821.h"
#include "../../sound/hc55516.h"
#include "../../sound/rbx_ym.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// ---- machine configurations ----
enum { MAP_DEF, MAP_MAYDAY, MAP_STD, MAP_SINISTAR, MAP_BUBBLES, MAP_SPDBALL, MAP_ALIENAR, MAP_BLASTER, MAP_W2RAM, MAP_W2ROM };
enum { IN_PLAIN, IN_MUXED, IN_49WAY, IN_CONQUEST, IN_SPDBALL, IN_LOTTOFUN, IN_BLASTKIT, IN_BLASTER, IN_W2MUX, IN_TSHOOT,
       IN_JOUST2 };
enum { SND_STD, SND_CVSD, SND_COCKPIT, SND_PLAYBALL, SND_BLASTER, SND_W2, SND_JOUST2 };
enum { TL_BASE, TL_MYSTIC, TL_JOUST2 };
enum { IRQ_W1, IRQ_W2, IRQ_MYSTIC, IRQ_TSHOOT };

typedef struct {
	const char *name;
	uint8_t gen, blit, map, in, snd, tiles, irq, proms;
	uint16_t clip;
	uint16_t x0, x1;   // visible area (inclusive)
	uint8_t y0, y1;
	uint16_t vtotal;
	uint32_t snd_clock;   // the sound CPU's clock (after its internal / 4)
} wcfg_t;

#define SND1 894886   // 3.579545 MHz / 4 (rounded; the exact ratio is SND1_STEP)
static const wcfg_t s_cfgs[] = {
	{ "defender",         1, 0, MAP_DEF,      IN_PLAIN,    SND_STD,      0, IRQ_W1, 0, 0xc000, 12, 303, 7, 246, 260, SND1 },
	{ "defender_6802snd", 1, 0, MAP_DEF,      IN_PLAIN,    SND_STD,      0, IRQ_W1, 0, 0xc000, 12, 303, 7, 246, 260, SND1 },
	{ "nextcent",         1, 0, MAP_DEF,      IN_PLAIN,    SND_STD,      0, IRQ_W1, 0, 0xc000, 12, 303, 7, 246, 260, 780000 },
	{ "jin",              1, 0, MAP_DEF,      IN_PLAIN,    SND_STD,      0, IRQ_W1, 0, 0xc000, 0, 315, 7, 246, 260, SND1 },
	{ "mayday",           1, 0, MAP_MAYDAY,   IN_PLAIN,    SND_STD,      0, IRQ_W1, 0, 0xc000, 12, 303, 7, 246, 260, SND1 },
	{ "williams_base",    1, 0, MAP_STD,      IN_PLAIN,    SND_STD,      0, IRQ_W1, 0, 0xc000, 6, 297, 7, 246, 260, SND1 },
	{ "williams_b1",      1, 1, MAP_STD,      IN_PLAIN,    SND_STD,      0, IRQ_W1, 0, 0xc000, 6, 297, 7, 246, 260, SND1 },
	{ "joust",            1, 1, MAP_STD,      IN_MUXED,    SND_STD,      0, IRQ_W1, 0, 0xc000, 6, 297, 7, 246, 260, SND1 },
	{ "splat",            1, 2, MAP_STD,      IN_MUXED,    SND_STD,      0, IRQ_W1, 0, 0xc000, 6, 297, 7, 246, 260, SND1 },
	{ "alienar",          1, 1, MAP_ALIENAR,  IN_MUXED,    SND_STD,      0, IRQ_W1, 0, 0xc000, 6, 297, 7, 246, 260, SND1 },
	{ "bubbles",          1, 1, MAP_BUBBLES,  IN_PLAIN,    SND_STD,      0, IRQ_W1, 0, 0xc000, 6, 297, 7, 246, 260, SND1 },
	{ "sinistar_upright", 1, 1, MAP_SINISTAR, IN_49WAY,    SND_CVSD,     0, IRQ_W1, 0, 0x7400, 6, 297, 7, 246, 260, SND1 },
	{ "sinistar_cockpit", 1, 1, MAP_SINISTAR, IN_49WAY,    SND_COCKPIT,  0, IRQ_W1, 0, 0x7400, 6, 297, 7, 246, 260, SND1 },
	{ "playball",         1, 1, MAP_STD,      IN_PLAIN,    SND_PLAYBALL, 0, IRQ_W1, 0, 0xc000, 6, 297, 8, 239, 260, SND1 },
	{ "spdball",          1, 1, MAP_SPDBALL,  IN_SPDBALL,  SND_STD,      0, IRQ_W1, 0, 0xc000, 6, 297, 7, 246, 260, SND1 },
	{ "lottofun",         1, 1, MAP_STD,      IN_LOTTOFUN, SND_STD,      0, IRQ_W1, 0, 0xc000, 6, 297, 7, 246, 260, SND1 },
	{ "blastkit",         1, 2, MAP_BLASTER,  IN_BLASTKIT, SND_STD,      0, IRQ_W1, 1, 0x9700, 6, 297, 7, 246, 260, SND1 },
	{ "blaster",          1, 2, MAP_BLASTER,  IN_BLASTER,  SND_BLASTER,  0, IRQ_W1, 1, 0x9700, 6, 297, 7, 246, 260, SND1 },
	{ "mysticm",          2, 2, MAP_W2RAM,    IN_PLAIN,    SND_W2,  TL_MYSTIC, IRQ_MYSTIC, 0, 0x9000, 8, 283, 8, 247, 256, 1000000 },
	{ "tshoot",           2, 2, MAP_W2ROM,    IN_TSHOOT,   SND_W2,  TL_BASE,   IRQ_TSHOOT, 0, 0x9000, 8, 283, 8, 247, 260, 1000000 },
	{ "inferno",          2, 2, MAP_W2RAM,    IN_W2MUX,    SND_W2,  TL_BASE,   IRQ_W2, 0, 0x9000, 8, 283, 8, 247, 260, 1000000 },
	{ "joust2",           2, 2, MAP_W2ROM,    IN_JOUST2,   SND_JOUST2, TL_JOUST2, IRQ_W2, 0, 0x9000, 8, 283, 8, 247, 260, 1000000 },
};

#define LINE_CLOCKS 64        // 512 pixels at 8 MHz = 64 main CPU clocks of 1 MHz
#define MAX_W 316
#define MAX_H 240
#define NCH 2                 // sound channels (each one CPU's domain)
#define FIFO 32

static const wcfg_t *s_c;
static int s_in;   // the input wiring: the config's, or Conquest's dial (williams_b1 with a DIAL port)
static int s_defndjeu;
static int s_vis_w, s_vis_h;

// ROM
static uint8_t *s_mrom, *s_banked, *s_srom, *s_srom2, *s_bgrom, *s_tiles_rom, *s_proms;
static uint32_t s_mrom_size, s_banked_size, s_bgrom_size, s_tiles_size, s_proms_size;

// CPUs and chips
static m6809_t s_main, s_bg;
static m6800_t s_snd[2];
static pia6821_t s_pia[4], s_bgpia;
static hc55516_t s_cvsd;    // the first sound board's (Sinistar, PlayBall) or the Joust 2 background board's

// memory
static uint8_t *s_vram;     // 0x0000-0xbfff (48 KB)
static uint8_t s_nvram[0x400], s_palram[0x800], s_tileram[0x800], s_sram[2][0x100], s_hiram[0x1000], s_bgram[0x800];
static uint8_t *s_remap_lookup;   // Blaster: 256 x 256 PROM remap
static uint8_t *s_tiles;          // second generation: decoded tiles, 24 x 16 bytes each
static int s_tile_count;
static uint32_t s_pal_rgb[1024];  // second generation: the palette's colors (calc_col)
static uint32_t s_rgb256[256];    // first generation: the 8-bit colors

// the MAME ports
static int s_port[16];
enum { P_IN0, P_IN1, P_IN2, P_IN3, P_IN4, P_INP1, P_INP2, P_INP1A, P_INP2A, P_X49, P_Y49, P_AN0, P_AN1, P_AN2, P_AN3,
       P_DIAL, P_COUNT };
static const char *const s_port_tags[P_COUNT] = { "IN0", "IN1", "IN2", "IN3", "IN4", "INP1", "INP2", "INP1A", "INP2A",
                                                  "49WAYX", "49WAYY", "AN0", "AN1", "AN2", "AN3", "DIAL" };

// sound channel: a CPU's output levels box-filtered on its clock (16.16 cycles) into samples
typedef struct {
	int64_t t, next, acc, acc2;
	int32_t level, level2;    // level2: the Joust 2 CVSD path (filtered apart)
	int32_t n, pad_;
	int32_t fifo[FIFO];
} chan_t;

// machine state (saved)
typedef struct {
	// video / banking
	uint8_t rom_view, cocktail, defbank, blaster_bank, video_control, w2bank, palette_view, fg_color, bg_select, pad0_;
	uint16_t xscroll;
	uint8_t blaster_color0;
	// blitter
	uint8_t window, control, solid_color, width, height, remap_index, blitting, pad1_;
	uint16_t sstart, dstart;
	// inputs
	uint8_t mux_sel, mux_sel_b, dial, an[4], gun[2], ticket_power, ticket_status, pad2_;
	int16_t x49, y49;
	int32_t ticket_timer;
	int32_t watchdog;          // main CPU clocks left (0: not running; MAME's 3 s default, started by the first kick)
	// IRQ lines
	uint8_t irq_in[4], snd_irq[2][2], bg_resetq, ym_reset, pad3_[4];
	uint8_t dac[2], bgdac;
	uint8_t pad4_;
	uint16_t line;
	uint16_t pad5_;
	uint64_t snd_tgt[2];     // sound CPU targets, 32.32 cycles
	int64_t main_tgt, bg_tgt;
	chan_t ch[NCH];
	int32_t hp_x[2], pad6_[2];
	int64_t hp_y[2];            // DC block (16.16)
	float f1[3], f2[3];         // the Joust 2 CVSD filters' state (w0..w2)
} wstate_t;
static wstate_t S;

static int s_nch;             // channels in use
static int s_stereo;
static int64_t s_P16[NCH], s_inv[NCH];
static uint64_t s_snd_step;    // the sound CPU's clocks per line, 32.32
static uint32_t s_rate;
static uint32_t s_frame_mhz;
static int32_t s_dac_gain, s_bgdac_gain;
static float s_cvsd_gain;     // first generation CVSD (x 1.0 = hc55516 scale)
static float s_bgcvsd_gain;   // Joust 2: the CVSD filters' output (MAME: x 4 x 0.1170, the board x 2.0)
static float s_b1[3], s_a1[3], s_b2[3], s_a2[3];   // the Joust 2 CVSD biquads (b0..b2, a1, a2)
static char s_nv_path[512];
static uint32_t s_nv_size;

// render snapshot
typedef struct {
	uint16_t pix[MAX_H * MAX_W];       // first generation: color indices; second: fg nibble (0 = transparent)
	uint32_t zero_rgb[MAX_H];          // Blaster: each line's color 0
	uint16_t fgbase[MAX_H], scroll[MAX_H];
	uint8_t bgsel[MAX_H];
	uint8_t tileram[0x800];
	uint32_t pal[1024];
	int valid;
} snap_t;
static snap_t *s_snap[2];
static int s_work;   // the snapshot being captured (the other one is drawn)

enum { WM_PA_MAIN = RBX_PA_DRV, WM_PA_SOUND, WM_PA_BG, WM_PA_BLIT, WM_PA_LINE, WM_PA_YM, WM_PA_END };
static const char *const s_pa_names[] = { "wms main 6809", "wms sound 6808", "wms bg 6809", "wms blitter",
                                          "wms line capture", "wms YM2151" };

static inline int bit(int v, int n) { return (v >> n) & 1; }
static inline uint8_t rport(int p) { return s_port[p] >= 0 ? (uint8_t)rbx_port_read(s_port[p]) : 0; }

// ---- sound channels ----
static void chan_emit(int i, chan_t *c);
static void chan_run(int i, int64_t cycles)
{
	chan_t *c = &S.ch[i];
	const int64_t t = cycles * 65536;
	if (t <= c->t)
		return;
	while (c->next <= t) {
		const int64_t d = c->next - c->t;
		c->acc += (int64_t)c->level * d;
		c->acc2 += (int64_t)c->level2 * d;
		c->t = c->next;
		c->next += s_P16[i];
		chan_emit(i, c);
	}
	const int64_t d = t - c->t;
	c->acc += (int64_t)c->level * d;
	c->acc2 += (int64_t)c->level2 * d;
	c->t = t;
}

static inline float biquad(float *w, const float *b, const float *a, float in)
{
	w[2] = w[1];
	w[1] = w[0];
	w[0] = -a[1] * w[1] - a[2] * w[2] + in;
	return b[0] * w[0] + b[1] * w[1] + b[2] * w[2];
}

static void chan_emit(int i, chan_t *c)
{
	int32_t s = (int32_t)((c->acc * s_inv[i]) >> 32);
	c->acc = 0;
	if (i == 1 && s_c->snd == SND_JOUST2) {
		// the background board: DAC + YM2151 + CVSD behind its filters (MAME's 1/4 before, x 4 after)
		const float cv = (float)((c->acc2 * s_inv[i]) >> 32) * 0.25f;
		const float y = biquad(S.f2, s_b2, s_a2, biquad(S.f1, s_b1, s_a1, cv));
		s += (int32_t)(y * s_bgcvsd_gain);
		int32_t ym = 0;
		const unsigned pa = rbx_prof_enter(WM_PA_YM);
		rbx_ym_advance(0, 64, &ym, 1);
		rbx_prof_leave(pa);
		s += ym;
	}
	c->acc2 = 0;
	if (c->n < FIFO)
		c->fifo[c->n++] = s;
}

static void chan_level(int i, int64_t cycles, int32_t level)
{
	chan_run(i, cycles);
	S.ch[i].level = level;
}

static inline int32_t dc_block(int side, int32_t x)
{
	// y = x - x1 + y1 * (1 - 2 pi 5 / rate): Q15 coefficient
	const int32_t k = 32768 - (int32_t)(32768.0 * 2 * 3.14159265 * 5 / 50000);
	const int64_t y = ((int64_t)(x - S.hp_x[side]) << 16) + ((S.hp_y[side] * k) >> 15);
	S.hp_x[side] = x;
	S.hp_y[side] = y;
	return (int32_t)(y >> 16);
}

static void mix_line(void)
{
	int k = S.ch[0].n;
	for (int i = 1; i < s_nch; i++)
		if (S.ch[i].n < k)
			k = S.ch[i].n;
	for (int j = 0; j < k; j++) {
		if (s_stereo)
			rbx_audio_put_stereo(dc_block(0, S.ch[0].fifo[j]), dc_block(1, S.ch[1].fifo[j]));
		else {
			int32_t s = S.ch[0].fifo[j];
			for (int i = 1; i < s_nch; i++)
				s += S.ch[i].fifo[j];
			rbx_audio_put(dc_block(0, s));
		}
	}
	for (int i = 0; i < s_nch; i++) {
		chan_t *c = &S.ch[i];
		for (int j = k; j < c->n; j++)
			c->fifo[j - k] = c->fifo[j];
		c->n -= k;
	}
}

// ---- the sound boards ----
static void snd_dac_w(void *ctx, uint8_t d)
{
	const int b = (int)(intptr_t)ctx;
	S.dac[b] = d;
	int32_t lv = ((int32_t)d - 128) * s_dac_gain;
	if (b == 0 && (s_c->snd == SND_CVSD || s_c->snd == SND_COCKPIT || s_c->snd == SND_PLAYBALL))
		lv += (int32_t)((float)hc55516_out(&s_cvsd) * s_cvsd_gain);
	chan_level(b, s_snd[b].cycles, lv);
}
static void snd_cvsd_update(void)
{
	snd_dac_w((void *)(intptr_t)0, S.dac[0]);
}
static void snd_ca2(void *ctx, int st) { (void)ctx; chan_run(0, s_snd[0].cycles); hc55516_digit_w(&s_cvsd, st); }
static void snd_cb2(void *ctx, int st)
{
	(void)ctx;
	chan_run(0, s_snd[0].cycles);
	hc55516_clock_w(&s_cvsd, st);
	snd_cvsd_update();
}
static void snd_irq_update(int b) { m6800_set_irq(&s_snd[b], S.snd_irq[b][0] | S.snd_irq[b][1]); }
static void snd_irqa(void *ctx, int st) { const int b = (int)(intptr_t)ctx; S.snd_irq[b][0] = (uint8_t)st; snd_irq_update(b); }
static void snd_irqb(void *ctx, int st) { const int b = (int)(intptr_t)ctx; S.snd_irq[b][1] = (uint8_t)st; snd_irq_update(b); }

static pia6821_t *snd_pia(int b) { return &s_pia[2 + b]; }

static uint8_t snd_read(void *ctx, uint16_t a)
{
	const int b = (int)(intptr_t)ctx;
	if (s_c->gen == 1) {
		if ((a & 0x7ffc) == 0x0400)
			return pia6821_read(snd_pia(b), a & 3);
	} else if ((a & 0xe000) == 0x2000)
		return pia6821_read(snd_pia(b), a & 3);
	return 0;
}
static void snd_write(void *ctx, uint16_t a, uint8_t v)
{
	const int b = (int)(intptr_t)ctx;
	if (s_c->gen == 1) {
		if ((a & 0x7ffc) == 0x0400)
			pia6821_write(snd_pia(b), a & 3, v);
	} else if ((a & 0xe000) == 0x2000)
		pia6821_write(snd_pia(b), a & 3, v);
}

// the sound command (MAME's deferred_snd_cmd_w): to the sound PIA's port B and CB1
static void snd_cmd_to(int b, uint8_t d)
{
	pia6821_portb_w(snd_pia(b), d);
	pia6821_cb1_w(snd_pia(b), d == 0xff ? 0 : 1);
}

// ---- Joust 2's background sound board (s11_obg) ----
static void bg_map_bank(int bank)
{
	for (int p = 0x80; p < 0x100; p++) {
		const uint32_t o = (uint32_t)bank * 0x8000 + (uint32_t)(p - 0x80) * 256;
		s_bg.rpage[p] = o + 256 <= s_bgrom_size ? s_bgrom + o : NULL;
	}
}
static uint8_t s_bgbank;
static uint8_t bg_read(void *ctx, uint16_t a)
{
	(void)ctx;
	if (a < 0x2000) return s_bgram[a & 0x7ff];
	if (a < 0x4000) { chan_run(1, s_bg.cycles); return rbx_ym_read(0, a & 1); }
	if (a < 0x6000) return pia6821_read(&s_bgpia, a & 3);
	return 0;
}
static void bg_write(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	if (a < 0x2000) { s_bgram[a & 0x7ff] = v; return; }
	if (a < 0x4000) { chan_run(1, s_bg.cycles); rbx_ym_write(0, a & 1, v); return; }
	if (a < 0x6000) { pia6821_write(&s_bgpia, a & 3, v); return; }
	if (a < 0x6800) {   // bg_cvsd_clock_set_w
		chan_run(1, s_bg.cycles);
		hc55516_clock_w(&s_cvsd, 1);
		S.ch[1].level2 = hc55516_out(&s_cvsd);
		return;
	}
	if (a < 0x7000) {   // bg_cvsd_digit_clock_clear_w
		chan_run(1, s_bg.cycles);
		hc55516_clock_w(&s_cvsd, 0);
		hc55516_digit_w(&s_cvsd, v & 1);
		S.ch[1].level2 = hc55516_out(&s_cvsd);
		return;
	}
	if (a >= 0x7800 && a < 0x8000) {   // bgbank_w: bits 1 0 3 2
		s_bgbank = (uint8_t)(((v & 3) << 2) | ((v >> 2) & 3));
		bg_map_bank(s_bgbank);
	}
}
static void bgpia_out_a(void *ctx, uint8_t d)
{
	(void)ctx;
	S.bgdac = d;
	chan_level(1, s_bg.cycles, ((int32_t)d - 128) * s_bgdac_gain);
}
static void bgpia_ca2(void *ctx, int st)
{
	(void)ctx;
	if (st != S.ym_reset) {
		S.ym_reset = (uint8_t)st;
		if (st) {
			chan_run(1, s_bg.cycles);
			rbx_ym_reset(0);
		}
	}
}
static void bgpia_cb2(void *ctx, int st) { (void)ctx; pia6821_cb1_w(&s_pia[2], st); }   // (to the sound PIA's CB1)
static void bgpia_irqa(void *ctx, int st) { (void)ctx; m6809_set_firq(&s_bg, st); }
static void bgpia_irqb(void *ctx, int st) { (void)ctx; m6809_set_nmi(&s_bg, st); }
static void bg_ym_irq(int st) { pia6821_ca1_w(&s_bgpia, !st); }   // (IRQ is not true state)
// a CPU reset in the middle of the run: its clock count kept (the cores' resets clear it)
static void reset6809_keep(m6809_t *c)
{
	const int64_t cy = c->cycles;
	m6809_reset(c);
	c->cycles = cy;
}
static void reset6800_keep(m6800_t *c)
{
	const int64_t cy = c->cycles;
	m6800_reset(c);
	c->cycles = cy;
}
static void bg_common_reset(void)
{
	s_bgbank = 0;
	bg_map_bank(0);
	reset6809_keep(&s_bg);
}
static void bg_resetq_w(int st)
{
	if (S.bg_resetq && !st)
		bg_common_reset();
	S.bg_resetq = (uint8_t)st;
}

// ---- inputs ----
static uint8_t way49(void)
{
	static const uint8_t translate49[7] = { 0x0, 0x4, 0x6, 0x7, 0xb, 0x9, 0x8 };
	return (uint8_t)(translate49[S.x49] << 4 | translate49[S.y49]);
}

static uint8_t pia0_in_a(void *ctx)
{
	(void)ctx;
	switch (s_in) {
	case IN_MUXED: {
		const uint8_t m0 = S.mux_sel ? rport(P_INP1) : rport(P_INP2);
		const uint8_t m1 = S.mux_sel ? rport(P_INP1A) : rport(P_INP2A);
		return (uint8_t)((rport(P_IN0) & 0x30) | (m0 & 0x0f) | ((m1 << 6) & 0xc0));
	}
	case IN_49WAY: return way49();
	case IN_CONQUEST: return (uint8_t)((rport(P_IN0) & ~0x0c) | ((S.dial >> 2 & 3) << 2));
	case IN_LOTTOFUN: return (uint8_t)((rport(P_IN0) & 0x7f) | (S.ticket_status ? 0x80 : 0));
	case IN_BLASTKIT: return S.mux_sel ? way49() : rport(P_IN3);
	case IN_BLASTER: return S.mux_sel ? rport(P_IN3) : way49();
	case IN_W2MUX: return S.mux_sel ? rport(P_INP2) : rport(P_INP1);
	case IN_TSHOOT: {
		const int p = S.mux_sel;
		const uint8_t raw = rport(p ? P_INP2 : P_INP1);
		const int g = S.gun[p] & 0x3f;
		return (uint8_t)((raw & 0xc0) | ((g ^ (g >> 1)) & 0x3f));
	}
	case IN_JOUST2: return (uint8_t)((rport(P_IN0) & 0xf0) | ((S.mux_sel ? rport(P_INP2) : rport(P_INP1)) & 0x0f));
	}
	return rport(P_IN0);
}
static uint8_t pia0_in_b(void *ctx)
{
	(void)ctx;
	switch (s_in) {
	case IN_MUXED: {
		const uint8_t m1 = S.mux_sel ? rport(P_INP1A) : rport(P_INP2A);
		return (uint8_t)((rport(P_IN1) & 0xfc) | ((m1 >> 2) & 0x03));
	}
	case IN_CONQUEST: return (uint8_t)((rport(P_IN1) & ~0x03) | (S.dial & 3));
	case IN_BLASTER: return (uint8_t)(((S.mux_sel ? rport(P_INP2) : rport(P_INP1)) & 0x0f) | (rport(P_IN1) & 0xf0));
	}
	return rport(P_IN1);
}
// First power on (blank CMOS): the games want the operator. As one would: High Score Reset held while the board
// powers on (the factory settings), then Advance pressed whenever the game sits polling the switches for it
// ("factory settings restored", the audit pages), for the first minute
#define SETUP_HSR_FRAMES 90
#define SETUP_FRAMES 3600
#define SETUP_LOTTO_UP 400
#define SETUP_POLLS 10    // IN2 reads in a frame: the game polls the switches for the operator (1-2 when it plays)
#define SETUP_POLL_FRAMES 30   // ... for this many frames in a row (Colony 7's attract reads them in bursts)
static int s_setup, s_setup_adv, s_setup_cool, s_setup_polling;
static uint32_t s_in2_reads;
static uint8_t pia1_in_a(void *ctx)
{
	(void)ctx;
	uint8_t v = rport(P_IN2);
	s_in2_reads++;
	if (s_in == IN_LOTTOFUN)
		v ^= 0x01;   // (Lotto Fun's Memory Protect toggle: down, as the game wants it to play, unless toggled)
	if (s_setup) {
		if (s_setup <= SETUP_HSR_FRAMES)
			v |= s_c->gen == 2 ? 0x04 : 0x08;
		if (s_setup_adv)
			v |= 0x02;
		// Auto Up: Advance steps forward, and ends the audits at "game over"; Lotto Fun's Memory Protect: up (0) for
		// the restore, then down
		if (s_in != IN_LOTTOFUN || s_setup > SETUP_LOTTO_UP)
			v |= 0x01;
		else
			v &= (uint8_t)~0x01;
	}
	return v;
}
static void setup_frame(void)
{
	if (!s_setup)
		return;
	// (over after the first minute; not on a start press: a player pressing START during the factory pages left the
	// game waiting for an Advance the Vita has no key for)
	if (++s_setup > SETUP_FRAMES) {
		s_setup = s_setup_adv = 0;
		return;
	}
	if (s_setup_adv)
		s_setup_adv--;
	if (s_setup_cool)
		s_setup_cool--;
	s_setup_polling = s_in2_reads > SETUP_POLLS ? s_setup_polling + 1 : 0;
	if (s_setup > SETUP_HSR_FRAMES + 30 && !s_setup_cool && s_setup_polling >= SETUP_POLL_FRAMES) {
		s_setup_adv = 3;
		s_setup_cool = 40;
	}
	s_in2_reads = 0;
}
static uint8_t pia3_in_a(void *ctx) { (void)ctx; return rport(P_IN3); }
static uint8_t pia3_in_b(void *ctx) { (void)ctx; return rport(P_IN4); }
static void pia0_mux_w(void *ctx, int st) { (void)ctx; S.mux_sel = (uint8_t)(st != 0); }
static void pia0_out_b(void *ctx, uint8_t d)
{
	(void)ctx;
	if (s_in != IN_LOTTOFUN)
		return;
	const int on = !bit(d, 7);   // (ticket motor_w, bit 7 inverted)
	if (on && !S.ticket_power) {
		S.ticket_timer = 70000;   // 70 ms of main CPU clocks
		S.ticket_power = 1;
		S.ticket_status = 0;
	} else if (!on && S.ticket_power) {
		S.ticket_timer = 0;
		S.ticket_power = 0;
	}
}

static void main_irq_update(void)
{
	const uint8_t *q = S.irq_in;
	switch (s_c->irq) {
	case IRQ_MYSTIC: m6809_set_irq(&s_main, q[1] | q[2] | q[3]); m6809_set_firq(&s_main, q[0]); break;
	case IRQ_TSHOOT: m6809_set_irq(&s_main, q[0] | q[1] | q[2] | q[3]); break;
	default: m6809_set_irq(&s_main, q[2] | q[3]); break;
	}
}
static void pia0_irqa(void *ctx, int st) { (void)ctx; S.irq_in[0] = (uint8_t)st; main_irq_update(); }
static void pia0_irqb(void *ctx, int st) { (void)ctx; S.irq_in[1] = (uint8_t)st; main_irq_update(); }
static void pia1_irqa(void *ctx, int st) { (void)ctx; S.irq_in[2] = (uint8_t)st; main_irq_update(); }
static void pia1_irqb(void *ctx, int st) { (void)ctx; S.irq_in[3] = (uint8_t)st; main_irq_update(); }

static void pia1_out_b(void *ctx, uint8_t d)
{
	(void)ctx;
	switch (s_c->snd) {
	case SND_STD: case SND_CVSD: snd_cmd_to(0, d | 0xc0); break;
	case SND_PLAYBALL: snd_cmd_to(0, d); break;
	case SND_COCKPIT: snd_cmd_to(0, d | 0xc0); snd_cmd_to(1, d | 0xc0); break;
	case SND_BLASTER: {
		const uint8_t l = d | 0x80, r = (uint8_t)((d >> 1 & 0x40) | (d & 0x3f) | 0x80);
		snd_cmd_to(0, l);
		snd_cmd_to(1, r);
		break;
	}
	case SND_W2: pia6821_porta_w(&s_pia[2], d); break;
	case SND_JOUST2:
		pia6821_portb_w(&s_bgpia, d);   // (bg data_w)
		pia6821_porta_w(&s_pia[2], d);
		break;
	}
}
static void pia1_ca2(void *ctx, int st) { (void)ctx; if (s_c->snd == SND_JOUST2) pia6821_cb1_w(&s_bgpia, st); }
static void pia1_cb2(void *ctx, int st) { (void)ctx; pia6821_ca1_w(&s_pia[2], st); }
// second generation sound PIA
static void w2snd_out_a(void *ctx, uint8_t d) { (void)ctx; pia6821_portb_w(&s_pia[1], d); }
static void w2snd_ca2(void *ctx, int st) { (void)ctx; pia6821_cb1_w(&s_pia[1], st); }
static void w2snd_cb2(void *ctx, int st) { (void)ctx; if (s_c->snd == SND_JOUST2) bg_resetq_w(st); }

static void pia_setup(void)
{
	for (int i = 0; i < 4; i++)
		pia6821_config(&s_pia[i]);
	pia6821_config(&s_bgpia);
	s_pia[0].in_a_cb = pia0_in_a;
	s_pia[0].in_b_cb = pia0_in_b;
	s_pia[0].out_b_cb = pia0_out_b;
	if (s_in == IN_MUXED || s_in == IN_BLASTKIT || s_in == IN_BLASTER)
		s_pia[0].cb2_cb = pia0_mux_w;
	if (s_in == IN_W2MUX || s_in == IN_TSHOOT || s_in == IN_JOUST2)
		s_pia[0].ca2_cb = pia0_mux_w;
	s_pia[1].in_a_cb = pia1_in_a;
	s_pia[1].out_b_cb = pia1_out_b;
	s_pia[1].irqa_cb = pia1_irqa;
	s_pia[1].irqb_cb = pia1_irqb;
	if (s_c->irq == IRQ_MYSTIC || s_c->irq == IRQ_TSHOOT) {
		s_pia[0].irqa_cb = pia0_irqa;
		s_pia[0].irqb_cb = pia0_irqb;
	}
	if (s_c->gen == 1) {
		const int nsnd = s_c->snd == SND_COCKPIT || s_c->snd == SND_BLASTER ? 2 : 1;
		for (int b = 0; b < nsnd; b++) {
			pia6821_t *p = snd_pia(b);
			p->ctx = (void *)(intptr_t)b;
			p->out_a_cb = snd_dac_w;
			p->irqa_cb = snd_irqa;
			p->irqb_cb = snd_irqb;
		}
		if (s_c->snd == SND_CVSD || s_c->snd == SND_COCKPIT || s_c->snd == SND_PLAYBALL) {
			s_pia[2].ca2_cb = snd_ca2;
			s_pia[2].cb2_cb = snd_cb2;
		}
		if (s_c->map == MAP_SPDBALL) {
			s_pia[3].in_a_cb = pia3_in_a;
			s_pia[3].in_b_cb = pia3_in_b;
		}
	} else {
		s_pia[1].ca2_cb = pia1_ca2;
		s_pia[1].cb2_cb = pia1_cb2;
		pia6821_t *p = &s_pia[2];
		p->ctx = (void *)(intptr_t)0;
		p->out_a_cb = w2snd_out_a;
		p->out_b_cb = snd_dac_w;
		p->ca2_cb = w2snd_ca2;
		p->cb2_cb = w2snd_cb2;
		p->irqa_cb = snd_irqa;
		p->irqb_cb = snd_irqb;
		if (s_in == IN_W2MUX)
			p->a_input_overrides_output_mask = 0xff;
		if (s_c->snd == SND_JOUST2) {
			s_bgpia.out_a_cb = bgpia_out_a;
			s_bgpia.ca2_cb = bgpia_ca2;
			s_bgpia.cb2_cb = bgpia_cb2;
			s_bgpia.irqa_cb = bgpia_irqa;
			s_bgpia.irqb_cb = bgpia_irqb;
		}
	}
}

// ---- the blitter (williamsblitter.cpp) ----
static uint8_t main_read(void *ctx, uint16_t a);
static void main_write(void *ctx, uint16_t a, uint8_t v);

static inline uint8_t bus_rd(uint16_t a)
{
	const uint8_t *p = s_main.rpage[a >> 8];
	return p ? p[a & 0xff] : main_read(NULL, a);
}
static inline void bus_wr(uint16_t a, uint8_t v)
{
	uint8_t *p = s_main.wpage[a >> 8];
	if (p)
		p[a & 0xff] = v;
	else
		main_write(NULL, a, v);
}

static int blit_core(int w, int h)
{
	const uint8_t ctl = S.control;
	const int dst256 = bit(ctl, 1), src256 = bit(ctl, 0);
	const int sxadv = src256 ? 0x100 : 1, syadv = src256 ? 1 : w;
	const int dxadv = dst256 ? 0x100 : 1, dyadv = dst256 ? 1 : w;
	const int no_even = bit(ctl, 7), no_odd = bit(ctl, 6), shift = bit(ctl, 5), solid = bit(ctl, 4), fg_only = bit(ctl, 3);
	const uint8_t solid_color = S.solid_color;
	const uint8_t *remap = s_remap_lookup ? s_remap_lookup + S.remap_index * 256 : NULL;
	const int win = S.window;
	const uint16_t clip = s_c->clip;
	// the kept part of the destination byte when the source's even / odd pixel is shown (D7-D4 / D3-D0) or not
	const uint8_t keep_on = (uint8_t)((no_even ? 0xf0 : 0) | (no_odd ? 0x0f : 0));
	uint16_t sstart = S.sstart, dstart = S.dstart;
	int pixdata = 0;
	for (int y = 0; y < h; y++) {
		uint16_t source = sstart, dest = dstart;
		for (int x = 0; x < w; x++) {
			uint8_t raw = bus_rd(source);
			if (remap)
				raw = remap[raw];
			int src;
			if (shift) {
				pixdata = (pixdata << 8) | raw;
				src = (pixdata >> 4) & 0xff;
			} else
				src = raw;
			uint8_t keep = keep_on;
			if (fg_only) {   // a transparent source pixel: kept unless NO_EVEN / NO_ODD (inverted sense)
				if (!(src & 0xf0)) keep ^= 0xf0;
				if (!(src & 0x0f)) keep ^= 0x0f;
			}
			// (always read from video RAM below 0xc000 regardless of the bank setting; written through the bus)
			uint8_t cur = dest < 0xc000 ? s_vram[dest] : bus_rd(dest);
			cur &= keep;
			cur |= (uint8_t)((solid ? solid_color : src) & ~keep);
			if (!win || dest < clip || dest >= 0xc000)
				bus_wr(dest, cur);
			source = (uint16_t)(source + sxadv);
			dest = (uint16_t)(dest + dxadv);
		}
		if (dst256)
			dstart = (uint16_t)((dstart & 0xff00) | ((dstart + dyadv) & 0xff));
		else
			dstart = (uint16_t)(dstart + dyadv);
		if (src256)
			sstart = (uint16_t)((sstart & 0xff00) | ((sstart + syadv) & 0xff));
		else
			sstart = (uint16_t)(sstart + syadv);
	}
	return 2 * w * h;
}

static void blitter_w(int offset, uint8_t data)
{
	switch (offset & 7) {
	case 0: {
		if (S.blitting)
			return;
		const unsigned pa = rbx_prof_enter(WM_PA_BLIT);
		S.control = data;
		const int xr = s_c->blit == 1 ? 4 : 0;
		int w = S.width ^ xr, h = S.height ^ xr;
		if (!w) w = 1;
		if (!h) h = 1;
		S.blitting = 1;
		const int accesses = blit_core(w, h);
		S.blitting = 0;
		int clocks = 4;
		if (bit(data, 2))
			clocks += 4 * (accesses + 2);
		else
			clocks += 2 * (accesses + 3);
		s_main.cycles += (clocks + 3) / 4;
		rbx_prof_leave(pa);
		break;
	}
	case 1: S.solid_color = data; break;
	case 2: S.sstart = (uint16_t)((S.sstart & 0x00ff) | (data << 8)); break;
	case 3: S.sstart = (uint16_t)((S.sstart & 0xff00) | data); break;
	case 4: S.dstart = (uint16_t)((S.dstart & 0x00ff) | (data << 8)); break;
	case 5: S.dstart = (uint16_t)((S.dstart & 0xff00) | data); break;
	case 6: S.width = data; break;
	case 7: S.height = data; break;
	}
}

// ---- the main CPU's memory ----
static void map_pages(void)
{
	m6809_t *c = &s_main;
	const int k = s_c->map;
	for (int p = 0; p < 0xc0; p++) {
		c->rpage[p] = s_vram + p * 256;
		c->wpage[p] = s_vram + p * 256;
	}
	for (int p = 0xc0; p < 0x100; p++) {
		c->rpage[p] = NULL;
		c->wpage[p] = NULL;
	}
	if (s_c->gen == 1) {
		if (k == MAP_DEF || k == MAP_MAYDAY) {
			const int v = S.defbank;
			if (v == 0) {
				for (int p = 0xc4; p < 0xc8; p++)
					c->rpage[p] = s_nvram;
			} else if (v <= 9 && (uint32_t)v * 0x1000 <= s_banked_size) {
				for (int p = 0xc0; p < 0xd0; p++)
					c->rpage[p] = s_banked + (v - 1) * 0x1000 + (p - 0xc0) * 256;
			}
			for (int p = 0xd0; p < 0x100; p++)
				c->rpage[p] = s_mrom + (p - 0xd0) * 256;
			if (k == MAP_MAYDAY)
				c->rpage[0xa1] = NULL;
			return;
		}
		if (S.rom_view) {
			if (k == MAP_BLASTER) {
				for (int p = 0; p < 0x40; p++)
					c->rpage[p] = s_mrom + 0x10000 + S.blaster_bank * 0x4000 + p * 256;
				for (int p = 0x40; p < 0x90; p++)
					c->rpage[p] = s_mrom + p * 256;
			} else
				for (int p = 0; p < 0x90; p++)
					c->rpage[p] = s_mrom + p * 256;
		}
		for (int p = 0xcc; p < 0xd0; p++) {
			c->rpage[p] = s_nvram + (p - 0xcc) * 256;
			if (k == MAP_BUBBLES)
				c->wpage[p] = s_nvram + (p - 0xcc) * 256;
		}
		if (k == MAP_SINISTAR) {
			for (int p = 0xd0; p < 0xe0; p++)
				c->rpage[p] = c->wpage[p] = s_hiram + (p - 0xd0) * 256;
			for (int p = 0xe0; p < 0x100; p++)
				c->rpage[p] = s_mrom + p * 256;
		} else
			for (int p = 0xd0; p < 0x100; p++)
				c->rpage[p] = s_mrom + p * 256;
		return;
	}
	// second generation
	if (S.rom_view)
		for (int p = 0; p < 0x80; p++)
			c->rpage[p] = s_mrom + 0x10000 + S.w2bank * 0x8000 + p * 256;
	if (S.palette_view)
		for (int p = 0x80; p < 0x88; p++) {
			c->rpage[p] = s_palram + (p - 0x80) * 256;
			c->wpage[p] = NULL;
		}
	for (int p = 0xc0; p < 0xc8; p++)
		c->rpage[p] = c->wpage[p] = s_tileram + (p - 0xc0) * 256;
	for (int p = 0xcc; p < 0xd0; p++)
		c->rpage[p] = s_nvram + (p - 0xcc) * 256;
	if (k == MAP_W2RAM) {
		for (int p = 0xd0; p < 0xe0; p++)
			c->rpage[p] = c->wpage[p] = s_hiram + (p - 0xd0) * 256;
		for (int p = 0xe0; p < 0x100; p++)
			c->rpage[p] = s_mrom + p * 256;
	} else
		for (int p = 0xd0; p < 0x100; p++)
			c->rpage[p] = s_mrom + p * 256;
}

static uint8_t video_counter(void)
{
	const int v = S.line;
	if (s_c->gen == 2)
		return (uint8_t)v;
	return v < 0x100 ? (uint8_t)(v & 0xfc) : 0xfc;
}

// the second generation palette: calc_col (frgb from MAME's netlist table)
static const float s_frgb[256] = {
	0.001889f, 0.002456f, 0.003196f, 0.004384f, 0.005272f, 0.007672f, 0.011345f, 0.017759f,
	0.018643f, 0.03103f,  0.051912f, 0.087251f, 0.1256f,   0.204967f, 0.331644f, 0.529628f,
	0.002524f, 0.003306f, 0.004371f, 0.006072f, 0.007386f, 0.010812f, 0.016096f, 0.02509f,
	0.02687f,  0.043747f, 0.070899f, 0.114042f, 0.159992f, 0.250108f, 0.391104f, 0.607433f,
	0.003522f, 0.004666f, 0.006254f, 0.008766f, 0.010795f, 0.015856f, 0.023585f, 0.036228f,
	0.039487f, 0.062206f, 0.096916f, 0.148811f, 0.20338f,  0.305447f, 0.462741f, 0.700199f,
	0.004941f, 0.006607f, 0.00895f,  0.012619f, 0.015683f, 0.022944f, 0.033832f, 0.050765f,
	0.056018f, 0.085033f, 0.127452f, 0.187886f, 0.25112f,  0.364998f, 0.53879f,  0.797876f,
	0.006857f, 0.009233f, 0.012592f, 0.017771f, 0.022172f, 0.03212f,  0.046615f, 0.06811f,
	0.075696f, 0.110906f, 0.160678f, 0.229149f, 0.300735f, 0.42593f,  0.61584f,  0.896241f,
	0.010113f, 0.01368f,  0.018726f, 0.02625f,  0.032735f, 0.046462f, 0.065809f, 0.092996f,
	0.103803f, 0.146292f, 0.204648f, 0.282479f, 0.364084f, 0.502781f, 0.712244f, 1.018759f,
	0.015744f, 0.021251f, 0.028944f, 0.039945f, 0.049446f, 0.068127f, 0.093445f, 0.127285f,
	0.14231f,  0.193004f, 0.261153f, 0.349656f, 0.443068f, 0.597642f, 0.830444f, 1.168267f,
	0.024223f, 0.032378f, 0.043467f, 0.058612f, 0.071646f, 0.09563f,  0.127147f, 0.167588f,
	0.187237f, 0.246001f, 0.32398f,  0.423282f, 0.528958f, 0.699986f, 0.957283f, 1.328146f,
	0.03429f,  0.045182f, 0.059626f, 0.078588f, 0.094882f, 0.123449f, 0.160233f, 0.206173f,
	0.230031f, 0.295596f, 0.382005f, 0.490617f, 0.607092f, 0.792615f, 1.071663f, 1.471949f,
	0.053106f, 0.068185f, 0.087712f, 0.112105f, 0.132967f, 0.167685f, 0.211656f, 0.265082f,
	0.295044f, 0.369837f, 0.467993f, 0.58972f,  0.721623f, 0.927771f, 1.23799f,  1.680648f,
	0.083576f, 0.103924f, 0.129726f, 0.160516f, 0.186838f, 0.228722f, 0.281144f, 0.343382f,
	0.381103f, 0.46708f,  0.5795f,   0.717377f, 0.86859f,  1.100548f, 1.449999f, 1.946101f,
	0.124032f, 0.149904f, 0.182177f, 0.219335f, 0.251408f, 0.300484f, 0.361692f, 0.433113f,
	0.479348f, 0.577076f, 0.704911f, 0.860274f, 1.032601f, 1.292736f, 1.685276f, 2.240223f,
	0.175706f, 0.207011f, 0.245689f, 0.289706f, 0.327912f, 0.384501f, 0.455038f, 0.536297f,
	0.592068f, 0.702442f, 0.847309f, 1.021943f, 1.217772f, 1.509243f, 1.949852f, 2.570528f,
	0.24927f,  0.287025f, 0.333877f, 0.385984f, 0.431717f, 0.497443f, 0.579727f, 0.673464f,
	0.741572f, 0.868017f, 1.034559f, 1.234123f, 1.460333f, 1.79207f,  2.295118f, 3.001228f,
	0.357357f, 0.403448f, 0.460931f, 0.52375f,  0.579557f, 0.656404f, 0.754715f, 0.865436f,
	0.950203f, 1.097887f, 1.293872f, 1.527752f, 1.795589f, 2.182008f, 2.770169f, 3.593798f,
	0.494255f, 0.549094f, 0.61884f,  0.693823f, 0.760527f, 0.851544f, 0.968276f, 1.098965f,
	1.203912f, 1.377026f, 1.608183f, 1.882244f, 2.200036f, 2.652345f, 3.342626f, 4.215708f
};
static float s_gain[3], s_offset[3];

static uint32_t calc_col(uint16_t lo, uint16_t hi)
{
	const int i = (hi >> 4) & 15, ub = hi & 15, ug = (lo >> 4) & 15, ur = lo & 15;
	float c[3] = { s_frgb[i * 16 + ur] / 4.22f, s_frgb[i * 16 + ug] / 4.22f, s_frgb[i * 16 + ub] / 4.22f };
	int v[3];
	for (int k = 0; k < 3; k++) {
		float x = c[k] + s_offset[k];
		if (x < 0) x = 0;
		x = x * s_gain[k] / 0.25f;
		if (x > 1) x = 1;
		v[k] = (int)(x * 255);
	}
	return (uint32_t)(v[0] << 16 | v[1] << 8 | v[2]);
}
static void w2_palette_w(int offset, uint8_t data)
{
	s_palram[offset] = data;
	s_pal_rgb[offset >> 1] = calc_col(s_palram[offset & ~1], s_palram[offset | 1]);
}

static void w2_bank_select(uint8_t data)
{
	switch (data & 3) {
	case 0: S.rom_view = 0; S.palette_view = 0; break;
	case 1: case 2: S.w2bank = (uint8_t)((data & 6) >> 1); S.rom_view = 1; S.palette_view = 0; break;
	case 3: S.w2bank = (uint8_t)((data & 4) >> 1); S.rom_view = 1; S.palette_view = 1; break;
	}
	map_pages();
}

static void watchdog_kick(void) { S.watchdog = 3000000; }

static uint8_t main_read(void *ctx, uint16_t a)
{
	(void)ctx;
	const int k = s_c->map;
	if (s_c->gen == 1) {
		if (k == MAP_DEF || k == MAP_MAYDAY) {
			if (a >= 0xa100 && a < 0xa200)   // (mayday: protection_r at 0xa190-0xa191)
				return (a == 0xa190 || a == 0xa191) ? s_vram[a + 3] : s_vram[a];
			if (a >= 0xc000 && a < 0xd000 && S.defbank == 0) {
				if (a >= 0xc800 && a < 0xcc00)
					return video_counter();
				if (a >= 0xcc00) {
					if ((a & 0x1c) == 0x00) return pia6821_read(&s_pia[1], a & 3);
					if ((a & 0x1c) == 0x04) return pia6821_read(&s_pia[0], a & 3);
				}
			}
			return 0;
		}
		if ((a & 0xff00) == 0xc800) {
			if (k == MAP_SPDBALL) {
				if (a < 0xc804)
					return S.an[a & 3];
				if ((a & 0x0c) == 0x08)
					return pia6821_read(&s_pia[3], a & 3);
			}
			if ((a & 0x0c) == 0x04) return pia6821_read(&s_pia[0], a & 3);
			if ((a & 0x0c) == 0x0c) return pia6821_read(&s_pia[1], a & 3);
			return 0;
		}
		if ((a & 0xff00) == 0xcb00)
			return video_counter();
		return 0;
	}
	// second generation
	if ((a & 0xff80) == 0xc980) {
		if ((a & 0x0c) == 0x00) return pia6821_read(&s_pia[1], a & 3);
		if ((a & 0x0c) == 0x04) return pia6821_read(&s_pia[0], a & 3);
		return 0;
	}
	if (a >= 0xcbe0 && a < 0xcbf0)
		return video_counter();
	return 0;
}

static void main_write(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	const int k = s_c->map;
	if (s_c->gen == 1) {
		if (k == MAP_DEF || k == MAP_MAYDAY) {
			if (a >= 0xd000) {
				if (a < 0xe000) {   // bank_select_w
					S.defbank = v & 0x0f;
					map_pages();
				}
				return;
			}
			if (S.defbank != 0)
				return;
			if (a < 0xc400) {
				if (a == 0xc3ff) {
					if (v == 0x39) watchdog_kick();
					return;
				}
				if (a & 0x10)
					S.cocktail = v & 1;   // video_control_w
				else
					s_palram[a & 0x0f] = v;
				return;
			}
			if (a < 0xc800) {
				s_nvram[a & 0xff] = v | 0xf0;
				return;
			}
			if (a >= 0xcc00) {
				if ((a & 0x1c) == 0x00) pia6821_write(&s_pia[1], a & 3, v);
				else if ((a & 0x1c) == 0x04) pia6821_write(&s_pia[0], a & 3, v);
			}
			return;
		}
		switch (a >> 8) {
		case 0xc0: case 0xc1: case 0xc2: case 0xc3:
			s_palram[a & 0x0f] = v;
			return;
		case 0xc8:
			if (k == MAP_SPDBALL && (a & 0x0c) == 0x08) { pia6821_write(&s_pia[3], a & 3, v); return; }
			if ((a & 0x0c) == 0x04) pia6821_write(&s_pia[0], a & 3, v);
			else if ((a & 0x0c) == 0x0c) pia6821_write(&s_pia[1], a & 3, v);
			return;
		case 0xc9:
			if (k == MAP_BLASTER) {
				switch (a & 0xc0) {
				case 0x00:   // blaster_vram_select_w
					S.rom_view = v & 1;
					S.cocktail = bit(v, 1);
					S.window = bit(v, 2);
					map_pages();
					break;
				case 0x40: S.remap_index = v; break;
				case 0x80: S.blaster_bank = v & 0x0f; map_pages(); break;
				case 0xc0: S.video_control = v; break;
				}
				return;
			}
			S.rom_view = v & 1;
			S.cocktail = bit(v, 1);
			if (k == MAP_SINISTAR)
				S.window = bit(v, 2);
			map_pages();
			return;
		case 0xca:
			if (s_c->blit)
				blitter_w(a & 7, v);
			return;
		case 0xcb:
			if (a == 0xcbff && v == 0x39 && k != MAP_ALIENAR)
				watchdog_kick();
			return;
		case 0xcc: case 0xcd: case 0xce: case 0xcf:
			s_nvram[a & 0x3ff] = v | 0xf0;   // cmos_4bit_w (Bubbles' pages are written directly)
			return;
		}
		return;
	}
	// second generation
	if (a >= 0x8000 && a < 0x8800) {   // the palette view
		w2_palette_w(a & 0x7ff, v);
		return;
	}
	if (a >= 0xcc00 && a < 0xd000) {
		s_nvram[a & 0x3ff] = v | 0xf0;
		return;
	}
	switch (a & 0xff80) {
	case 0xc800: w2_bank_select(v); return;
	case 0xc880: blitter_w(a & 7, v); return;
	case 0xc900:
		if ((v & 0x3f) == 0x14)
			watchdog_kick();
		return;
	case 0xc980:
		if ((a & 0x0c) == 0x00) pia6821_write(&s_pia[1], a & 3, v);
		else if ((a & 0x0c) == 0x04) pia6821_write(&s_pia[0], a & 3, v);
		return;   // (0xc98c: the diagnostic segments)
	case 0xcb00: case 0xcb80:
		switch (a & 0xe0) {
		case 0x00: S.fg_color = v & 0x3f; break;
		case 0x20: S.bg_select = v; break;
		case 0x40: S.xscroll = (uint16_t)((S.xscroll & ~0x00f) | ((v & 0x80) >> 4) | (v & 0x07)); break;
		case 0x60: S.xscroll = (uint16_t)((S.xscroll & 0x00f) | (v << 4)); break;
		case 0x80: S.cocktail = v & 1; break;
		case 0xa0: S.window = v & 1; break;
		}
		return;
	}
}

// ---- video ----
static void build_rgb256(void)
{
	// compute_resistor_weights(0, 255, -1.0, ...) of 1200 / 560 / 330 (red, green) and 560 / 330 (blue), no pull-downs
	static const int res_rg[3] = { 1200, 560, 330 }, res_b[2] = { 560, 330 };
	double wrg[3], wb[2], srg = 0, sb = 0;
	for (int n = 0; n < 2; n++) {
		const int cnt = n ? 2 : 3;
		const int *res = n ? res_b : res_rg;
		double *w = n ? wb : wrg;
		for (int k = 0; k < cnt; k++) {
			double r0 = 1.0 / 1e12, r1 = 1.0 / 1e12;
			for (int j = 0; j < cnt; j++) {
				if (j == k) r1 += 1.0 / res[j];
				else r0 += 1.0 / res[j];
			}
			r0 = 1.0 / r0;
			r1 = 1.0 / r1;
			w[k] = 255.0 * r0 / (r1 + r0);
			if (n) sb += w[k];
			else srg += w[k];
		}
	}
	const double scale = 255.0 / (srg > sb ? srg : sb);
	for (int i = 0; i < 256; i++) {
		const int r = (int)(scale * (wrg[0] * bit(i, 0) + wrg[1] * bit(i, 1) + wrg[2] * bit(i, 2)) + 0.5);
		const int g = (int)(scale * (wrg[0] * bit(i, 3) + wrg[1] * bit(i, 4) + wrg[2] * bit(i, 5)) + 0.5);
		const int b = (int)(scale * (wb[0] * bit(i, 6) + wb[1] * bit(i, 7)) + 0.5);
		s_rgb256[i] = (uint32_t)(r << 16 | g << 8 | b);
	}
}

// Mystic Marathon: IC79, a 74LS85 comparator, on the low color bit
static int mystic_color_decode(uint8_t base_col, int sig_j1, int y)
{
	const int v = y << 6;
	const int w11 = (v >> 11) & 1, w12 = (v >> 12) & 1, w13 = (v >> 13) & 1;
	const int a = 1 | ((base_col & 1) << 2) | ((base_col & 1) << 3);
	const int b = (w12 << 0) | (w13 << 1) | (sig_j1 << 3);
	const int color = (a > b) || ((a == b) && !w11);
	return (base_col & 0x3e) | color;
}

static void capture_line(int y)
{
	const int row = y - s_c->y0;
	snap_t *sn = s_snap[s_work];
	uint16_t *d = sn->pix + row * s_vis_w;
	const int c0 = s_c->x0 >> 1, nc = s_vis_w >> 1;
	const uint8_t *src = s_vram + c0 * 256 + y;
	if (s_c->gen == 1) {
		uint16_t pens[16];
		for (int i = 0; i < 16; i++)
			pens[i] = s_palram[i];
		if (s_c->map == MAP_BLASTER) {
			uint8_t *vs = s_vram + c0 * 256 + y;
			const uint8_t *palette_0 = s_vram + 0xbb00, *scanline_control = s_vram + 0xbc00;
			if (y == s_c->y0 || !(S.video_control & 1))
				S.blaster_color0 = palette_0[0] ^ 0xff;
			const int erase = S.video_control & scanline_control[y] & 2;
			if (S.video_control & scanline_control[y] & 1)
				S.blaster_color0 = palette_0[y] ^ 0xff;
			sn->zero_rgb[row] = s_rgb256[S.blaster_color0] | s_rgb256[s_palram[0]];
			pens[0] = (uint16_t)(256 + row);
			for (int c = 0; c < nc; c++) {
				const uint8_t pix = vs[c * 256];
				if (erase)
					vs[c * 256] = 0;
				d[2 * c] = pens[pix >> 4];
				d[2 * c + 1] = pens[pix & 15];
			}
			return;
		}
		for (int c = 0; c < nc; c++) {
			const uint8_t pix = src[c * 256];
			d[2 * c] = pens[pix >> 4];
			d[2 * c + 1] = pens[pix & 15];
		}
		return;
	}
	for (int c = 0; c < nc; c++) {
		const uint8_t pix = src[c * 256];
		d[2 * c] = pix >> 4;
		d[2 * c + 1] = pix & 15;
	}
	sn->fgbase[row] = (uint16_t)((s_c->tiles == TL_MYSTIC ? mystic_color_decode(S.fg_color, 1, y) : S.fg_color) * 16);
	sn->scroll[row] = (uint16_t)((S.xscroll & 7) + ((S.xscroll >> 3) * 6));
	sn->bgsel[row] = S.bg_select;
}

static void present(void)
{
	rbx_video_wait_idle();
	snap_t *sn = s_snap[s_work];
	if (s_c->gen == 2) {
		memcpy(sn->tileram, s_tileram, sizeof(s_tileram));
		memcpy(sn->pal, s_pal_rgb, sizeof(s_pal_rgb));
	}
	sn->valid = 1;
	s_work ^= 1;
	rbx_video_frame_done();
}

static void wm_render(void)
{
	const snap_t *sn = s_snap[s_work ^ 1];
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch, W = s_vis_w, H = s_vis_h;
	if (!sn->valid) {
		for (int y = 0; y < H; y++)
			memset(bm + y * pitch, 0, W * sizeof(uint16_t));
		return;
	}
	if (s_c->gen == 1) {
		memcpy(rbx_video.palette, s_rgb256, sizeof(s_rgb256));
		if (s_c->map == MAP_BLASTER)
			for (int y = 0; y < H; y++)
				rbx_video.palette[256 + y] = sn->zero_rgb[y];
		for (int y = 0; y < H; y++)
			memcpy(bm + y * pitch, sn->pix + y * W, W * sizeof(uint16_t));
		return;
	}
	memcpy(rbx_video.palette, sn->pal, sizeof(sn->pal));
	const int mask = s_tile_count - 1;
	for (int y = 0; y < H; y++) {
		const int sy = s_c->y0 + y;
		const int trow = sy >> 4, py = sy & 15;
		uint16_t *d = bm + y * pitch;
		const uint16_t *fg = sn->pix + y * W;
		const uint16_t fgb = sn->fgbase[y];
		// the tile layer: source x = screen x + scroll - 2 (MAME's scrolldx 2), 128 tiles of 24 wide
		int tx = s_c->x0 + sn->scroll[y] - 2;
		while (tx < 0) tx += 3072;
		while (tx >= 3072) tx -= 3072;
		int col = 0, sub = tx;
		while (sub >= 24 * 16) { sub -= 24 * 16; col += 16; }
		while (sub >= 24) { sub -= 24; col++; }
		int base;
		const uint8_t bgsel = sn->bgsel[y];
		switch (s_c->tiles) {
		case TL_MYSTIC: base = mystic_color_decode(bgsel, 0, (trow * 16) & 0xff) * 16; break;
		case TL_JOUST2: base = (bgsel & 0x3f) << 4; break;
		default: base = ((bgsel & 0x38) << 4) + ((trow >> 1) & 7) * 16; break;
		}
		int x = 0;
		while (x < W) {
			const int data = sn->tileram[col * 16 + trow];
			const uint8_t *t = s_tiles + (data & mask) * 384 + py * 24;
			const int flip = (data & ~mask) != 0;
			int n = 24 - sub;
			if (n > W - x) n = W - x;
			for (int i = 0; i < n; i++, x++) {
				const uint16_t f = fg[x];
				d[x] = f ? (uint16_t)(fgb + f) : (uint16_t)(base + t[flip ? 23 - (sub + i) : sub + i]);
			}
			sub = 0;
			if (++col == 128) col = 0;
		}
	}
}

// ---- inputs, a frame at a time ----
static int stick49(int v, int dpad_neg, int dpad_pos)
{
	// a 7-position axis: the stick's deflection (counts a frame, up to ~9) in thirds, else the d-pad at full
	int off;
	if (v) {
		const int m = v < 0 ? -v : v;
		off = m >= 7 ? 3 : m >= 4 ? 2 : 1;
		if (v < 0) off = -off;
	} else
		off = dpad_pos ? 3 : dpad_neg ? -3 : 0;
	return 3 + off;
}

static void inputs_frame(void)
{
	const uint32_t in = rbx_player_input(1);
	if (s_in == IN_49WAY || s_in == IN_BLASTKIT || s_in == IN_BLASTER) {
		// 49WAYX: right = larger; 49WAYY (PORT_REVERSE): up = larger
		const int ax = rbx_analog(1, RBX_IPT_AD_STICK_X), ay = rbx_analog(1, RBX_IPT_AD_STICK_Y);
		S.x49 = (int16_t)stick49(ax, (in & RBX_IN_LEFT) != 0, (in & RBX_IN_RIGHT) != 0);
		S.y49 = (int16_t)stick49(-ay, (in & RBX_IN_DOWN) != 0, (in & RBX_IN_UP) != 0);
	}
	if (s_in == IN_CONQUEST)
		S.dial = (uint8_t)((S.dial - rbx_analog_dpad(1, RBX_IPT_DIAL, 2)) & 0x0f);
	if (s_in == IN_SPDBALL)
		for (int p = 0; p < 2; p++) {
			S.an[p * 2] = (uint8_t)(S.an[p * 2] - rbx_analog_dpad(p + 1, RBX_IPT_TRACKBALL_Y, 8));
			S.an[p * 2 + 1] = (uint8_t)(S.an[p * 2 + 1] + rbx_analog_dpad(p + 1, RBX_IPT_TRACKBALL_X, 8));
		}
	if (s_in == IN_TSHOOT) {
		// the gun as a crosshair the stick / d-pad moves: GUNX (INP1) from the Y axis, GUNY (INP2) from the X axis
		int gy = S.gun[0] + rbx_analog_dpad(1, RBX_IPT_AD_STICK_Y, 1);
		int gx = S.gun[1] + rbx_analog_dpad(1, RBX_IPT_AD_STICK_X, 1);
		S.gun[0] = (uint8_t)(gy < 0 ? 0 : gy > 0x3f ? 0x3f : gy);
		S.gun[1] = (uint8_t)(gx < 0 ? 0 : gx > 0x3f ? 0x3f : gx);
	}
}

// ---- init / reset / frame ----
static const wcfg_t *find_cfg(const char *machine, int *defndjeu)
{
	char name[64];
	snprintf(name, sizeof(name), "%s", machine);
	char *colon = strchr(name, ':');
	if (defndjeu)
		*defndjeu = colon && !strcmp(colon + 1, "defndjeu");
	if (colon)
		*colon = 0;
	for (size_t i = 0; i < sizeof(s_cfgs) / sizeof(s_cfgs[0]); i++)
		if (!strcmp(s_cfgs[i].name, name))
			return &s_cfgs[i];
	return NULL;
}

static void cpu_setup(void)
{
	m6809_init(&s_main);
	s_main.read = main_read;
	s_main.write = main_write;
	map_pages();
	const int nsnd = s_c->snd == SND_COCKPIT || s_c->snd == SND_BLASTER ? 2 : 1;
	for (int b = 0; b < 2; b++) {
		m6800_t *c = &s_snd[b];
		m6800_init(c);
		c->read = snd_read;
		c->write = snd_write;
		c->ctx = (void *)(intptr_t)b;
		if (b >= nsnd)
			continue;
		c->rpage[0] = c->wpage[0] = s_sram[b];
		const uint8_t *rom = b ? s_srom2 : s_srom;
		for (int p = s_c->gen == 1 ? 0xb0 : 0xe0; p < 0x100; p++)
			c->rpage[p] = rom + p * 256;
	}
	if (s_c->snd == SND_JOUST2) {
		m6809_init(&s_bg);
		s_bg.read = bg_read;
		s_bg.write = bg_write;
		for (int p = 0; p < 0x20; p++)
			s_bg.rpage[p] = s_bg.wpage[p] = s_bgram + (p & 7) * 256;
		bg_map_bank(s_bgbank);
	}
}

// the watchdog's soft reset (MAME schedule_soft_reset): every device reset, the memory kept
static void soft_reset(void)
{
	S.rom_view = S.defbank = S.blaster_bank = S.palette_view = S.w2bank = 0;
	S.control = S.solid_color = S.width = S.height = S.remap_index = 0;
	S.sstart = S.dstart = 0;
	S.watchdog = 0;
	for (int i = 0; i < 4; i++)
		pia6821_reset(&s_pia[i]);
	if (s_c->gen == 2)
		w2_bank_select(0);
	map_pages();
	reset6809_keep(&s_main);
	reset6800_keep(&s_snd[0]);
	reset6800_keep(&s_snd[1]);
	if (s_c->snd == SND_JOUST2) {
		chan_run(1, s_bg.cycles);
		pia6821_reset(&s_bgpia);
		rbx_ym_reset(0);
		S.bg_resetq = 1;
		bg_common_reset();
	}
}

static void wm_reset(void)
{
	rbx_video_wait_idle();
	memset(&S, 0, sizeof(S));
	memset(s_vram, 0, 0xc000);
	memset(s_palram, 0, sizeof(s_palram));
	memset(s_tileram, 0, sizeof(s_tileram));
	memset(s_sram, 0, sizeof(s_sram));
	memset(s_hiram, 0, sizeof(s_hiram));
	memset(s_bgram, 0, sizeof(s_bgram));
	for (int i = 0; i < 1024; i++)
		s_pal_rgb[i] = calc_col(0, 0);
	S.x49 = S.y49 = 3;
	S.gun[0] = S.gun[1] = 0x20;
	S.bg_resetq = 1;
	s_bgbank = 0;
	for (int i = 0; i < NCH; i++)
		S.ch[i].next = s_P16[i];
	hc55516_reset(&s_cvsd);
	cpu_setup();
	for (int i = 0; i < 4; i++)
		pia6821_reset(&s_pia[i]);
	pia6821_reset(&s_bgpia);
	if (s_c->gen == 2)
		w2_bank_select(0);
	map_pages();
	m6809_reset(&s_main);
	m6800_reset(&s_snd[0]);
	m6800_reset(&s_snd[1]);
	if (s_c->snd == SND_JOUST2) {
		rbx_ym_reset(0);
		bg_common_reset();
	}
	S.ch[0].level = ((int32_t)S.dac[0] - 128) * s_dac_gain;
	for (int i = 0; i < 2; i++)
		s_snap[i]->valid = 0;
}

static void wm_exit(void);

// the D-11298's CVSD filters (opamp_mfb_lowpass_setup, recalc at the output rate)
static void biquad_setup(double r1, double r2, double r3, double c1, double c2, float *b, float *a)
{
	const double pi = 3.14159265358979323846;
	const double gain = -r3 / r1;
	double fc, q = sqrt(2.0) / 2.0;
	const int one_pole = c1 == 0;
	if (one_pole)
		fc = (r1 * r3) / (2 * pi * ((r1 * r2) + (r1 * r3) + (r2 * r3)) * r3 * c2);
	else {
		fc = 1.0 / (2 * pi * sqrt(r2 * r3 * c1 * c2));
		q = sqrt(r2 * r3 * c1 * c2) / ((r3 * c2) + (r2 * c2) + ((r2 * c2) * -gain));
	}
	double b0, b1, b2, a1, a2;
	const double sr = (double)s_rate;
	if (fc >= sr / 2) {
		b0 = 1; b1 = b2 = a1 = a2 = 0;
	} else if (one_pole) {
		a1 = exp(-2.0 * pi * (fc / sr));
		b0 = 1.0 - a1;
		a1 = -a1;
		b1 = b2 = a2 = 0;
	} else {
		const double K = tan(pi * fc / sr), K2 = K * K, KoQ = K / q, normal = 1.0 / (1.0 + KoQ + K2);
		b0 = K2 * normal;
		b1 = 2.0 * b0;
		b2 = b0;
		a1 = 2.0 * (K2 - 1.0) * normal;
		a2 = (1.0 - KoQ + K2) * normal;
	}
	b[0] = (float)(b0 * gain);
	b[1] = (float)(b1 * gain);
	b[2] = (float)(b2 * gain);
	a[0] = 1;
	a[1] = (float)a1;
	a[2] = (float)a2;
}

static int wm_init(const rbx_game *g)
{
	s_c = find_cfg(g->machine, &s_defndjeu);
	if (!s_c) {
		rbx_fail("%s: machine %s is not emulated", g->set, g->machine);
		return -1;
	}
	uint32_t ssize = 0, ssize2 = 0;
	s_mrom = rbx_region("maincpu", &s_mrom_size);
	s_srom = rbx_region("soundcpu", &ssize);
	s_banked = s_srom2 = s_bgrom = s_tiles_rom = s_proms = NULL;
	s_banked_size = s_bgrom_size = s_tiles_size = s_proms_size = 0;
	if (s_c->map == MAP_DEF || s_c->map == MAP_MAYDAY)
		s_banked = rbx_region("banked", &s_banked_size);
	if (s_c->snd == SND_COCKPIT || s_c->snd == SND_BLASTER)
		s_srom2 = rbx_region("soundcpu_b", &ssize2);
	if (s_c->snd == SND_JOUST2)
		s_bgrom = rbx_region("bg:cpu", &s_bgrom_size);
	if (s_c->gen == 2)
		s_tiles_rom = rbx_region("tiles", &s_tiles_size);
	if (s_c->proms)
		s_proms = rbx_region("proms", &s_proms_size);
	const uint32_t need_main = s_c->map == MAP_DEF || s_c->map == MAP_MAYDAY ? 0x3000 :
	                           s_c->map == MAP_BLASTER ? 0x50000 : s_c->gen == 2 ? 0x30000 : 0x10000;
	if (!s_mrom || s_mrom_size < need_main || !s_srom || ssize < 0x10000 ||
	    ((s_c->map == MAP_DEF || s_c->map == MAP_MAYDAY) && !s_banked) ||
	    ((s_c->snd == SND_COCKPIT || s_c->snd == SND_BLASTER) && (!s_srom2 || ssize2 < 0x10000)) ||
	    (s_c->snd == SND_JOUST2 && !s_bgrom) || (s_c->gen == 2 && (!s_tiles_rom || s_tiles_size < 0x6000)) ||
	    (s_c->proms && (!s_proms || s_proms_size < 0x800))) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	if (s_defndjeu) {   // init_defndjeu: bits 0 and 7 swapped
		for (uint32_t i = 0; i < s_mrom_size; i++) {
			const uint8_t v = s_mrom[i];
			s_mrom[i] = (uint8_t)((v & 0x7e) | (v >> 7) | ((v & 1) << 7));
		}
		for (uint32_t i = 0; i < s_banked_size; i++) {
			const uint8_t v = s_banked[i];
			s_banked[i] = (uint8_t)((v & 0x7e) | (v >> 7) | ((v & 1) << 7));
		}
	}
	s_vram = (uint8_t *)calloc(1, 0xc000);
	s_snap[0] = (snap_t *)calloc(1, sizeof(snap_t));
	s_snap[1] = (snap_t *)calloc(1, sizeof(snap_t));
	if (!s_vram || !s_snap[0] || !s_snap[1]) {
		rbx_fail("%s: out of memory", g->set);
		wm_exit();
		return -1;
	}
	s_work = 0;
	if (s_c->proms) {   // the remap table (device_start)
		s_remap_lookup = (uint8_t *)malloc(256 * 256);
		if (!s_remap_lookup) {
			rbx_fail("%s: out of memory", g->set);
			wm_exit();
			return -1;
		}
		for (int i = 0; i < 256; i++) {
			const uint8_t *t = s_proms + (i & 0x7f) * 16;
			for (int j = 0; j < 256; j++)
				s_remap_lookup[i * 256 + j] = (uint8_t)((t[j >> 4] << 4) | t[j & 0x0f]);
		}
	}
	if (s_c->gen == 2) {   // williams2_layout: 24 x 16, 4 bpp, the three thirds of the region side by side
		const uint32_t third = s_tiles_size / 3;
		s_tile_count = (int)(third / 64);
		s_tiles = (uint8_t *)malloc((size_t)s_tile_count * 384);
		if (!s_tiles) {
			rbx_fail("%s: out of memory", g->set);
			wm_exit();
			return -1;
		}
		for (int t = 0; t < s_tile_count; t++)
			for (int y = 0; y < 16; y++)
				for (int x = 0; x < 24; x++) {
					const uint8_t b = s_tiles_rom[((x % 6) >> 1) * third + t * 64 + y * 4 + x / 6];
					s_tiles[t * 384 + y * 24 + x] = (x & 1) ? (b & 15) : (b >> 4);
				}
		if (s_c->tiles == TL_MYSTIC) {
			s_gain[0] = 0.8f; s_gain[1] = 0.73f; s_gain[2] = 0.81f;
			s_offset[0] = -0.27f; s_offset[1] = 0.0f; s_offset[2] = -0.22f;
		} else {
			s_gain[0] = s_gain[1] = s_gain[2] = 0.25f;
			s_offset[0] = s_offset[1] = s_offset[2] = 0.0f;
		}
	} else
		build_rgb256();
	for (int i = 0; i < P_COUNT; i++)
		s_port[i] = rbx_port_find(s_port_tags[i]);
	s_in = s_c->in;
	if (s_port[P_DIAL] >= 0)
		s_in = IN_CONQUEST;
	// video
	s_vis_w = s_c->x1 - s_c->x0 + 1;
	s_vis_h = s_c->y1 - s_c->y0 + 1;
	const int npens = s_c->gen == 2 ? 1024 : 512;
	s_frame_mhz = (uint32_t)(8000000000.0 / (512.0 * s_c->vtotal) + 0.5);
	// sound: the channels' sample grid (the same period in real time for all)
	// MAME's levels x 2 on the DAC-only boards; as they are with a CVSD or the Joust 2 background board
	const int boost = (s_c->snd == SND_CVSD || s_c->snd == SND_COCKPIT || s_c->snd == SND_PLAYBALL ||
	                   s_c->snd == SND_JOUST2) ? 1 : 2;
	s_stereo = s_c->snd == SND_BLASTER;
	s_nch = (s_c->snd == SND_COCKPIT || s_c->snd == SND_BLASTER || s_c->snd == SND_JOUST2) ? 2 : 1;
	if (s_c->gen == 1) {
		const double clk = s_c->snd_clock == SND1 ? 3579545.0 / 4 : (double)s_c->snd_clock;
		const int per = s_c->snd_clock == SND1 ? 18 : 16;
		s_P16[0] = s_P16[1] = (int64_t)per << 16;
		s_rate = (uint32_t)(clk / per + 0.5);
		s_snd_step = (uint64_t)(clk * LINE_CLOCKS / 1000000.0 * 4294967296.0 + 0.5);
		s_dac_gain = (int32_t)(128.5 * 0.25 * boost);
		s_cvsd_gain = 0.8f * boost;
	} else {
		s_snd_step = (uint64_t)LINE_CLOCKS << 32;
		s_dac_gain = (int32_t)(128.5 * 0.5 * boost);
		if (s_c->snd == SND_JOUST2) {
			const double per = 1000000.0 * 64 / 3579545.0;
			s_P16[0] = (int64_t)(per * 65536 + 0.5);
			s_P16[1] = 2 * s_P16[0];
			s_rate = (uint32_t)(1000000.0 * 65536 / (double)s_P16[0] + 0.5);
			s_bgdac_gain = (int32_t)(128.5 * 0.1170 * 2.0 * boost);
			s_bgcvsd_gain = (float)(0.1170 * 4.0 * 2.0 * boost);
			rbx_ym_config yc;
			memset(&yc, 0, sizeof(yc));
			yc.type = RBX_YM2151;
			yc.clock = 3579545;
			yc.gain[0] = yc.gain[1] = (int)(0.1170 * 2.0 * boost * 256 + 0.5);
			yc.irq = bg_ym_irq;
			if (rbx_ym_init(0, &yc)) {
				rbx_fail("%s: YM2151 init failed", g->set);
				wm_exit();
				return -1;
			}
		} else {
			s_P16[0] = s_P16[1] = (int64_t)20 << 16;
			s_rate = 50000;
		}
	}
	for (int i = 0; i < NCH; i++)
		s_inv[i] = (int64_t)(4294967296.0 / (double)s_P16[i] + 0.5);
	if (s_c->snd == SND_JOUST2) {
		biquad_setup(43000, 36000, 180000, 0, 180e-12, s_b1, s_a1);
		biquad_setup(27000, 15000, 27000, 4700e-12, 1200e-12, s_b2, s_a2);
	}
	// NVRAM (MAME default all 0)
	s_nv_size = (s_c->map == MAP_DEF || s_c->map == MAP_MAYDAY) ? 0x100 : 0x400;
	memset(s_nvram, 0, sizeof(s_nvram));
	s_nv_path[0] = 0;
	extern const char *rbx_nvram_dir(void);
	const char *dir = rbx_nvram_dir();
	if (dir) {
		snprintf(s_nv_path, sizeof(s_nv_path), "%s%s.nv", dir, g->set);
		FILE *f = fopen(s_nv_path, "rb");
		if (f) {
			if (fread(s_nvram, 1, s_nv_size, f) != s_nv_size)
				memset(s_nvram, 0, sizeof(s_nvram));
			fclose(f);
		}
	}
	{
		int blank = 1;
		for (uint32_t i = 0; i < s_nv_size; i++)
			blank &= s_nvram[i] == 0;
		s_setup = blank && s_in != IN_CONQUEST;
		s_setup_adv = s_setup_cool = s_setup_polling = 0;
		s_in2_reads = 0;
	}
	if ((s_stereo ? rbx_audio_setup_stereo(s_rate) : rbx_audio_setup(s_rate)) ||
	    rbx_video_setup(s_vis_w, s_vis_h, npens)) {
		rbx_fail("%s: out of memory", g->set);
		wm_exit();
		return -1;
	}
	rbx_prof_name(RBX_PA_DRV, s_pa_names, WM_PA_END - RBX_PA_DRV);
	pia_setup();
	wm_reset();
	return 0;
}

static void wm_exit(void)
{
	rbx_video_wait_idle();
	if (s_nv_path[0]) {
		rbx_file_put_now(s_nv_path, s_nvram, s_nv_size);
		s_nv_path[0] = 0;
	}
	if (s_c && s_c->snd == SND_JOUST2)
		rbx_ym_exit(0);
	free(s_vram);
	free(s_snap[0]);
	free(s_snap[1]);
	free(s_remap_lookup);
	free(s_tiles);
	s_vram = NULL;
	s_snap[0] = s_snap[1] = NULL;
	s_remap_lookup = NULL;
	s_tiles = NULL;
	s_c = NULL;
}

static void line_start(int line)
{
	S.line = (uint16_t)line;
	if (s_c->gen == 1) {
		if ((line & 31) == 0 && line != 256)
			pia6821_cb1_w(&s_pia[1], bit(line, 5));   // VA11
		if (line == 0 || line == 240)
			pia6821_ca1_w(&s_pia[1], line >= 240);    // COUNT240
	} else {
		if ((line & 31) == 0 && line != 256) {
			pia6821_cb1_w(&s_pia[0], bit(line, 5));
			pia6821_ca1_w(&s_pia[1], bit(line, 5));
		}
		if (line == 8 || line == 254)
			pia6821_ca1_w(&s_pia[0], line >= 254 ? 0 : 1);   // /ENDSCREEN
	}
	if (S.watchdog > 0) {
		S.watchdog -= LINE_CLOCKS;
		if (S.watchdog <= 0)
			soft_reset();
	}
	if (S.ticket_power) {
		S.ticket_timer -= LINE_CLOCKS;
		if (S.ticket_timer <= 0) {
			S.ticket_status ^= 1;
			S.ticket_timer += 70000;
		}
	}
}

static void wm_frame(void)
{
	rbx_audio.frame_mhz = s_frame_mhz;
	inputs_frame();
	setup_frame();
	const int nsnd = s_c->snd == SND_COCKPIT || s_c->snd == SND_BLASTER ? 2 : 1;
	for (int line = 0; line < s_c->vtotal; line++) {
		line_start(line);
		rbx_prof_set(WM_PA_MAIN);
		S.main_tgt += LINE_CLOCKS;
		m6809_run_until(&s_main, S.main_tgt);
		rbx_prof_set(WM_PA_SOUND);
		for (int b = 0; b < nsnd; b++) {
			S.snd_tgt[b] += s_snd_step;
			const int64_t t = (int64_t)(S.snd_tgt[b] >> 32);
			m6800_run_until(&s_snd[b], t);
			chan_run(b, t);
		}
		if (s_c->snd == SND_JOUST2) {
			rbx_prof_set(WM_PA_BG);
			S.bg_tgt += 2 * LINE_CLOCKS;
			m6809_run_until(&s_bg, S.bg_tgt);
			chan_run(1, S.bg_tgt);
		}
		mix_line();
		if (line >= s_c->y0 && line <= s_c->y1) {
			rbx_prof_set(WM_PA_LINE);
			capture_line(line);
			if (line == s_c->y1)
				present();
		}
	}
	rbx_prof_set(RBX_PA_DRIVER);
}

// ---- save states ----
static int wm_state(void *ctx, int load)
{
	int rc = 0;
	rc |= rbx_state_io(ctx, "wm", &S, sizeof(S));
	rc |= rbx_state_io(ctx, "wm_main", &s_main, M6809_STATE_SIZE);
	rc |= rbx_state_io(ctx, "wm_snd0", &s_snd[0], M6800_STATE_SIZE);
	rc |= rbx_state_io(ctx, "wm_snd1", &s_snd[1], M6800_STATE_SIZE);
	char tag[12];
	for (int i = 0; i < 4; i++) {
		snprintf(tag, sizeof(tag), "wm_pia%d", i);
		rc |= rbx_state_io(ctx, tag, &s_pia[i], PIA6821_STATE_SIZE);
	}
	rc |= rbx_state_io(ctx, "wm_cvsd", &s_cvsd, sizeof(s_cvsd));
	rc |= rbx_state_io(ctx, "wm_vram", s_vram, 0xc000);
	rc |= rbx_state_io(ctx, "wm_nv", s_nvram, sizeof(s_nvram));
	rc |= rbx_state_io(ctx, "wm_pal", s_palram, sizeof(s_palram));
	rc |= rbx_state_io(ctx, "wm_sram", s_sram, sizeof(s_sram));
	rc |= rbx_state_io(ctx, "wm_hiram", s_hiram, sizeof(s_hiram));
	if (s_c->gen == 2)
		rc |= rbx_state_io(ctx, "wm_tile", s_tileram, sizeof(s_tileram));
	if (s_c->snd == SND_JOUST2) {
		rc |= rbx_state_io(ctx, "wm_bg", &s_bg, M6809_STATE_SIZE);
		rc |= rbx_state_io(ctx, "wm_bgpia", &s_bgpia, PIA6821_STATE_SIZE);
		rc |= rbx_state_io(ctx, "wm_bgram", s_bgram, sizeof(s_bgram));
		rc |= rbx_state_io(ctx, "wm_bgbank", &s_bgbank, 1);
		rc |= rbx_ym_state(0, ctx, load);
	}
	if (rc)
		return -1;
	if (load) {
		s_setup = s_setup_adv = 0;   // (a loaded game is set up)
		map_pages();
		if (s_c->snd == SND_JOUST2)
			bg_map_bank(s_bgbank);
		if (s_c->gen == 2)
			for (int i = 0; i < 1024; i++)
				s_pal_rgb[i] = calc_col(s_palram[2 * i], s_palram[2 * i + 1]);
	}
	return 0;
}

static int wm_save(void *ctx) { return wm_state(ctx, 0); }
static int wm_load(void *ctx) { return wm_state(ctx, 1); }
static int wm_supported(const char *machine) { return find_cfg(machine, NULL) != NULL; }

const rbx_driver rbx_drv_williams = { "Williams 6809", wm_init, wm_exit, wm_reset, wm_frame, wm_save, wm_load,
                                      wm_supported, wm_render, NULL };
