/*  Jaleco Mega System 32: an NEC V70, a Z80 with the YMF271, the MS32 sprite chip (zoomed 256x256 textures), a ROZ
    layer (whole-screen or per-line "super" mode), a 16x16 background (two layouts), an 8x8 text layer and the
    priority RAM mixer. Ported from MAME src/mame/jaleco/ms32.cpp, ms32.h, ms32_v.cpp (BSD-3-Clause,
    copyright-holders: David Haywood, Paul Priest, Luca Elia), ms32_sprite.cpp (BSD-3-Clause, copyright-holders:
    Luca Elia, David Haywood, Paul Priest), jaleco_ms32_sysctrl.cpp (BSD-3-Clause, copyright-holders: Angelo Salese)
    and jalcrpt.cpp (BSD-3-Clause, copyright-holders: Luca Elia, David Haywood).
    The V70 is src/core/cpu/v60 with its 26-bit bus (v60_bus_set_bits: the 64 MB image MAME's map mirrors through the
    last GB), the YMF271 src/core/sound/ymf271.
    Not run: f1superb (MAME: not working, its road layer's maths DSP and the comms are not emulated).
    The mahjong games read the KEY0-4 matrix as MAME does (mahjong_ctrl_r); the frontend has no mahjong keys yet.
    The frame is drawn as MAME's screen_update draws it, at the vblank, from a copy of the video RAM, the registers and
    the sprite RAM buffered at the vblank before (MAME's sprite buffer), on the render side in 8 bands of lines
    (decoband: the render thread and its helpers), written straight into the frontend's frame.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/v60/v60.h"
#include "../../cpu/z80/z80.h"
#include "../../sound/ymf271.h"
#include "../../video/decoband.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// ---- time: ticks of 240 MHz (the V70's 20 MHz = 12, the Z80's 8 MHz = 30, the 6 MHz dot clock = 40, 8 MHz = 30)
#define T_V70 12u
#define T_Z80 30u
#define TICKS_HZ 240000000u
#define YMF_CLOCK 16934400u
#define SPR_WORDS 0x8000             // (the object RAM: 0x10000 bytes)
#define PENS 0x10000                 // (0x8000 colours, then the same halved: the mixer's shadow)
#define PEN_BLACK 0x3fff             // (an unused colour, kept black: the mixer's "unhandled" pixels)
#define VIS_W 320
#define VIS_H 224

enum { MS_PA_V70 = RBX_PA_DRV, MS_PA_Z80, MS_PA_SND, MS_PA_CAPT, MS_PA_END };
static const char *const s_pa_names[] = { "v70", "z80", "ymf271", "capture" };

// ---- the board's state (saved whole) ----
static struct {
	uint64_t vt, zt;                 // the V70's and the Z80's time (ticks)
	uint64_t frame_t;                // the frame's start
	uint64_t prg_at;                 // the programmable timer's next tick (0: off)
	uint64_t ymf_at[2];              // the YMF271's timers (0: off)
	uint64_t snd_acc;                // the next sample's time x 44100 (ticks x 44100)
	uint32_t frame;                  // MAME's frame_number
	uint16_t irqreq;
	uint8_t irq_line;
	uint8_t latch, latch_pending, to_main;
	uint8_t zbank[2];
	uint8_t z_nmi_line;
	// sysctrl
	uint8_t dotclock, flip, timer_en;
	uint16_t hblank, hdisp, vblank, vdisp, interval;
	// video registers (32-bit RAM at fce00xxx)
	uint32_t sprite_ctrl[0x80 / 4];
	uint32_t roz_ctrl[0x60 / 4];
	uint32_t tx_scroll[0x18 / 4];
	uint32_t bg_scroll[0x18 / 4];
	uint32_t brt[4];
	int32_t brt_r, brt_g, brt_b;
	uint32_t layout;                 // (fce00a7c: m_tilemaplayoutcontrol)
	uint32_t mj_select;
} s_st;

// ---- memory ----
static uint8_t *s_rom;
static uint32_t s_rom_size;
static uint8_t *s_ram;               // work RAM 128 KB (V70 order)
static uint8_t s_nvram[0x2000];      // (not battery backed in MAME: no nvram device)
static uint8_t *s_priram;            // 0x2000
static uint16_t *s_palram;           // 0x10000 words
static uint16_t *s_rozram;           // 0x8000
static uint16_t *s_lineram;          // 0x800
static uint16_t *s_sprram;           // 0x8000
static uint16_t *s_txram;            // 0x2000
static uint16_t *s_bgram;            // 0x2000
static uint16_t *s_sprbuf;           // MAME's m_sprram_buffer (copied at the vblank)
static uint32_t *s_pens;             // the colours (0x8000, RGB888) as update_color makes them

// ---- the gfx ----
static uint8_t **s_spr_blk;          // the sprite ROM as 8 KB blocks (ADR-222); 256x256 textures in 8x8 tiles
static uint32_t s_spr_count;         // textures (64 KB each)
static const uint8_t *s_roz_gfx, *s_bg_gfx, *s_tx_gfx;
static uint32_t s_roz_n, s_bg_n, s_tx_n;   // tiles
static uint32_t s_roz_m, s_bg_m, s_tx_m;   // (count - 1 when a power of two, else 0)

// ---- the sound ----
static z80 s_z;
static const uint8_t *s_zmap[256];
static uint8_t *s_zrom;
static uint32_t s_zrom_size;
static uint8_t s_zram[0x4000];
static ymf271_t *s_ymf;
static int32_t *s_mix;
static int s_mix_n, s_mix_cap;
static int s_zline;                  // (the Z80's IRQ: not connected)

// ---- the run ----
static const rbx_game *s_game;
static int s_invert_lines, s_mahjong;
static int s_in_v70, s_in_z80;
static int s_irq_dirty;
static uint64_t s_v_slice0;          // (the V70's time at its slice's start)
static unsigned long s_zstart;
static uint64_t s_z_slice0;
static uint32_t s_spin;              // (ticks the V70 stops for after the instruction: sound_command_w)
static int s_port_in, s_port_dsw, s_port_key[5];
static int s_dbg;                    // RBX_MS32_DBG: the CPUs' places every second

// ---- the snapshot drawn by render ----
static struct {
	uint16_t tx[0x2000], bg[0x2000], roz[0x8000], line[0x800];
	uint8_t pri[0x2000];
	uint32_t roz_ctrl[0x60 / 4], tx_scroll[6], bg_scroll[6], sprite_ctrl4, layout;
} *s_snap;
static uint16_t *s_snap_spr;         // (the sprite buffer drawn: swapped with s_sprbuf at the vblank)
static uint32_t *s_snap_pens;        // (0x8000)
static uint8_t s_pen_dirty[0x8000 / 256], s_snap_pen_dirty[0x8000 / 256];
static int s_pens_all;
// the render's buffers
static uint16_t *s_tbuf, *s_sbuf;    // tiles, sprites (VIS_W x VIS_H)
static uint8_t *s_pbuf;              // the tiles' priority bits

static inline uint64_t v_now(void) { return s_in_v70 ? s_v_slice0 + (uint64_t)v60_cycles_run() * T_V70 : s_st.vt; }
static inline uint64_t z_now(void) { return s_in_z80 ? s_z_slice0 + (uint64_t)(s_z.cyc - s_zstart) * T_Z80 : s_st.zt; }
static inline uint32_t wrapn(uint32_t v, uint32_t n, uint32_t m) { return m ? v & m : rbx_wrap(v, n); }

// ---------------------------------------------------------------------------------------------------- the palette
static inline uint32_t make_pen(int color)
{
	const uint16_t rg = s_palram[color * 2], bb = s_palram[color * 2 + 1];
	uint32_t r = (rg >> 8) & 0xff, g = rg & 0xff, b = bb & 0xff;
	if (~color & 0x4000) {   // (update_color: the brightness on all but the colours above 0x4000)
		r = (r * (uint32_t)s_st.brt_r) >> 8;
		g = (g * (uint32_t)s_st.brt_g) >> 8;
		b = (b * (uint32_t)s_st.brt_b) >> 8;
	}
	return r << 16 | g << 8 | b;
}
static void pens_all(void)
{
	for (int i = 0; i < 0x8000; i++)
		s_pens[i] = make_pen(i);
	memset(s_pen_dirty, 1, sizeof(s_pen_dirty));
}

static void brightness_w(int offset, uint16_t data, uint16_t mask, int hi)
{
	const uint32_t old = s_st.brt[offset];
	const uint32_t m = hi ? (uint32_t)mask << 16 : mask, d = hi ? (uint32_t)data << 16 : data;
	s_st.brt[offset] = (old & ~m) | (d & m);
	if (s_st.brt[offset] != old && !(offset & 2)) {
		s_st.brt_r = 0x100 - (int32_t)((s_st.brt[0] & 0xff00) >> 8);
		s_st.brt_g = 0x100 - (int32_t)(s_st.brt[0] & 0x00ff);
		s_st.brt_b = 0x100 - (int32_t)(s_st.brt[1] & 0x00ff);
		pens_all();
	}
}

// ---------------------------------------------------------------------------------------------------- interrupts
static void apply_irq(void)
{
	const int state = s_st.irqreq ? 1 : 0;
	if (state != s_st.irq_line) {
		s_st.irq_line = (uint8_t)state;
		v60_set_irq_line(state);
	}
}
static void irq_raise(int level, int state)
{
	if (state)
		s_st.irqreq |= (uint16_t)(1u << level);
	else
		s_st.irqreq &= (uint16_t)~(1u << level);
	if (s_in_v70) {
		s_irq_dirty = 1;
		if ((s_st.irqreq ? 1 : 0) != s_st.irq_line)
			v60_abort_timeslice();
	} else
		apply_irq();
}
static uint8_t irq_callback(void)
{
	int i;
	for (i = 15; i >= 0 && !(s_st.irqreq & (1 << i)); i--) {}
	return (uint8_t)i;
}
static inline void vblank_cb(int s) { irq_raise(10, s); }
static inline void field_cb(int s) { irq_raise(9, s); }

// ---------------------------------------------------------------------------------------------------- the sound
static void snd_render_to(uint64_t now)
{
	// samples at 44100 Hz: sample k is due at k * TICKS_HZ / 44100
	const uint64_t lim = now * 44100u;
	int n = 0;
	while (s_st.snd_acc <= lim) {
		s_st.snd_acc += TICKS_HZ;
		n++;
	}
	if (!n)
		return;
	const unsigned prof = rbx_prof_enter(MS_PA_SND);
	if (s_mix_n + n > s_mix_cap)
		n = s_mix_cap - s_mix_n;
	if (n > 0) {
		ymf271_render(s_ymf, s_mix + s_mix_n * 4, n);
		s_mix_n += n;
	}
	rbx_prof_leave(prof);
}
static void ymf_flush(void)
{
	if (s_ymf->wk)
		ymf271_async_take(s_ymf, s_mix, s_mix_n);
	for (int i = 0; i < s_mix_n; i++) {
		const int32_t *m = s_mix + i * 4;   // (outputs 0 / 1 to the speakers at 1.0)
		rbx_audio_put_stereo(m[0] / 4, m[1] / 4);
	}
	s_mix_n = 0;
}
static void ymf_job(void) { ymf271_async_job(s_ymf, 0); }
static void ymf_kick(void *u) { (void)u; rbx_aux_kick(); }
static void ymf_sync(void *u) { (void)u; rbx_aux_sync(); }
static void ymf_timer(void *user, int which, int64_t clocks)
{
	(void)user;
	if (clocks < 0)
		s_st.ymf_at[which] = 0;
	else {
		const uint64_t t = rbx_div64_small((uint64_t)clocks * TICKS_HZ + YMF_CLOCK - 1, YMF_CLOCK);
		s_st.ymf_at[which] = z_now() + (t ? t : 1);
	}
}
static uint8_t ymf_mem_r(void *user, uint32_t addr) { (void)user; (void)addr; return 0; }
static void ymf_mem_w(void *user, uint32_t addr, uint8_t data) { (void)user; (void)addr; (void)data; }

// the Z80's banks (ms32_snd_bank_w: 16 entries of 16 KB from 0x4000)
static void zbank_update(void)
{
	for (int b = 0; b < 2; b++) {
		uint32_t base = 0x4000 + (uint32_t)(s_st.zbank[b] & 0x0f) * 0x4000;
		if (base + 0x4000 > s_zrom_size)
			base = base % (s_zrom_size & ~0x3fffu ? s_zrom_size & ~0x3fffu : 0x4000);
		for (int p = 0; p < 64; p++)
			s_zmap[0x80 + b * 64 + p] = s_zrom + base + p * 256;
	}
}

static uint8_t z_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x3f00)
		return s_zrom[a];
	if (a < 0x4000) {
		if (a <= 0x3f0f) {
			snd_render_to(z_now());
			return ymf271_read(s_ymf, a & 0xf);
		}
		if (a == 0x3f10) {   // latch_r (the generic latch's read clears its pending flag: the NMI line)
			s_st.latch_pending = 0;
			return (uint8_t)(s_st.latch ^ 0xff);
		}
		return 0xff;
	}
	if (a < 0x8000)
		return s_zram[a & 0x3fff];
	return s_zmap[a >> 8][a & 0xff];
}
static void z_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0x4000 && a < 0x8000) {
		s_zram[a & 0x3fff] = v;
		return;
	}
	if (a >= 0x3f00 && a <= 0x3f0f) {
		snd_render_to(z_now());
		ymf271_write(s_ymf, a & 0xf, v);
	} else if (a == 0x3f10) {   // to_main_w
		if (s_dbg == 3) fprintf(stderr, "ms32 f%u to_main %02x\n", s_st.frame, v);
		s_st.to_main = v;
		irq_raise(1, 1);
	} else if (a == 0x3f80) {   // ms32_snd_bank_w
		s_st.zbank[0] = v & 0x0f;
		s_st.zbank[1] = v >> 4;
		zbank_update();
	}
}
static uint8_t z_in(z80 *z, uint8_t p) { (void)z; (void)p; return 0xff; }
static void z_out(z80 *z, uint8_t p, uint8_t v) { (void)z; (void)p; (void)v; }

static void z_setup(void)
{
	z80_init(&s_z);
	s_z.read_byte = z_read;
	s_z.write_byte = z_write;
	s_z.port_in = z_in;
	s_z.port_out = z_out;
	s_z.read_map = s_zmap;
}

// the Z80 (and the YMF271's timers) up to time t
static void z_run_to(uint64_t t)
{
	if (s_st.zt >= t)
		return;
	const unsigned prof = rbx_prof_enter(MS_PA_Z80);
	while (s_st.zt < t) {
		uint64_t end = t;
		for (int i = 0; i < 2; i++)
			if (s_st.ymf_at[i] && s_st.ymf_at[i] <= s_st.zt) {
				s_st.ymf_at[i] = 0;
				snd_render_to(s_st.zt);
				ymf271_timer_expired(s_ymf, i);   // (may set it again)
			}
		for (int i = 0; i < 2; i++)
			if (s_st.ymf_at[i] && s_st.ymf_at[i] < end)
				end = s_st.ymf_at[i];
		const uint32_t n = ((uint32_t)(end - s_st.zt) + T_Z80 - 1) / T_Z80;   // (a slice: below 2^32 ticks)
		if (s_st.latch_pending && !s_st.z_nmi_line)
			z80_gen_nmi(&s_z);
		s_st.z_nmi_line = s_st.latch_pending;
		s_in_z80 = 1;
		s_zstart = s_z.cyc;
		s_z_slice0 = s_st.zt;
		z80_run(&s_z, s_zstart + n, &s_zline, 0);
		s_in_z80 = 0;
		const unsigned long ran = s_z.cyc - s_zstart;
		s_st.zt += (uint64_t)(ran ? ran : 1) * T_Z80;
		// (the NMI line follows the latch's pending flag: a read in the run cleared it)
		if (!s_st.latch_pending)
			s_st.z_nmi_line = 0;
	}
	rbx_prof_leave(prof);
}

// ---------------------------------------------------------------------------------------------------- the V70's bus
static inline uint32_t port32(int p) { return p >= 0 ? rbx_port_read(p) : 0xffffffffu; }

// mahjong_ctrl_r: the KEY rows selected by mahjong_select, rotated so the Start column is the lowest bit
static uint32_t mahjong_ctrl(void)
{
	uint32_t mj = 0x3f;
	for (int i = 0; i < 5; i++)
		if ((s_st.mj_select >> i) & 1)
			mj &= s_port_key[i] >= 0 ? rbx_port_read(s_port_key[i]) : 0x3f;
	// bitswap<6>(mj, 4, 2, 3, 1, 0, 5)
	const uint32_t r = ((mj >> 4) & 1) << 5 | ((mj >> 2) & 1) << 4 | ((mj >> 3) & 1) << 3 | ((mj >> 1) & 1) << 2 |
	                   (mj & 1) << 1 | ((mj >> 5) & 1);
	return r | 0xc0;
}

static uint32_t inputs_r(void)
{
	uint32_t v = port32(s_port_in);
	if (s_mahjong)
		v = (v & ~0xffu) | mahjong_ctrl();
	return v;
}

static inline uint16_t ram32_r(const uint32_t *r, uint32_t i, int hi) { return (uint16_t)(hi ? r[i] >> 16 : r[i]); }
static inline void ram32_w(uint32_t *r, uint32_t i, int hi, uint16_t d, uint16_t mask)
{
	const uint32_t m = hi ? (uint32_t)mask << 16 : mask, v = hi ? (uint32_t)d << 16 : d;
	r[i] = (r[i] & ~m) | (v & m);
}
static inline uint16_t combine(uint16_t old, uint16_t v, uint16_t mask) { return (uint16_t)((old & ~mask) | (v & mask)); }

static void dbg_acc(const char *k, uint32_t a, uint16_t d)
{
	static uint32_t seen[512];
	static int n;
	for (int i = 0; i < n; i++)
		if (seen[i] == (a | (k[0] == 'w' ? 0x80000000u : 0)))
			return;
	if (n < 512)
		seen[n++] = a | (k[0] == 'w' ? 0x80000000u : 0);
	fprintf(stderr, "ms32 %s %08x %04x pc %08x\n", k, a | 0xfc000000u, d, v60_ppc());
}
static uint16_t main_read16_(uint32_t a, uint16_t mask);
static uint16_t main_read16(uint32_t a, uint16_t mask)
{
	const uint16_t v = main_read16_(a, mask);
	if (s_dbg > 1)
		dbg_acc("r", a, v);
	return v;
}
static uint16_t main_read16_(uint32_t a, uint16_t mask)
{
	(void)mask;
	const int hi = (a & 2) != 0;
	const uint32_t top = a & 0x3e00000;   // (2 MB areas of the 64 MB image)
	switch (top) {
	case 0x0000000:   // fc000000 NVRAM (8 bits)
		return hi ? 0 : s_nvram[(a & 0x7fff) >> 2];
	case 0x0800000:   // fc800000 sound latch: no read
		return 0;
	case 0x0c00000:   // fcc00000 inputs
		if ((a & 0x1ffffc) == 0x04) {
			const uint32_t v = inputs_r();
			return (uint16_t)(hi ? v >> 16 : v);
		}
		if ((a & 0x1ffffc) == 0x10) {
			const uint32_t v = port32(s_port_dsw);
			return (uint16_t)(hi ? v >> 16 : v);
		}
		return 0;
	case 0x0e00000: {   // fce00000 system / video registers
		const uint32_t o = a & 0x1ffffc;
		if (o >= 0x200 && o < 0x280)
			return ram32_r(s_st.sprite_ctrl, (o - 0x200) >> 2, hi);
		if (o >= 0x600 && o < 0x660)
			return ram32_r(s_st.roz_ctrl, (o - 0x600) >> 2, hi);
		if (o >= 0xa00 && o < 0xa18)
			return ram32_r(s_st.tx_scroll, (o - 0xa00) >> 2, hi);
		if (o >= 0xa20 && o < 0xa38)
			return ram32_r(s_st.bg_scroll, (o - 0xa20) >> 2, hi);
		return 0;
	}
	case 0x1000000:   // fd000000 sound result; fd180000 priority RAM
		if ((a & 0x1fffff) < 4) {
			if (hi)
				return 0;
			z_run_to(v_now());
			irq_raise(1, 0);   // (sound_result_r: the irq acked here)
			if (s_dbg == 3) fprintf(stderr, "ms32 f%u result %02x\n", s_st.frame, s_st.to_main);
			return (uint16_t)(s_st.to_main ^ 0xff);
		}
		if ((a & 0x1fffff) >= 0x180000 && (a & 0x1fffff) < 0x1c0000)
			return hi ? 0 : s_priram[(a & 0x7fff) >> 2];
		return 0;
	case 0x1400000:   // fd400000 palette
		return hi ? 0 : s_palram[(a & 0x3ffff) >> 2];
	case 0x2000000:
		return hi ? 0 : s_rozram[(a & 0x1ffff) >> 2];
	case 0x2200000:
		return hi ? 0 : s_lineram[(a & 0x1fff) >> 2];
	case 0x2800000:
		return hi ? 0 : s_sprram[(a & 0x1ffff) >> 2];
	case 0x2c00000:
		if (hi)
			return 0;
		return (a & 0x8000) ? s_bgram[(a & 0x7fff) >> 2] : s_txram[(a & 0x7fff) >> 2];
	}
	return 0;
}

static void sysctrl_w(uint32_t reg, uint16_t data);

static void sound_command_w(uint8_t data)
{
	z_run_to(v_now());
	if (s_dbg == 3)
		fprintf(stderr, "ms32 f%u cmd %02x (pending %d) z80 pc %04x\n", s_st.frame, data, s_st.latch_pending, s_z.pc);
	s_st.latch = data;
	s_st.latch_pending = 1;
	// (spin_until_time 40 us: the Z80 answers meanwhile)
	if (s_in_v70) {
		s_spin = 40 * (TICKS_HZ / 1000000u);
		v60_abort_timeslice();
	}
}

static void main_write16(uint32_t a, uint16_t data, uint16_t mask)
{
	if (s_dbg > 1)
		dbg_acc("w", a, data);
	const int hi = (a & 2) != 0;
	const uint32_t top = a & 0x3e00000;
	switch (top) {
	case 0x0000000:
		if (!hi && (mask & 0xff))
			s_nvram[(a & 0x7fff) >> 2] = (uint8_t)data;
		return;
	case 0x0800000:
		if ((a & 0x1ffffc) == 0 && !hi && (mask & 0xff))
			sound_command_w((uint8_t)data);
		return;
	case 0x0e00000: {
		const uint32_t o = a & 0x1ffffc;
		if (o < 0x60) {
			if (!hi)
				sysctrl_w(o >> 2, data);
		} else if (o >= 0x200 && o < 0x280)
			ram32_w(s_st.sprite_ctrl, (o - 0x200) >> 2, hi, data, mask);
		else if (o >= 0x280 && o < 0x290)
			brightness_w((int)((o - 0x280) >> 2), data, mask, hi);
		else if (o >= 0x600 && o < 0x660)
			ram32_w(s_st.roz_ctrl, (o - 0x600) >> 2, hi, data, mask);
		else if (o >= 0xa00 && o < 0xa18)
			ram32_w(s_st.tx_scroll, (o - 0xa00) >> 2, hi, data, mask);
		else if (o >= 0xa20 && o < 0xa38)
			ram32_w(s_st.bg_scroll, (o - 0xa20) >> 2, hi, data, mask);
		else if (o == 0xa7c)
			ram32_w(&s_st.layout, 0, hi, data, mask);   // (bgmode_w)
		return;   // (e00: the coin counters)
	}
	case 0x1000000:
		if ((a & 0x1fffff) >= 0x180000 && (a & 0x1fffff) < 0x1c0000) {
			if (!hi && (mask & 0xff))
				s_priram[(a & 0x7fff) >> 2] = (uint8_t)data;
		} else if ((a & 0x1ffffc) == 0x1c0000)
			ram32_w(&s_st.mj_select, 0, hi, data, mask);
		return;
	case 0x1400000:
		if (!hi) {
			const uint32_t i = (a & 0x3ffff) >> 2;
			const uint16_t v = combine(s_palram[i], data, mask);
			if (v != s_palram[i]) {
				s_palram[i] = v;
				if (i < 0x10000) {
					s_pens[i >> 1] = make_pen((int)(i >> 1));
					s_pen_dirty[i >> 9] = 1;
				}
			}
		}
		return;
	case 0x2000000:
		if (!hi) { uint16_t *p = &s_rozram[(a & 0x1ffff) >> 2]; *p = combine(*p, data, mask); }
		return;
	case 0x2200000:
		if (!hi) { uint16_t *p = &s_lineram[(a & 0x1fff) >> 2]; *p = combine(*p, data, mask); }
		return;
	case 0x2800000:
		if (!hi) { uint16_t *p = &s_sprram[(a & 0x1ffff) >> 2]; *p = combine(*p, data, mask); }
		return;
	case 0x2c00000:
		if (!hi) {
			uint16_t *p = (a & 0x8000) ? &s_bgram[(a & 0x7fff) >> 2] : &s_txram[(a & 0x7fff) >> 2];
			*p = combine(*p, data, mask);
		}
		return;
	}
}

// ---------------------------------------------------------------------------------------------------- the sysctrl
static inline uint16_t neg12(uint16_t d) { return (uint16_t)(0x1000 - (d & 0xfff)); }
static void prg_timer_flush(void)
{
	if (s_st.timer_en)
		s_st.prg_at = v_now() + (uint64_t)s_st.interval * (TICKS_HZ / 2000u);   // (500 us x interval)
	else {
		s_st.prg_at = 0;
		irq_raise(0, 0);
	}
	if (s_in_v70)
		v60_abort_timeslice();   // (the slice ends at the new time)
}
static void sysctrl_w(uint32_t reg, uint16_t data)
{
	switch (reg) {
	case 0x0:   // control_w
		s_st.dotclock = data & 1;
		s_st.flip = (data >> 1) & 1;
		s_st.timer_en = (data >> 3) & 1;
		prg_timer_flush();
		break;
	case 0x1: s_st.hblank = neg12(data); break;
	case 0x2: s_st.hdisp = neg12(data); break;
	case 0x5: s_st.vblank = neg12(data); break;
	case 0x6: s_st.vdisp = neg12(data); break;
	case 0xc: s_st.interval = neg12(data); prg_timer_flush(); break;
	case 0xd: irq_raise(0, 0); break;   // timer_ack_w
	case 0xe:   // sound_reset_w: pulse_input_line(RESET, zero)
		if (data & 1) {
			z_run_to(v_now());
			const unsigned long cyc = s_z.cyc;
			z_setup();
			s_z.cyc = cyc;
		}
		break;
	case 0xf: vblank_cb(0); field_cb(0); break;   // irq_ack_w
	case 0x13: s_st.to_main = 0xff; break;          // sound_ack_w
	case 0x16: field_cb(0); break;
	case 0x17: vblank_cb(0); break;
	}
}

// ---------------------------------------------------------------------------------------------------- drawing
// The frame in BANDS bands of lines drawn apart by the render thread and its helpers (decoband): render makes the
// frame's lists once (the visible sprites with their clipping worked out, the layer order, the mixer's table, the ROZ
// map's tiles), then each band draws MAME's screen_update clipped to its lines (tiles, sprites, mixing are per pixel,
// the result is the same).
#define BANDS 8
#define BAND_H (VIS_H / BANDS)

enum { A_SPR, A_TILE, A_SHADOW, A_BLACK };
enum { TC_EMPTY, TC_MIXED, TC_OPAQUE };   // (a tile's pens: all 0, some, none 0)

typedef struct {
	int16_t sx, sy, ylast;       // the first column / line drawn, the last line (inclusive)
	int16_t nx;                  // columns drawn a line (the texture's edge and the screen's right side counted)
	uint8_t flipy;
	uint16_t color;
	uint32_t incy, srcy0, srcendy, tyo8;
	uint32_t tex;                // the texture's offset (code << 16)
	uint32_t pos0;               // the first column's source x (8.8, the texture's tx included)
	int32_t dpos;                // its step a column
} spr_t;

static spr_t *s_list;                 // (SPR_WORDS / 8 entries)
static int s_nlist;
static int s_rot_pri, s_scr_pri, s_asc_pri;
static uint8_t s_act[16][8];
static struct { uint16_t spr, and_, or_, pad; } s_mixm[16 * 8];   // (s_act as masks: [sprite priority << 3 | tile bits])
static int s_mix_simple, s_mix_zplain;              // (every case A_SPR: the sprite's pixel when it has one, else the tiles')
static uint32_t s_tx_sx, s_tx_sy, s_bg_sx, s_bg_sy;
static uint8_t *s_roz_cls, *s_bg_cls, *s_tx_cls;   // (the gfx tiles' TC_*)
static uint32_t *s_olut;              // (PENS: the colours in the frontend's format, rbx_video.out_color)
static uint32_t (*s_olut_fn)(int r, int g, int b);
static int s_olut_bpp;
static int s_noout;                   // (RBX_MS32_NOOUT: the bitmap and the core's conversion, to check the frame written here)
static void *s_out;                   // (the frontend's frame this render: the bands write it, else the bitmap)
static struct { int super_; uint32_t sx, sy, ixx, ixy, iyx, iyy; int32_t startx, starty, offsx, offsy; } s_rz;

static uint8_t *tile_classes(const uint8_t *gfx, uint32_t n, int bytes)
{
	uint8_t *c = (uint8_t *)malloc(n ? n : 1);
	if (!c)
		return NULL;
	for (uint32_t i = 0; i < n; i++) {
		const uint8_t *p = gfx + (size_t)i * bytes;
		int zero = 0, set = 0;
		for (int k = 0; k < bytes; k++) {
			if (p[k])
				set = 1;
			else
				zero = 1;
		}
		c[i] = (uint8_t)(!set ? TC_EMPTY : zero ? TC_MIXED : TC_OPAQUE);
	}
	return c;
}

// the lines [y0, y1) of a scrolling tilemap (8x8 or 16x16 raw 8bpp tiles, row-major): opaque pens over the tiles'
// buffer, the layer's priority bit or'ed into the priority buffer
static void draw_tilemap(int y0, int y1, const uint16_t *ram, int cols, int rows, int tsz, const uint8_t *gfx,
                         const uint8_t *cls, uint32_t n, uint32_t m, uint32_t colbase, uint32_t scrollx, uint32_t scrolly,
                         uint8_t pri)
{
	const int shift = tsz == 16 ? 4 : 3, tmask = tsz - 1;
	const uint32_t wmask = (uint32_t)(cols * tsz - 1), hmask = (uint32_t)(rows * tsz - 1);
	const int tbytes = tsz * tsz;
	for (int y = y0; y < y1; y++) {
		const uint32_t sy = (uint32_t)(y + scrolly) & hmask;
		const uint16_t *trow = ram + (size_t)(sy >> shift) * cols * 2;
		const int fy = (int)(sy & tmask);
		uint16_t *dt = s_tbuf + y * VIS_W;
		uint8_t *dp = s_pbuf + y * VIS_W;
		int x = 0;
		uint32_t sx = scrollx & wmask;
		while (x < VIS_W) {
			const int fx = (int)(sx & tmask);
			int len = tsz - fx;
			if (len > VIS_W - x)
				len = VIS_W - x;
			const uint32_t ti = (sx >> shift) * 2;
			const uint32_t code = wrapn(trow[ti], n, m);
			const uint8_t c = cls[code];
			if (c != TC_EMPTY) {
				const uint16_t col = (uint16_t)(colbase + ((trow[ti + 1] & 0xf) << 8));
				const uint8_t *src = gfx + (size_t)code * tbytes + fy * tsz + fx;
				uint16_t *t = dt + x;
				uint8_t *p = dp + x;
				if (c == TC_OPAQUE)
					for (int i = 0; i < len; i++) {
						t[i] = (uint16_t)(col | src[i]);
						p[i] |= pri;
					}
				else
					for (int i = 0; i < len; i++) {
						const uint8_t pix = src[i];
						if (pix) {
							t[i] = (uint16_t)(col | pix);
							p[i] |= pri;
						}
					}
			}
			x += len;
			sx = (sx + (uint32_t)len) & wmask;
		}
	}
}

// a ROZ map tile: its colour | its TC_* in the low bits, its gfx offset
static inline uint16_t roz_tile(uint32_t ti, uint32_t *off)
{
	const uint32_t code = wrapn(s_snap->roz[ti * 2], s_roz_n, s_roz_m);
	*off = code * 256u;
	return (uint16_t)((0x2000 + ((s_snap->roz[ti * 2 + 1] & 0xf) << 8)) | s_roz_cls[code]);
}

// the ROZ layer (128x128 16x16 tiles, wrapping): MAME draw_roz with wrap, 16.16 coordinates, one line
static void roz_line(int y, uint32_t cx, uint32_t cy, uint32_t dxx, uint32_t dxy)
{
	uint16_t *dt = s_tbuf + y * VIS_W;
	uint8_t *dp = s_pbuf + y * VIS_W;
	const uint8_t *gfx = s_roz_gfx;
	if (dxy == 0 && dxx == 0x10000) {   // (unscaled, unrotated: whole tile pieces)
		const uint32_t py = (cy >> 16) & 2047;
		const uint32_t rowt = (py >> 4) << 7, fy = (py & 15) << 4;
		uint32_t px = (cx >> 16) & 2047;
		int x = 0;
		while (x < VIS_W) {
			const uint32_t fx = px & 15;
			int len = 16 - (int)fx;
			if (len > VIS_W - x)
				len = VIS_W - x;
			const uint32_t ti = rowt | (px >> 4);
			uint32_t off;
			const uint16_t ct = roz_tile(ti, &off);
			if ((ct & 3) != TC_EMPTY) {
				const uint8_t *src = gfx + off + fy + fx;
				const uint16_t col = (uint16_t)(ct & 0xff00);
				for (int i = 0; i < len; i++) {
					const uint8_t pix = src[i];
					if (pix) {
						dt[x + i] = (uint16_t)(col | pix);
						dp[x + i] |= 2;
					}
				}
			}
			x += len;
			px = (px + (uint32_t)len) & 2047;
		}
		return;
	}
	uint32_t last = 0xffffffffu;
	const uint8_t *tile = gfx;
	uint16_t col = 0;
	int empty = 1;
	for (int x = 0; x < VIS_W; x++, cx += dxx, cy += dxy) {
		const uint32_t px = (cx >> 16) & 2047, py = (cy >> 16) & 2047;
		const uint32_t ti = (py >> 4) << 7 | (px >> 4);
		if (ti != last) {
			last = ti;
			uint32_t off;
			const uint16_t ct = roz_tile(ti, &off);
			empty = (ct & 3) == TC_EMPTY;
			col = (uint16_t)(ct & 0xff00);
			tile = gfx + off;
		}
		if (empty)
			continue;
		const uint8_t pix = tile[((py & 15) << 4) + (px & 15)];
		if (pix) {
			dt[x] = (uint16_t)(col | pix);
			dp[x] |= 2;
		}
	}
}
static inline int32_t sext(uint32_t v, uint32_t sign) { return (v & sign) ? (int32_t)(v | ~(sign * 2 - 1)) : (int32_t)v; }
static void roz_prep(void)
{
	const uint32_t *rc = s_snap->roz_ctrl;
	s_rz.startx = sext((rc[0] & 0xffff) | ((rc[1] & 3) << 16), 0x20000);
	s_rz.starty = sext((rc[2] & 0xffff) | ((rc[3] & 3) << 16), 0x20000);
	s_rz.offsx = (int32_t)(rc[0x30 / 4] + (rc[0x38 / 4] & 1) * 0x400);
	s_rz.offsy = (int32_t)(rc[0x34 / 4] + (rc[0x3c / 4] & 1) * 0x400);
	s_rz.super_ = rc[0x5c / 4] & 1;
	s_rz.ixx = (uint32_t)sext((rc[4] & 0xffff) | ((rc[5] & 1) << 16), 0x10000) << 8;
	s_rz.ixy = (uint32_t)sext((rc[6] & 0xffff) | ((rc[7] & 1) << 16), 0x10000) << 8;
	s_rz.iyy = (uint32_t)sext((rc[8] & 0xffff) | ((rc[9] & 1) << 16), 0x10000) << 8;
	s_rz.iyx = (uint32_t)sext((rc[10] & 0xffff) | ((rc[11] & 1) << 16), 0x10000) << 8;
	s_rz.sx = (uint32_t)(s_rz.startx + s_rz.offsx) << 16;
	s_rz.sy = (uint32_t)(s_rz.starty + s_rz.offsy) << 16;
}
static void draw_roz(int y0, int y1)
{
	for (int y = y0; y < y1; y++) {
		if (s_rz.super_) {   // "super" mode: per line
			const uint16_t *l = &s_snap->line[8 * (y & 0xff)];
			const int32_t s2x = sext(l[0] | ((l[1] & 3u) << 16), 0x20000);
			const int32_t s2y = sext(l[2] | ((l[3] & 3u) << 16), 0x20000);
			const int32_t ixx = sext(l[4] | ((l[5] & 1u) << 16), 0x10000);
			const int32_t ixy = sext(l[6] | ((l[7] & 1u) << 16), 0x10000);
			roz_line(y, (uint32_t)(s2x + s_rz.startx + s_rz.offsx) << 16, (uint32_t)(s2y + s_rz.starty + s_rz.offsy) << 16,
			         (uint32_t)ixx << 8, (uint32_t)ixy << 8);
		} else
			roz_line(y, s_rz.sx + (uint32_t)y * s_rz.iyx, s_rz.sy + (uint32_t)y * s_rz.iyy, s_rz.ixx, s_rz.ixy);
	}
}

// the visible sprites in drawing order (draw_sprites + ms32_sprite_device::prio_zoom_transpen_raw), their clipping
// worked out once: the first opaque pixel at a place stays
static void build_sprites(const uint16_t *ram)
{
	s_nlist = 0;
	if (!s_spr_count)
		return;
	const int reverse = (s_snap->sprite_ctrl4 & 0x8000) == 0;
	const int step = reverse ? -8 : 8;
	int i = reverse ? SPR_WORDS - 8 : 0;   // (sprite_tail = size - 8: reverse starts there, forward stops before it)
	const int end = reverse ? -8 : SPR_WORDS - 8;
	for (; i != end; i += step) {
		const uint16_t *s = ram + i;
		const uint16_t attr = s[0];
		if (!(attr & 4))
			continue;
		const uint32_t incx = s[6], incy = s[7];
		if (!incx || !incy)
			continue;
		const int flipx = attr & 1;
		const uint32_t txo = s[1] & 0xff, tyo = s[1] >> 8;
		const int32_t w = (s[3] & 0xff) + 1, h = (s[3] >> 8) + 1;
		int32_t sx = (s[5] & 0x3ff) - (s[5] & 0x400);
		int32_t sy = (s[4] & 0x1ff) - (s[4] & 0x200);
		const uint32_t srcendx = (uint32_t)w << 8, srcendy = (uint32_t)h << 8;
		uint32_t srcx = 0, srcy = 0;
		if (sx < 0) {
			srcx = (uint32_t)(-sx) * incx;
			sx = 0;
		}
		if (srcx >= srcendx || sx >= VIS_W)
			continue;
		if (sy < 0) {
			srcy = (uint32_t)(-sy) * incy;
			sy = 0;
		}
		if (srcy >= srcendy || sy >= VIS_H)
			continue;
		// the columns: while the source is inside the sprite and the screen; the texture's columns past 255 skipped
		uint32_t nx = (srcendx - srcx + incx - 1) / incx;
		if (nx > (uint32_t)(VIS_W - sx))
			nx = (uint32_t)(VIS_W - sx);
		uint32_t pos = (txo << 8) + (flipx ? srcendx - srcx - 1 : srcx);
		if (!flipx) {   // (increasing: the columns from 256 on are past the texture)
			const uint32_t ok = pos < 0x10000 ? (0x10000 - pos + incx - 1) / incx : 0;
			if (nx > ok)
				nx = ok;
		} else if (pos >= 0x10000) {   // (decreasing: the first ones past it skipped)
			const uint32_t skip = (pos - 0x10000) / incx + 1;
			if (skip >= nx)
				continue;
			nx -= skip;
			sx += (int32_t)skip;
			srcx += skip * incx;
			pos -= skip * incx;
		}
		if (!nx)
			continue;
		const uint32_t ny = (srcendy - srcy + incy - 1) / incy;
		const int32_t ylast = sy + (int32_t)ny - 1;
		spr_t *e = &s_list[s_nlist++];
		e->sx = (int16_t)sx;
		e->sy = (int16_t)sy;
		e->ylast = (int16_t)(ylast < VIS_H - 1 ? ylast : VIS_H - 1);
		e->nx = (int16_t)nx;
		e->flipy = (uint8_t)((attr & 2) != 0);
		e->color = (uint16_t)((((s[2] >> 12) & 0xf) << 8) | ((attr & 0xf0) << 8));
		e->incy = incy;
		e->srcy0 = srcy;
		e->srcendy = srcendy;
		e->tyo8 = tyo << 8;
		e->tex = rbx_wrap(s[2] & 0x0fff, s_spr_count) << 16;
		e->pos0 = pos;
		e->dpos = flipx ? -(int32_t)incx : (int32_t)incx;
	}
}
static void draw_sprites(int y0, int y1)
{
	for (int k = 0; k < s_nlist; k++) {
		const spr_t *e = &s_list[k];
		if (e->sy >= y1 || e->ylast < y0)
			continue;
		const int ya = e->sy > y0 ? e->sy : y0, yb = e->ylast < y1 - 1 ? e->ylast : y1 - 1;
		uint32_t srcy = e->srcy0 + (uint32_t)(ya - e->sy) * e->incy;
		// the columns' places in a texture row (8x8 tiles: x & 7 | (x >> 3) << 6), the same on every line
		uint16_t cofs[VIS_W];
		const int nx = e->nx;
		uint32_t pos = e->pos0;
		for (int x = 0; x < nx; x++, pos += (uint32_t)e->dpos) {
			const uint32_t dx = pos >> 8;
			cofs[x] = (uint16_t)((dx & 7) | (dx >> 3) << 6);
		}
		for (int y = ya; y <= yb; y++, srcy += e->incy) {
			const uint32_t drawy = (e->tyo8 + (e->flipy ? e->srcendy - srcy - 1 : srcy)) >> 8;
			if (drawy >= 256)
				continue;
			const uint32_t off = e->tex | (drawy >> 3) << 11 | (drawy & 7) << 3;
			const uint8_t *row = s_spr_blk[off >> RBX_BLK_SHIFT] + (off & RBX_BLK_MASK);
			uint16_t *d = s_sbuf + y * VIS_W + e->sx;
			const uint16_t color = e->color;
			for (int x = 0; x < nx; x++) {
				const uint8_t pix = row[cofs[x]];
				if (pix && !d[x])
					d[x] = (uint16_t)(color | pix);
			}
		}
	}
}

// one line of the tile-sprite mixing: the colour index (MAME's cases through s_act: A_SPR the sprite's pixel when
// opaque, A_SHADOW the tiles' | 0x8000, A_BLACK PEN_BLACK, A_TILE the tiles'), stored as an index (lut NULL) or as
// the frontend's colour of bpp bytes; pixels without a sprite whose case is the tiles' taken first
static inline __attribute__((always_inline)) void mix_line(void *dst, const uint16_t *t, const uint16_t *s,
                                                            const uint8_t *p, const uint32_t *lut, int bpp)
{
	const int simple = s_mix_simple, zplain = s_mix_zplain;
	for (int x = 0; x < VIS_W; x++) {
		const uint32_t sv = s[x];
		uint32_t v;
		if (!sv && zplain)
			v = t[x];
		else if (simple)
			v = (sv & 0xff) ? (sv & 0xfff) : t[x];
		else {
			const uint32_t k = (sv >> 12) << 3 | (p[x] & 7u);
			const uint32_t sel = s_mixm[k].spr & (0u - ((sv & 0xff) != 0));
			v = (((sv & 0xfff) & sel) | (t[x] & ~sel)) & s_mixm[k].and_;
			v |= s_mixm[k].or_;
		}
		if (bpp == 2)
			((uint16_t *)dst)[x] = (uint16_t)lut[v];
		else if (bpp == 4)
			((uint32_t *)dst)[x] = lut[v];
		else
			((uint16_t *)dst)[x] = (uint16_t)v;
	}
}

static void band_draw(int job)
{
	const int y0 = job * BAND_H, y1 = y0 + BAND_H;
	memset(s_tbuf + y0 * VIS_W, 0, (size_t)BAND_H * VIS_W * 2);
	memset(s_pbuf + y0 * VIS_W, 0, (size_t)BAND_H * VIS_W);
	memset(s_sbuf + y0 * VIS_W, 0, (size_t)BAND_H * VIS_W * 2);
	draw_sprites(y0, y1);
	for (int prin = 0; prin < 4; prin++) {
		if (s_rot_pri == prin)
			draw_roz(y0, y1);
		else if (s_scr_pri == prin) {
			if (s_snap->layout & 1)
				draw_tilemap(y0, y1, s_snap->bg, 256, 16, 16, s_bg_gfx, s_bg_cls, s_bg_n, s_bg_m, 0x1000, s_bg_sx, s_bg_sy, 1);
			else
				draw_tilemap(y0, y1, s_snap->bg, 64, 64, 16, s_bg_gfx, s_bg_cls, s_bg_n, s_bg_m, 0x1000, s_bg_sx, s_bg_sy, 1);
		} else if (s_asc_pri == prin)
			draw_tilemap(y0, y1, s_snap->tx, 64, 64, 8, s_tx_gfx, s_tx_cls, s_tx_n, s_tx_m, 0x6000, s_tx_sx, s_tx_sy, 4);
	}
	// the tile-sprite mixing (into the frontend's frame through the colour table when there is one)
	for (int y = y0; y < y1; y++) {
		const uint16_t *t = s_tbuf + y * VIS_W, *s = s_sbuf + y * VIS_W;
		const uint8_t *p = s_pbuf + y * VIS_W;
		if (!s_out) {
			mix_line(rbx_video.bitmap + (size_t)y * rbx_video.pitch, t, s, p, NULL, 0);
			continue;
		}
		uint8_t *op = (uint8_t *)s_out + (size_t)y * rbx_video.out_pitch;
		if (s_olut_bpp == 2)
			mix_line(op, t, s, p, s_olut, 2);
		else
			mix_line(op, t, s, p, s_olut, 4);
	}
}
static inline void olut_set(int i)
{
	const uint32_t c = rbx_video.palette[i];
	s_olut[i] = rbx_video.out_color((int)(c >> 16) & 0xff, (int)(c >> 8) & 0xff, (int)c & 0xff);
}
static decoband_t s_dband = { 0, 0, 0, 0, BANDS, band_draw };
static int ms32_help(int helper) { return decoband_help(&s_dband, helper); }

static void ms32_render(void)
{
	// the palette: the colours changed since the last frame, and their halves (the mixer's shadow); the frontend's
	// frame written by the bands through s_olut (made again whole when its format changes)
	s_out = !s_noout && rbx_video.out && (rbx_video.out_bpp == 2 || rbx_video.out_bpp == 4) && rbx_video.out_color ? rbx_video.out : NULL;
	if (!s_out)
		s_olut_fn = NULL;   // (the colours not followed meanwhile: made whole at the next frame written)
	const int olut_all = s_out &&(s_olut_fn != rbx_video.out_color || s_olut_bpp != rbx_video.out_bpp);
	for (int b = 0; b < 0x8000 / 256; b++) {
		if (!s_snap_pen_dirty[b])
			continue;
		s_snap_pen_dirty[b] = 0;
		for (int i = b * 256; i < b * 256 + 256; i++) {
			const uint32_t c = s_snap_pens[i];
			rbx_video.palette[i] = c;
			rbx_video.palette[0x8000 + i] = (c >> 1) & 0x7f7f7f;
			if (s_out && !olut_all) {
				olut_set(i);
				olut_set(0x8000 + i);
			}
		}
	}
	rbx_video.palette[PEN_BLACK] = 0;
	if (s_out) {
		if (olut_all) {
			for (int i = 0; i < PENS; i++)
				olut_set(i);
			s_olut_fn = rbx_video.out_color;
			s_olut_bpp = rbx_video.out_bpp;
		}
		olut_set(PEN_BLACK);
	}

	const uint8_t *pri = s_snap->pri;
	int asc_pri = 0, scr_pri = 0, rot_pri = 0;
	if ((pri[0x2b00 / 2] & 0xff) == 0x34) asc_pri++; else rot_pri++;
	if ((pri[0x2e00 / 2] & 0xff) == 0x34) asc_pri++; else scr_pri++;
	if (pri[0x3a00 / 2] == 0x09)
		asc_pri = 3;
	if ((pri[0x3a00 / 2] & 0x30) == 0x00) scr_pri++; else rot_pri++;
	s_asc_pri = asc_pri;
	s_scr_pri = scr_pri;
	s_rot_pri = rot_pri;
	const uint32_t *ts = s_snap->tx_scroll, *bs = s_snap->bg_scroll;
	s_tx_sx = ts[0] + ts[2] + 0x18;
	s_tx_sy = ts[3] + ts[5];
	s_bg_sx = bs[0] + bs[2] + 0x10;
	s_bg_sy = bs[3] + bs[5];
	if (rot_pri < 4)
		roz_prep();

	// the tile-sprite mixing (MAME's per-pixel cases as a table: the sprite priority's primask, the tiles' bits)
	int simple = 1;
	for (int sp = 0; sp < 16; sp++) {
		static const uint16_t off[8] = { 0x1500, 0x1400, 0x1100, 0x1000, 0x0500, 0x0400, 0x0100, 0x0000 };
		int pm = 0;
		for (int k = 0; k < 8; k++)
			if (pri[((sp << 4) | 0x0a00 | off[k]) / 2] & 0x38)
				pm |= 1 << k;
		for (int tp = 0; tp < 8; tp++) {
			uint8_t a;
			switch (pm) {
			case 0x00: a = A_SPR; break;
			case 0xc0: a = A_TILE; break;   // (MAME: random pens)
			case 0xcc: a = (tp & 2) ? A_TILE : A_SPR; break;
			case 0xf0: a = tp < 4 ? A_SPR : A_TILE; break;
			case 0xfc: a = tp < 2 ? A_SPR : A_TILE; break;
			case 0xfe: a = tp == 0 ? A_SPR : tp <= 3 ? A_SHADOW : A_TILE; break;
			case 0xf8: a = tp == 2 ? A_SPR : A_TILE; break;
			default: a = A_BLACK; break;
			}
			s_act[sp][tp] = a;
			if (a != A_SPR)
				simple = 0;
			const int k = sp << 3 | tp;
			s_mixm[k].spr = a == A_SPR ? 0xffff : 0;
			s_mixm[k].and_ = a == A_BLACK ? 0 : 0xffff;
			s_mixm[k].or_ = a == A_BLACK ? PEN_BLACK : a == A_SHADOW ? 0x8000 : 0;
		}
	}
	s_mix_simple = simple;
	s_mix_zplain = 1;   // (a pixel without a sprite: its case for priority 0 is the tiles' on every tile bits)
	for (int tp = 0; tp < 8; tp++)
		if (s_act[0][tp] != A_SPR && s_act[0][tp] != A_TILE)
			s_mix_zplain = 0;
	build_sprites(s_snap_spr);
	decoband_run(&s_dband);
	if (s_out)
		rbx_video.out_done = 1;
}

// at the vblank: what render draws (MAME's screen_update before the vblank callback copies the sprite buffer)
static void capture(void)
{
	const unsigned prof = rbx_prof_enter(MS_PA_CAPT);
	rbx_video_wait_idle();
	memcpy(s_snap->tx, s_txram, sizeof(s_snap->tx));
	memcpy(s_snap->bg, s_bgram, sizeof(s_snap->bg));
	memcpy(s_snap->roz, s_rozram, sizeof(s_snap->roz));
	memcpy(s_snap->line, s_lineram, sizeof(s_snap->line));
	memcpy(s_snap->pri, s_priram, sizeof(s_snap->pri));
	memcpy(s_snap->roz_ctrl, s_st.roz_ctrl, sizeof(s_snap->roz_ctrl));
	memcpy(s_snap->tx_scroll, s_st.tx_scroll, sizeof(s_snap->tx_scroll));
	memcpy(s_snap->bg_scroll, s_st.bg_scroll, sizeof(s_snap->bg_scroll));
	s_snap->sprite_ctrl4 = s_st.sprite_ctrl[0x10 / 4];
	s_snap->layout = s_st.layout;
	for (int b = 0; b < 0x8000 / 256; b++)
		if (s_pen_dirty[b]) {
			s_pen_dirty[b] = 0;
			s_snap_pen_dirty[b] = 1;
			memcpy(s_snap_pens + b * 256, s_pens + b * 256, 256 * 4);
		}
	// the buffer drawn is the one copied at the last vblank; this vblank's copy is the next frame's
	uint16_t *t = s_snap_spr;
	s_snap_spr = s_sprbuf;
	s_sprbuf = t;
	memcpy(s_sprbuf, s_sprram, SPR_WORDS * 2);
	rbx_video_frame_done();
	rbx_prof_leave(prof);
}

// ---------------------------------------------------------------------------------------------------- the run
// the V70 up to time t (the programmable timer on the way), then the Z80
static void v_run_to(uint64_t t)
{
	while (s_st.vt < t) {
		if (s_st.prg_at && s_st.prg_at <= s_st.vt) {   // prg_timer_tick
			irq_raise(0, 1);
			s_st.prg_at = s_st.vt + (uint64_t)s_st.interval * (TICKS_HZ / 2000u);
		}
		uint64_t end = t;
		if (s_st.prg_at && s_st.prg_at < end)
			end = s_st.prg_at;
		const uint32_t n = ((uint32_t)(end - s_st.vt) + T_V70 - 1) / T_V70;
		rbx_prof_set(MS_PA_V70);
		s_in_v70 = 1;
		s_v_slice0 = s_st.vt;
		s_spin = 0;
		const int ran = v60_execute((int)n);
		s_in_v70 = 0;
		rbx_prof_set(RBX_PA_DRIVER);
		s_st.vt += (uint64_t)(ran > 0 ? ran : 1) * T_V70 + s_spin;
		if (s_irq_dirty) {
			s_irq_dirty = 0;
			apply_irq();
		}
	}
}

static void ms32_frame(void)
{
	const uint32_t div = s_st.dotclock ? 6 : 8;
	uint32_t htotal = (uint32_t)s_st.hblank + s_st.hdisp, vtotal = (uint32_t)s_st.vblank + s_st.vdisp;
	if (htotal < 64 || htotal > 2048)
		htotal = 384;
	if (vtotal < 32 || vtotal > 1024)
		vtotal = 263;
	const uint32_t line_t = htotal * div * (TICKS_HZ / 48000000u);
	const uint32_t vdisp = s_st.vdisp && s_st.vdisp < vtotal ? s_st.vdisp : 224;
	for (uint32_t line = 0; line < vtotal; line++) {
		const uint64_t lt = s_st.frame_t + (uint64_t)line * line_t;
		v_run_to(lt);
		z_run_to(lt);
		if (line == 0 && (s_st.frame & 1))
			s_invert_lines ? vblank_cb(1) : field_cb(1);
		if (line == vdisp) {
			s_st.frame++;
			capture();
			s_invert_lines ? field_cb(1) : vblank_cb(1);
		}
		// (MAME's 60 kHz quantum: the Z80 and the V70 in quarter lines)
		for (int q = 1; q <= 4; q++) {
			const uint64_t qt = lt + (uint64_t)line_t * (uint32_t)q / 4;
			v_run_to(qt);
			z_run_to(qt);
		}
	}
	if (s_dbg && !(s_st.frame % 60))
		fprintf(stderr, "ms32 f%u pc %08x irq %04x z80 pc %04x crtc %u+%u %u+%u dot %u\n", s_st.frame, v60_pc(),
		        s_st.irqreq, s_z.pc, s_st.hdisp, s_st.hblank, s_st.vdisp, s_st.vblank, s_st.dotclock);
	s_st.frame_t += (uint64_t)vtotal * line_t;
	snd_render_to(s_st.frame_t);
	ymf_flush();
	rbx_audio.frame_mhz = (uint32_t)rbx_div64_small(48000000000ull, (uint64_t)div * htotal * vtotal);
}

// ---------------------------------------------------------------------------------------------------- init / reset
static void ms32_exit(void);

static void map_main(void)
{
	v60_bus_clear();
	v60_bus_set_bits(26);
	v60_bus_map(0x2e00000, 0x2efffff, s_ram, 0x20000, 1, 1);   // fee00000 work RAM (mirror 0x0e0000)
	uint32_t rsz = 0x1000;
	while (rsz < s_rom_size && rsz < 0x200000)
		rsz <<= 1;
	v60_bus_map(0x3e00000, 0x3ffffff, s_rom, rsz, 1, 0);       // ffe00000 ROM
	v60_bus.read16 = main_read16;
	v60_bus.write16 = main_write16;
	v60_bus.irq_cycle = irq_callback;
}

static void ms32_reset(void)
{
	rbx_video_wait_idle();
	memset(s_ram, 0, 0x20000);
	memset(s_nvram, 0, sizeof(s_nvram));
	memset(s_priram, 0, 0x2000);
	memset(s_palram, 0, 0x20000);
	memset(s_rozram, 0, 0x10000);
	memset(s_lineram, 0, 0x1000);
	memset(s_sprram, 0, SPR_WORDS * 2);
	memset(s_txram, 0, 0x4000);
	memset(s_bgram, 0, 0x4000);
	memset(s_sprbuf, 0, SPR_WORDS * 2);
	memset(s_snap_spr, 0, SPR_WORDS * 2);
	memset(s_zram, 0, sizeof(s_zram));
	memset(&s_st, 0, sizeof(s_st));
	// sysctrl device_reset
	s_st.hblank = 64;
	s_st.hdisp = 320;
	s_st.vblank = 39;
	s_st.vdisp = 224;
	s_st.interval = 1;
	// video_start: tp2m32 sets no brightness
	s_st.brt[0] = s_st.brt[1] = 0xffff;
	s_st.brt_r = s_st.brt_g = s_st.brt_b = 0x100;
	s_st.sprite_ctrl[0x10 / 4] = 0x8000;
	s_st.zbank[0] = 0;
	s_st.zbank[1] = 1;
	s_st.snd_acc = TICKS_HZ;   // (the first sample at 1 / 44100 s)
	pens_all();
	zbank_update();
	z_setup();
	ymf271_reset(s_ymf);
	s_mix_n = 0;
	s_irq_dirty = 0;
	v60_init();
	v60_set_pir(0x00007000);   // (the V70)
	v60_reset();
}

// jalcrpt.cpp decrypt_ms32_tx / decrypt_ms32_bg
static int decrypt_tx(uint8_t *src, uint32_t size, int addr_xor, int data_xor)
{
	uint8_t *res = (uint8_t *)malloc(size);
	if (!res)
		return -1;
	addr_xor ^= 0x1005d;
	for (uint32_t i0 = 0; i0 < size; i0++) {
		const uint32_t i = i0 ^ (uint32_t)addr_xor;
		uint32_t j = 0;
		if (i & (1u << 18)) j ^= 0x40000;
		if (i & (1u << 17)) j ^= 0x60000;
		if (i & (1u << 7)) j ^= 0x70000;
		if (i & (1u << 3)) j ^= 0x78000;
		if (i & (1u << 14)) j ^= 0x7c000;
		if (i & (1u << 13)) j ^= 0x7e000;
		if (i & (1u << 0)) j ^= 0x7f000;
		if (i & (1u << 11)) j ^= 0x7f800;
		if (i & (1u << 10)) j ^= 0x7fc00;
		if (i & (1u << 9)) j ^= 0x00200;
		if (i & (1u << 8)) j ^= 0x00300;
		if (i & (1u << 16)) j ^= 0x00380;
		if (i & (1u << 6)) j ^= 0x003c0;
		if (i & (1u << 12)) j ^= 0x003e0;
		if (i & (1u << 4)) j ^= 0x003f0;
		if (i & (1u << 15)) j ^= 0x003f8;
		if (i & (1u << 2)) j ^= 0x003fc;
		if (i & (1u << 1)) j ^= 0x003fe;
		if (i & (1u << 5)) j ^= 0x003ff;
		res[i0] = (uint8_t)((j < size ? src[j] : 0) ^ (i0 & 0xff) ^ (uint32_t)data_xor);
	}
	memcpy(src, res, size);
	free(res);
	return 0;
}
static int decrypt_bg(uint8_t *src, uint32_t size, int addr_xor, int data_xor)
{
	uint8_t *res = (uint8_t *)malloc(size);
	if (!res)
		return -1;
	addr_xor ^= 0xc1c5b;
	for (uint32_t i0 = 0; i0 < size; i0++) {
		uint32_t j = i0 & ~0xfffffu;
		const uint32_t i = i0 ^ (uint32_t)addr_xor;
		if (i & (1u << 19)) j ^= 0x80000;
		if (i & (1u << 8)) j ^= 0xc0000;
		if (i & (1u << 17)) j ^= 0xe0000;
		if (i & (1u << 2)) j ^= 0xf0000;
		if (i & (1u << 15)) j ^= 0xf8000;
		if (i & (1u << 14)) j ^= 0xfc000;
		if (i & (1u << 13)) j ^= 0xfe000;
		if (i & (1u << 12)) j ^= 0xff000;
		if (i & (1u << 1)) j ^= 0xff800;
		if (i & (1u << 10)) j ^= 0xffc00;
		if (i & (1u << 9)) j ^= 0x00200;
		if (i & (1u << 3)) j ^= 0x00300;
		if (i & (1u << 7)) j ^= 0x00380;
		if (i & (1u << 6)) j ^= 0x003c0;
		if (i & (1u << 5)) j ^= 0x003e0;
		if (i & (1u << 4)) j ^= 0x003f0;
		if (i & (1u << 18)) j ^= 0x003f8;
		if (i & (1u << 16)) j ^= 0x003fc;
		if (i & (1u << 11)) j ^= 0x003fe;
		if (i & (1u << 0)) j ^= 0x003ff;
		res[i0] = (uint8_t)((j < size ? src[j] : 0) ^ (i0 & 0xff) ^ (uint32_t)data_xor);
	}
	memcpy(src, res, size);
	free(res);
	return 0;
}

static uint32_t pow2m(uint32_t n) { return n && !(n & (n - 1)) ? n - 1 : 0; }

static int ms32_init(const rbx_game *g)
{
	s_game = g;
	s_noout = getenv("RBX_MS32_NOOUT") != NULL;
	s_dbg = getenv("RBX_MS32_DBG") ? atoi(getenv("RBX_MS32_DBG")) : 0;
	char mname[48];
	snprintf(mname, sizeof(mname), "%s", g->machine);
	char *colon = strchr(mname, ':');
	const char *init = colon ? colon + 1 : "";
	if (colon)
		*colon = 0;
	if (strcmp(mname, "ms32") && strcmp(mname, "ms32_invert_lines")) {
		rbx_fail("%s: machine %s is not emulated", g->set, g->machine);
		return -1;
	}
	s_invert_lines = !strcmp(mname, "ms32_invert_lines");
	int txa, txd, bga, bgd;
	if (!strcmp(init, "ss91022_10")) { txa = 0x00000; txd = 0x35; bga = 0x00000; bgd = 0xa3; }
	else if (!strcmp(init, "ss92046_01")) { txa = 0x00020; txd = 0x7e; bga = 0x00001; bgd = 0x9b; }
	else if (!strcmp(init, "ss92047_01")) { txa = 0x24000; txd = 0x18; bga = 0x24000; bgd = 0x55; }
	else if (!strcmp(init, "ss92048_01")) { txa = 0x20400; txd = 0xd6; bga = 0x20400; bgd = 0xd4; }
	else {
		rbx_fail("%s: init %s is not emulated", g->set, init);
		return -1;
	}

	s_rom = rbx_region("maincpu", &s_rom_size);
	uint32_t rsz, bsz, tsz, ssz;
	uint8_t *roz = rbx_region("roztiles", &rsz), *bg = rbx_region("bgtiles", &bsz), *tx = rbx_region("txtiles", &tsz);
	s_spr_blk = rbx_region_blocks("sprite", &ssz);
	s_zrom = rbx_region("audiocpu", &s_zrom_size);
	uint32_t ysz;
	const uint8_t *yrom = rbx_region("ymf", &ysz);
	if (!s_rom || !roz || !bg || !tx || !s_spr_blk || !s_zrom || !yrom || s_zrom_size < 0x8000)
		return -1;
	if (decrypt_tx(tx, tsz, txa, txd) || decrypt_bg(bg, bsz, bga, bgd)) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	s_roz_gfx = roz; s_roz_n = rsz / 256; s_roz_m = pow2m(s_roz_n);
	s_bg_gfx = bg; s_bg_n = bsz / 256; s_bg_m = pow2m(s_bg_n);
	s_tx_gfx = tx; s_tx_n = tsz / 64; s_tx_m = pow2m(s_tx_n);
	s_spr_count = ssz >> 16;

	s_ram = (uint8_t *)calloc(1, 0x20000);
	s_priram = (uint8_t *)calloc(1, 0x2000);
	s_palram = (uint16_t *)calloc(1, 0x20000);
	s_rozram = (uint16_t *)calloc(1, 0x10000);
	s_lineram = (uint16_t *)calloc(1, 0x1000);
	s_sprram = (uint16_t *)calloc(1, SPR_WORDS * 2);
	s_txram = (uint16_t *)calloc(1, 0x4000);
	s_bgram = (uint16_t *)calloc(1, 0x4000);
	s_sprbuf = (uint16_t *)calloc(1, SPR_WORDS * 2);
	s_snap_spr = (uint16_t *)calloc(1, SPR_WORDS * 2);
	s_pens = (uint32_t *)calloc(0x8000, 4);
	s_snap_pens = (uint32_t *)calloc(0x8000, 4);
	s_snap = calloc(1, sizeof(*s_snap));
	s_tbuf = (uint16_t *)calloc(VIS_W * VIS_H, 2);
	s_sbuf = (uint16_t *)calloc(VIS_W * VIS_H, 2);
	s_pbuf = (uint8_t *)calloc(VIS_W * VIS_H, 1);
	s_list = (spr_t *)malloc(sizeof(spr_t) * (SPR_WORDS / 8));
	s_olut = (uint32_t *)malloc(PENS * 4);
	s_olut_fn = NULL;
	s_olut_bpp = 0;
	s_roz_cls = tile_classes(roz, s_roz_n, 256);
	s_bg_cls = tile_classes(bg, s_bg_n, 256);
	s_tx_cls = tile_classes(tx, s_tx_n, 64);
	s_ymf = ymf271_create(YMF_CLOCK, ymf_mem_r, ymf_mem_w, NULL, ymf_timer, NULL);
	s_mix_cap = 44100 / 30 + 64;
	s_mix = (int32_t *)calloc((size_t)s_mix_cap * 4, sizeof(int32_t));
	if (!s_ram || !s_priram || !s_palram || !s_rozram || !s_lineram || !s_sprram || !s_txram || !s_bgram || !s_sprbuf ||
	    !s_snap_spr || !s_pens || !s_snap_pens || !s_snap || !s_tbuf || !s_sbuf || !s_pbuf || !s_olut ||
	    !s_list || !s_roz_cls || !s_bg_cls || !s_tx_cls ||
	    !s_ymf || !s_mix || rbx_audio_setup_stereo(44100) || rbx_video_setup(VIS_W, VIS_H, PENS)) {
		rbx_fail("%s: out of memory", g->set);
		ms32_exit();
		return -1;
	}
	ymf271_set_rom(s_ymf, yrom, ysz);
	if (!ymf271_set_async(s_ymf, 1, ymf_kick, ymf_sync, NULL))
		rbx_aux_set_job(ymf_job);
	rbx_audio.frame_mhz = 59406;
	rbx_prof_name(RBX_PA_DRV, s_pa_names, MS_PA_END - RBX_PA_DRV);
	for (int i = 0; i < 0x3f; i++)
		s_zmap[i] = s_zrom + i * 256;
	s_zmap[0x3f] = NULL;
	for (int i = 0x40; i < 0x80; i++)
		s_zmap[i] = s_zram + (i - 0x40) * 256;
	s_port_in = rbx_port_find("INPUTS");
	s_port_dsw = rbx_port_find("DSW");
	s_mahjong = 0;
	for (int i = 0; i < 5; i++) {
		char tag[8];
		snprintf(tag, sizeof(tag), "KEY%d", i);
		s_port_key[i] = rbx_port_find(tag);
		if (s_port_key[i] >= 0)
			s_mahjong = 1;
	}
	ms32_reset();
	map_main();
	memset(s_snap_pen_dirty, 1, sizeof(s_snap_pen_dirty));
	return 0;
}

static void ms32_exit(void)
{
	rbx_video_wait_idle();
	rbx_aux_set_job(NULL);
	v60_release();
	if (s_ymf)
		ymf271_destroy(s_ymf);
	s_ymf = NULL;
	free(s_ram); free(s_priram); free(s_palram); free(s_rozram); free(s_lineram); free(s_sprram); free(s_txram);
	free(s_bgram); free(s_sprbuf); free(s_snap_spr); free(s_pens); free(s_snap_pens); free(s_snap); free(s_tbuf);
	free(s_sbuf); free(s_pbuf); free(s_mix);
	free(s_list); free(s_olut); free(s_roz_cls); free(s_bg_cls); free(s_tx_cls);
	s_list = NULL;
	s_olut = NULL;
	s_olut_fn = NULL;
	s_roz_cls = s_bg_cls = s_tx_cls = NULL;
	s_ram = s_priram = s_pbuf = NULL;
	s_palram = s_rozram = s_lineram = s_sprram = s_txram = s_bgram = s_sprbuf = s_snap_spr = s_tbuf = s_sbuf = NULL;
	s_pens = s_snap_pens = NULL;
	s_snap = NULL;
	s_mix = NULL;
	s_spr_blk = NULL;
	s_game = NULL;
}

// ---------------------------------------------------------------------------------------------------- save states
static int ms32_state(void *ctx, int load)
{
	rbx_video_wait_idle();
	v60_state_t vs;
	if (!load) {
		v60_get_state(&vs);
		ymf271_state_save_prep(s_ymf);
	}
	z80 zc = s_z;
	int rc = 0;
	rc |= rbx_state_io(ctx, "v70", &vs, sizeof(vs));
	rc |= rbx_state_io(ctx, "z80", &zc, sizeof(zc));
	rc |= rbx_state_io(ctx, "st", &s_st, sizeof(s_st));
	rc |= rbx_state_io(ctx, "ram", s_ram, 0x20000);
	rc |= rbx_state_io(ctx, "nvram", s_nvram, sizeof(s_nvram));
	rc |= rbx_state_io(ctx, "pri", s_priram, 0x2000);
	rc |= rbx_state_io(ctx, "pal", s_palram, 0x20000);
	rc |= rbx_state_io(ctx, "roz", s_rozram, 0x10000);
	rc |= rbx_state_io(ctx, "line", s_lineram, 0x1000);
	rc |= rbx_state_io(ctx, "spr", s_sprram, SPR_WORDS * 2);
	rc |= rbx_state_io(ctx, "tx", s_txram, 0x4000);
	rc |= rbx_state_io(ctx, "bg", s_bgram, 0x4000);
	rc |= rbx_state_io(ctx, "sprbuf", s_sprbuf, SPR_WORDS * 2);
	rc |= rbx_state_io(ctx, "sprsnap", s_snap_spr, SPR_WORDS * 2);
	rc |= rbx_state_io(ctx, "zram", s_zram, sizeof(s_zram));
	rc |= rbx_state_io(ctx, "ymf", s_ymf, YMF271_STATE_SIZE);
	if (rc)
		return -1;
	if (load) {
		v60_set_state(&vs);
		zc.read_byte = s_z.read_byte;
		zc.write_byte = s_z.write_byte;
		zc.port_in = s_z.port_in;
		zc.port_out = s_z.port_out;
		zc.userdata = s_z.userdata;
		zc.read_map = s_zmap;
		zc.op_rom = s_z.op_rom;
		s_z = zc;
		ymf271_state_loaded(s_ymf);
		zbank_update();
		pens_all();
		s_mix_n = 0;
	}
	return 0;
}
static int ms32_save(void *ctx) { return ms32_state(ctx, 0); }
static int ms32_load(void *ctx) { return ms32_state(ctx, 1); }
static int ms32_supported(const char *machine)
{
	return !strncmp(machine, "ms32:", 5) || !strncmp(machine, "ms32_invert_lines:", 18);   // (not f1superb)
}

static const char *const s_block_regions[] = { "sprite", NULL };
const rbx_driver rbx_drv_ms32 = { "Jaleco Mega System 32", ms32_init, ms32_exit, ms32_reset, ms32_frame, ms32_save,
                                  ms32_load, ms32_supported, ms32_render, ms32_help, NULL, NULL, s_block_regions };
