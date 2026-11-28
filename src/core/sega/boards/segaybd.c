/*  Sega Y-Board: Galaxy Force 2, Power Drift, G-LOC, Rail Chase. Ported from MAME src/mame/sega/segaybd.cpp,
    segaic16.cpp (the rotation), sega16sp.cpp (the Y-Board and System 16B sprites), segaic16_m.cpp, 315_5296.cpp,
    src/devices/machine/msm6253.cpp and src/devices/sound/segapcm.cpp (BSD-3-Clause, copyright-holders: Aaron Giles,
    David Haywood, Hiromitsu Shioya, Olivier Galibert, AJR).
    Three 68000s at 12.5 MHz (main, X, Y), each with a multiplier and a divider; IRQ 2 (line 170), 4 (the vblank, line
    223) and 6 (both) held a line on all three. The X CPU builds the Y-Board sprite list (zooming sprites into a 512x512
    frame), the Y CPU the rotation (that frame turned and scaled onto the screen) and the System 16B sprites over it.
    8192 colours with the shadow / hilight copies. Sound: a Z80 at 4 MHz with a YM2151 and a SegaPCM, the latch on its
    NMI. 342x262 at 60 Hz, 320x224 shown.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../rbx_core.h"
#include "../../cpu/m68k_bus.h"
#include "../../cpu/m68k/m68k.h"
#include "../../cpu/m68k_spin.h"
#include "../../cpu/z80/z80.h"
#include "../../sound/rbx_ym.h"
#include "../../video/decoband.h"
#include "../machine/segaic16m.h"
#include "../machine/segapcm.h"
#include "../machine/sega_zspin.h"

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CPU_CLOCK 12500000         // 50 MHz / 4
#define Z80_CLOCK 4026988          // 32.2159 MHz / 8
#define FRAME_MHZ 60000            // (set_refresh_hz(60), 262 lines)
#define LINES 262
#define VIS_W 320
#define VIS_H 224
#define NPAL 8192
#define PAL_BLACK (2 * NPAL)
#define NCPU 3

// ---- ROMs ----
static uint8_t *s_rom[NCPU], *s_bspr, *s_yspr, *s_zrom, *s_pcmrom;
static uint32_t s_rom_size[NCPU], s_bspr_size, s_yspr_size, s_zrom_size, s_pcm_size;
static int s_port_p1, s_port_general, s_port_limitsw, s_port_dsw, s_port_coinage, s_port_adc[8];
static uint8_t s_adc_def[8], s_adc_type[8];   // (the analog fields' defaults and kinds: 0 none, 1 x, 2 y, 3 pedal, 4 pedal 2,
                                               // 5 a throttle: rbx_throttle)

// ---- the state (saved; on the heap while the game runs, ADR-040) ----
static struct ybd_state {
	uint16_t mainram[0x8000];      // main 0x1f0000
	uint16_t shareram[0x8000];     // all 0x0c0000
	uint16_t xram[0x2000];         // X 0x1f8000
	uint16_t backupram[0x2000];    // X 0x1fc000
	uint16_t ysprites[0x8000];     // X 0x180000
	uint16_t yram[0x8000];         // Y 0x1f0000
	uint16_t rotateram[0x400], rotatebuf[0x400];   // Y 0x180000
	uint16_t bsprites[0x800];      // Y 0x188000
	uint16_t paletteram[0x2000];   // Y 0x190000
	uint16_t mult[NCPU][2], divr[NCPU][8];
	uint8_t io_latch[8], io_dir, io_cnt;
	uint8_t misc_io, adc_shift, latch, latch_pending;
	uint8_t timer_irq, vblank_irq, mute, display_enable;
	uint8_t held[NCPU], sound_reset;   // (the X / Y CPUs and the Z80 held in reset by port E)
	uint8_t adc_val[8];
	uint64_t main_time;
	uint64_t cpu_frac[NCPU], z80_frac, ym_frac;
	uint32_t pcm_phase;
	int32_t pcm_hl, pcm_hr;
	uint8_t state_end;
} *s_bp;
#define b (*s_bp)

static m68k_spin_t s_spin[NCPU];
static int s_spin_on = 1;
static uint8_t s_ctx[NCPU][2048];
static int s_cur;
static uint32_t s_irqp[NCPU];
static uint8_t s_lines[NCPU];   // (the interrupt lines last given to each CPU: bits 2, 4, 6)
static m68k_bus_t s_bus[NCPU];   // (CPU 0 uses m68k_bus)

static z80 s_z80;
static int s_zirq;
static uint8_t s_zram[0x800];
static const uint8_t *s_zmap[256];
static segapcm_t s_pcm;
static int32_t *s_mix_l, *s_mix_r;
static int s_mix_n, s_mix_cap;

// ---- the three 68000s on one Musashi core ----
static void apply_lines(void)   // update_irqs, for the selected CPU
{
	const uint8_t want = (uint8_t)((b.timer_irq ? 4 : 0) | (b.vblank_irq ? 16 : 0) | (b.timer_irq && b.vblank_irq ? 64 : 0));
	if (want == s_lines[s_cur])
		return;
	for (int lv = 2; lv <= 6; lv += 2)
		if (((want ^ s_lines[s_cur]) >> lv) & 1)
			m68k_bus_set_line(lv, (want >> lv) & 1);
	s_lines[s_cur] = want;
}
static void cpu_switch(int n)
{
	if (n == s_cur)
		return;
	s_irqp[s_cur] = m68k_bus_irq_pending();
	m68k_switch_context(s_ctx[s_cur], s_ctx[n]);
	m68k_bus_select(n ? &s_bus[n] : NULL);
	s_cur = n;   // (before the interrupt check: an interrupt taken there runs the hooks for this CPU)
	m68k_bus_cur->irq_pending = s_irqp[n];   // (one interrupt check: a second one right after it changes nothing)
	m68k_bus_update_irq();
}
static uint64_t main_now(void) { return b.main_time + (uint64_t)(s_cur == 0 ? m68k_cycles_run() : 0); }

// ---- sound ----
// The sound board (the Z80, the YM2151's timers and status, the SegaPCM) takes nothing back to the 68000s: the main
// CPU only writes the latch and holds the Z80 in reset. So it runs on a worker (rbx_aux2, ADR-074) from a queue the
// emulation thread fills in order: each line's start and end, the latch's writes (with the main CPU's time in the
// line) and the resets. The worker does them in that order, as the emulation thread did; the frame's end (the mix),
// the save state and the reset wait for it (rbx_aux2_sync). Without a worker thread the kick runs it at once
static unsigned long s_zt0, s_zt1;
static uint64_t s_line_mt;
static uint8_t s_wsres;   // (the worker's sound_reset: b.sound_reset as of the step it is on)
enum { SQ_BEGIN, SQ_LATCH, SQ_RESET, SQ_END };
typedef struct { uint8_t kind, data; uint32_t n; } snd_op;
#define SQ_SIZE 4096
static snd_op s_sq[SQ_SIZE];
static unsigned s_sq_in, s_sq_done;   // (the emulation thread / the worker)
static int s_snd_async;
// the ports the Z80 reads the same all through a run: all but the latch (the YM2151's status changes only with its
// writes and as it is advanced, after the run)
static int z_port_fixed(const z80 *z, uint8_t p) { (void)z; return (p & 0xc0) != 0x40; }
static void z_run(unsigned long end)
{
	if (s_spin_on)
		sega_zspin(&s_z80, end, &s_zirq, z_port_fixed);
	z80_run(&s_z80, end, &s_zirq, 0);
}
static void sound_catchup(uint64_t el)   // (the latch's write is a synchronize(): the Z80 reaches that time first)
{
	if (s_wsres)
		return;
	unsigned long t = s_zt0 + (unsigned long)(el * Z80_CLOCK / CPU_CLOCK);
	if ((long)(t - s_zt1) > 0)
		t = s_zt1;
	if ((long)(t - s_z80.cyc) > 0)
		z_run(t);
}
static void snd_sync(void);
static void snd_job(void);
static void snd_push(uint8_t kind, uint8_t data, uint32_t n)
{
	if (s_sq_in - __atomic_load_n(&s_sq_done, __ATOMIC_ACQUIRE) >= SQ_SIZE)
		snd_sync();   // (full: let the worker catch up)
	snd_op *op = &s_sq[s_sq_in & (SQ_SIZE - 1)];
	op->kind = kind;
	op->data = data;
	op->n = n;
	__atomic_store_n(&s_sq_in, s_sq_in + 1, __ATOMIC_RELEASE);
}
static void latch_w(uint8_t data)   // generic_latch_8: data_pending on the Z80's NMI
{
	snd_push(SQ_LATCH, data, (uint32_t)(main_now() - s_line_mt));
}

static uint8_t z_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0xf000) return a < s_zrom_size ? s_zrom[a] : 0xff;
	if (a < 0xf800) return segapcm_r(&s_pcm, (uint8_t)a);
	return s_zram[a & 0x7ff];
}
static void z_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0xf800) s_zram[a & 0x7ff] = v;
	else if (a >= 0xf000) segapcm_w(&s_pcm, (uint8_t)a, v);
}
static uint8_t z_in(z80 *z, uint8_t p)
{
	(void)z;
	if ((p & 0xc0) == 0x00)
		return rbx_ym_read(0, p & 1);
	if ((p & 0xc0) == 0x40) {
		b.latch_pending = 0;
		return b.latch;
	}
	return 0xff;
}
static void z_out(z80 *z, uint8_t p, uint8_t v)
{
	(void)z;
	if ((p & 0xc0) == 0x00)
		rbx_ym_write(0, p & 1, v);
}
static void ym_irq(int state) { s_zirq = state; }
static void z80_bind(void)
{
	s_z80.read_byte = z_read;
	s_z80.write_byte = z_write;
	s_z80.port_in = z_in;
	s_z80.port_out = z_out;
	s_z80.userdata = NULL;
	s_z80.read_map = s_zmap;
}

static void sound_line_begin(void)
{
	snd_push(SQ_BEGIN, 0, (uint32_t)rbx_line_cycles(&b.z80_frac, Z80_CLOCK, FRAME_MHZ, LINES));
	s_line_mt = b.main_time;
}
static void sound_line(void)
{
	snd_push(SQ_END, 0, (uint32_t)rbx_line_cycles(&b.ym_frac, Z80_CLOCK, FRAME_MHZ, LINES));
	if (s_snd_async)
		rbx_aux2_kick();
	else
		snd_job();
}
// the worker's steps (the line's end: the Z80 to it, the YM2151 and the PCM through it)
static void snd_line_end(uint32_t clocks)
{
	if (!s_wsres && (long)(s_zt1 - s_z80.cyc) > 0)
		z_run(s_zt1);
	const int n = rbx_ym_advance_lr(0, clocks, s_mix_l + s_mix_n, s_mix_r + s_mix_n, s_mix_cap - s_mix_n);
	for (int k = 0; k < n; k++) {   // (the PCM at half the YM's rate: clock / 128 against clock / 64)
		if (!(b.pcm_phase++ & 1)) {
			int32_t l, r;
			segapcm_tick(&s_pcm, &l, &r);
			b.pcm_hl = l * 179 / 256;   // 0.70
			b.pcm_hr = r * 179 / 256;
		}
		s_mix_l[s_mix_n + k] += b.pcm_hl;
		s_mix_r[s_mix_n + k] += b.pcm_hr;
	}
	s_mix_n += n;
}
static void snd_job(void)
{
	const unsigned end = __atomic_load_n(&s_sq_in, __ATOMIC_ACQUIRE);
	unsigned i = s_sq_done;
	for (; i != end; i++) {
		const snd_op *op = &s_sq[i & (SQ_SIZE - 1)];
		switch (op->kind) {
		case SQ_BEGIN:
			s_zt0 = s_z80.cyc;
			s_zt1 = s_zt0 + (unsigned long)op->n;
			break;
		case SQ_LATCH:
			sound_catchup(op->n);
			b.latch = op->data;
			if (!b.latch_pending)
				z80_gen_nmi(&s_z80);
			b.latch_pending = 1;
			break;
		case SQ_RESET:
			if (op->data & 2) {   // (held in reset: started again on release)
				const unsigned long cyc = s_z80.cyc;
				z80_init(&s_z80);
				z80_bind();
				s_z80.cyc = cyc;
			}
			s_wsres = op->data & 1;
			break;
		case SQ_END:
			snd_line_end(op->n);
			break;
		}
	}
	__atomic_store_n(&s_sq_done, i, __ATOMIC_RELEASE);
}
static void snd_sync(void)   // (the board's sound done up to here)
{
	if (s_snd_async)
		rbx_aux2_sync();
	else
		snd_job();
}
static void sound_frame_end(void)
{
	snd_sync();
	rbx_ym_async_take_lr(0, s_mix_l, s_mix_r, s_mix_n);
	if (b.mute)
		for (int i = 0; i < s_mix_n; i++)
			rbx_audio_put_stereo(0, 0);
	else
		for (int i = 0; i < s_mix_n; i++)
			rbx_audio_put_stereo(s_mix_l[i], s_mix_r[i]);
	memset(s_mix_l, 0, sizeof(int32_t) * (size_t)s_mix_n);
	memset(s_mix_r, 0, sizeof(int32_t) * (size_t)s_mix_n);
	s_mix_n = 0;
}

// ---- the 315-5296 I/O chip, the MSM6253 ADC ----
static void misc_output_w(uint8_t data)   // port E
{
	b.display_enable = (data & 0x80) != 0;
	const uint8_t sres = !(data & 0x10), xres = (data & 0x08) != 0, yres = (data & 0x04) != 0;
	if (sres != b.sound_reset)   // (the Z80 reset at its place in the sound's steps, as the worker reaches it)
		snd_push(SQ_RESET, (uint8_t)(sres | (sres && !b.sound_reset ? 2 : 0)), 0);
	b.sound_reset = sres;
	const uint8_t res[3] = { 0, xres, yres };
	for (int i = 1; i < NCPU; i++) {
		if (res[i] && !b.held[i])
			b.held[i] = 2;   // (reset on its next turn, then held)
		else if (!res[i])
			b.held[i] = 0;
	}
	b.misc_io = data;
}
static void io_out_port(int port, uint8_t data)
{
	if (port == 4)
		misc_output_w(data);
	else if (port == 7)   // output2_w: D7 /MUTE
		b.mute = !(data & 0x80);
}
static uint8_t io_r(int offset)
{
	offset &= 0xf;
	if (offset < 8) {
		if ((b.io_dir >> offset) & 1)
			return b.io_latch[offset];
		const int ports[8] = { s_port_p1, s_port_general, s_port_limitsw, -1, -1, s_port_dsw, s_port_coinage, -1 };
		return ports[offset] >= 0 ? (uint8_t)rbx_port_read(ports[offset]) : 0xff;
	}
	switch (offset) {
	case 0x8: return 'S';
	case 0x9: return 'E';
	case 0xa: return 'G';
	case 0xb: return 'A';
	case 0xc: case 0xe: return b.io_cnt;
	case 0xd: case 0xf: return b.io_dir;
	}
	return 0xff;
}
static void io_w(int offset, uint8_t data)
{
	offset &= 0xf;
	if (offset < 8) {
		if ((b.io_dir >> offset) & 1)
			io_out_port(offset, data);
		b.io_latch[offset] = data;
	} else if (offset == 0xe)
		b.io_cnt = data;
	else if (offset == 0xf) {
		for (int i = 0; i < 8; i++)
			if (((b.io_dir ^ data) >> i) & 1)
				io_out_port(i, ((data >> i) & 1) ? b.io_latch[i] : 0);
		b.io_dir = data;
	}
}
static uint8_t adc_input(int ch)   // the MSM6253's inputs: ADC.0-2, then the HC4052 picking ADC.3-6
{
	const int p = ch < 3 ? ch : 3 + (b.misc_io & 3);
	return s_port_adc[p] >= 0 ? b.adc_val[p] : ch < 3 ? 0xff : 0x80;   // (unassigned: 0xff; the mux's read_safe(0x80))
}

// ---- buses ----
static uint16_t common_r16(int cpu, uint32_t a)   // the multiplier and divider of each CPU
{
	if ((a & 0x1fe000) == 0x080000) return sega_mult_r(b.mult[cpu], a >> 1);
	if ((a & 0x1fe000) == 0x084000) return sega_div_r(b.divr[cpu], a >> 1);
	return 0xffff;
}
static int common_w16(int cpu, uint32_t a, uint16_t v, uint16_t mask)
{
	if ((a & 0x1fe000) == 0x080000) sega_mult_w(b.mult[cpu], a >> 1, v, mask);
	else if ((a & 0x1fe000) == 0x084000) sega_div_w(b.divr[cpu], a >> 1, v, mask);
	else return 0;
	return 1;
}

static uint16_t main_r16(uint32_t a)
{
	a &= 0x1fffff;
	if ((a & 0x1fffe0) == 0x100000) return (uint16_t)(0xff00 | io_r((int)(a >> 1)));
	if ((a & 0x1ffff8) == 0x100040) {   // d7_r: the shift register's MSB out
		const uint8_t bit = b.adc_shift >> 7;
		b.adc_shift = (uint8_t)(b.adc_shift << 1);
		return (uint16_t)(0xff00 | bit << 7 | 0x7f);
	}
	return common_r16(0, a);
}
static uint8_t main_r8(uint32_t a)
{
	const uint16_t v = main_r16(a & ~1u);
	return (uint8_t)(a & 1 ? v : v >> 8);
}
static void main_w16m(uint32_t a, uint16_t v, uint16_t mask)
{
	a &= 0x1fffff;
	if (common_w16(0, a, v, mask))
		return;
	if ((a & 0x1fe000) == 0x082000) {
		if (mask & 0xff)
			latch_w((uint8_t)v);
	} else if ((a & 0x1fffe0) == 0x100000) {
		if (mask & 0xff)
			io_w((int)(a >> 1), (uint8_t)v);
	} else if ((a & 0x1ffff8) == 0x100040) {
		if (mask & 0xff)   // address_w: the shift register from input (offset)
			b.adc_shift = adc_input((int)((a >> 1) & 3));
	}
}
static void main_w16(uint32_t a, uint16_t v) { main_w16m(a, v, 0xffff); }
static void main_w8(uint32_t a, uint8_t v)
{
	const int sh = a & 1 ? 0 : 8;
	main_w16m(a & ~1u, (uint16_t)(v << sh), (uint16_t)(0xff << sh));
}

static uint16_t x_r16(uint32_t a) { return common_r16(1, a & 0x1fffff); }
static uint8_t x_r8(uint32_t a)
{
	const uint16_t v = x_r16(a & ~1u);
	return (uint8_t)(a & 1 ? v : v >> 8);
}
static void x_w16m(uint32_t a, uint16_t v, uint16_t mask) { common_w16(1, a & 0x1fffff, v, mask); }
static void x_w16(uint32_t a, uint16_t v) { x_w16m(a, v, 0xffff); }
static void x_w8(uint32_t a, uint8_t v)
{
	const int sh = a & 1 ? 0 : 8;
	x_w16m(a & ~1u, (uint16_t)(v << sh), (uint16_t)(0xff << sh));
}

static uint16_t rotate_control_r(void)   // swap the halves of the rotation RAM
{
	uint32_t *s = (uint32_t *)b.rotateram, *d = (uint32_t *)b.rotatebuf;
	for (int i = 0; i < 0x800 / 4; i++) {
		const uint32_t t = s[i];
		s[i] = d[i];
		d[i] = t;
	}
	return 0xffff;
}
static uint16_t y_r16(uint32_t a)
{
	a &= 0x1fffff;
	if ((a & 0x1f8000) == 0x180000) return b.rotateram[(a >> 1) & 0x3ff];
	if ((a & 0x1f8000) == 0x198000) return rotate_control_r();
	return common_r16(2, a);
}
static uint8_t y_r8(uint32_t a)
{
	const uint16_t v = y_r16(a & ~1u);
	return (uint8_t)(a & 1 ? v : v >> 8);
}
static void y_w16m(uint32_t a, uint16_t v, uint16_t mask)
{
	a &= 0x1fffff;
	if ((a & 0x1f8000) == 0x180000) {
		uint16_t *w = &b.rotateram[(a >> 1) & 0x3ff];
		*w = (uint16_t)((*w & ~mask) | (v & mask));
	} else
		common_w16(2, a, v, mask);
}
static void y_w16(uint32_t a, uint16_t v) { y_w16m(a, v, 0xffff); }
static void y_w8(uint32_t a, uint8_t v)
{
	const int sh = a & 1 ? 0 : 8;
	y_w16m(a & ~1u, (uint16_t)(v << sh), (uint16_t)(0xff << sh));
}
// long words (an even address, both words on one unmapped page): the two word accesses, high word first
// (the reads in order: some have side effects)
static uint32_t main_r32(uint32_t a) { const uint32_t hi = main_r16(a); return hi << 16 | main_r16(a + 2); }
static uint32_t x_r32(uint32_t a) { const uint32_t hi = x_r16(a); return hi << 16 | x_r16(a + 2); }
static uint32_t y_r32(uint32_t a) { const uint32_t hi = y_r16(a); return hi << 16 | y_r16(a + 2); }
static void main_w32(uint32_t a, uint32_t v) { main_w16(a, (uint16_t)(v >> 16)); main_w16(a + 2, (uint16_t)v); }
static void x_w32(uint32_t a, uint32_t v) { x_w16(a, (uint16_t)(v >> 16)); x_w16(a + 2, (uint16_t)v); }
static void y_w32(uint32_t a, uint32_t v) { y_w16(a, (uint16_t)(v >> 16)); y_w16(a + 2, (uint16_t)v); }

// ---- video ----
// (two of each: the render thread draws one while the next frame's is taken, rbx_video_set_depth(2); s_snap is the one
// render draws, picked by rbx_video_render_seq; s_snap_last the one posted last)
typedef struct {
	uint16_t paletteram[0x2000];
	uint16_t ysprites[0x8000];
	uint16_t rotatebuf[0x400];
	uint16_t bsprites[0x800];
	uint8_t display_enable;
	int pal_dirty;
} ybd_snap;
static ybd_snap *s_snapv[2], *s_snap;
static int s_snap_last;
static uint16_t *s_pal_conv;
static int s_pal_valid;
static uint8_t s_pal_normal[32], s_pal_shadow[32], s_pal_hilight[32];
static uint16_t (*s_ybm)[512];   // the Y-Board sprites' frame (512 x 512, 0xffff empty)
static uint16_t (*s_bbm)[VIS_W];   // the 16B sprites (0xffff empty: the mix puts it back)
static int16_t s_ylo[512], s_yhi[512], s_blo[VIS_H], s_bhi[VIS_H];   // each row's columns written (lo > hi: none)
// the frame's rows the screen reads this frame (ysamp_rows): only those drawn. A row not read is left empty, as
// drawn and not read: the screen is the same
static uint8_t s_ysamp[512];
static int s_allrows;   // (RBX_YBD_ALLROWS=1: every row drawn, as before)

static uint64_t *s_yspr64;   // the Y sprite ROM as MAME's big-endian 64-bit words

static void palette_init(void)   // sega_16bit_common_base::palette_init (compute_resistor_weights, 6 resistors)
{
	static const double rn[6] = { 3900, 2000, 1000, 1000.0 / 2, 1000.0 / 4, 0 };
	static const double rs[6] = { 3900, 2000, 1000, 1000.0 / 2, 1000.0 / 4, 470 };
	double wn[6], ws[6];
	for (int set = 0; set < 2; set++) {
		const double *r = set ? rs : rn;
		double *w = set ? ws : wn, max = 0;
		for (int n = 0; n < 6; n++) {
			double r0 = 1.0 / 1e12, r1 = 1.0 / 1e12;
			for (int j = 0; j < 6; j++) {
				if (r[j] == 0)
					continue;
				if (j == n)
					r1 += 1.0 / r[j];
				else
					r0 += 1.0 / r[j];
			}
			r0 = 1.0 / r0;
			r1 = 1.0 / r1;
			double v = 255.0 * r0 / (r1 + r0);
			if (v > 255) v = 255;
			w[n] = r[n] != 0 ? v : 0;
			max += w[n];
		}
		const double scale = 255.0 / max;
		for (int n = 0; n < 6; n++)
			w[n] *= scale;
	}
	for (int v = 0; v < 32; v++) {
		double n = 0, s = 0;
		for (int bit = 0; bit < 5; bit++)
			if ((v >> bit) & 1) {
				n += wn[bit];
				s += ws[bit];
			}
		s_pal_normal[v] = (uint8_t)(int)(n + 0.5);
		s_pal_shadow[v] = (uint8_t)(int)(s + 0.5);
		s_pal_hilight[v] = (uint8_t)(int)(s + ws[5] + 0.5);
	}
}

// a word's 8 nibbles in the other order (nibble j to 7 - j)
static inline uint32_t nib_rev32(uint32_t w)
{
	w = __builtin_bswap32(w);
	return (w >> 4 & 0x0f0f0f0fu) | (w & 0x0f0f0f0fu) << 4;
}

// the Y-Board sprites of the frame's rows [r0, r1) (sega_yboard_sprite_device::draw; origin 0x600, clip 0-511)
static void draw_ysprites(int r0, int r1)
{
	const uint16_t *ram = s_snap->ysprites;
	const uint16_t *rot = s_snap->rotatebuf;   // (rotatebase - yorigin: indexed by the frame's row)
	for (int r = r0; r < r1; r++) {   // (the last frame's columns cleared)
		if (s_ylo[r] <= s_yhi[r])
			memset(s_ybm[r] + s_ylo[r], 0xff, (size_t)(s_yhi[r] - s_ylo[r] + 1) * sizeof(uint16_t));
		s_ylo[r] = 512;
		s_yhi[r] = -1;
	}
	uint8_t visited[0x1000];
	memset(visited, 0, sizeof(visited));
	const uint8_t numbanks = (uint8_t)(s_yspr_size / 0x80000);
	int next = 0;
	for (const uint16_t *data = ram; !(data[0] & 0x8000) && !visited[next]; data = ram + next * 8) {
		const int hide = data[0] & 0x5000;
		const uint16_t *indirect = ram + ((data[0] & 0x7ff) << 4);
		int bank = ((data[1] >> 8) & 0x10) | ((data[2] >> 12) & 0x0f);
		const int xpos = (data[1] & 0xfff) - 0x600;
		const int top = (data[2] & 0xfff) - 0x600;
		uint16_t addr = data[3];
		const int height = data[4];
		const int ydelta = (data[5] & 0x4000) ? 1 : -1;
		const int flip = (~data[5] >> 13) & 1;
		const int xdelta = (data[5] & 0x1000) ? 1 : -1;
		int zoom = data[5] & 0x7ff;
		const int colpri = (data[6] << 1) & 0xfe00;
		const int pitch = (int8_t)data[6];
		visited[next] = 1;
		next = data[7] & 0xfff;
		if (hide || height == 0)
			continue;
		if (numbanks)
			bank %= numbanks;
		const uint64_t *sd = s_yspr64 + 0x10000 * bank;
		if (zoom == 0)
			zoom = 1;
		// (rows outside [r0, r1): nothing to draw)
		if (ydelta > 0 ? (top >= r1 || top + height <= r0) : (top < r0 || top - height >= r1 - 1))
			continue;
		// (each pen's pixel: colpri | indirect, 0xffff for none; no pixel drawn is 0xffff: ind < 0x1fe)
		uint16_t pen[16];
		for (int i = 0; i < 16; i++)
			pen[i] = indirect[i] < 0x1fe ? (uint16_t)(colpri | indirect[i]) : 0xffff;
		const int ytarget = top + ydelta * height;
		int yacc = 0;
		for (int y = top; y != ytarget; y += ydelta) {
			if (ydelta > 0 ? y >= r1 : y < r0)
				break;
			if (y >= 0 && y <= 511) {
				int minx = rot[y & ~1], maxx = rot[y | 1];
				if ((minx & 0x8000) && ydelta < 0)
					break;
				if ((minx & 0x4000) && ydelta > 0)
					break;
				if (!(minx & 0xc000) && y >= r0 && y < r1 && s_ysamp[y]) {
					minx -= 0x600;
					maxx -= 0x600;
					if (minx < 0) minx = 0;
					if (maxx > 511) maxx = 511;
					uint16_t *dest = s_ybm[y];
					int xacc = 0, x = xpos, pix = 0, sk = 0;
					uint16_t offs = flip ? (uint16_t)(addr + 1) : (uint16_t)(addr - 1);
					// (past the clip's far edge nothing more is drawn)
					while ((xdelta > 0 && x <= maxx) || (xdelta < 0 && x >= minx)) {
						const uint64_t pixels = sd[flip ? --offs : ++offs];
						// the word's pens in drawing order, 8 to a half (pen k of the half at bits 28 - 4k): flipped
						// words with their nibbles reversed. 32-bit halves: the ARM's 64-bit shifts take several
						// instructions
						uint32_t hw, lw;
						if (flip) {
							hw = nib_rev32((uint32_t)pixels);
							lw = nib_rev32((uint32_t)(pixels >> 32));
						} else {
							hw = (uint32_t)(pixels >> 32);
							lw = (uint32_t)pixels;
						}
						pix = (int)(lw & 0xf);   // (the word's last pen: 0x0f ends the row)
						if (zoom == 0x200 && (xdelta > 0 ? x >= minx && x + 15 <= maxx : x <= maxx && x - 15 >= minx)) {
							// (1:1 with the 16 pixels within the clip; xacc stays 0)
							uint32_t q = hw;
							for (int k = 0; k < 16; k++, x += xdelta, q <<= 4) {
								if (k == 8)
									q = lw;
								const uint16_t v = pen[q >> 28];
								if (v != 0xffff)
									dest[x] = v;
							}
							if (pix == 0x0f)
								break;
							continue;
						}
						if (zoom >= 0x200) {
							// 1:1 or smaller: a pixel or none per pen. After a pen drawn, xacc + zoom - 0x200 >= 0x200
							// skips that many 0x200s of pens (xacc >> 9, the rest in xacc), across words (sk): pen
							// k * 0x200 + xacc goes up by zoom a pixel. The word's 16 pens looked up first
							uint16_t vals[16];
							for (int k = 0; k < 8; k++, hw <<= 4, lw <<= 4) {
								vals[k] = pen[hw >> 28];
								vals[k + 8] = pen[lw >> 28];
							}
							unsigned pos = (unsigned)sk * 0x200 + (unsigned)xacc;
							const unsigned zu = (unsigned)zoom;
							// (at most 16 pixels a word: all of them within the clip, no test per pixel)
							if (xdelta > 0 ? x >= minx && x + 15 <= maxx : x <= maxx && x - 15 >= minx)
								for (; pos < 16 * 0x200; pos += zu, x += xdelta) {
									const uint16_t v = vals[pos >> 9];
									if (v != 0xffff)
										dest[x] = v;
								}
							else
								for (; pos < 16 * 0x200; pos += zu, x += xdelta) {
									const uint16_t v = vals[pos >> 9];
									if (v != 0xffff && x >= minx && x <= maxx)
										dest[x] = v;
								}
							xacc = (int)(pos & 0x1ff);
							sk = (int)(pos >> 9) - 16;
							if (xdelta > 0 ? x > maxx : x < minx)
								goto row_done;
							if (pix == 0x0f)
								break;
							continue;
						}
						// (enlarged: each pen's run of n pixels filled within the clip)
						uint32_t q = hw;
						for (int k = 0; k < 16; k++, q <<= 4) {
							if (k == 8)
								q = lw;
							const uint16_t v = pen[q >> 28];
							int n = 0;
							while (xacc < 0x200) {
								n++;
								xacc += zoom;
							}
							xacc -= 0x200;
							if (xdelta > 0) {
								if (n && v != 0xffff) {
									int i = x < minx ? minx : x, e = x + n - 1;
									if (e > maxx)
										e = maxx;
									for (; i <= e; i++)
										dest[i] = v;
								}
								x += n;
								if (x > maxx)
									goto row_done;
							} else {
								if (n && v != 0xffff) {
									int i = x - n + 1, e = x > maxx ? maxx : x;
									if (i < minx)
										i = minx;
									for (; i <= e; i++)
										dest[i] = v;
								}
								x -= n;
								if (x < minx)
									goto row_done;
							}
						}
						if (pix == 0x0f)
							break;
					}
				row_done:;
					int lo = xdelta > 0 ? xpos : x, hi = xdelta > 0 ? x : xpos;
					if (lo < minx) lo = minx;
					if (hi > maxx) hi = maxx;
					if (lo <= hi) {
						if (lo < s_ylo[y]) s_ylo[y] = (int16_t)lo;
						if (hi > s_yhi[y]) s_yhi[y] = (int16_t)hi;
					}
				}
			}
			yacc += zoom;
			addr = (uint16_t)(addr + pitch * (yacc >> 9));
			yacc &= 0x1ff;
		}
	}
}

// the System 16B sprites of the screen's rows [y0, y1) (sega_sys16b_sprite_device::draw, origin 184)
static void draw_bsprites(int y0, int y1)
{
	const uint16_t *spritebase = (const uint16_t *)s_bspr;
	const uint8_t numbanks = (uint8_t)(s_bspr_size / 0x20000);
	const uint16_t *ram = s_snap->bsprites;
	for (const uint16_t *data = ram; data < ram + 0x800; data += 8) {
		if (data[2] & 0x8000)
			break;
		const int bottom = data[0] >> 8, top = data[0] & 0xff;
		const int xpos = data[1] & 0x1ff;
		const int hide = data[2] & 0x4000, flip = data[2] & 0x100;
		const int pitch = (int8_t)(data[2] & 0xff);
		uint16_t addr = data[3];
		int bank = (data[4] >> 8) & 0xf;
		const int colpri = ((data[4] & 0xff) << 4) | (((data[1] >> 9) & 0xf) << 12);
		const int vzoom = (data[5] >> 5) & 0x1f, hzoom = data[5] & 0x1f;
		if (hide || top >= bottom || top >= y1 || bottom <= y0)
			continue;
		if (numbanks)
			bank %= numbanks;
		const uint16_t *sd = spritebase + 0x10000 * bank;
		uint16_t zacc = (uint16_t)(data[5] & 0x03ff);
		for (int y = top; y < bottom; y++) {
			addr = (uint16_t)(addr + pitch);
			zacc = (uint16_t)(zacc + (vzoom << 10));
			if (zacc & 0x8000) {
				addr = (uint16_t)(addr + pitch);
				zacc &= ~0x8000;
			}
			if (y < y0 || y >= y1)
				continue;
			uint16_t *dest = s_bbm[y];
			int xacc = 4 * hzoom, x;
			uint16_t a7;
			if (!flip) {
				a7 = (uint16_t)(addr - 1);
				for (x = xpos; ((xpos - x) & 0x1ff) != 1;) {
					const uint16_t pixels = sd[++a7];
					int pix = 0;
					for (int sh = 12; sh >= 0; sh -= 4) {
						pix = (pixels >> sh) & 15;
						xacc = (xacc & 0x3f) + hzoom;
						if (xacc < 0x40) {
							const int sx = x - 184;
							if (sx >= 0 && sx < VIS_W && pix != 0 && pix != 15)
								dest[sx] = (uint16_t)(colpri | pix);
							x++;
						}
					}
					if (pix == 15 || x - 184 >= VIS_W)   // (past the screen's right edge nothing more is drawn)
						break;
				}
			} else {
				a7 = (uint16_t)(addr + 1);
				for (x = xpos; ((xpos - x) & 0x1ff) != 1;) {
					const uint16_t pixels = sd[--a7];
					int pix = 0;
					for (int sh = 0; sh <= 12; sh += 4) {
						pix = (pixels >> sh) & 15;
						xacc = (xacc & 0x3f) + hzoom;
						if (xacc < 0x40) {
							const int sx = x - 184;
							if (sx >= 0 && sx < VIS_W && pix != 0 && pix != 15)
								dest[sx] = (uint16_t)(colpri | pix);
							x++;
						}
					}
					if (pix == 15 || x - 184 >= VIS_W)   // (past the screen's right edge nothing more is drawn)
						break;
				}
			}
			int lo = xpos - 184, hi = x - 184;
			if (lo < 0) lo = 0;
			if (hi > VIS_W - 1) hi = VIS_W - 1;
			if (lo <= hi) {
				if (lo < s_blo[y]) s_blo[y] = (int16_t)lo;
				if (hi > s_bhi[y]) s_bhi[y] = (int16_t)hi;
			}
		}
	}
}

// two phases of bands: the sprites' frame (512 rows), then the screen (rotate_draw and the 16B sprites mixed in, and
// the rows written to the frontend's frame). The frame's sprites crowd a few rows (pdrift: a quarter of the rows held
// half the work), so its bands are smaller
#define BANDS 8
#define YBANDS 32
static int s_phase;
static void band_out(int y0, int y1);
static int s_out;
static void ybd_band(int band)
{
	if (s_phase == 0) {
		draw_ysprites(512 * band / YBANDS, 512 * (band + 1) / YBANDS);
		return;
	}
	const int y0 = VIS_H * band / BANDS, y1 = VIS_H * (band + 1) / BANDS;
	draw_bsprites(y0, y1);
	const uint16_t *rb = s_snap->rotatebuf;
	int32_t currx = (int32_t)((uint32_t)rb[0x3f0] << 16 | rb[0x3f1]);
	int32_t curry = (int32_t)((uint32_t)rb[0x3f2] << 16 | rb[0x3f3]);
	const int32_t dyy = (int32_t)((uint32_t)rb[0x3f4] << 16 | rb[0x3f5]);
	const int32_t dxx = (int32_t)((uint32_t)rb[0x3f6] << 16 | rb[0x3f7]);
	const int32_t dxy = (int32_t)((uint32_t)rb[0x3f8] << 16 | rb[0x3f9]);
	const int32_t dyx = (int32_t)((uint32_t)rb[0x3fa] << 16 | rb[0x3fb]);
	currx += dxx * 27 + dxy * y0;   // (cliprect.min_x + 27; the band's first row)
	curry += dyx * 27 + dyy * y0;
	uint8_t pl[VIS_W];   // (the Y sprites' priority: 0x1f where none)
	for (int y = y0; y < y1; y++) {
		uint16_t *d = rbx_video.bitmap + y * rbx_video.pitch;
		int32_t tx = currx, ty = curry;
		const int sy0 = (ty >> 14) & 0x1ff;
		if (dyx == 0 && s_ylo[sy0] > s_yhi[sy0]) {   // (a row of the frame with no sprite: its scanline colour)
			for (int x = 0; x < VIS_W; x++)
				d[x] = (uint16_t)sy0;
			memset(pl, 0x1f, sizeof(pl));
		} else if (dyx == 0) {   // (one row of the frame)
			const uint16_t *row = s_ybm[sy0];
			for (int x = 0; x < VIS_W; x++) {
				const uint16_t pix = row[(tx >> 14) & 0x1ff];
				if (pix != 0xffff) {
					d[x] = (uint16_t)((pix & 0x1ff) | ((pix >> 6) & 0x200) | ((pix >> 3) & 0xc00) | 0x1000);
					pl[x] = (uint8_t)(((pix >> 8) | 1) & 0x1f);
				} else {
					d[x] = (uint16_t)sy0;
					pl[x] = 0x1f;
				}
				tx += dxx;
			}
		} else
			for (int x = 0; x < VIS_W; x++) {
				const int sx = (tx >> 14) & 0x1ff, sy = (ty >> 14) & 0x1ff;
				const uint16_t pix = s_ybm[sy][sx];
				if (pix != 0xffff) {
					d[x] = (uint16_t)((pix & 0x1ff) | ((pix >> 6) & 0x200) | ((pix >> 3) & 0xc00) | 0x1000);
					pl[x] = (uint8_t)(((pix >> 8) | 1) & 0x1f);
				} else {
					d[x] = (uint16_t)sy;   // (the scanline colour: colorbase 0)
					pl[x] = 0x1f;
				}
				tx += dxx;
				ty += dyx;
			}
		// the 16B sprites (and the row put back to none)
		const int lo = s_blo[y], hi = s_bhi[y];
		if (lo <= hi) {
			uint16_t *sp = s_bbm[y];
			for (int x = lo; x <= hi; x++) {
				const uint16_t bp = sp[x];
				if (bp == 0xffff)
					continue;
				sp[x] = 0xffff;
				if (((bp >> 11) & 0x1e) < pl[x]) {
					if ((bp & 0xf) == 0xe)
						d[x] = (uint16_t)(d[x] + NPAL);
					else
						d[x] = (uint16_t)(0x800 | (bp & 0x7ff));
				}
			}
			s_blo[y] = VIS_W;
			s_bhi[y] = -1;
		}
		currx += dxy;
		curry += dyy;
	}
	if (s_out)
		band_out(y0, y1);
}
static decoband_t s_bands = { 0, 0, 0, 0, BANDS, ybd_band };
static int ybd_help(int helper) { return decoband_help(&s_bands, helper); }

// the frame's output written by the bands (as segas24.c, ADR-066): the palette converted to the frontend's colour (s_olut,
// the entries that changed, as video_out's table) and each band's rows written with it. The same pixels as video_out
static uint32_t *s_olut, *s_olut_src;
static uint32_t (*s_olut_fn)(int, int, int);   // (no parameter names: b is the board macro)
static int s_olut_bpp, s_olut_stale;
static void band_out(int y0, int y1)
{
	const int mask = rbx_video.palette_size - 1;
	for (int y = y0; y < y1; y++) {
		const uint16_t *sp = rbx_video.bitmap + y * rbx_video.pitch;
		uint8_t *op = (uint8_t *)rbx_video.out + (size_t)y * rbx_video.out_pitch;
		if (s_olut_bpp == 2) {
			uint16_t *d = (uint16_t *)op;
			for (int x = 0; x < VIS_W; x++)
				d[x] = (uint16_t)s_olut[sp[x] & mask];
		} else {
			uint32_t *d = (uint32_t *)op;
			for (int x = 0; x < VIS_W; x++)
				d[x] = s_olut[sp[x] & mask];
		}
	}
}
static void out_table(void)   // (video_out's table: the entries that changed, all of them for another format)
{
	const int all = s_olut_fn != rbx_video.out_color || s_olut_bpp != rbx_video.out_bpp;
	if (!all && !s_olut_stale)   // (no palette entry changed since the last table)
		return;
	uint32_t (*fn)(int, int, int) = rbx_video.out_color;
	for (int i = 0; i < rbx_video.palette_size; i++) {
		const uint32_t c = rbx_video.palette[i];
		if (!all && s_olut_src[i] == c)
			continue;
		s_olut_src[i] = c;
		s_olut[i] = fn((int)(c >> 16) & 0xff, (int)(c >> 8) & 0xff, (int)c & 0xff);
	}
	s_olut_fn = fn;
	s_olut_bpp = rbx_video.out_bpp;
	s_olut_stale = 0;
}

// the rows of the sprites' frame the screen reads (ybd_band's walk): without the rotation's dyx each screen row reads
// one row of the frame, (curry >> 14) & 0x1ff; with it, any row (all of them drawn)
static void ysamp_rows(void)
{
	const uint16_t *rb = s_snap->rotatebuf;
	const uint32_t curry = (uint32_t)rb[0x3f2] << 16 | rb[0x3f3];
	const uint32_t dyy = (uint32_t)rb[0x3f4] << 16 | rb[0x3f5];
	const uint32_t dyx = (uint32_t)rb[0x3fa] << 16 | rb[0x3fb];
	if (dyx || s_allrows) {
		memset(s_ysamp, 1, sizeof(s_ysamp));
		return;
	}
	memset(s_ysamp, 0, sizeof(s_ysamp));
	for (int y = 0; y < VIS_H; y++)   // (ybd_band: curry + dyx * 27 + dyy * y, wrapping as the int32 sums)
		s_ysamp[((curry + dyy * (uint32_t)y) >> 14) & 0x1ff] = 1;
}

static void ybd_render(void)
{
	s_snap = s_snapv[rbx_video_render_seq() & 1];
	if (s_snap->pal_dirty || !s_pal_valid) {
		for (int i = 0; i < NPAL; i++) {
			const uint16_t v = s_snap->paletteram[i];
			if (s_pal_valid && s_pal_conv[i] == v)
				continue;
			s_pal_conv[i] = v;
			const int r = ((v >> 12) & 1) | ((v << 1) & 0x1e), g = ((v >> 13) & 1) | ((v >> 3) & 0x1e),
			          bb = ((v >> 14) & 1) | ((v >> 7) & 0x1e);
			rbx_video.palette[i] = (uint32_t)(s_pal_normal[r] << 16 | s_pal_normal[g] << 8 | s_pal_normal[bb]);
			rbx_video.palette[i + NPAL] = (v & 0x8000)
				? (uint32_t)(s_pal_hilight[r] << 16 | s_pal_hilight[g] << 8 | s_pal_hilight[bb])
				: (uint32_t)(s_pal_shadow[r] << 16 | s_pal_shadow[g] << 8 | s_pal_shadow[bb]);
		}
		rbx_video.palette[PAL_BLACK] = 0;
		s_pal_valid = 1;
		s_snap->pal_dirty = 0;
		s_olut_stale = 1;
	}
	s_out = 0;
	if (!s_snap->display_enable) {   // (the core converts it)
		for (int y = 0; y < VIS_H; y++)
			for (int x = 0; x < VIS_W; x++)
				rbx_video.bitmap[y * rbx_video.pitch + x] = PAL_BLACK;
		return;
	}
	s_out = rbx_video.out && !rbx_video.blank && s_olut && (rbx_video.out_bpp == 2 || rbx_video.out_bpp == 4);
	if (s_out)
		out_table();
	ysamp_rows();
	s_phase = 0;
	s_bands.jobs = YBANDS;
	decoband_run(&s_bands);
	s_phase = 1;
	s_bands.jobs = BANDS;
	decoband_run(&s_bands);
	if (s_out)
		rbx_video.out_done = 1;
}

// The 16B sprite chip writes back while it draws (MAME's draw at the vblank): each entry's +E gets the address it
// reached and +A its row zoom accumulator. The games read them (Power Drift hides its text without), so the
// emulation thread steps the same addresses here, without the pixels (the drawing itself uses the snapshot)
static void bsprites_writeback(void)
{
	const uint16_t *spritebase = (const uint16_t *)s_bspr;
	const uint8_t numbanks = (uint8_t)(s_bspr_size / 0x20000);
	uint16_t *ram = b.bsprites;
	for (uint16_t *data = ram; data < ram + 0x800; data += 8) {
		if (data[2] & 0x8000)
			break;
		const int bottom = data[0] >> 8, top = data[0] & 0xff;
		const int xpos = data[1] & 0x1ff;
		const int hide = data[2] & 0x4000, flip = data[2] & 0x100;
		const int pitch = (int8_t)(data[2] & 0xff);
		uint16_t addr = data[3];
		int bank = (data[4] >> 8) & 0xf;
		const int vzoom = (data[5] >> 5) & 0x1f, hzoom = data[5] & 0x1f;
		data[7] = addr;
		if (hide || top >= bottom)
			continue;
		if (numbanks)
			bank %= numbanks;
		const uint16_t *sd = spritebase + 0x10000 * bank;
		data[5] &= 0x03ff;
		for (int y = top; y < bottom; y++) {
			addr = (uint16_t)(addr + pitch);
			data[5] = (uint16_t)(data[5] + (vzoom << 10));
			if (data[5] & 0x8000) {
				addr = (uint16_t)(addr + pitch);
				data[5] &= ~0x8000;
			}
			if (y > VIS_H - 1)
				continue;
			int xacc = 4 * hzoom, x = xpos;
			data[7] = flip ? (uint16_t)(addr + 1) : (uint16_t)(addr - 1);
			while (((xpos - x) & 0x1ff) != 1) {
				const uint16_t pixels = sd[flip ? --data[7] : ++data[7]];
				int pix = 0;
				for (int k = 0; k < 4; k++) {
					pix = (pixels >> (flip ? 4 * k : 12 - 4 * k)) & 15;
					xacc = (xacc & 0x3f) + hzoom;
					if (xacc < 0x40)
						x++;
				}
				if (pix == 15)
					break;
			}
		}
	}
}

static void vblank_draw(void)
{
	rbx_video_wait_slot();   // (a free snapshot: the frame before this one may still be being drawn)
	const int slot = (int)(rbx_video_post_seq() & 1);
	ybd_snap *const sn = s_snapv[slot];
	// (changed against the frame posted last, whose palette render converts last)
	const int dirty = memcmp(s_snapv[s_snap_last]->paletteram, b.paletteram, sizeof(b.paletteram)) != 0;
	if (dirty || (slot != s_snap_last && memcmp(sn->paletteram, b.paletteram, sizeof(b.paletteram))))
		memcpy(sn->paletteram, b.paletteram, sizeof(b.paletteram));
	sn->pal_dirty = dirty;
	memcpy(sn->ysprites, b.ysprites, sizeof(b.ysprites));
	memcpy(sn->rotatebuf, b.rotatebuf, sizeof(b.rotatebuf));
	memcpy(sn->bsprites, b.bsprites, sizeof(b.bsprites));
	sn->display_enable = b.display_enable;
	s_snap_last = slot;
	rbx_video_frame_done();
	if (b.display_enable)   // (MAME's screen_update returns early when the display is off: no drawing)
		bsprites_writeback();
}

// ---- the analog controls: d-pad / stick moving them, back to the centre when let go ----
static void analog_frame(void)
{
	for (int i = 0; i < 8; i++) {
		const int p = s_port_adc[i];
		if (p < 0)
			continue;
		const int def = s_adc_def[i];
		if (s_adc_type[i] == 5) {   // (the throttle lever: the general inputs' keys, held where they leave it; MINMAX 1..)
			const int v = rbx_throttle();
			b.adc_val[i] = (uint8_t)(v < 1 ? 1 : v);
			continue;
		}
		if (s_adc_type[i] >= 3) {   // pedals (MAME's keys for them: button 1 gas, 2 brake, KEYDELTA 20 / 40 a frame;
			                        // the stick up presses the gas too), back to rest when let go
			const int gas = s_adc_type[i] == 3, step = gas ? 20 : 40;
			const int on = (rbx_input_bits(1) & (gas ? RBX_IN_B1 : RBX_IN_B2)) || (gas && rbx_analog(1, RBX_IPT_PEDAL) > 2);
			int v = b.adc_val[i] + (on ? step : -step);
			v = v < def ? def : v > 255 ? 255 : v;
			b.adc_val[i] = (uint8_t)v;
			continue;
		}
		const int type = s_adc_type[i] == 1 ? RBX_IPT_AD_STICK_X : s_adc_type[i] == 2 ? RBX_IPT_AD_STICK_Y : -1;
		if (type < 0) {
			b.adc_val[i] = (uint8_t)def;
			continue;
		}
		const int d = rbx_analog_dpad(1, type, 4);
		int v = b.adc_val[i];
		if (d)
			v += s_adc_type[i] == 2 ? -d : d;
		else if (v < def)
			v = v + 4 > def ? def : v + 4;
		else if (v > def)
			v = v - 4 < def ? def : v - 4;
		if (v < 1) v = 1;
		if (v > 255) v = 255;
		b.adc_val[i] = (uint8_t)v;
	}
}

// ---- init / frame ----
static void ybd_reset(void)
{
	snd_sync();
	memset(s_bp, 0, sizeof(*s_bp));
	s_wsres = 0;
	for (int i = 0; i < 8; i++)
		b.adc_val[i] = s_adc_def[i];
	memset(s_zram, 0, sizeof(s_zram));
	memset(s_pcm.v, 0, sizeof(s_pcm.v));
	memset(s_pcm.ram, 0xff, sizeof(s_pcm.ram));
	z80_init(&s_z80);
	z80_bind();
	s_zirq = 0;
	rbx_ym_reset(0);
	s_mix_n = 0;
	for (int i = 0; i < NCPU; i++)
		m68k_spin_clear(&s_spin[i]);
	// the CPUs from the last, CPU 0 left selected
	m68k_bus_select(NULL);
	s_cur = 0;
	for (int i = NCPU - 1; i >= 0; i--) {
		m68k_bus_select(i ? &s_bus[i] : NULL);
		m68k_bus_irq_reset();
		m68k_bus_cur->irq_asserted = 0;
		m68k_pulse_reset();
		if (i)
			m68k_get_context(s_ctx[i]);
		s_irqp[i] = 0;
		s_lines[i] = 0;
	}
	s_pal_valid = 0;
}

static void ybd_exit(void);

static int ybd_init(const rbx_game *g)
{
	if (!s_bp && !(s_bp = calloc(1, sizeof(*s_bp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	static const char *const cpus[NCPU] = { "maincpu", "subx", "suby" };
	for (int i = 0; i < NCPU; i++)
		s_rom[i] = rbx_region(cpus[i], &s_rom_size[i]);
	s_bspr = rbx_region("bsprites", &s_bspr_size);
	s_yspr = rbx_region("ysprites", &s_yspr_size);
	s_zrom = rbx_region("soundcpu", &s_zrom_size);
	s_pcmrom = rbx_region("pcm", &s_pcm_size);
	if (!s_rom[0] || s_rom_size[0] < 0x80000 || !s_rom[1] || s_rom_size[1] < 0x40000 || !s_rom[2] || s_rom_size[2] < 0x40000 ||
	    !s_bspr || !s_yspr || !s_zrom || !s_pcmrom) {
		rbx_fail("%s: ROM regions missing", g->set);
		ybd_exit();
		return -1;
	}
	for (int i = 0; i < NCPU; i++)
		m68k_bus_swap16(s_rom[i], s_rom_size[i]);
	m68k_bus_swap16(s_bspr, s_bspr_size);   // (ROM_REGION16_BE)
	s_yspr64 = (uint64_t *)malloc(s_yspr_size);   // (ROM_REGION64_BE: the first byte the top one)
	s_snapv[0] = calloc(1, sizeof(ybd_snap));
	s_snapv[1] = calloc(1, sizeof(ybd_snap));
	s_snap = s_snapv[0];
	s_snap_last = 0;
	s_pal_conv = (uint16_t *)calloc(NPAL, sizeof(uint16_t));
	s_ybm = (uint16_t (*)[512])malloc(512 * 512 * sizeof(uint16_t));
	s_bbm = (uint16_t (*)[VIS_W])malloc(VIS_H * VIS_W * sizeof(uint16_t));
	if (!s_yspr64 || !s_snapv[0] || !s_snapv[1] || !s_pal_conv || !s_ybm || !s_bbm) {
		rbx_fail("%s: out of memory", g->set);
		ybd_exit();
		return -1;
	}
	memset(s_ybm, 0xff, 512 * 512 * sizeof(uint16_t));
	memset(s_bbm, 0xff, VIS_H * VIS_W * sizeof(uint16_t));
	for (int r = 0; r < 512; r++) {
		s_ylo[r] = 512;
		s_yhi[r] = -1;
	}
	for (int y = 0; y < VIS_H; y++) {
		s_blo[y] = VIS_W;
		s_bhi[y] = -1;
	}
	for (uint32_t i = 0; i < s_yspr_size / 8; i++) {
		uint64_t v = 0;
		for (int k = 0; k < 8; k++)
			v = v << 8 | s_yspr[i * 8 + (uint32_t)k];
		s_yspr64[i] = v;
	}
	palette_init();
	{
		uint32_t p2 = 1;
		while (p2 < s_pcm_size)
			p2 <<= 1;
		s_pcm.rom = s_pcmrom;
		s_pcm.mask = p2 - 1;
		s_pcm.bankshift = 13;   // (BANK_12M | BANK_MASKF8)
		s_pcm.bankmask = 0xf8;
	}
	s_port_p1 = rbx_port_find("P1");
	s_port_general = rbx_port_find("GENERAL");
	s_port_limitsw = rbx_port_find("LIMITSW");
	s_port_dsw = rbx_port_find("DSW");
	s_port_coinage = rbx_port_find("COINAGE");
	for (int i = 0; i < 8; i++) {
		char tag[8];
		snprintf(tag, sizeof(tag), "ADC.%d", i);
		s_port_adc[i] = rbx_port_find(tag);
		s_adc_def[i] = 0x80;
		s_adc_type[i] = 0;
		for (const rbx_ipt_port *pt = g->ports; pt && pt->tag; pt++)
			if (!strcmp(pt->tag, tag) && pt->count) {   // (the field's own default: MAME's analog value at rest)
				const rbx_ipt_field *f = &g->fields[pt->first];
				s_adc_def[i] = (uint8_t)f->def;
				s_adc_type[i] = f->type == RBX_IPT_AD_STICK_X || f->type == RBX_IPT_PADDLE ? 1
				              : f->type == RBX_IPT_AD_STICK_Y ? 2 : f->type == RBX_IPT_PEDAL ? 3 : f->type == RBX_IPT_PEDAL2 ? 4
				              : f->type == RBX_IPT_AD_STICK_Z ? 5 : 0;
			}
	}
	// sound: YM2151 0.30 (L / R), the SegaPCM 0.70
	rbx_ym_config yc;
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM2151;
	yc.clock = Z80_CLOCK;
	yc.gain[0] = yc.gain[1] = 77;
	yc.irq = ym_irq;
	if (rbx_ym_init(0, &yc) || rbx_ym_set_async(0, 2)) {
		rbx_fail("%s: out of memory", g->set);
		ybd_exit();
		return -1;
	}
	s_mix_cap = (int)(rbx_ym_rate(0) / 50) + 256;
	s_mix_l = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	s_mix_r = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	if (!s_mix_l || !s_mix_r || rbx_audio_setup_stereo(rbx_ym_rate(0)) || rbx_video_setup(VIS_W, VIS_H, PAL_BLACK + 1)) {
		rbx_fail("%s: out of memory", g->set);
		ybd_exit();
		return -1;
	}
	s_olut = (uint32_t *)calloc((size_t)rbx_video.palette_size, sizeof(uint32_t));
	s_olut_src = (uint32_t *)calloc((size_t)rbx_video.palette_size, sizeof(uint32_t));
	s_olut_fn = NULL;
	s_olut_bpp = 0;
	s_olut_stale = 1;
	if (!s_olut || !s_olut_src) {
		rbx_fail("%s: out of memory", g->set);
		ybd_exit();
		return -1;
	}
	memset(s_zmap, 0, sizeof(s_zmap));
	for (uint32_t p = 0; p < 0xf0 && p * 256 + 256 <= s_zrom_size; p++)
		s_zmap[p] = s_zrom + p * 256;
	for (uint32_t p = 0; p < 8; p++)
		s_zmap[0xf8 + p] = s_zram + p * 256;
	if (m68k_context_size() > sizeof(s_ctx[0])) {
		rbx_fail("%s: CPU context too large", g->set);
		ybd_exit();
		return -1;
	}
	s_spin_on = getenv("RBX_NOSPIN") == NULL;
	m68k_init();
	m68k_set_cpu_type(M68K_CPU_TYPE_68000);
	// the maps (a 21-bit bus: mirrored every 2 MB)
	for (int i = 0; i < NCPU; i++) {
		m68k_bus_select(i ? &s_bus[i] : NULL);
		m68k_bus_reset_map();
		for (uint32_t mir = 0; mir < 0x1000000; mir += 0x200000) {
			m68k_bus_map(mir + 0x000000, mir + (i ? 0x03ffff : 0x07ffff), s_rom[i], i ? 0x40000 : 0x80000, 1, 0);
			m68k_bus_map(mir + 0x0c0000, mir + 0x0cffff, b.shareram, sizeof(b.shareram), 1, 1);
			if (i == 0)
				m68k_bus_map(mir + 0x1f0000, mir + 0x1fffff, b.mainram, sizeof(b.mainram), 1, 1);
			else if (i == 1) {
				m68k_bus_map(mir + 0x180000, mir + 0x18ffff, b.ysprites, sizeof(b.ysprites), 1, 1);
				m68k_bus_map(mir + 0x1f8000, mir + 0x1fbfff, b.xram, sizeof(b.xram), 1, 1);
				m68k_bus_map(mir + 0x1fc000, mir + 0x1fffff, b.backupram, sizeof(b.backupram), 1, 1);
			} else {
				// (the rotation RAM, 2 KB mirrored through 0x180000-0x187fff, smaller than a page: in y_r16 / y_w16m,
				// so that no bus has partial ranges and the handlers' accesses go straight to them)
				m68k_bus_map(mir + 0x188000, mir + 0x18ffff, b.bsprites, sizeof(b.bsprites), 1, 1);
				m68k_bus_map(mir + 0x190000, mir + 0x197fff, b.paletteram, sizeof(b.paletteram), 1, 1);
				m68k_bus_map(mir + 0x1f0000, mir + 0x1fffff, b.yram, sizeof(b.yram), 1, 1);
			}
		}
		m68k_bus_cur->read8 = i == 0 ? main_r8 : i == 1 ? x_r8 : y_r8;
		m68k_bus_cur->read16 = i == 0 ? main_r16 : i == 1 ? x_r16 : y_r16;
		m68k_bus_cur->write8 = i == 0 ? main_w8 : i == 1 ? x_w8 : y_w8;
		m68k_bus_cur->write16 = i == 0 ? main_w16 : i == 1 ? x_w16 : y_w16;
		// (the multiplier's and divider's long words in one call, high word first as two word accesses do)
		m68k_bus_cur->read32 = i == 0 ? main_r32 : i == 1 ? x_r32 : y_r32;
		m68k_bus_cur->write32 = i == 0 ? main_w32 : i == 1 ? x_w32 : y_w32;
	}
	m68k_bus_select(NULL);
	s_sq_in = s_sq_done = 0;
	s_snd_async = 0;
	ybd_reset();
	// the sound board on a worker (ADR-074); RBX_YBD_SND_SYNC=1: on the emulation thread at each line's end
	if (!getenv("RBX_YBD_SND_SYNC")) {
		s_snd_async = 1;
		rbx_aux2_set_job(snd_job);
	}
	s_allrows = getenv("RBX_YBD_ALLROWS") != NULL;
	// the next frame's snapshot taken while this one is drawn (ADR-071); RBX_YBD_DEPTH=1: the vblank waits for the
	// drawing as before
	if (!getenv("RBX_YBD_DEPTH") || atoi(getenv("RBX_YBD_DEPTH")) != 1)
		rbx_video_set_depth(2);
	return 0;
}

static void ybd_exit(void)
{
	if (s_snd_async)
		rbx_aux2_set_job(NULL);   // (after its last steps)
	s_snd_async = 0;
	for (int i = 0; i < NCPU; i++)
		m68k_spin_free(&s_spin[i]);
	m68k_bus_select(NULL);
	s_cur = 0;
	rbx_ym_exit(0);
	free(s_mix_l);
	free(s_mix_r);
	s_mix_l = s_mix_r = NULL;
	free(s_yspr64);
	s_yspr64 = NULL;
	free(s_snapv[0]);
	free(s_snapv[1]);
	s_snapv[0] = s_snapv[1] = s_snap = NULL;
	free(s_olut);
	free(s_olut_src);
	s_olut = s_olut_src = NULL;
	free(s_pal_conv);
	s_pal_conv = NULL;
	free(s_ybm);
	s_ybm = NULL;
	free(s_bbm);
	s_bbm = NULL;
	free(s_bp);
	s_bp = NULL;
}

static int run_cpu(int i, int budget)
{
	cpu_switch(i);
	apply_lines();
	if (b.held[i] == 2) {   // (port E's reset: from its vectors, then held)
		m68k_pulse_reset();
		b.held[i] = 1;
	}
	if (b.held[i])
		return budget;
	return s_spin_on ? m68k_spin_execute(&s_spin[i], budget, 0) : m68k_execute(budget);
}

static void ybd_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	analog_frame();
	for (int line = 0; line < LINES; line++) {
		if (line == VIS_H)   // the frame drawn at the vblank's start
			vblank_draw();
		if (line == 170 || line == 171)   // irq2_gen_tick
			b.timer_irq = line == 170;
		else if (line == 223 || line == 224)
			b.vblank_irq = line == 223;
		sound_line_begin();
		// the three CPUs through the line, the X and Y CPUs kept level with the main one
		int left[NCPU];
		for (int i = 0; i < NCPU; i++)
			left[i] = rbx_line_cycles(&b.cpu_frac[i], CPU_CLOCK, FRAME_MHZ, LINES);
		const int e = run_cpu(0, left[0]);
		b.main_time += (uint64_t)e;
		for (int i = 1; i < NCPU; i++)
			run_cpu(i, left[i]);
		sound_line();
	}
	cpu_switch(0);
	sound_frame_end();
}

static int ybd_state_io(void *ctx, int load)
{
	snd_sync();
	cpu_switch(0);
	const uint32_t csize = m68k_context_size();
	if (!load)
		m68k_get_context(s_ctx[0]);
	uint32_t irq[2 * NCPU + 1];
	for (int i = 0; i < NCPU; i++) {
		irq[2 * i] = i ? s_irqp[i] : m68k_bus_irq_pending();
		irq[2 * i + 1] = (i ? &s_bus[i] : &m68k_bus)->irq_asserted;
	}
	irq[2 * NCPU] = (uint32_t)(s_lines[0] | s_lines[1] << 8 | s_lines[2] << 16);
	z80 z = s_z80;
	int32_t zi = s_zirq;
	int rc = 0;
	rc |= rbx_state_io(ctx, "m68k", s_ctx[0], csize);
	rc |= rbx_state_io(ctx, "m68k_x", s_ctx[1], csize);
	rc |= rbx_state_io(ctx, "m68k_y", s_ctx[2], csize);
	rc |= rbx_state_io(ctx, "irq", irq, sizeof(irq));
	rc |= rbx_state_io(ctx, "ybd", s_bp, offsetof(struct ybd_state, state_end));
	rc |= rbx_state_io(ctx, "z80", &z, sizeof(z));
	rc |= rbx_state_io(ctx, "zirq", &zi, sizeof(zi));
	rc |= rbx_state_io(ctx, "zram", s_zram, sizeof(s_zram));
	rc |= rbx_state_io(ctx, "pcmv", s_pcm.v, sizeof(s_pcm.v));
	rc |= rbx_state_io(ctx, "pcmr", s_pcm.ram, sizeof(s_pcm.ram));
	rc |= rbx_ym_state(0, ctx, load);
	if (!rc && load) {
		m68k_set_context(s_ctx[0]);
		m68k_bus_set_irq_pending(irq[0]);
		for (int i = 0; i < NCPU; i++) {
			if (i)
				s_irqp[i] = irq[2 * i];
			(i ? &s_bus[i] : &m68k_bus)->irq_asserted = irq[2 * i + 1];
			s_lines[i] = (uint8_t)(irq[2 * NCPU] >> (8 * i));
		}
		m68k_bus_update_irq();
		s_z80 = z;
		z80_bind();
		s_zirq = zi;
		s_wsres = b.sound_reset;
		s_pal_valid = 0;
	}
	return rc ? -1 : 0;
}

static int ybd_save(void *ctx) { return ybd_state_io(ctx, 0); }
static int ybd_load(void *ctx) { return ybd_state_io(ctx, 1); }
static int ybd_supported(const char *machine)
{
	return strncmp(machine, "yboard", 6) == 0 && strncmp(machine, "yboard_link", 11) != 0;
}

const rbx_driver rbx_drv_segaybd = { "Sega Y-Board", ybd_init, ybd_exit, ybd_reset, ybd_frame, ybd_save, ybd_load,
                                     ybd_supported, ybd_render, ybd_help };
