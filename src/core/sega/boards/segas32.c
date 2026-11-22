/*  Sega System 32 (the single-screen boards). Ported from MAME 0.289 src/mame/sega/segas32.cpp, segas32.h,
    segas32_v.cpp, segas32_m.cpp, s32comm.cpp, 315_5296.cpp and src/devices/machine/msm6253.cpp, upd4701.cpp
    (BSD-3-Clause, copyright-holders: Aaron Giles, David Haywood, Ariane Fugmann, AJR).
    A V60 at 16.1 MHz (src/core/cpu/v60), a Z80 at 8 MHz with two YM3438 and a RF5C68 (shared RAM with the V60, an
    interrupt controller each way), the 315-5296 I/O chip (CNT1 the display enable, CNT2 the Z80's reset), a 93C46.
    Video: four 512x256 tilemap pages a layer (NBG0/1 zooming, NBG2/3 row scroll / row select), a text layer, a
    bitmap layer and a background colour, five clip windows; sprites drawn by the sprite chip into one of two frame
    buffers (the list run 50 us after the vblank's end, the buffers swapped then); the mixer sorts the layers by
    priority, blends, shades and offsets the colours in RGB555 (the bitmap: 15-bit colour indices, a fixed palette).
    416 or 320 x 224 at 60 Hz (262 lines). Timing follows MAME's scheduler: the CPUs run in attoseconds to the next
    timer, an input line change ends the running CPU's slice and is applied when the other one has caught up.
    Golden Axe 2 and Arabian Fight: an encrypted V25 at 10 MHz (src/core/cpu/nec, as the V35 core) sharing a MB8421
    dual-port RAM with the V60.
    Multi 32 (sega_multi32_state): a V70 at 20 MHz (the V60 core, PIR 0x7000), a second palette / mixer / sprite
    layer and 315-5296 for the second monitor (drawn to the right of the first: one image twice as wide), 128 KB of
    work RAM, a Z80 with one YM3438 and a MultiPCM (src/core/sound/multipcm) on two mono speakers.
    The dual-board (F1 Exhaust Note, Air Rescue) and CD boards are not emulated here.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#if defined(__ARM_NEON)
#include <arm_neon.h>
#endif
#include "../../rbx.h"
#include "../../rbx_core.h"
#include "../../cpu/v60/v60.h"
#include "../../cpu/z80/z80.h"
#include "../../cpu/nec/nec.h"
#include "../../sound/rbx_ym.h"
#include "../../sound/rf5c68.h"
#include "../../sound/multipcm.h"
#include "../../machine/eepromser.h"
#include "../../machine/i8255.h"
#include "../../video/decoband.h"

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// ---- time (attoseconds from the start of the current frame: rebased each frame) ----
#define ATTO 1000000000000000000ull
static uint32_t s_main_clock = 32215900u, s_cpu_clock = 32215900u / 2;   // (Multi 32: 32 MHz, the V70 40 MHz / 2)
#define MAIN_CLOCK s_main_clock
#define V60_CLOCK s_cpu_clock
#define Z80_CLOCK (MAIN_CLOCK / 4)
#define YM_CLOCK (MAIN_CLOCK / 4)
#define RF_CLOCK (50000000u / 4)
static uint64_t s_p_v60, s_p_z80;   // ATTO / V60_CLOCK, ATTO / Z80_CLOCK (set with the clocks)
static double s_inv_v60, s_inv_z80;
#define P_V60 s_p_v60
#define P_Z80 s_p_z80
// d / p exactly (d >= 0): a floating point quotient, corrected
static inline uint64_t udiv_p(uint64_t d, uint64_t p, double inv)
{
	uint64_t q = (uint64_t)((double)d * inv);
	while (q && q * p > d)
		q--;
	while ((q + 1) * p <= d)
		q++;
	return q;
}
#define V25_CLOCK 10000000u
#define P_V25 (ATTO / V25_CLOCK)
#define FRAME_AS (ATTO / 60)
#define LINES 262
#define LINE_AS (FRAME_AS / LINES)
#define VIS_LINES 224
#define NEVER INT64_MAX
#define SW 416
#define SH 224
#define NPAL 32768

enum { MAIN_IRQ_VBSTART, MAIN_IRQ_VBSTOP, MAIN_IRQ_SOUND, MAIN_IRQ_TIMER0, MAIN_IRQ_TIMER1 };
enum { SOUND_IRQ_YM3438, SOUND_IRQ_V60 };
enum { TM_VBSTART, TM_VBEND, TM_SPRITES, TM_IRQ0, TM_IRQ1, TM_LINE, TM_SYNC, NTIMERS };
enum { CPU_NONE = -1, CPU_V60 = 0, CPU_Z80 = 1, CPU_V25 = 2 };

// ---- configuration ----
enum { BD_REGULAR, BD_ANALOG, BD_TRACK, BD_4P };
enum { PROT_NONE, PROT_SONIC, PROT_BRIVAL, PROT_DARKEDGE, PROT_DBZVRVS, PROT_JLEAGUE };
static int s_board, s_prot, s_prot_vblank;
static int s_screen_flipy;
static int s_v25;          // the V25 protection board (segas32_v25_state)
static uint8_t *s_mcu;
static uint32_t s_mcu_size;
static nec_t s_nec;
static int s_linktype;   // s32comm set_linktype: 14084 Rad Rally, 15612 F1 Super Lap (0: none)   // ORIENTATION_FLIP_Y (Holosseum): the screen mirrored here   // s_prot_vblank: 1 Dark Edge, 2 F1 Super Lap (the FD1149's vblank)
static uint8_t *s_rom, *s_zrom, *s_tiles, *s_sprites;
static uint32_t s_rom_size, s_zrom_size, s_tiles_size, s_sprites_size;
static int s_port_p1, s_port_p2, s_port_pc, s_port_s12, s_port_s34, s_port_an[8], s_port_ex[3], s_port_tr[6];
static int s_port_b[5];    // Multi 32: the second I/O chip's P1_B, P2_B, PORTC_B, SERVICE12_B, SERVICE34_B
// the MSM6253's inputs: the analog field of each ANALOGn port (type, player, default, mask)
static struct { int type, player; uint32_t def, mask; } s_an[8];
static int s_multi;        // Multi 32 (m_is_multi32)
static int s_scross;       // (init_scross: scross_bank_w on the Z80's program space at 0xb0)

// ---- the state (saved; on the heap while the game runs, ADR-040) ----
static struct s32_state {
	uint8_t workram[0x10000];
	uint16_t videoram[0x10000];
	uint16_t spriteram[0x10000];
	uint16_t paletteram[0x4000];
	uint16_t mixer[0x40];
	uint8_t zram[0x2000];          // the Z80's shared RAM (0xe000)
	uint16_t sprbuf[2][SW * SH];   // the sprite frame buffers (A, B)
	uint16_t protram[0x800];       // (Burning Rival)
	uint8_t comm_shared[0x800];
	uint8_t dpram[0x800];          // the MB8421 (V25 boards)
	// the timers (MAME's emu_timers): expire time and insertion order
	int64_t texp[NTIMERS];
	uint32_t tseq[NTIMERS], seq;
	int64_t base, lt[3];           // the scheduler's time, each CPU's local time (attoseconds)
	int64_t v60_cycles;            // the V60's cycles before the current slice (the EEPROM's clock)
	uint64_t ym_frac;
	// interrupts, lines
	uint8_t v60_irq_control[16];
	uint8_t v60_line, v60_want;
	uint8_t sound_irq_control[4], sound_irq_input;
	uint8_t z_line, z_want, z_vector_want;
	uint8_t z_reset, z_reset_want;   // the Z80 held in reset (CNT2 low)
	uint8_t sound_dummy;
	uint16_t sound_bank;
	// 315-5296
	uint8_t io_dir, io_cnt, io_latch[8];
	// video registers
	uint8_t displayenable, tilebank_external;
	uint8_t sprite_render_count, sprite_control[8], sprite_control_latched[8];
	uint8_t spr_front;             // the displayed buffer (MIXER_LAYER_SPRITES): 0 = A
	// s32comm, analog, trackballs
	uint8_t comm_cn, comm_fg, comm_zfg;
	// S32COMM_SIMULATION with MAME's default options (the remote port is the local one: a ring of one node)
	uint8_t link_enable, link_alive, link_id, link_count;
	uint16_t link_timer;
	uint8_t link_buf[0x100];       // m_buffer0
	uint8_t link_fifo[8][0x100];   // (the frames sent, read back in order)
	uint8_t link_fifo_n, link_fifo_r;
	uint8_t adc_shift;
	uint16_t upd_x[3], upd_y[3];
	int16_t an_val[4];             // the analog inputs' values (from the controls)
	uint32_t rng;
	uint8_t state_end;
} *s_bp;
#define b (*s_bp)

// ---- Multi 32's second monitor and the rest (saved; on the heap, only for Multi 32) ----
static struct m32_state {
	uint8_t workram[0x20000];
	uint16_t paletteram[0x4000];   // (m_paletteram[1])
	uint16_t mixer[0x40];          // (m_mixer_control[1])
	uint16_t sprbuf[2][SW * SH];   // MIXER_LAYER_MULTISPR, MULTISPR_2 (swapped with the first pair)
	uint8_t io_dir, io_cnt, io_latch[8];   // io_chip_1
	uint8_t displayenable;
	uint8_t analog_bank;
	int16_t an_val[8];
	uint8_t state_end;
} *s_mp;
#define mb (*s_mp)
static multipcm_t *s_mpcm;

static z80 s_z80;
static int s_zirq;
static const uint8_t *s_zmap[256];
static rf5c68_t *s_rfp;   // (64 KB of wave RAM: on the heap, ADR-040)
#define s_rf (*s_rfp)
static eepser_t s_eeprom;
static i8255_t s_ppi;
static int s_exec = CPU_NONE;      // the CPU running
static unsigned long s_zstart;     // the Z80's cycle count at its slice's start
static uint16_t s_zscan_h[2];      // the sound program's table scan loops (zscan_find), their BIT n,(hl)'s address
static int s_zscan_n, s_zscan_hit;
static int64_t s_target;
static int64_t s_cbtime;          // the time outside the CPUs (a timer's expire time)
static int s_frame_done;
static int32_t *s_mix_l, *s_mix_r;
static int s_mix_n, s_mix_cap;
// Multi 32's MultiPCM on the YM3438's worker (ADR-070): its writes and each line's samples go into that chip's queue
// (rbx_ym_async_user) in their order; pcm_user makes them there into s_pcm_l / r, added to the mix at the frame's end
// after a sync. Nothing the Z80 reads comes from the chip (its reads give 0). RBX_S32_PCM_SYNC=1: here, as before
static int s_pcm_async, s_pcm_pos;
static uint32_t s_pcm_rate;
static int32_t *s_pcm_l, *s_pcm_r;

// ---- the video snapshot (taken at the vblank's start, drawn by render) ----
typedef struct {
	uint8_t scan_extent[256];
	uint16_t extent[32][16];
	uint8_t clipdraw_start;
} extents_t;
static struct s32_snap {
	uint16_t videoram[0x10000];
	uint16_t paletteram[0x4000];
	uint16_t mixer[0x40];
	const uint16_t *spr;           // the shown sprite frame buffer (s_sprp: not copied, ADR-069)
	uint8_t spi;                   // (its physical buffer)
	uint8_t latched[8];
	uint8_t displayenable;
	int width;
	uint8_t enablemask;            // update_tilemaps' result
	uint8_t tilebank[4];           // each NBG layer's tile bank (System 32: the same for all)
	extents_t ext[6];              // the clip extents of TEXT (unused), NBG0-3, BITMAP
	uint16_t palx[3][0x8000];     // (render) the palette as drawn with each colour offset, no blend / shadow (entries
	                               // 0x4000-0x7fff repeat 0-0x3fff: a layer's palette base + its pixel needs no wrap)
	uint16_t palprev[0x4000];      // (render) the palette palx was made from: only the entries that change are made again
	uint32_t palkey[2];            // (render) the colour offsets palx[0], [1] hold (PALKEY_NONE: none; calloc: [2] not made)
	uint8_t palok;
	uint32_t paldirty[0x4000 / 8 / 32];   // (s_paldirty moved here at each snapshot, cleared by palx_build)
} *s_snap, *s_snap1;   // (s_snap1: Multi 32's second monitor, its palette / mixer / sprites)
// (two of each: the render thread draws one while the next frame's is taken, rbx_video_set_depth(2); s_snap / s_snap1
// are the pair render draws, picked by rbx_video_render_seq)
static struct s32_snap *s_snapv[2], *s_snap1v[2];

// ---- the scheduler ----
static void abort_slice(void)
{
	if (s_exec == CPU_V60)
		v60_abort_timeslice();
	else if (s_exec == CPU_Z80) {
		z80_break = 1;
		s_zscan_hit = 0;
	}
}

static int64_t now_t(void)
{
	if (s_exec == CPU_V60)
		return b.lt[CPU_V60] + (int64_t)v60_cycles_run() * (int64_t)P_V60;
	if (s_exec == CPU_Z80)
		return b.lt[CPU_Z80] + (int64_t)(s_z80.cyc - s_zstart) * (int64_t)P_Z80;
	return s_cbtime;
}

static void t_adjust(int i, int64_t when)
{
	b.texp[i] = when;
	b.tseq[i] = ++b.seq;
	if (s_exec != CPU_NONE && when < s_target)   // (the new head timer: the running CPU's slice ends)
		abort_slice();
}

// set_input_line: synced (a timer at now), applied once the other CPU has caught up
static void apply_sync(void);
static void sync_request(void)
{
	if (s_exec == CPU_NONE) {   // (from a timer: at once)
		apply_sync();
		return;
	}
	const int64_t t = now_t();
	if (b.texp[TM_SYNC] == NEVER || t < b.texp[TM_SYNC]) {
		b.texp[TM_SYNC] = t;
		b.tseq[TM_SYNC] = ++b.seq;
	}
	abort_slice();
}

static uint8_t z_read(void *u, uint16_t a);
static void z_write(void *u, uint16_t a, uint8_t v);
static uint8_t z_in(z80 *z, uint8_t p);
static void z_out(z80 *z, uint8_t p, uint8_t v);
static void z80_reset_cpu(void)   // (z80_init clears the callbacks)
{
	z80_init(&s_z80);
	s_z80.read_byte = z_read;
	s_z80.write_byte = z_write;
	s_z80.port_in = z_in;
	s_z80.port_out = z_out;
	s_z80.userdata = NULL;
	s_z80.read_map = s_zmap;
}

static void apply_sync(void)
{
	if (b.v60_want != b.v60_line) {
		b.v60_line = b.v60_want;
		v60_set_irq_line(b.v60_line);
	}
	if (b.z_reset_want != b.z_reset) {
		b.z_reset = b.z_reset_want;
		if (b.z_reset) {   // (asserted: the CPU reset and held)
			const unsigned long c = s_z80.cyc;
			z80_reset_cpu();
			s_z80.cyc = c;
		}
	}
	b.z_line = b.z_want;
	s_z80.int_data = b.z_vector_want;
	s_zirq = b.z_line;
}
static void tm_sync(void)
{
	b.texp[TM_SYNC] = NEVER;
	apply_sync();
}

// ---- the interrupt controller ----
static void update_irq_state(void)
{
	const uint8_t effirq = b.v60_irq_control[7] & ~b.v60_irq_control[6] & 0x1f;
	const uint8_t want = effirq ? 1 : 0;
	if (want != b.v60_want) {
		b.v60_want = want;
		sync_request();
	}
}

static void signal_v60_irq(int which)
{
	if (which >= 0 && which < 5)
		b.v60_irq_control[7] |= (uint8_t)(1 << which);
	update_irq_state();
}

static uint8_t irq_callback(void)
{
	const uint8_t effirq = b.v60_irq_control[7] & ~b.v60_irq_control[6] & 0x1f;
	for (int vector = 0; vector < 5; vector++)
		if ((effirq >> vector) & 1)
			return b.v60_irq_control[vector];
	return 0;
}

static void signal_sound_irq(int which);
static void int_control_w(int offset, uint8_t data)
{
	switch (offset) {
	case 0: case 1: case 2: case 3: case 4:   // vectors
	case 5:
		b.v60_irq_control[offset] = data;
		break;
	case 6:   // mask
		b.v60_irq_control[offset] = data;
		update_irq_state();
		break;
	case 7:   // acknowledge
		b.v60_irq_control[offset] &= data;
		update_irq_state();
		break;
	case 8: case 9: {   // timer 0 count
		b.v60_irq_control[offset] = data;
		const uint16_t duration = (uint16_t)(b.v60_irq_control[8] | ((b.v60_irq_control[9] << 8) & 0xf00));
		if (duration)
			t_adjust(TM_IRQ0, now_t() + (int64_t)(0x800 * duration) * (int64_t)(ATTO / (MAIN_CLOCK / 2)));
		break;
	}
	case 10: case 11: {   // timer 1 count
		b.v60_irq_control[offset] = data;
		const uint16_t duration = (uint16_t)(b.v60_irq_control[10] | ((b.v60_irq_control[11] << 8) & 0xf00));
		if (duration)
			t_adjust(TM_IRQ1, now_t() + (int64_t)(0x100 * duration) * (int64_t)(ATTO / (50000000u / 16)));
		break;
	}
	case 12: case 13: case 14: case 15:   // signal IRQ to sound CPU
		signal_sound_irq(SOUND_IRQ_V60);
		break;
	}
}

// ---- the sound interrupt controller ----
static void update_sound_irq_state(void)
{
	const uint8_t effirq = b.sound_irq_input & ~b.sound_irq_control[3] & 0x07;
	int vector;
	for (vector = 0; vector < 3; vector++)
		if ((effirq >> vector) & 1) {
			b.z_want = 1;
			b.z_vector_want = (uint8_t)(2 * vector);
			break;
		}
	if (vector == 3)
		b.z_want = 0;
	sync_request();
}

static void signal_sound_irq(int which)
{
	for (int i = 0; i < 3; i++)
		if (b.sound_irq_control[i] == which)
			b.sound_irq_input |= (uint8_t)(1 << i);
	update_sound_irq_state();
}

static void clear_sound_irq(int which)
{
	for (int i = 0; i < 3; i++)
		if (b.sound_irq_control[i] == which)
			b.sound_irq_input &= (uint8_t)~(1 << i);
	update_sound_irq_state();
}

static void ym3438_irq_handler(int state)
{
	if (state)
		signal_sound_irq(SOUND_IRQ_YM3438);
	else
		clear_sound_irq(SOUND_IRQ_YM3438);
}

// ---- the sprite chip ----
static void sprite_render_list(void);
// the sprite list drawn on the rbx_aux2 worker (by the emulation thread without one): its inputs as they were at
// update_sprites (the sprite RAM, the latched controls, the buffer drawn into); the next update_sprites, a save and
// the exit wait for it (the frame buffer it draws into is shown only after the next swap)
static struct { uint16_t *ram; uint8_t latched[8]; uint8_t front, erase, dsti, srci; volatile int pending; } s_sj;
// The worker is safe by construction: it reads only this snapshot and writes only the hidden buffer (front ^ 1, a
// physical buffer no frame snapshot names), which nothing else touches until the next update_sprites (which syncs
// first: the erase and the swap come after it). RBX_S32_SPRITE_SYNC=1: drawn on the emulation thread.
// (11-10 the Vita stopped answering after the first build that used it, and again later with it off: the cause
// not shown to be the worker. It runs on CPU1 with the sound threads: CPU3 has the drawing helper, s32_help)
static int s_sprite_async;
// The two sprite frame buffers (A, B; each monitor's) are physical buffers of a pool of four (ADR-069): 0, 1 are
// b.sprbuf / mb.sprbuf (the saved ones), 2, 3 the heap's. s_spmap[A or B] is the one holding it. The frame snapshot
// keeps the shown one's pointer (spi) instead of a copy; a buffer a snapshot slot still names is not written again:
// the erase or the drawing of a buffer goes to a free one instead (of the four, at most three are named: the other
// buffer and the two slots), with a copy of the old contents first when there is no erase.
static uint16_t *s_sprp[2][4];
static uint16_t *s_sprx;           // (the pool's heap part: [2 monitors][2][SW * SH])
static uint8_t s_spmap[2];
static int spr_named(int i)
{
	return (s_snapv[0] && s_snapv[0]->spi == i) || (s_snapv[1] && s_snapv[1]->spi == i);
}
static int spr_free(int other)   // a buffer neither the other one nor a snapshot names
{
	for (int i = 0; i < 4; i++)
		if (i != other && !spr_named(i))
			return i;
	return -1;   // (not reached)
}
static void spr_canonical(void)   // (before a save / load: A, B in b.sprbuf / mb.sprbuf again)
{
	const int pa = s_spmap[0], pb = s_spmap[1];
	if (pa == 0 && pb == 1)
		return;
	for (int m = 0; m < (s_multi ? 2 : 1); m++) {
		uint16_t *const *q = s_sprp[m];
		if (pa == 1 && pb == 0) {   // (swapped: through buffer 2, free then)
			memcpy(q[2], q[0], SW * SH * 2);
			memcpy(q[0], q[1], SW * SH * 2);
			memcpy(q[1], q[2], SW * SH * 2);
		} else if (pa == 1) {
			memcpy(q[0], q[1], SW * SH * 2);
			memcpy(q[1], q[pb], SW * SH * 2);
		} else {
			if (pb != 1)
				memcpy(q[1], q[pb], SW * SH * 2);
			if (pa != 0)
				memcpy(q[0], q[pa], SW * SH * 2);
		}
	}
	s_spmap[0] = 0;
	s_spmap[1] = 1;
}
static void sprite_erase_phys(int i)
{
	for (int m = 0; m < (s_multi ? 2 : 1); m++) {   // (the other monitor's buffer as well)
		uint16_t *p = s_sprp[m][i];
		for (int k = 0; k < SW * SH; k++)
			p[k] = 0xffff;
	}
}
static void sprite_job(void)
{
	if (s_sj.pending) {
		if (s_sj.erase)   // (the erase before the swap: the buffer drawn into now, ADR-068)
			sprite_erase_phys(s_sj.dsti);
		else if (s_sj.srci != s_sj.dsti)   // (moved off a buffer a snapshot names: its contents first)
			for (int m = 0; m < (s_multi ? 2 : 1); m++)
				memcpy(s_sprp[m][s_sj.dsti], s_sprp[m][s_sj.srci], SW * SH * 2);
		sprite_render_list();
		s_sj.pending = 0;
	}
}
static void sprite_swap_buffers(void)
{
	b.spr_front ^= 1;
	memcpy(b.sprite_control_latched, b.sprite_control, sizeof(b.sprite_control_latched));
}
static void tm_sprites(void)   // update_sprites
{
	b.texp[TM_SPRITES] = NEVER;
	if (!((b.sprite_control[3] >> 1) & 1)) {
		if (b.sprite_render_count-- == 0) {
			b.sprite_control[0] = 3;
			b.sprite_render_count = b.sprite_control[3] & 1;
		}
	}
	rbx_aux2_sync();
	const int erase = (b.sprite_control[0] >> 1) & 1;
	if (erase && !(b.sprite_control[0] & 1)) {   // (no swap: the shown buffer, here)
		const int f = b.spr_front;
		if (spr_named(s_spmap[f]))
			s_spmap[f] = (uint8_t)spr_free(s_spmap[f ^ 1]);
		sprite_erase_phys(s_spmap[f]);
	}
	if (b.sprite_control[0] & 1) {
		// the erase of the buffer about to be hidden goes with its drawing (on the worker): nothing reads it before
		sprite_swap_buffers();
		const int h = b.spr_front ^ 1;
		s_sj.srci = s_spmap[h];
		if (spr_named(s_spmap[h]))
			s_spmap[h] = (uint8_t)spr_free(s_spmap[h ^ 1]);
		s_sj.dsti = s_spmap[h];
		s_sj.erase = (uint8_t)erase;
		memcpy(s_sj.ram, b.spriteram, sizeof(b.spriteram));
		memcpy(s_sj.latched, b.sprite_control_latched, sizeof(s_sj.latched));
		s_sj.front = b.spr_front;
		s_sj.pending = 1;
		if (s_sprite_async)
			rbx_aux2_kick();
		else
			sprite_job();
	}
	b.sprite_control[0] = 0;
}

static uint8_t sprite_control_r(int offset)
{
	switch (offset) {
	case 0: return (uint8_t)(0xfc | (b.spr_front == 0));
	case 1: return 0xfc | 1;
	case 2: return (uint8_t)(0xfc | b.sprite_control_latched[2]);
	case 3: return (uint8_t)(0xfc | b.sprite_control_latched[3]);
	case 4: return (uint8_t)(0xfc | b.sprite_control_latched[4]);
	case 5: return (uint8_t)(0xfc | b.sprite_control_latched[5]);
	case 6: return (uint8_t)(0xfc | (b.sprite_control_latched[6] & 1));
	case 7: return 0xfc;
	}
	return 0xff;
}

// ---- the 315-5296, the EEPROM ----
static int64_t eep_now(void) { return b.v60_cycles + (s_exec == CPU_V60 ? v60_cycles_run() : 0); }
static uint8_t port8(int p) { return p >= 0 ? (uint8_t)rbx_port_read(p) : 0xff; }
static void io_out_port(int chip, int port, uint8_t data)
{
	switch (port) {
	case 3:   // port D: misc_output_0_w (coin counters, lamps), the EEPROM's DI / CS / CLK (System 32)
		if (s_multi)
			break;
		eepser_di_write(&s_eeprom, (data >> 7) & 1);
		eepser_cs_write(&s_eeprom, (data >> 5) & 1, eep_now());
		eepser_clk_write(&s_eeprom, (data >> 6) & 1, eep_now());
		break;
	case 7:   // port H: tilebank_external_w (Multi 32's second chip: the EEPROM)
		if (chip == 0)
			b.tilebank_external = data;
		else {
			eepser_di_write(&s_eeprom, (data >> 7) & 1);
			eepser_cs_write(&s_eeprom, (data >> 5) & 1, eep_now());
			eepser_clk_write(&s_eeprom, (data >> 6) & 1, eep_now());
		}
		break;
	}
}
static uint8_t io_in_port(int chip, int port)
{
	if (chip) {
		switch (port) {
		case 0: return port8(s_port_b[0]);
		case 1: return port8(s_port_b[1]);
		case 2: return port8(s_port_b[2]);
		case 4: return port8(s_port_b[3]);
		case 5:
			if (s_port_b[4] >= 0)
				rbx_port_custom(s_port_b[4], 0x80, eepser_do_read_93cxx(&s_eeprom, eep_now()) ? 0x80u : 0);
			return port8(s_port_b[4]);
		}
		return 0xff;
	}
	switch (port) {
	case 0: return port8(s_port_p1);
	case 1: return port8(s_port_p2);
	case 2: return port8(s_port_pc);
	case 4: return port8(s_port_s12);
	case 5:
		if (s_port_s34 >= 0)
			rbx_port_custom(s_port_s34, 0x80, eepser_do_read_93cxx(&s_eeprom, eep_now()) ? 0x80u : 0);
		return port8(s_port_s34);
	}
	return 0xff;   // (D, G, H: outputs)
}
static void cnt_w(int chip, int i, int state)
{
	if (chip) {   // io_chip_1: CNT1 the second monitor's display enable
		if (i == 1)
			mb.displayenable = (uint8_t)state;
		return;
	}
	if (i == 1)
		b.displayenable = (uint8_t)state;
	else if (i == 2) {   // the Z80's reset (inverted)
		const uint8_t want = (uint8_t)!state;
		if (want != b.z_reset_want) {
			b.z_reset_want = want;
			sync_request();
		}
	}
}
static uint8_t io_r(int chip, int offset)
{
	const uint8_t dir = chip ? mb.io_dir : b.io_dir, cnt = chip ? mb.io_cnt : b.io_cnt;
	const uint8_t *latch = chip ? mb.io_latch : b.io_latch;
	offset &= 0x3f;
	if (offset < 8)
		return ((dir >> offset) & 1) ? latch[offset] : io_in_port(chip, offset);
	switch (offset) {
	case 0x8: return 'S';
	case 0x9: return 'E';
	case 0xa: return 'G';
	case 0xb: return 'A';
	case 0xc: case 0xe: return cnt;
	case 0xd: case 0xf: return dir;
	}
	return 0xff;
}
static void io_w(int chip, int offset, uint8_t data)
{
	uint8_t *dir = chip ? &mb.io_dir : &b.io_dir, *cnt = chip ? &mb.io_cnt : &b.io_cnt;
	uint8_t *latch = chip ? mb.io_latch : b.io_latch;
	offset &= 0x3f;
	if (offset < 8) {
		if ((*dir >> offset) & 1)
			io_out_port(chip, offset, data);
		latch[offset] = data;
	} else if (offset == 0xe) {
		for (int i = 0; i < 3; i++)
			cnt_w(chip, i, (data >> i) & 1);
		*cnt = data;
	} else if (offset == 0xf) {
		for (int i = 0; i < 8; i++)
			if (((*dir ^ data) >> i) & 1)
				io_out_port(chip, i, ((data >> i) & 1) ? latch[i] : 0);
		*dir = data;
	}
}

// ---- palette RAM ----
static inline uint16_t to_xBGR(uint16_t value)   // xBBBBBGGGGGRRRRR_to_xBGRBBBBGGGGRRRR
{
	const int r = (value >> 0) & 0x1f, g = (value >> 5) & 0x1f, bl = (value >> 10) & 0x1f;
	value = (uint16_t)((value & 0x8000) | ((bl & 0x01) << 14) | ((g & 0x01) << 13) | ((r & 0x01) << 12));
	value |= (uint16_t)(((bl & 0x1e) << 7) | ((g & 0x1e) << 3) | ((r & 0x1e) >> 1));
	return value;
}
static inline uint16_t from_xBGR(uint16_t value)
{
	const int r = ((value >> 12) & 0x01) | ((value << 1) & 0x1e);
	const int g = ((value >> 13) & 0x01) | ((value >> 3) & 0x1e);
	const int bl = ((value >> 14) & 0x01) | ((value >> 7) & 0x1e);
	return (uint16_t)((value & 0x8000) | (bl << 10) | (g << 5) | (r << 0));
}
static uint16_t pal_r(const uint16_t *pal, int offset)
{
	const int convert = (offset >> 14) & 1;
	offset &= 0x3fff;
	return convert ? to_xBGR(pal[offset]) : pal[offset];
}
// the palette's blocks of 8 entries written since each snapshot slot's last copy, by monitor (ADR-073): palx_build
// looks only at those (a superset of the blocks that changed; all of them after a reset or a load)
static uint32_t s_paldirty[2][2][0x4000 / 8 / 32];
static void pal_dirty_all(void) { memset(s_paldirty, 0xff, sizeof(s_paldirty)); }
static void pal_w(uint16_t *pal, const uint16_t *mixer, int offset, uint16_t data, uint16_t mask)
{
	const int convert = (offset >> 14) & 1;
	offset &= 0x3fff;
	uint32_t (*const dirty)[0x4000 / 8 / 32] = s_paldirty[s_mp && pal == mb.paletteram];
	for (int pass = 0; pass < 2; pass++) {
		dirty[0][offset >> 8] |= 1u << ((offset >> 3) & 31);
		dirty[1][offset >> 8] |= 1u << ((offset >> 3) & 31);
		uint16_t value = pal[offset];
		if (convert) value = to_xBGR(value);
		value = (uint16_t)((value & ~mask) | (data & mask));
		if (convert) value = from_xBGR(value);
		pal[offset] = value;
		if (!(mixer[0x4e / 2] & 0x0880))   // (blending on: writes go to both halves)
			break;
		offset ^= 0x2000;
	}
}
static uint16_t paletteram_r(int offset) { return pal_r(b.paletteram, offset); }
static void paletteram_w(int offset, uint16_t data, uint16_t mask) { pal_w(b.paletteram, b.mixer, offset, data, mask); }

// ---- protection and other handlers ----
static uint16_t ws16(uint32_t off) { return (uint16_t)(b.workram[off & 0xffff] | (b.workram[(off + 1) & 0xffff] << 8)); }
static void ww16(uint32_t off, uint16_t v) { b.workram[off & 0xffff] = (uint8_t)v; b.workram[(off + 1) & 0xffff] = (uint8_t)(v >> 8); }
static void sonic_level_load_protection(uint16_t data, uint16_t mask)
{
	ww16(0xe5c4, (uint16_t)((ws16(0xe5c4) & ~mask) | (data & mask)));
	uint16_t level;
	const uint16_t cl = ws16(0xe5c4);
	if (cl == 0)
		level = 0x0007;
	else {
		const uint32_t o = 0x263a + cl * 2;
		level = (uint16_t)(s_rom[(o - 1) % s_rom_size] | (s_rom[(o - 2) % s_rom_size] << 8));
	}
	ww16(0xf06e, level);
	ww16(0xf0bc, 0);
	ww16(0xf0be, 0);
}
static void brival_protection_w(int offset, uint16_t data)
{
	static const int prot_address[6][2] = {
		{ 0x109517, 0x00/2 }, { 0x109597, 0x10/2 }, { 0x109597, 0x20/2 },
		{ 0x109597, 0x30/2 }, { 0x109597, 0x40/2 }, { 0x109617, 0x50/2 },
	};
	(void)data;
	if (offset < 0x800 / 2 || offset > 0x80a / 2)
		return;
	const int t = offset - 0x800 / 2;
	uint8_t ret[16];
	for (int i = 0; i < 16; i++)
		ret[i] = s_rom[(prot_address[t][0] + i) % s_rom_size];
	memcpy((uint8_t *)&b.protram[prot_address[t][1]], ret, 16);
}
static uint16_t main_read16(uint32_t a, uint16_t mask);
static void main_write16(uint32_t a, uint16_t d, uint16_t mask);
static void space_wb(uint32_t a, uint8_t v)
{
	if (a & 1) main_write16(a & ~1u, (uint16_t)(v << 8), 0xff00);
	else main_write16(a, v, 0x00ff);
}
static void darkedge_fd1149_vblank(void)
{
	ww16(0xf072, 0);
	ww16(0xf082, 0);
	if (b.workram[0xa12c] != 0) {
		b.workram[0xa12c]--;
		if (b.workram[0xa12c] == 0)
			b.workram[0xa12e] = 1;
	}
}
static void f1lap_fd1149_vblank(void)
{
	b.workram[0xf7c6] = 0;
	if (b.workram[0xee81] == 0xff)
		b.workram[0xee81] = 0;
}
static void jleague_protection_w(int offset, uint16_t data, uint16_t mask)
{
	const uint32_t o = 0xf700 + offset * 2;
	ww16(o, (uint16_t)((ws16(o) & ~mask) | (data & mask)));
	if (offset == 0) {
		const uint32_t ra = 0x7bbc0 + data * 2;
		space_wb(0x20f708, (uint8_t)(s_rom[ra % s_rom_size] | (s_rom[(ra + 1) % s_rom_size] << 8)));
	} else if (offset == 2)
		b.workram[0x0016] = (uint8_t)data;
}

// ---- the communication board (s32comm.cpp with S32COMM_SIMULATION; the link: MAME's default sockets, the remote
// port the local one, so the frames sent come back: a ring of one node, the master's link up after 232 vblanks) ----
static void link_send(int size)
{
	if (b.link_fifo_n < 8) {
		memcpy(b.link_fifo[(b.link_fifo_r + b.link_fifo_n) & 7], b.link_buf, (size_t)size);
		b.link_fifo_n++;
	}
}
static int link_read(int size)
{
	if (!b.link_fifo_n)
		return 0;
	memcpy(b.link_buf, b.link_fifo[b.link_fifo_r], (size_t)size);
	b.link_fifo_r = (b.link_fifo_r + 1) & 7;
	b.link_fifo_n--;
	return size;
}
static void link_send_data(uint8_t type, int start, int size, int datasize)
{
	b.link_buf[0] = type;
	for (int i = 0; i < size; i++)
		b.link_buf[1 + i] = b.comm_shared[(start + i) & 0x7ff];
	link_send(datasize);
}
// comm_tick_14084 / comm_tick_15612 / comm_tick_15033: the shared memory's layout of each
static void comm_tick(void)
{
	if (!s_linktype || b.link_enable != 1)
		return;
	const int t15612 = s_linktype == 15612, t15033 = s_linktype == 15033;
	const int i_status = t15612 ? 0 : 4, i_mode = t15612 ? 1 : 2, i_id = t15612 ? 2 : 1, i_count = t15612 ? 3 : 0;
	const int i_rts = t15612 ? 4 : 3;
	const int frame_start = t15033 ? 0x710 : t15612 ? 0x10 : 0x480, frame_size = (t15612 || t15033) ? 0xe0 : 0x80;
	const int save_base = t15033 ? 0x10 - 0xe0 : t15612 ? 0x10 : 0;   // (15033: frameStartRX + (idx - 1) * frameSize)
	const int datasize = frame_size + 1;
	uint8_t *sh = b.comm_shared;
	const int is_master = sh[i_mode] == 1;
	const int is_slave = t15612 ? sh[i_mode] == 2 : sh[i_mode] == 0;
	const int is_relay = t15612 ? sh[i_mode] == 0 : t15033 ? sh[i_mode] == 2 : sh[i_mode] == 0xff;
	if (b.link_alive == 2) {
		sh[t15033 ? 0 : i_status] = 0xff;
		return;
	}
	if (b.link_alive == 0) {
		if (t15033 && sh[0] == 0x56 && sh[1] == 0x37 && sh[2] == 0x30) {
			for (int j = 0x003; j < 0x0800; j++)
				sh[j] = 0;
			sh[0x08] = 0x5a;
			sh[0x09] = 0x38;
			sh[0x0a] = 0x30;
		}
		sh[i_status] = t15612 ? 0x05 : 0x00;
		int recv = link_read(datasize);
		while (recv > 0) {
			const int idx = b.link_buf[0];
			if (idx == 0xff) {
				if (is_master) {
					b.link_id = 1;
					b.link_count = b.link_buf[1];
					b.link_timer = 0;
				} else if (is_slave || is_relay) {
					if (is_slave) {
						b.link_buf[1]++;
						b.link_id = b.link_buf[1];
					}
					link_send(datasize);
				}
			} else if (idx == 0xfe) {
				if (is_slave || is_relay) {
					b.link_count = b.link_buf[1];
					link_send(datasize);
				}
				b.link_alive = 1;
				sh[i_status] = 1;
				sh[i_id] = b.link_id;
				sh[i_count] = b.link_count;
			}
			recv = b.link_alive == 0 ? link_read(datasize) : 0;
		}
		if (is_master && b.link_alive == 0) {
			if (b.link_timer == 1) {
				b.link_buf[0] = 0xff;
				b.link_buf[1] = 0x01;
				link_send(datasize);
			} else if (b.link_timer == 0) {
				b.link_buf[0] = 0xfe;
				b.link_buf[1] = b.link_count;
				link_send(datasize);
				b.link_alive = 1;
				sh[i_status] = 1;
				sh[i_id] = b.link_id;
				sh[i_count] = b.link_count;
			} else
				b.link_timer--;
		}
	}
	if (b.link_alive == 1) {
		do {
			int recv = link_read(datasize);
			while (recv > 0) {
				const int idx = b.link_buf[0];
				if (idx > 0 && idx <= b.link_count) {
					if (idx != b.link_id) {
						for (int j = 0; j < frame_size; j++)
							sh[(save_base + idx * frame_size + j) & 0x7ff] = b.link_buf[1 + j];
						link_send(datasize);
					}
				} else {
					if (idx == 0xfc) {
						b.link_timer = 0;
						if (!is_master)
							link_send(datasize);
					}
					if (idx == 0xfd && !is_master) {
						for (int j = 0; j < 0x0b; j++)
							sh[0x05 + j] = b.link_buf[1 + j];
						link_send(datasize);
					}
				}
				recv = link_read(datasize);
			}
		} while (b.link_timer == 1);
		b.link_timer = 0;   // (m_framesync: comm_framesync off)
		if (b.link_id != 0) {
			if (sh[i_rts] != 0) {
				link_send_data(b.link_id, frame_start, frame_size, datasize);
				for (int j = 0; j < frame_size; j++)
					sh[(save_base + b.link_id * frame_size + j) & 0x7ff] = b.link_buf[1 + j];
			}
			if (is_master) {
				link_send_data(0xfd, 0x05, 0x0b, datasize);
				b.link_buf[0] = 0xfc;
				b.link_buf[1] = 0x01;
				link_send(datasize);
			}
		}
		sh[i_rts] = 0;
	}
}
static void comm_cn_w(uint8_t data)
{
	b.comm_cn = data & 1;
	if (!b.comm_cn)
		b.link_enable = 0;
	else {
		b.link_enable = 1;
		b.link_id = 0;
		b.link_alive = 0;
		b.link_count = 0;
		b.link_timer = 0x00e8;   // (58 fps x 4 s)
		comm_tick();
	}
}

// the MSM6253's channel: the analog field's value (its default without input), a missing port 0xff
static uint8_t adc_input(int ch)
{
	if (s_multi) {   // (sega_multi32_analog: inputs 2 and 3 from ANALOG3 / 4 or ANALOG7 / 8, by analog_bank_w)
		if (ch >= 2)
			ch += mb.analog_bank * 4;
		return s_port_an[ch] < 0 ? 0 : (uint8_t)mb.an_val[ch];   // (read_safe(0))
	}
	if (s_port_an[ch] < 0)
		return 0xff;
	return (uint8_t)b.an_val[ch];
}

// ---- the main CPU's bus ----
static void map_main(void);
static uint16_t main_read16(uint32_t a, uint16_t mask)
{
	a &= 0xffffff;
	switch (a >> 20) {
	case 0x2:   // (the protection hooks' page of work RAM)
		if (s_prot == PROT_BRIVAL && a >= 0x20ba00 && a <= 0x20ba07) {
			const int o = (int)((a - 0x20ba00) >> 1);
			if (mask == 0xffff && (o == 0 || o == 2 || o == 3))
				return 0;
		}
		return ws16(a & 0xffff);
	case 0x5:
		return (uint16_t)(0xff00 | sprite_control_r((int)((a >> 1) & 7)));
	case 0x6:
		if (!(a & 0x10000))
			return paletteram_r((int)((a >> 1) & 0x7fff));
		return b.mixer[(a >> 1) & 0x3f];
	case 0x8:
		if (a < 0x801000)
			return (uint16_t)(0xff00 | b.comm_shared[(a >> 1) & 0x7ff]);
		if (a == 0x801000)
			return (uint16_t)(0xff00 | b.comm_cn | 0xfe);
		if (a == 0x801002)
			return (uint16_t)(0xff00 | ((b.comm_fg | (~b.comm_zfg << 7) | 0x7e) & 0xff));
		return 0xffff;
	case 0xa:
		if (s_v25 && a < 0xa01000)   // the MB8421 right port (umask 0x00ff)
			return (uint16_t)(0xff00 | b.dpram[(a >> 1) & 0x7ff]);
		if (s_prot == PROT_DARKEDGE && a < 0xa80000)
			return 0xffff;
		if (s_prot == PROT_DBZVRVS && a < 0xa80000)
			return 0xffff;
		if (s_prot == PROT_BRIVAL && a < 0xa01000)
			return b.protram[(a >> 1) & 0x7ff];
		return 0xffff;
	case 0xc: {
		const uint32_t o = a & 0x7f;
		if (o < 0x20)
			return (uint16_t)(0xff00 | io_r(0, (int)((o >> 1) & 0xf)));
		if (s_board == BD_ANALOG && o >= 0x50 && o < 0x58) {   // msm6253 d7_r
			const uint8_t bit = (b.adc_shift >> 7) & 1;
			b.adc_shift = (uint8_t)(b.adc_shift << 1);
			return (uint16_t)(0xff00 | (bit << 7) | 0x7f);
		}
		if (s_board == BD_TRACK && o >= 0x40 && o < 0x58) {   // upd4701 read_xy
			const int n = (int)((o - 0x40) >> 3), off = (int)((o >> 1) & 3);
			const uint16_t v = (uint16_t)(((off & 2) ? b.upd_y[n] : b.upd_x[n]) & 0xfff);
			return (uint16_t)(0xff00 | ((off & 1) ? (v >> 8) : (v & 0xff)));
		}
		if (s_board == BD_4P && o >= 0x60 && o < 0x68)
			return (uint16_t)(0xff00 | i8255_read(&s_ppi, (int)((o >> 1) & 3)));
		return 0xffff;
	}
	case 0xd:
		if (a < 0xd80000)   // int_control_r: all 0xff (the timer counts are not read back)
			return 0xffff;
		{
			b.rng = 1664525u * b.rng + 1013904223u;   // machine().rand()
			return (uint16_t)(b.rng >> 16);   // (rotated by 16 bits: the low half is the seed's high half)
		}
	}
	return 0xffff;
}

static void main_write16(uint32_t a, uint16_t d, uint16_t mask)
{
	a &= 0xffffff;
	switch (a >> 20) {
	case 0x2:
		if (s_prot == PROT_SONIC && a == 0x20e5c4) {
			sonic_level_load_protection(d, mask);
			return;
		}
		if (s_prot == PROT_JLEAGUE && a >= 0x20f700 && a <= 0x20f705) {
			jleague_protection_w((int)((a - 0x20f700) >> 1), d, mask);
			return;
		}
		ww16(a & 0xffff, (uint16_t)((ws16(a & 0xffff) & ~mask) | (d & mask)));
		return;
	case 0x5:
		if (mask & 0xff)
			b.sprite_control[(a >> 1) & 7] = (uint8_t)d;
		return;
	case 0x6:
		if (!(a & 0x10000))
			paletteram_w((int)((a >> 1) & 0x7fff), d, mask);
		else {
			uint16_t *r = &b.mixer[(a >> 1) & 0x3f];
			*r = (uint16_t)((*r & ~mask) | (d & mask));
		}
		return;
	case 0x8:
		if (!(mask & 0xff))
			return;
		if (a < 0x801000)
			b.comm_shared[(a >> 1) & 0x7ff] = (uint8_t)d;
		else if (a == 0x801000)
			comm_cn_w((uint8_t)d);
		else if (a == 0x801002) {
			if (b.comm_cn)
				b.comm_fg = d & 1;
		}
		return;
	case 0xa:
		if (s_v25 && a < 0xa01000) {
			if (mask & 0xff)
				b.dpram[(a >> 1) & 0x7ff] = (uint8_t)d;
		} else if (s_prot == PROT_BRIVAL && a < 0xa01000)
			brival_protection_w((int)((a >> 1) & 0x7ff), d);
		else if (s_prot == PROT_DBZVRVS && a < 0xa80000)   // write16mo: copy a word of work RAM
			ww16(0x80c8, ws16(0x0044));
		return;
	case 0xc: {
		const uint32_t o = a & 0x7f;
		if (!(mask & 0xff))
			return;
		if (o < 0x20)
			io_w(0, (int)((o >> 1) & 0xf), (uint8_t)d);
		else if (s_board == BD_ANALOG && o >= 0x50 && o < 0x58)   // msm6253 address_w
			b.adc_shift = adc_input((o >> 1) & 3);
		else if (s_board == BD_TRACK && o >= 0x40 && o < 0x58) {   // upd4701 reset_xy_w
			const int n = (int)((o - 0x40) >> 3);
			b.upd_x[n] = b.upd_y[n] = 0;
		} else if (s_board == BD_4P && o >= 0x60 && o < 0x68)
			i8255_write(&s_ppi, (int)((o >> 1) & 3), (uint8_t)d);
		return;
	}
	case 0xd:
		if (a < 0xd80000) {   // int_control_w: an 8-bit handler, both byte lanes
			if (mask & 0x00ff)
				int_control_w((int)(a & 0xe), (uint8_t)d);
			if (mask & 0xff00)
				int_control_w((int)((a & 0xe) | 1), (uint8_t)(d >> 8));
		}
		return;
	}
}

// multi32_map (+ multi32_analog_map / multi32_6player_map): the second palette / mixer at 0x680000, the second I/O
// chip at 0xc80000; the 8-bit devices on umask32 0x00ff00ff (the same byte lanes as the V60's 16-bit accesses)
static uint16_t multi_read16(uint32_t a, uint16_t mask)
{
	a &= 0xffffff;
	switch (a >> 20) {
	case 0x5:
		return (uint16_t)(0xff00 | sprite_control_r((int)((a >> 1) & 7)));
	case 0x6: {
		const int which = (a >> 19) & 1;
		if (!(a & 0x10000))
			return pal_r(which ? mb.paletteram : b.paletteram, (int)((a >> 1) & 0x7fff));
		return (which ? mb.mixer : b.mixer)[(a >> 1) & 0x3f];
	}
	case 0x8:
	case 0xd:
		return main_read16(a, mask);
	case 0xc: {
		const uint32_t o = a & 0x7f;
		if (a & 0x80000)
			return o < 0x20 ? (uint16_t)(0xff00 | io_r(1, (int)((o >> 1) & 0xf))) : 0xffff;
		if (o < 0x20)
			return (uint16_t)(0xff00 | io_r(0, (int)((o >> 1) & 0xf)));
		if (s_board == BD_ANALOG && o >= 0x50 && o < 0x58) {   // msm6253 d7_r
			const uint8_t bit = (b.adc_shift >> 7) & 1;
			b.adc_shift = (uint8_t)(b.adc_shift << 1);
			return (uint16_t)(0xff00 | (bit << 7) | 0x7f);
		}
		if (s_board == BD_4P && o >= 0x60 && o < 0x68)
			return (uint16_t)(0xff00 | i8255_read(&s_ppi, (int)((o >> 1) & 3)));
		return 0xffff;
	}
	}
	return 0xffff;
}
static void multi_write16(uint32_t a, uint16_t d, uint16_t mask)
{
	a &= 0xffffff;
	switch (a >> 20) {
	case 0x5:
		if (mask & 0xff)
			b.sprite_control[(a >> 1) & 7] = (uint8_t)d;
		return;
	case 0x6: {
		const int which = (a >> 19) & 1;
		uint16_t *mixer = which ? mb.mixer : b.mixer;
		if (!(a & 0x10000))
			pal_w(which ? mb.paletteram : b.paletteram, mixer, (int)((a >> 1) & 0x7fff), d, mask);
		else {
			uint16_t *r = &mixer[(a >> 1) & 0x3f];
			*r = (uint16_t)((*r & ~mask) | (d & mask));
		}
		return;
	}
	case 0x8:
	case 0xd:
		main_write16(a, d, mask);
		return;
	case 0xc: {
		const uint32_t o = a & 0x7f;
		if (!(mask & 0xff))
			return;
		if (a & 0x80000) {
			if (o < 0x20)
				io_w(1, (int)((o >> 1) & 0xf), (uint8_t)d);
		} else if (o < 0x20)
			io_w(0, (int)((o >> 1) & 0xf), (uint8_t)d);
		else if (s_board == BD_ANALOG && o >= 0x50 && o < 0x58)   // msm6253 address_w
			b.adc_shift = adc_input((o >> 1) & 3);
		else if (s_board == BD_ANALOG && o == 0x60)   // analog_bank_w
			mb.analog_bank = d & 1;
		else if (s_board == BD_4P && o >= 0x60 && o < 0x68)
			i8255_write(&s_ppi, (int)((o >> 1) & 3), (uint8_t)d);
		return;
	}
	}
}

static void map_main(void)
{
	v60_bus_clear();
	if (s_multi) {
		v60_bus_map(0x000000, 0x1fffff, s_rom, 0x200000, 1, 0);
		v60_bus_map(0x200000, 0x2fffff, mb.workram, 0x20000, 1, 1);
		v60_bus_map(0x300000, 0x3fffff, b.videoram, 0x20000, 1, 1);
		v60_bus_map(0x400000, 0x4fffff, b.spriteram, 0x20000, 1, 1);
		v60_bus_map(0x700000, 0x7fffff, b.zram, 0x2000, 1, 1);
		v60_bus_map(0xf00000, 0xffffff, s_rom, 0x100000, 1, 0);
		v60_bus.read16 = multi_read16;
		v60_bus.write16 = multi_write16;
		v60_bus.irq_cycle = irq_callback;
		return;
	}
	v60_bus_map(0x000000, 0x1fffff, s_rom, 0x200000, 1, 0);
	v60_bus_map(0x200000, 0x2fffff, b.workram, 0x10000, 1, 1);
	v60_bus_map(0x300000, 0x3fffff, b.videoram, 0x20000, 1, 1);
	v60_bus_map(0x400000, 0x4fffff, b.spriteram, 0x20000, 1, 1);
	v60_bus_map(0x700000, 0x7fffff, b.zram, 0x2000, 1, 1);
	v60_bus_map(0xf00000, 0xffffff, s_rom, 0x100000, 1, 0);
	// the protection hooks on work RAM (not on its mirrors)
	if (s_prot == PROT_SONIC) v60_bus.wr[0x20e000 >> V60_PAGE_SHIFT] = NULL;
	if (s_prot == PROT_JLEAGUE) v60_bus.wr[0x20f000 >> V60_PAGE_SHIFT] = NULL;
	if (s_prot == PROT_BRIVAL) v60_bus.rd[0x20b000 >> V60_PAGE_SHIFT] = NULL;
	v60_bus.read16 = main_read16;
	v60_bus.write16 = main_write16;
	v60_bus.irq_cycle = irq_callback;
}

static const uint8_t s_ga2_opcode_table[256] = {
	0x00, 0x00, 0xEA, 0x00, 0x00, 0x8B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFA,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3B, 0x00, 0x49, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0xE8, 0x00, 0x00, 0x75, 0x00, 0x00, 0x00, 0x00, 0x3A, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x8D, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xBF, 0x00, 0x88, 0x00,
	0x00, 0x00, 0x00, 0x81, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xBC,
	0x00, 0x00, 0x00, 0x8A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x83, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xB8, 0x26, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xB5, 0x00, 0xEB,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xB2, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0xC3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xB9, 0xBB, 0x00, 0x43, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x8E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xBE, 0x00, 0x80, 0x00, 0x00,
};
static const uint8_t s_arf_opcode_table[256] = {
	0x00, 0x00, 0x43, 0x00, 0x00, 0x00, 0x83, 0x00, 0x00, 0x00, 0xEA, 0x00, 0x00, 0xBC, 0x73, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x3A, 0x00, 0x00, 0xBE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00,
	0x00, 0xB5, 0x00, 0x00, 0x00, 0x00, 0x00, 0x26, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE8, 0x8D, 0x00, 0x8B, 0x00,
	0x00, 0x00, 0x00, 0xFA, 0x00, 0x8A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xBA, 0x88, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xBB, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x75, 0x00, 0xBF,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x3B, 0x8E, 0x74, 0x00, 0x00, 0x81, 0x00,
	0x00, 0x00, 0x00, 0xC3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xB9, 0xB2, 0x00, 0x00, 0x00, 0x00, 0x49, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xEB, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0xB8,
};

// ---- the V25 (v25_map: the ROM at 0x00000 and 0xf0000, the MB8421 left port at 0x10000) ----
static uint8_t v25_r8(uint32_t a)
{
	a &= 0xfffff;
	if (a < 0x10000 || a >= 0xf0000)
		return s_mcu[a & 0xffff];
	if (a < 0x20000)
		return b.dpram[a & 0x7ff];
	return 0xff;
}
static uint16_t v25_r16(uint32_t a) { return (uint16_t)(v25_r8(a) | (v25_r8(a + 1) << 8)); }
static void v25_w8(uint32_t a, uint8_t v)
{
	a &= 0xfffff;
	if (a >= 0x10000 && a < 0x20000)
		b.dpram[a & 0x7ff] = v;
}
static void v25_w16(uint32_t a, uint16_t v)
{
	v25_w8(a, (uint8_t)v);
	v25_w8(a + 1, (uint8_t)(v >> 8));
}
static void v25_map(void)
{
	for (int i = 0; i < 256; i++)
		s_nec.omap[i] = s_nec.rmap[i] = NULL, s_nec.wmap[i] = NULL;
	nec_map(&s_nec, 0x00000, 0x10000, s_mcu, NULL);
	nec_map(&s_nec, 0xf0000, 0x100000, s_mcu, NULL);
}

// ---- the sound CPU ----
static void sound_bank_update(void)
{
	const uint32_t off = (uint32_t)b.sound_bank * 0x2000;
	for (int i = 0; i < 0x20; i++)
		s_zmap[0xa0 + i] = off + 0x2000 <= s_zrom_size ? s_zrom + off + i * 256 : NULL;
}
static uint8_t z_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0xa000) {
		if (s_zscan_n && s_exec == CPU_Z80 && (a == s_zscan_h[0] || a == s_zscan_h[s_zscan_n - 1]) && s_z80.pc == (uint16_t)(a + 1))
			s_zscan_hit = z80_break = 1;   // (a scan loop's BIT fetched: z80_run ends after it, zscan goes on)
		return a < s_zrom_size ? s_zrom[a] : 0xff;
	}
	if (a < 0xc000) {
		const uint32_t o = (uint32_t)b.sound_bank * 0x2000 + (a - 0xa000);
		return o < s_zrom_size ? s_zrom[o] : 0xff;
	}
	if (a < 0xe000) {
		if (s_multi)
			return 0;   // (multipcm read)
		if (a >= 0xd000)
			return s_rf.ram[(s_rf.wbank | (a & 0xfff)) & 0xffff];
		return 0xff;
	}
	return b.zram[a - 0xe000];
}
static void pcm_user(uint8_t a, uint16_t v, uint32_t n)   // (the worker's: in the order queued)
{
	switch (a) {
	case 0: multipcm_write(s_mpcm, v >> 8, (uint8_t)v); break;
	case 1: multipcm_set_banks(s_mpcm, v >> 8, v & 0xff); break;
	default:
		multipcm_render_lr(s_mpcm, s_pcm_l + s_pcm_pos, s_pcm_r + s_pcm_pos, (int)n, s_pcm_rate, 90);
		s_pcm_pos += (int)n;
		break;
	}
}
static void pcm_write(int offset, uint8_t v)
{
	if (s_pcm_async)
		rbx_ym_async_user(0, 0, (uint16_t)(offset << 8 | v), 0);
	else
		multipcm_write(s_mpcm, offset, v);
}
static void pcm_banks(uint32_t lo, uint32_t hi)
{
	if (s_pcm_async)
		rbx_ym_async_user(0, 1, (uint16_t)(lo << 8 | hi), 0);
	else
		multipcm_set_banks(s_mpcm, lo, hi);
}
static void z_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0xe000)
		b.zram[a - 0xe000] = v;
	else if (s_multi) {
		if (a >= 0xc000 && a < 0xc003)
			pcm_write(a - 0xc000, v);
		else if (s_scross && a >= 0xb0 && a <= 0xbf)   // scross_bank_w
			pcm_banks(v & 7, v & 7);
	} else if (a >= 0xd000)
		rf5c68_mem_w(&s_rf, a & 0xfff, v);
	else if (a >= 0xc000 && (a & 0x0f) <= 8)
		rf5c68_w(&s_rf, a & 0x0f, v);
}
static uint8_t z_in(z80 *z, uint8_t p)
{
	(void)z;
	if ((p & 0xf0) == 0x80) return rbx_ym_read(0, p & 3);
	if ((p & 0xf0) == 0x90) return s_multi ? 0xff : rbx_ym_read(1, p & 3);
	if (p == 0xf1) return b.sound_dummy;
	return 0xff;
}
static void z_out(z80 *z, uint8_t p, uint8_t v)
{
	(void)z;
	switch (p & 0xf0) {
	case 0x80: rbx_ym_write(0, p & 3, v); break;
	case 0x90: if (!s_multi) rbx_ym_write(1, p & 3, v); break;
	case 0xa0:
		b.sound_bank = (uint16_t)((b.sound_bank & ~0x3f) | (v & 0x3f));
		sound_bank_update();
		break;
	case 0xb0:
		if (s_multi) {   // multipcm_bank_w
			pcm_banks(v & 7, (v >> 3) & 7);
			break;
		}
		b.sound_bank = (uint16_t)((b.sound_bank & 0x3f) | ((v & 0x04) << 4) | ((v & 0x03) << 7));
		sound_bank_update();
		break;
	case 0xc0:   // sound_int_control_lo_w
		if (p & 1) {
			b.sound_irq_input &= v;
			update_sound_irq_state();
		}
		if (p & 4)
			signal_v60_irq(MAIN_IRQ_SOUND);
		break;
	case 0xd0:
		if ((p & 0x0f) < 8) {
			b.sound_irq_control[p & 3] = v;
			update_sound_irq_state();
		}
		break;
	case 0xf0:
		if (p == 0xf1)
			b.sound_dummy = v;
		break;
	}
}

// ---- sound out ----
static void sound_line(void)
{
	const uint32_t clocks = (uint32_t)rbx_line_cycles(&b.ym_frac, YM_CLOCK, 60000, LINES);
	if (s_multi) {   // two mono speakers: sleft from the chips' outputs 1, sright from 0
		const int n = rbx_ym_advance_lr(0, clocks, s_mix_r + s_mix_n, s_mix_l + s_mix_n, s_mix_cap - s_mix_n);
		if (s_pcm_async)
			rbx_ym_async_user(0, 2, 0, (uint32_t)n);   // (made at s_pcm_l / r + s_mix_n)
		else
			multipcm_render_lr(s_mpcm, s_mix_r + s_mix_n, s_mix_l + s_mix_n, n, rbx_ym_rate(0), 90);
		s_mix_n += n;
		return;
	}
	const int n = rbx_ym_advance_lr(0, clocks, s_mix_l + s_mix_n, s_mix_r + s_mix_n, s_mix_cap - s_mix_n);
	rbx_ym_advance_lr(1, clocks, s_mix_l + s_mix_n, s_mix_r + s_mix_n, s_mix_cap - s_mix_n);
	rf5c68_render_lr(&s_rf, s_mix_l + s_mix_n, s_mix_r + s_mix_n, n, rbx_ym_rate(0), 102);
	s_mix_n += n;
}
static void sound_frame_end(void)
{
	if (s_multi) {
		rbx_ym_async_take_lr(0, s_mix_r, s_mix_l, s_mix_n);
		if (s_pcm_async) {
			rbx_aux_sync();   // (all the MultiPCM steps queued: s_pcm_pos == s_mix_n)
			for (int i = 0; i < s_mix_n; i++) {
				s_mix_r[i] += s_pcm_l[i];
				s_mix_l[i] += s_pcm_r[i];
			}
			memset(s_pcm_l, 0, sizeof(int32_t) * (size_t)s_mix_n);
			memset(s_pcm_r, 0, sizeof(int32_t) * (size_t)s_mix_n);
			s_pcm_pos = 0;
		}
	} else {
		rbx_ym_async_take_lr(0, s_mix_l, s_mix_r, s_mix_n);
		rbx_ym_async_take_lr(1, s_mix_l, s_mix_r, s_mix_n);
	}
	for (int i = 0; i < s_mix_n; i++)
		rbx_audio_put_stereo(s_mix_l[i], s_mix_r[i]);
	memset(s_mix_l, 0, sizeof(int32_t) * (size_t)s_mix_n);
	memset(s_mix_r, 0, sizeof(int32_t) * (size_t)s_mix_n);
	s_mix_n = 0;
}

// =====================================================================================================================
// ---- video (segas32_v.cpp) ----
enum { L_TEXT, L_NBG0, L_NBG1, L_NBG2, L_NBG3, L_BITMAP, L_SPRITES, L_BACKGROUND };

static inline int sext(int v, int bits) { return (int)((uint32_t)v << (32 - bits)) >> (32 - bits); }

// the NBG tiles: 16x16 4bpp, rearranged at init to rows of 8 bytes, the even pixel in the high nibble
static uint32_t s_ntiles;
// one row of a tile of a page's pixmap (512x256), decoded once for the pixels that fall in it (pen 0: 0, as drawn)
typedef struct { int key; uint16_t px[16]; } tilerow_t;
// a byte's two pixels as a little-endian pair (colour | pen, 0 for pen 0): [0] the high nibble first, [1] last
static uint32_t s_nibv[2][256], s_nibm[2][256];
static void nib_init(void)
{
	for (int v = 0; v < 256; v++) {
		const uint32_t hi = (uint32_t)v >> 4, lo = (uint32_t)v & 15;
		s_nibv[0][v] = hi | (lo << 16);
		s_nibm[0][v] = (hi ? 0xffffu : 0) | (lo ? 0xffff0000u : 0);
		s_nibv[1][v] = lo | (hi << 16);
		s_nibm[1][v] = (lo ? 0xffffu : 0) | (hi ? 0xffff0000u : 0);
	}
}
// the 16 pixels of the row (rev: right to left)
static inline __attribute__((always_inline)) void tile_row_px(uint16_t *px, const uint16_t *vram, int page, int bank, int x,
                                                              int y, int rev)
{
	const uint16_t data = vram[((page & 0x7f) << 9) | ((y >> 4) << 5) | (x >> 4)];
	const uint32_t code = rbx_wrap(((uint32_t)bank << 13) | (data & 0x1fff), s_ntiles);
	const int py = (data & 0x8000) ? 15 - (y & 15) : (y & 15);
	const uint8_t *row = s_tiles + code * 128 + py * 8;
	const uint32_t cc = (((data >> 4) & 0x1ffu) << 4) * 0x10001u;
	const int f = ((data >> 14) & 1) ^ rev;
#if defined(__ARM_NEON)
	{   // (the same pixels 16 at a time: the nibbles zipped, widened, the colour or'ed where the pen is not 0)
		const uint8x8_t nb = vld1_u8(row);
		const uint8x8x2_t z = vzip_u8(vshr_n_u8(nb, 4), vand_u8(nb, vdup_n_u8(0x0f)));
		const uint16x8_t c16 = vdupq_n_u16((uint16_t)cc);
		uint16x8_t p0 = vmovl_u8(z.val[0]), p1 = vmovl_u8(z.val[1]);
		p0 = vandq_u16(vorrq_u16(p0, c16), vtstq_u16(p0, p0));
		p1 = vandq_u16(vorrq_u16(p1, c16), vtstq_u16(p1, p1));
		if (f) {   // (right to left)
			const uint16x8_t r0 = vrev64q_u16(p1), r1 = vrev64q_u16(p0);
			p0 = vcombine_u16(vget_high_u16(r0), vget_low_u16(r0));
			p1 = vcombine_u16(vget_high_u16(r1), vget_low_u16(r1));
		}
		vst1q_u16(px, p0);
		vst1q_u16(px + 8, p1);
		return;
	}
#endif
	const uint32_t *pv = s_nibv[f], *pm = s_nibm[f];
	for (int i = 0; i < 8; i++) {
		const uint32_t w = (pv[row[i]] | cc) & pm[row[i]];
		memcpy(px + (f ? 14 - 2 * i : 2 * i), &w, 4);
	}
}
static inline __attribute__((always_inline)) const uint16_t *tm_row(tilerow_t *c, const uint16_t *vram, int page, int bank, int x, int y)
{
	const int key = ((page & 0x7f) << 13) | ((x >> 4) << 8) | y;
	if (c->key == key)
		return c->px;
	c->key = key;
	tile_row_px(c->px, vram, page, bank, x, y, 0);
	return c->px;
}

// n pixels of a page pair's row from the integer source x u (bit 9: the page), stepping dir (+1 / -1): tile by tile
static inline void span_px(tilerow_t *tc, const uint16_t *vram, int pa, int pb, int bank, int sy, int u, int dir,
                           uint16_t *dst, int n)
{
	while (n > 0) {
		const int o = u & 15;
		if (n >= 16 && o == (dir > 0 ? 0 : 15)) {   // (a whole tile: straight into the line)
			tile_row_px(dst, vram, ((u >> 9) & 1) ? pb : pa, bank, u & 0x1ff, sy, dir < 0);
			dst += 16;
			u += dir * 16;
			n -= 16;
			continue;
		}
		const uint16_t *px = tm_row(tc, vram, ((u >> 9) & 1) ? pb : pa, bank, u & 0x1ff, sy);
		int m = dir > 0 ? 16 - o : o + 1;
		if (m > n)
			m = n;
		if (dir > 0)
			for (int i = 0; i < m; i++)
				dst[i] = px[o + i];
		else
			for (int i = 0; i < m; i++)
				dst[i] = px[o - i];
		dst += m;
		u += dir * m;
		n -= m;
	}
}

static void compute_clipping_extents(const uint16_t *vram, int width, int enable, int clipout, int clipmask, extents_t *list)
{
	const int flip = (vram[0x1ff00 / 2] >> 9) & 1;
	const int tminx = 0, tmaxx = width, tminy = 0, tmaxy = SH;   // (the cliprect expanded by one)
	list->extent[0][0] = (uint16_t)tminx;
	list->extent[0][1] = (uint16_t)tmaxx;
	if (!enable) {
		memset(&list->scan_extent[tminy], 0, (size_t)(tmaxy - tminy));
		list->clipdraw_start = 1;
		return;
	}
	int cminx[5], cminy[5], cmaxx[5], cmaxy[5], sorted[5];
	for (int i = 0; i < 5; i++) {
		if (!flip) {
			cminx[i] = vram[0x1ff60 / 2 + i * 4] & 0x1ff;
			cminy[i] = vram[0x1ff62 / 2 + i * 4] & 0x0ff;
			cmaxx[i] = (vram[0x1ff64 / 2 + i * 4] & 0x1ff) + 1;
			cmaxy[i] = (vram[0x1ff66 / 2 + i * 4] & 0x0ff) + 1;
		} else {
			cmaxx[i] = width - (vram[0x1ff60 / 2 + i * 4] & 0x1ff);
			cmaxy[i] = SH - (vram[0x1ff62 / 2 + i * 4] & 0x0ff);
			cminx[i] = width - ((vram[0x1ff64 / 2 + i * 4] & 0x1ff) + 1);
			cminy[i] = SH - ((vram[0x1ff66 / 2 + i * 4] & 0x0ff) + 1);
		}
		// clips[i] &= tempclip (rectangle intersection, inclusive max as MAME's: max kept as given)
		if (cminx[i] < tminx) cminx[i] = tminx;
		if (cminy[i] < tminy) cminy[i] = tminy;
		if (cmaxx[i] > tmaxx) cmaxx[i] = tmaxx;
		if (cmaxy[i] > tmaxy) cmaxy[i] = tmaxy;
		sorted[i] = i;
	}
	for (int i = 0; i < 5; i++)
		for (int j = i + 1; j < 5; j++)
			if (cminx[sorted[i]] > cminx[sorted[j]]) { const int t = sorted[i]; sorted[i] = sorted[j]; sorted[j] = t; }
	for (int i = 1; i < 32; i++) {
		if (!(i & clipmask))
			continue;
		uint16_t *extent = &list->extent[i][0];
		*extent++ = (uint16_t)tminx;
		for (int j = 0; j < 5; j++) {
			if ((i >> sorted[j]) & 1) {
				const int k = sorted[j];
				if (extent != &list->extent[i][1] && cminx[k] <= extent[-1]) {
					if (cmaxx[k] > extent[-1])
						extent[-1] = (uint16_t)cmaxx[k];
				} else {
					*extent++ = (uint16_t)cminx[k];
					*extent++ = (uint16_t)cmaxx[k];
				}
			}
		}
		*extent++ = (uint16_t)tmaxx;
	}
	for (int y = tminy; y < tmaxy; y++) {
		int sect = 0;
		for (int i = 0; i < 5; i++)
			if (((clipmask >> i) & 1) && y >= cminy[i] && y < cmaxy[i])
				sect |= 1 << i;
		list->scan_extent[y] = (uint8_t)sect;
	}
	list->clipdraw_start = (uint8_t)clipout;
}

static void compute_tilemap_flips(const uint16_t *vram, int bgnum, int *flipx, int *flipy)
{
	const int global_flip = (vram[0x1ff00 / 2] >> 9) & 1;
	const int layer_flip = (vram[0x1ff00 / 2] >> bgnum) & 1;
	const int prohibit_flipy = (vram[0x1ff00 / 2] >> 8) & 1;
	*flipx = layer_flip ? !global_flip : global_flip;
	*flipy = (layer_flip && !prohibit_flipy) ? !global_flip : global_flip;
}

static void get_pages(const uint16_t *vram, int bgnum, int pages[4])
{
	pages[0] = (vram[0x1ff40 / 2 + 2 * bgnum + 0] >> 0) & 0x7f;
	pages[1] = (vram[0x1ff40 / 2 + 2 * bgnum + 0] >> 8) & 0x7f;
	pages[2] = (vram[0x1ff40 / 2 + 2 * bgnum + 1] >> 0) & 0x7f;
	pages[3] = (vram[0x1ff40 / 2 + 2 * bgnum + 1] >> 8) & 0x7f;
}

// one line of a layer through its clip extents: draw(x) for the drawn spans, 0 for the clipped ones
#define EXTENT_LOOP(EXT, Y, W, DRAWSPAN, SKIPSPAN)                                        \
	{                                                                                     \
		const uint16_t *extents = &(EXT)->extent[(EXT)->scan_extent[Y]][0];               \
		int clipdraw = (EXT)->clipdraw_start;                                             \
		if (clipdraw || extents[1] <= (W) - 1) {                                          \
			for (;;) {                                                                    \
				const int x0 = extents[0], x1 = extents[1];                               \
				if (clipdraw) { DRAWSPAN } else { SKIPSPAN }                              \
				if (extents[1] > (W) - 1)                                                 \
					break;                                                                \
				clipdraw = !clipdraw;                                                     \
				extents++;                                                                \
			}                                                                             \
		}                                                                                 \
	}

// a zooming layer's line-independent values (the frame's, made by zoom_setup before the bands: two divisions a line
// were on the line's path, the Cortex-A9 has no divide instruction)
typedef struct {
	int pages[4];
	uint32_t srcx_start, srcxstep, srcy0, srcystep;
	int unit;   // (no zoom: spans, the direction)
} zoomp_t;
static zoomp_t s_zp[2];
static void zoom_setup(const struct s32_snap *sn, int bgnum, zoomp_t *z)
{
	const uint16_t *vram = sn->videoram;
	const int W = sn->width;
	get_pages(vram, bgnum, z->pages);
	int flipx, flipy;
	compute_tilemap_flips(vram, bgnum, &flipx, &flipy);
	int dstxstep = vram[0x1ff50 / 2 + 2 * bgnum] & 0xfff;
	int dstystep = ((vram[0x1ff00 / 2] >> 14) & 1) ? (vram[0x1ff52 / 2 + 2 * bgnum] & 0xfff) : dstxstep;
	if (dstxstep < 0x80) dstxstep = 0x80;
	if (dstystep < 0x80) dstystep = 0x80;
	uint32_t srcxstep = (0x200u << 20) / (uint32_t)dstxstep;
	uint32_t srcystep = (0x200u << 20) / (uint32_t)dstystep;
	uint32_t srcx_start = (uint32_t)(vram[0x1ff12 / 2 + 4 * bgnum] & 0x3ff) << 20;
	srcx_start += (uint32_t)(vram[0x1ff10 / 2 + 4 * bgnum] & 0xff00) << 4;
	uint32_t srcy = (uint32_t)(vram[0x1ff16 / 2 + 4 * bgnum] & 0x1ff) << 20;
	srcy += (uint32_t)(vram[0x1ff14 / 2 + 4 * bgnum] & 0xfe00) << 4;
	srcx_start -= (uint32_t)(sext(vram[0x1ff30 / 2 + 2 * bgnum], (dstxstep != 0x200) ? 10 : 9) * (int32_t)srcxstep);
	srcy -= (uint32_t)(sext(vram[0x1ff32 / 2 + 2 * bgnum], (dstystep != 0x200) ? 10 : 9) * (int32_t)srcystep);
	if (flipy) {
		srcy += (uint32_t)(SH - 1) * srcystep;
		srcystep = (uint32_t)-(int32_t)srcystep;
	}
	if (flipx) {
		srcx_start += (uint32_t)(W - 1) * srcxstep;
		srcxstep = (uint32_t)-(int32_t)srcxstep;
	}
	z->srcx_start = srcx_start;
	z->srcxstep = srcxstep;
	z->srcy0 = srcy;
	z->srcystep = srcystep;
	z->unit = srcxstep == (1u << 20) ? 1 : srcxstep == (uint32_t)-(1 << 20) ? -1 : 0;
}
static void line_zoom(const struct s32_snap *sn, int bgnum, int y, uint16_t *dst)
{
	const uint16_t *vram = sn->videoram;
	const int W = sn->width;
	const zoomp_t *const z = &s_zp[bgnum];
	const int *const pages = z->pages;
	const uint32_t srcxstep = z->srcxstep;
	const uint32_t srcy = z->srcy0 + (uint32_t)y * z->srcystep;
	const int bank = sn->tilebank[bgnum];
	const int pa = pages[((srcy >> 27) & 2) + 0], pb = pages[((srcy >> 27) & 2) + 1];
	const int sy = (int)((srcy >> 20) & 0xff);
	uint32_t srcx = z->srcx_start;
	tilerow_t tc;
	tc.key = -1;
	const int unit = z->unit;   // (no zoom: spans)
	EXTENT_LOOP(&sn->ext[1 + bgnum], y, W,
		if (unit) {
			span_px(&tc, vram, pa, pb, bank, sy, (int)(srcx >> 20), unit, dst + x0, x1 - x0);
			srcx += srcxstep * (uint32_t)(x1 - x0);
		} else
		for (int x = x0; x < x1; x++) {
			const int sx = (int)((srcx >> 20) & 0x1ff);
			dst[x] = tm_row(&tc, vram, ((srcx >> 29) & 1) ? pb : pa, bank, sx, sy)[sx & 15];
			srcx += srcxstep;
		},
		srcx += srcxstep * (uint32_t)(x1 - x0);)
}

static void line_rowscroll(const struct s32_snap *sn, int bgnum, int y, uint16_t *dst)
{
	const uint16_t *vram = sn->videoram;
	const int W = sn->width;
	int pages[4];
	get_pages(vram, bgnum, pages);
	int flipx, flipy;
	compute_tilemap_flips(vram, bgnum, &flipx, &flipy);
	int rowscroll = (vram[0x1ff04 / 2] >> (bgnum - 2)) & 1;
	int rowselect = (vram[0x1ff04 / 2] >> bgnum) & 1;
	if ((vram[0x1ff04 / 2] >> (bgnum + 2)) & 1)
		rowscroll = rowselect = 0;
	const uint16_t *table = &vram[(vram[0x1ff04 / 2] >> 10) * 0x400];
	const int xscroll = (vram[0x1ff12 / 2 + 4 * bgnum] & 0x3ff) - (vram[0x1ff30 / 2 + 2 * bgnum] & 0x1ff);
	const int yscroll = (vram[0x1ff16 / 2 + 4 * bgnum] & 0x1ff);
	int srcx, srcxstep;
	if (!flipx) {
		srcx = xscroll;
		srcxstep = 1;
	} else {
		srcx = W - 1 + xscroll;
		srcxstep = -1;
	}
	int srcy, ylookup;
	if (!flipy) {
		srcy = yscroll + y;
		ylookup = y;
	} else {
		srcy = yscroll + SH - 1 - y;
		ylookup = SH - 1 - y;
	}
	if (rowscroll)
		srcx += table[0x000 + 0x100 * (bgnum - 2) + ylookup] & 0x3ff;
	if (rowselect)
		srcy = (yscroll + table[0x200 + 0x100 * (bgnum - 2) + ylookup]) & 0x1ff;
	const int bank = sn->tilebank[bgnum];
	const int pa = pages[((srcy >> 7) & 2) + 0], pb = pages[((srcy >> 7) & 2) + 1];
	const int sy = srcy & 0xff;
	tilerow_t tc;
	tc.key = -1;
	EXTENT_LOOP(&sn->ext[1 + bgnum], y, W,
		span_px(&tc, vram, pa, pb, bank, sy, srcx, srcxstep, dst + x0, x1 - x0);
		srcx += srcxstep * (x1 - x0);,
		srcx += srcxstep * (x1 - x0);)
}

static void line_text(const struct s32_snap *sn, int y, uint16_t *dst)
{
	const uint16_t *vram = sn->videoram;
	const int W = sn->width;
	const int flip = (vram[0x1ff00 / 2] >> 9) & 1;
	const uint16_t ctl = vram[0x1ff5c / 2];
	const uint32_t base = (uint32_t)((ctl >> 4) & 0x1f) << 11;
	const int ly = flip ? ((SH - 1 - y) & 0xff) : y;
	const int ty = ly >> 3, py = ly & 7;
	for (int x = 0; x < W; x += 8) {   // (W a multiple of 8: a character's 8 pixels, in reverse when flipped)
		const int lx = flip ? ((W - 1 - x) & 0x1ff) : x;
		const uint16_t data = vram[base | (ty << 6) | (lx >> 3)];
		const uint32_t code = (uint32_t)((ctl & 7) << 9) | (data & 0x1ff);
		const uint32_t ba = code * 32 + (uint32_t)py * 4;   // (the gfx's byte address in video RAM)
		const uint16_t w0 = vram[(ba >> 1) & 0xffff], w1 = vram[((ba >> 1) + 1) & 0xffff];
		const uint32_t row = (uint32_t)(((w0 & 0xff) << 24) | ((w0 >> 8) << 16) | ((w1 & 0xff) << 8) | (w1 >> 8));   // (pixel 0 in the top nibble)
		uint16_t *d = dst + x;
		if (!row) {
			memset(d, 0, 16);
			continue;
		}
		// (a byte's two pixels at once, as tile_row_px: colour | pen, 0 for pen 0; flipped: the bytes from the last, the
		// low nibble first)
		const uint32_t cc = (((data >> 9) & 0x7fu) << 4) * 0x10001u;
		uint32_t w[4];
		if (!flip)
			for (int i = 0; i < 4; i++) {
				const uint8_t by = (uint8_t)(row >> (24 - 8 * i));
				w[i] = (s_nibv[0][by] | cc) & s_nibm[0][by];
			}
		else
			for (int i = 0; i < 4; i++) {
				const uint8_t by = (uint8_t)(row >> (8 * i));
				w[i] = (s_nibv[1][by] | cc) & s_nibm[1][by];
			}
		memcpy(d, w, 16);
	}
}

static void line_bitmap(const struct s32_snap *sn, int y, uint16_t *dst)
{
	const uint16_t *vram = sn->videoram;
	const int W = sn->width;
	const int bpp = ((vram[0x1ff00 / 2] >> 11) & 1) ? 8 : 4;
	const int xscroll = vram[0x1ff88 / 2] & 0x1ff;
	const int yscroll = vram[0x1ff8a / 2] & 0x1ff;
	const int color = (vram[0x1ff8c / 2] << 4) & 0x1fff0 & ~((1 << bpp) - 1);
	EXTENT_LOOP(&sn->ext[5], y, W,
		if (bpp == 8) {
			const uint16_t *src = &vram[512 / 2 * ((y + yscroll) & 0xff)];
			for (int x = x0; x < x1; x++) {
				const int effx = (x + xscroll) & 0x1ff;
				int pix = ((src[effx >> 1] >> ((effx & 1) * 8)) & 0xff) + color;
				if ((pix & 0xff) == 0) pix = 0;
				dst[x] = (uint16_t)(pix & 0x1fff);   // (mix_line's mask, applied here: a pixel is drawn when not 0)
			}
		} else {
			const uint16_t *src = &vram[512 / 4 * ((y + yscroll) & 0x1ff)];
			for (int x = x0; x < x1; x++) {
				const int effx = (x + xscroll) & 0x1ff;
				int pix = ((src[effx / 4] >> (4 * (effx & 3))) & 0x0f) + color;
				if ((pix & 0x0f) == 0) pix = 0;
				dst[x] = (uint16_t)(pix & 0x1fff);
			}
		},
		(void)x0; (void)x1;)
}

// ---- sprites (draw_one_sprite / sprite_render_list, in the emulation thread at the sprite chip's time) ----
typedef struct { int min_x, max_x, min_y, max_y; } rect_t;
static inline uint32_t spr_ram32(uint32_t i)
{
	const uint16_t w0 = s_sj.ram[(i * 2) & 0xffff], w1 = s_sj.ram[(i * 2 + 1) & 0xffff];
	return (uint32_t)(uint16_t)((w1 >> 8) | (w1 << 8)) | ((uint32_t)(uint16_t)((w0 >> 8) | (w0 << 8)) << 16);
}
static inline uint32_t spr_rom32(uint32_t i)
{
	uint32_t v;
	memcpy(&v, s_sprites + (size_t)i * 4, 4);
	return v;
}


static int draw_one_sprite(const uint16_t *data, int xoffs, int yoffs, rect_t clipin, rect_t clipout)
{
	static const int transparency_masks[4][4] = {
		{ 0x7fff, 0x3fff, 0x1fff, 0x0fff },
		{ 0x3fff, 0x1fff, 0x0fff, 0x07ff },
		{ 0x3fff, 0x1fff, 0x0fff, 0x07ff },
		{ 0x1fff, 0x0fff, 0x07ff, 0x03ff }
	};
	uint16_t *bitmap = s_sprp[s_multi && ((data[3] >> 11) & 1)][s_sj.dsti];   // MIXER_LAYER_SPRITES_2 / MULTISPR_2
	const uint8_t numbanks = (uint8_t)((s_sprites_size / 4) >> 20);   // (m_sprite_region.length(): in 32-bit words)
	const int indirect = (data[0] >> 13) & 1;
	const int indlocal = (data[0] >> 12) & 1;
	const int shadow = ((data[0] >> 11) & 1) && (s_sj.latched[0x0a / 2] & 1);
	const int fromram = (data[0] >> 10) & 1;
	const int bpp8 = (data[0] >> 9) & 1;
	const int transp = ((data[0] >> 8) & 1) ? 0 : (bpp8 ? 0xff : 0x0f);
	const int flipy = (data[0] >> 7) & 1;
	const int flipx = (data[0] >> 6) & 1;
	const int offsety = (data[0] >> 5) & 1;
	const int offsetx = (data[0] >> 4) & 1;
	const int adjusty = (data[0] >> 2) & 3;
	const int adjustx = (data[0] >> 0) & 3;
	const int srch = (data[1] >> 8);
	const int srcw = bpp8 ? (data[1] & 0x3f) : ((data[1] >> 1) & 0x3f);
	int bank = s_multi ? ((data[3] & 0x2000) >> 13) | ((data[3] & 0x8000) >> 14)
	                   : ((data[3] & 0x0800) >> 11) | ((data[3] & 0x4000) >> 13);
	const int dsth = data[2] & 0x3ff;
	const int dstw = data[3] & 0x3ff;
	int ypos = sext(data[4], 12);
	int xpos = sext(data[5], 12);
	uint32_t addr = data[6] | ((uint32_t)(data[2] & 0xf000) << 4);
	const int color = 0x8000 | (data[7] & (bpp8 ? 0x7f00 : 0x7ff0));
	int xdelta = 1, ydelta = 1, yacc = 0, transmask;
	uint32_t addrmask, sbase = 0;
	uint16_t indtable[16];
	memset(indtable, 0, sizeof(indtable));

	if (srcw == 0 || srch == 0 || dstw == 0 || dsth == 0)
		goto bail;
	transmask = transparency_masks[s_sj.latched[0x08 / 2] & 3][s_sj.latched[0x0a / 2] & 3];
	if (bpp8)
		transmask &= 0xfff0;
	if (indirect) {
		const uint16_t *src = indlocal ? &data[8] : &s_sj.ram[8 * (data[7] & 0x1fff)];
		for (int x = 0; x < 16; x++)
			indtable[x] = (uint16_t)((src[x] & (bpp8 ? 0xfff0 : 0xffff)) | ((s_sj.latched[0x0a / 2] & 1) ? 0x8000 : 0x0000));
	}
	if (fromram)
		addrmask = (0x20000 / 4) - 1;
	else {
		if (numbanks)
			bank %= numbanks;
		sbase = (uint32_t)bank << 20;   // (in 32-bit words)
		addrmask = 0xfffff;
		if (((size_t)sbase + addrmask + 1) * 4 > s_sprites_size)
			addrmask = 0;   // (no ROM there)
	}
	{
		const int hzoom = (((bpp8 ? 4 : 8) * srcw) << 16) / dstw;
		const int vzoom = (srch << 16) / dsth;
		if (offsetx)
			xpos += xoffs;
		switch (adjustx) {
		case 0: case 3: xpos -= (dstw - 1) / 2; break;
		case 1: xpos -= dstw - 1; break;
		case 2: break;
		}
		if (offsety)
			ypos += yoffs;
		switch (adjusty) {
		case 0: case 3: ypos -= (dsth - 1) / 2; break;
		case 1: ypos -= dsth - 1; break;
		case 2: break;
		}
		if (flipx) {
			xpos += dstw - 1;
			xdelta = -1;
		}
		if (flipy) {
			ypos += dsth - 1;
			ydelta = -1;
		}
		int xtarget = xpos + xdelta * dstw;
		const int ytarget = ypos + ydelta * dsth;
		if (xdelta > 0 && xtarget > clipin.max_x) {
			xtarget = clipin.max_x + 1;
			if (xpos >= xtarget)
				goto bail;
		}
		if (xdelta < 0 && xtarget < clipin.min_x) {
			xtarget = clipin.min_x - 1;
			if (xpos <= xtarget)
				goto bail;
		}
		const int n = (xtarget - xpos) * xdelta;   // (the destination pixels of a row)
		// the destination pixel k is the source pixel (k * hzoom) >> 16 (MAME's xacc loop); the row's words are
		// read up to the one of the last pixel, a transparent end marker (the last pixel 0x0f / 0xff) stopping it:
		// a pixel of word w is drawn when no word below min(w, wlast) is a marker
		const uint32_t wlast = (uint32_t)((((uint64_t)(n - 1) * (uint32_t)hzoom) >> 16) >> (bpp8 ? 2 : 3));
		const int wsh = bpp8 ? 2 : 3, np = 1 << wsh;
#define SPR_WORD(W) (fromram ? spr_ram32((addr + (W)) & addrmask) : (addrmask ? spr_rom32(sbase + ((addr + (W)) & addrmask)) : 0))
#define SPR_MARK(P) (((P) & (uint32_t)transp) == (uint32_t)transp)
		// the k of clipin (every row's drawn pixels are among them)
		int kc0, kc1;
		if (xdelta > 0) { kc0 = clipin.min_x - xpos; kc1 = clipin.max_x - xpos; }
		else { kc0 = xpos - clipin.max_x; kc1 = xpos - clipin.min_x; }
		if (kc0 < 0) kc0 = 0;
		if (kc1 > n - 1) kc1 = n - 1;
		if (kc0 > kc1)
			goto bail;
		// k -> source pixel, the same for every row (s < 8 * srcw <= 504: n <= dstw); made at the first drawn row
		uint16_t smap[1024];
		int smap_ok = 0;
		// a 4bpp pixel's value as drawn (-1: none), inside the word (trans 0) and at its ends (trans = transp)
		int32_t lut[2][16];
		if (!bpp8)
			for (int e = 0; e < 2; e++)
				for (int pix = 0; pix < 16; pix++) {
					int32_t v = -1;
					if (pix != (e ? transp : 0)) {
						if (!indirect) {
							if (pix != 0)
								v = color | pix;
						} else if ((indtable[pix] & transmask) != transmask)
							v = indtable[pix];
					}
					lut[e][pix] = v;
				}
		int32_t rowv[64 * 8];   // (the row's source pixels as drawn, by s)
		// a 4bpp sprite shrunk: each drawn pixel taken from its word (most source pixels are not drawn), by k the
		// word and the nibble's shift | 16 at the word's ends (lut[1])
		const int direct = !bpp8 && hzoom > 0x10000;
		uint8_t kw[1024], kse[1024];
		uint32_t wv[64];
		for (int y = ypos; y != ytarget; y += ydelta) {
			if (y >= clipin.min_y && y <= clipin.max_y) {
				if (!smap_ok) {
					for (int k = kc0; k <= kc1; k++) {
						const unsigned s = (unsigned)(((uint64_t)k * (uint32_t)hzoom) >> 16);
						smap[k] = (uint16_t)s;
						kw[k] = (uint8_t)(s >> 3);
						kse[k] = (uint8_t)((28 - 4 * (s & 7)) | ((s & 7) == 0 || (s & 7) == 7 ? 1 : 0));
					}
					smap_ok = 1;
				}
				const int do_clipout = (y >= clipout.min_y && y <= clipout.max_y);
				uint16_t *const dest = &bitmap[y * SW];
				// the words of the row's pixels, the end marker
				const uint32_t wa = (uint32_t)smap[kc0] >> wsh;
				uint32_t wb = (uint32_t)smap[kc1] >> wsh, slimit = 0xffffffffu;
				if (transp != 0) {
					const uint32_t we = wb < wlast ? wb : wlast;
					for (uint32_t i = 0; i < we; i++)
						if (SPR_MARK(SPR_WORD(i))) {
							slimit = (i + 1) << wsh;
							wb = i;
							break;
						}
				}
				if (wa > wb)
					goto next_row;
				if (direct)
					for (uint32_t w = wa; w <= wb; w++)
						wv[w] = SPR_WORD(w);
				else for (uint32_t w = wa; w <= wb; w++) {
					const uint32_t pixels = SPR_WORD(w);
					int32_t *const rv = &rowv[w << wsh];
					if (!bpp8) {
						rv[0] = lut[1][pixels >> 28];
						for (int p = 1; p < 7; p++)
							rv[p] = lut[0][(pixels >> (28 - 4 * p)) & 0xf];
						rv[7] = lut[1][pixels & 0xf];
					} else
						for (int p = 0; p < 4; p++) {
							const int pix = (int)(pixels >> (24 - 8 * p)) & 0xff;
							const int trans = (p == 0 || p == 3) ? transp : 0;
							int32_t v = -1;
							if (pix != trans) {
								if (!indirect) {
									if (pix != 0)
										v = color | pix;
								} else {
									const int indpix = indtable[pix >> 4] | (pix & 0x0f);
									if ((indpix & transmask) != transmask)
										v = indpix;
								}
							}
							rv[p] = v;
						}
				}
				(void)np;
				// the drawn x: clipin, less clipout (when it is on this row)
				int xa[2], xb[2], ni;
				if (do_clipout && clipout.min_x <= clipout.max_x) {
					xa[0] = clipin.min_x; xb[0] = clipin.max_x < clipout.min_x - 1 ? clipin.max_x : clipout.min_x - 1;
					xa[1] = clipin.min_x > clipout.max_x + 1 ? clipin.min_x : clipout.max_x + 1; xb[1] = clipin.max_x;
					ni = 2;
				} else {
					xa[0] = clipin.min_x; xb[0] = clipin.max_x;
					ni = 1;
				}
				for (int i = 0; i < ni; i++) {
					if (xa[i] > xb[i])
						continue;
					int k0, k1;
					if (xdelta > 0) { k0 = xa[i] - xpos; k1 = xb[i] - xpos; }
					else { k0 = xpos - xb[i]; k1 = xpos - xa[i]; }
					if (k0 < kc0) k0 = kc0;
					if (k1 > kc1) k1 = kc1;
					while (k1 >= k0 && smap[k1] >= slimit)   // (the pixels after the marker's word)
						k1--;
					int x = xpos + xdelta * k0;
					if (direct) {
						const int32_t *const lf = &lut[0][0];
						if (!shadow) {
							for (int k = k0; k <= k1; k++, x += xdelta) {
								const unsigned se = kse[k];
								const int32_t v = lf[((se & 1) << 4) | ((wv[kw[k]] >> (se & 0x1e)) & 0xf)];
								if (v >= 0)
									dest[x] = (uint16_t)v;
							}
						} else
							for (int k = k0; k <= k1; k++, x += xdelta) {
								const unsigned se = kse[k];
								if (lf[((se & 1) << 4) | ((wv[kw[k]] >> (se & 0x1e)) & 0xf)] >= 0)
									dest[x] &= 0x7fff;
							}
					} else if (!shadow) {
						for (int k = k0; k <= k1; k++, x += xdelta) {
							const int32_t v = rowv[smap[k]];
							if (v >= 0)
								dest[x] = (uint16_t)v;
						}
					} else
						for (int k = k0; k <= k1; k++, x += xdelta)
							if (rowv[smap[k]] >= 0)
								dest[x] &= 0x7fff;
				}
			}
		next_row:
			yacc += vzoom;
			addr += (uint32_t)(srcw * (yacc >> 16));
			yacc &= 0xffff;
		}
#undef SPR_WORD
#undef SPR_MARK
	}
bail:
	return (indirect && indlocal) ? 2 : 0;
}

static void sprite_render_list(void)
{
	rect_t outerclip, clipin, clipout;
	int xoffs = 0, yoffs = 0, numentries = 0, spritenum = 0;
	outerclip.min_x = outerclip.min_y = 0;
	outerclip.max_x = (s_sj.latched[0x0c / 2] & 1) ? 415 : 319;
	outerclip.max_y = 223;
	clipin = outerclip;
	clipout.min_x = clipout.min_y = 0;
	clipout.max_x = clipout.max_y = -1;
	while (numentries++ < 0x20000 / 16) {
		const uint16_t *const sprite = &s_sj.ram[8 * (spritenum & 0x1fff)];
		switch (sprite[0] >> 14) {
		case 0:
			spritenum += 1 + draw_one_sprite(sprite, xoffs, yoffs, clipin, clipout);
			break;
		case 1:
			if ((sprite[0] >> 12) & 1) {
				clipin.min_y = sext(sprite[0], 12);
				clipin.max_y = sext(sprite[1], 12);
				clipin.min_x = sext(sprite[2], 12);
				clipin.max_x = sext(sprite[3], 12);
				// clipin &= outerclip
				if (clipin.min_x < outerclip.min_x) clipin.min_x = outerclip.min_x;
				if (clipin.max_x > outerclip.max_x) clipin.max_x = outerclip.max_x;
				if (clipin.min_y < outerclip.min_y) clipin.min_y = outerclip.min_y;
				if (clipin.max_y > outerclip.max_y) clipin.max_y = outerclip.max_y;
			}
			if ((sprite[0] >> 13) & 1) {
				clipout.min_y = sext(sprite[4], 12);
				clipout.max_y = sext(sprite[5], 12);
				clipout.min_x = sext(sprite[6], 12);
				clipout.max_x = sext(sprite[7], 12);
			}
			spritenum++;
			break;
		case 2:
			if ((sprite[0] >> 13) & 1) {
				yoffs = sext(sprite[1], 12);
				xoffs = sext(sprite[2], 12);
			}
			spritenum = sprite[0] & 0x1fff;
			break;
		case 3:
			numentries = 0x20000 / 16;
			break;
		}
	}
}

// ---- the mixer ----
typedef struct {
	uint16_t palbase, sprblendmask;
	uint8_t blendmask, index, effpri, mixshift, coloroffs;
} mixer_layer_info;
typedef struct {
	mixer_layer_info layerorder[16][8];
	int rgboffs[3][3];
	uint8_t sprgroup_shift, sprgroup_mask;
	int sprshadowmask, sprpixmask, sprshadow, blendfactor;
	int sprx_start, sprdx, spry_start, sprdy;
	const uint16_t *palx[3];   // (the colour offsets' palettes: the same table when an offset is 0)
	// mix_line's layer scans as tables: by sprite group and the mask of the layers with a pixel (bits L_TEXT..L_BITMAP,
	// L_SPRITES: a sprite pixel neither transparent nor shadow; bit 7: the sprite pixel shadows when passed), the position
	// in layerorder of the first such layer (the background ends every scan), 0x08: it blends, 0x10: shadow (the
	// sprites come no later and shadow); where the sprites are
	uint8_t first[16][256];
	// the same scans for mix_line: group * 8 + the first layer's position, 0x80 when it blends or shadows; and for
	// mix_slow_run: group * 8 + the second scan's layer's position (the first's when it does not blend), 0x80: the two
	// blend, 0x100: shadow
	uint8_t firstk[16][256];
	uint16_t slow[16][256];
	uint32_t rgbf[3];   // (mix_slow_run's fields of the colour offsets, + 32)
	uint8_t sprpos[16];
	// (mixer_remap) a tilemap layer's (or the background's) palx by its pixel when its mixshift is not 0: palx[palbase +
	// mix_index(pixel)] for the pixels below 0x2000 (all of these layers' pixels), made each frame
	uint16_t remap[8][0x2000];
	// (mixer_setup's inputs the tables above were made from: the same ones make the same tables, ADR-073)
	uint16_t key_mixer[0x40];
	int key_width;
	uint8_t key_en, key_lat, key_ok;
} mixer_t;
static mixer_t *s_mxs;   // (each monitor's: [2], init's calloc)

static uint8_t compute_color_offsets(const uint16_t *mix, int layerbit, int layerflag)
{
	const int mode = ((mix[0x3e / 2] & 0x8000) >> 14) | (layerbit & 1);
	switch (mode) {
	case 1: return 2;
	case 2: return (!layerflag) ? 2 : 0;
	default: return (uint8_t)!layerflag;
	}
}
static uint16_t compute_sprite_blend(uint8_t encoding)
{
	const int value = encoding & 0xf;
	switch ((encoding >> 4) & 3) {
	case 0: return (uint16_t)(1 << value);
	case 1: return (uint16_t)((1 << value) | ((1 << value) - 1));
	case 2: return (uint16_t)(~((1 << value) - 1) & 0xffff);
	default: return 0xffff;
	}
}

static void mixer_setup(const struct s32_snap *sn, mixer_t *mx)
{
	const uint16_t *mix = sn->mixer;
	const uint8_t lat = sn->latched[0x04 / 2] & 3;
	if (mx->key_ok && mx->key_width == sn->width && mx->key_en == sn->enablemask && mx->key_lat == lat &&
	    !memcmp(mx->key_mixer, mix, sizeof(mx->key_mixer)))
		return;
	mx->key_ok = 1;
	mx->key_width = sn->width;
	mx->key_en = sn->enablemask;
	mx->key_lat = lat;
	memcpy(mx->key_mixer, mix, sizeof(mx->key_mixer));
	const int blendenable = (mix[0x4e / 2] >> 11) & 1;
	mx->blendfactor = (mix[0x4e / 2] >> 8) & 7;
	mixer_layer_info layersort[8];
	mx->rgboffs[0][0] = sext(mix[0x40 / 2], 6);
	mx->rgboffs[0][1] = sext(mix[0x42 / 2], 6);
	mx->rgboffs[0][2] = sext(mix[0x44 / 2], 6);
	mx->rgboffs[1][0] = sext(mix[0x46 / 2], 6);
	mx->rgboffs[1][1] = sext(mix[0x48 / 2], 6);
	mx->rgboffs[1][2] = sext(mix[0x4a / 2], 6);
	mx->rgboffs[2][0] = mx->rgboffs[2][1] = mx->rgboffs[2][2] = 0;
	for (int c = 0; c < 3; c++)
		mx->rgbf[c] = (uint32_t)(mx->rgboffs[c][0] + 32) | ((uint32_t)(mx->rgboffs[c][1] + 32) << 10) |
		              ((uint32_t)(mx->rgboffs[c][2] + 32) << 20);
	static const uint8_t gshift[16] = { 14, 14, 13, 12, 14, 13, 12, 11, 14, 13, 12, 11, 13, 12, 11, 10 };
	static const uint8_t gmask[16] = { 0x00, 0x01, 0x03, 0x07, 0x01, 0x03, 0x07, 0x0f, 0x01, 0x03, 0x07, 0x0f, 0x01, 0x03, 0x07, 0x0f };
	static const uint8_t gor[16] = { 0x01, 0x02, 0x04, 0x08, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	const int gsel = mix[0x4c / 2] & 0x0f;
	const uint8_t sprgroup_or = gor[gsel];
	mx->sprgroup_shift = gshift[gsel];
	mx->sprgroup_mask = gmask[gsel];
	mx->sprshadowmask = ((mix[0x4c / 2] >> 2) & 1) ? 0x8000 : 0x0000;
	mx->sprpixmask = ((1 << mx->sprgroup_shift) - 1) & 0x3fff;
	mx->sprshadow = 0x7ffe & mx->sprpixmask;
	int numlayers = 0;
	for (int laynum = L_TEXT; laynum <= L_BITMAP; laynum++) {
		const int priority = mix[0x20 / 2 + laynum] & 0x0f;
		if (((sn->enablemask >> laynum) & 1) && priority != 0) {
			mixer_layer_info *l = &layersort[numlayers++];
			l->index = (uint8_t)laynum;
			l->effpri = (uint8_t)((priority << 3) | (6 - laynum));
			l->palbase = (uint16_t)((mix[0x20 / 2 + laynum] & 0x00f0) << 6);
			l->mixshift = (uint8_t)((mix[0x20 / 2 + laynum] >> 8) & 3);
			l->blendmask = (uint8_t)(blendenable ? ((mix[0x30 / 2 + laynum] >> 6) & 0xff) : 0);
			l->sprblendmask = compute_sprite_blend((uint8_t)(mix[0x30 / 2 + laynum] & 0x3f));
			l->coloroffs = compute_color_offsets(mix, (mix[0x3e / 2] >> laynum) & 1, (mix[0x30 / 2 + laynum] >> 14) & 1);
		}
	}
	{
		mixer_layer_info *l = &layersort[numlayers++];
		l->index = L_BACKGROUND;
		l->effpri = (1 << 3) | 0;
		l->palbase = (uint16_t)((mix[0x2c / 2] & 0x00f0) << 6);
		l->mixshift = (uint8_t)((mix[0x2c / 2] >> 8) & 3);
		l->blendmask = 0;
		l->sprblendmask = 0;
		l->coloroffs = compute_color_offsets(mix, (mix[0x3e / 2] >> 8) & 1, (mix[0x3e / 2] >> 14) & 1);
	}
	for (int laynum = 0; laynum < numlayers; laynum++)
		for (int i = laynum + 1; i < numlayers; i++)
			if (layersort[i].effpri > layersort[laynum].effpri) {
				const mixer_layer_info t = layersort[i];
				layersort[i] = layersort[laynum];
				layersort[laynum] = t;
			}
	for (int groupnum = 0; groupnum <= mx->sprgroup_mask; groupnum++) {
		const int effgroup = sprgroup_or | groupnum;
		const int priority = mix[0x00 / 2 + effgroup] & 0x0f;
		const int effpri = (priority << 3) | 7;
		int sprindex = numlayers, dstnum = 0;
		for (int laynum = 0; laynum < numlayers; laynum++) {
			if (effpri > layersort[laynum].effpri && sprindex == numlayers)
				sprindex = dstnum++;
			mx->layerorder[groupnum][dstnum++] = layersort[laynum];
		}
		mixer_layer_info *l = &mx->layerorder[groupnum][sprindex];
		l->index = L_SPRITES;
		l->effpri = (uint8_t)effpri;
		if ((mix[0x4c / 2] & 3) != 3)
			l->palbase = (uint16_t)((mix[0x00 / 2 + effgroup] & 0x00f0) << 6);
		else
			l->palbase = (uint16_t)((mix[0x4c / 2] & 0x00f0) << 6);
		l->mixshift = (uint8_t)((mix[0x00 / 2 + effgroup] >> 8) & 3);
		l->blendmask = 0;
		l->sprblendmask = 0;
		l->coloroffs = compute_color_offsets(mix, (mix[0x3e / 2] >> 6) & 1, (mix[0x4c / 2] >> 15) & 1);
	}
	for (int g = 0; g <= mx->sprgroup_mask; g++) {
		const mixer_layer_info *o = mx->layerorder[g];
		mx->sprpos[g] = 8;
		for (int i = 0; i < 8; i++)
			if (o[i].index == L_SPRITES) {
				mx->sprpos[g] = (uint8_t)i;
				break;
			}
		for (int m = 0; m < 256; m++) {
			int i = 0;
			while (o[i].index != L_BACKGROUND && !((m >> o[i].index) & 1))
				i++;
			mx->first[g][m] = (uint8_t)(i | (o[i].blendmask ? 0x08 : 0) | ((m & 0x80) && mx->sprpos[g] <= i ? 0x10 : 0));
		}
		for (int m = 0; m < 256; m++) {
			const int e = mx->first[g][m];
			mx->firstk[g][m] = (uint8_t)((g * 8 + (e & 7)) | (e & 0x18 ? 0x80 : 0));
			int t = (g * 8 + (e & 7)) | (e & 0x10 ? 0x100 : 0);
			if (e & 0x08) {   // (the first layer is a tilemap: its bit leaves the mask for the second scan)
				const mixer_layer_info *first = &o[e & 7];
				const int e2 = mx->first[g][m & ~(1 << first->index)];
				const int laynum = o[e2 & 7].index;
				t = (g * 8 + (e2 & 7)) | (e2 & 0x10 ? 0x100 : 0);
				if (((first->blendmask >> laynum) & 1) && (laynum != L_SPRITES || ((first->sprblendmask >> g) & 1)))
					t |= 0x80;
			}
			mx->slow[g][m] = (uint16_t)t;
			if (!(t & 0x180))   // (the first layer's blend leaves out the second, no shadow: its colour + offset, the fast path)
				mx->firstk[g][m] &= 0x7f;
		}
	}
	const int W = sn->width;
	if (sn->latched[0x04 / 2] & 1) {
		mx->sprx_start = W - 1;
		mx->sprdx = -1;
	} else {
		mx->sprx_start = 0;
		mx->sprdx = 1;
	}
	if ((sn->latched[0x04 / 2] >> 1) & 1) {
		mx->spry_start = SH - 1;
		mx->sprdy = -1;
	} else {
		mx->spry_start = 0;
		mx->sprdy = 1;
	}
}

static inline int clamp_ch(int v) { return v > 31 ? 31 : v > 0 ? v : 0; }

// mix_line's colour of a pixel without blend / shadow, for each colour offset (one table for the offsets of 0). Kept from
// frame to frame: the entries whose palette word changed are made again, a table whose offsets changed all of it
#define PALKEY_NONE 0xffffffffu
static inline uint16_t palx_px(uint16_t v, const int *d)
{
	return (uint16_t)((clamp_ch((v & 0x1f) + d[0]) << 10) | (clamp_ch(((v >> 5) & 0x1f) + d[1]) << 5) |
	                  clamp_ch(((v >> 10) & 0x1f) + d[2]));
}
static void palx_build(struct s32_snap *sn, mixer_t *mx)
{
	const uint16_t *pal = sn->paletteram;
	uint16_t *t2 = sn->palx[2], *prev = sn->palprev;
	mx->palx[2] = t2;
	int upd[2] = { 0, 0 };   // (the offset tables kept up by the changed entries)
	for (int c = 0; c < 2; c++) {
		const int *d = mx->rgboffs[c];
		if (!d[0] && !d[1] && !d[2]) {
			mx->palx[c] = t2;
			sn->palkey[c] = PALKEY_NONE;   // (not kept up)
			continue;
		}
		mx->palx[c] = sn->palx[c];
		if (sn->palok && sn->palkey[c] == mx->rgbf[c])
			upd[c] = 1;
		else
			sn->palkey[c] = PALKEY_NONE;   // (made whole below)
	}
	static const int zero[3] = { 0, 0, 0 };
	if (!sn->palok) {
		for (int i = 0; i < 0x4000; i++)
			t2[i] = t2[i | 0x4000] = palx_px(pal[i], zero);
		memcpy(prev, pal, sizeof(sn->palprev));
		sn->palok = 1;
	} else
		for (int w = 0; w < 0x4000 / 8 / 32; w++)   // (only the blocks written since: the others are as prev)
			for (uint32_t bits = sn->paldirty[w]; bits; bits &= bits - 1) {
				const int i = (w * 32 + __builtin_ctz(bits)) * 8;
				uint64_t a[2], q[2];
				memcpy(a, pal + i, 16);
				memcpy(q, prev + i, 16);
				if (a[0] == q[0] && a[1] == q[1])
					continue;
				for (int k = i; k < i + 8; k++) {
					const uint16_t v = pal[k];
					prev[k] = v;
					t2[k] = t2[k | 0x4000] = palx_px(v, zero);
					if (upd[0])
						sn->palx[0][k] = sn->palx[0][k | 0x4000] = palx_px(v, mx->rgboffs[0]);
					if (upd[1])
						sn->palx[1][k] = sn->palx[1][k | 0x4000] = palx_px(v, mx->rgboffs[1]);
				}
			}
	memset(sn->paldirty, 0, sizeof(sn->paldirty));
	for (int c = 0; c < 2; c++)
		if (mx->palx[c] != t2 && sn->palkey[c] == PALKEY_NONE) {
			uint16_t *t = sn->palx[c];
			for (int i = 0; i < 0x4000; i++)
				t[i] = t[i | 0x4000] = palx_px(pal[i], mx->rgboffs[c]);
			sn->palkey[c] = mx->rgbf[c];
		}
}

// MAME's mixer scans, per pixel, the sprite group's layer order for the first layer with a pixel (the sprites: one neither
// transparent nor shadow; the background always) and, when that layer blends, again for the next one; passing the sprite
// layer sets shadow (~pixel & the shadow mask, or 1 for a shadow pixel). Here a mask of the layers with a pixel and the
// tables of mixer_setup give the layers, and shadow is the sprite pixel's when the sprites come no later than the last
// layer scanned.
typedef struct {
	const uint16_t *pal;
	const mixer_t *mx;
	struct {                 // layerorder[group][i] (k = group * 8 + i)
		const uint16_t *line;   // its line (0 where no pixel; L_BACKGROUND's colour; L_SPRITES's masked)
		const uint16_t *palxb;  // its colour offset's palx from its palette base (palx repeats: no wrap)
		uint32_t rgbf;          // (mix_slow_run) its colour offset's fields
		uint16_t palbase;
		uint8_t mixshift;
		uint8_t shpalx;         // (mix_slow_run) no colour offset field above 0: a shadow is palx's colour halved
		uint8_t msf;            // (mix_line's table pbf: palxb, or the remap table with mixshift 0)
		const uint16_t *pbf;
	} fi[16 * 8];
	int shift0;              // every layer's mixshift is 0
	uint16_t sp[SW];         // (the sprite line)
} mixctx_t;
static void mixer_remap(mixer_t *mx)
{
	for (int i = 0; i < 8; i++) {
		const mixer_layer_info *l = &mx->layerorder[0][i];
		if (l->index != L_SPRITES && l->mixshift) {   // (a pixel: colour * 16 + pen, the pen kept by mix_index)
			const uint16_t *const pb = mx->palx[l->coloroffs] + l->palbase;
			uint16_t *const t = mx->remap[l->index];
			for (int c = 0; c < 0x200; c++)
				memcpy(t + c * 16, pb + (((c << 4) >> l->mixshift) & 0xfff0), 32);
		}
		if (l->index == L_BACKGROUND)
			break;
	}
}
static void mix_begin(mixctx_t *c, const struct s32_snap *sn, const mixer_t *mx, uint16_t *const *lb)
{
	c->pal = sn->paletteram;
	c->mx = mx;
	c->shift0 = 1;
	for (int g = 0; g <= mx->sprgroup_mask; g++)
		for (int i = 0; i < 8; i++) {
			const mixer_layer_info *l = &mx->layerorder[g][i];
			const int k = g * 8 + i;
			c->fi[k].line = l->index == L_SPRITES ? c->sp : lb[l->index & 7];
			c->fi[k].palxb = mx->palx[l->coloroffs] + l->palbase;
			c->fi[k].rgbf = mx->rgbf[l->coloroffs];
			c->fi[k].palbase = l->palbase;
			c->fi[k].mixshift = l->mixshift;
			const int *d = mx->rgboffs[l->coloroffs];
			c->fi[k].shpalx = d[0] <= 0 && d[1] <= 0 && d[2] <= 0;
			const int remap = l->index != L_SPRITES && l->mixshift;
			c->fi[k].pbf = remap ? mx->remap[l->index] : c->fi[k].palxb;
			c->fi[k].msf = remap ? 0 : l->mixshift;
			if (c->fi[k].msf)
				c->shift0 = 0;
			if (l->index == L_BACKGROUND)
				break;
		}
}

// MAME's channel arithmetic (colour + offset, the blend's (a * (7 - f) + b * (f + 1)) >> 3, the shadow's >> 1, then the
// clamp to 0..31) on the three channels at once: 10-bit fields at bits 0, 10, 20, each the channel + 32 (0..94: no field
// borrows or carries, and the shifts' floors stay those of the signed values)
#define MIXF(v) ((uint32_t)(v) | ((uint32_t)(v) << 10) | ((uint32_t)(v) << 20))
static inline uint32_t mix_spread(int c) { return (uint32_t)(c & 0x1f) | ((uint32_t)(c & 0x3e0) << 5) | ((uint32_t)(c & 0x7c00) << 10); }
static uint8_t s_clamp32[128];   // (a field's channel + 32 clamped to 0..31)

static inline int mix_index(int pix, int mixshift) { return ((pix >> mixshift) & 0xfff0) + (pix & 0x0f); }

// the pixels x0..x1 - 1 of the same layers that blend or shadow (k: firstk's layer, t: the slow table's entry; the
// tables' values held in registers)
static __attribute__((noinline)) void mix_slow_run(const mixctx_t *c, int k, int t, int x0, int x1, uint16_t *out)
{
	const uint16_t *const pal = c->pal;
	const uint16_t *const ln = c->fi[k].line;
	const int pb = c->fi[k].palbase, ms = c->fi[k].mixshift;
	const uint32_t rgbf = c->fi[k].rgbf;
	const uint32_t sh = (t & 0x100) ? 1 : 0;
	if (t & 0x80) {
		const int k2 = t & 0x7f;
		const uint16_t *const ln2 = c->fi[k2].line;
		const int pb2 = c->fi[k2].palbase, ms2 = c->fi[k2].mixshift;
		const uint32_t rgbf2 = c->fi[k2].rgbf;
		const int bf = c->mx->blendfactor;
		const uint32_t w1 = (uint32_t)(7 - bf), w2 = (uint32_t)(bf + 1);
		for (int x = x0; x < x1; x++) {
			const uint32_t v1 = mix_spread(pal[(pb + mix_index(ln[x], ms)) & 0x3fff]) + rgbf;
			const uint32_t v2 = mix_spread(pal[(pb2 + mix_index(ln2[x], ms2)) & 0x3fff]) + rgbf2;
			uint32_t v = ((v1 * w1 + v2 * w2) >> 3) & MIXF(0x7f);
			if (sh)
				v = ((v >> 1) & MIXF(0x7f)) + MIXF(16);
			out[x] = (uint16_t)((s_clamp32[v & 0x7f] << 10) | (s_clamp32[(v >> 10) & 0x7f] << 5) | s_clamp32[(v >> 20) & 0x7f]);
		}
	} else if (sh && c->fi[k].shpalx) {   // (the channel + offset is at most 31: clamp(floor(v / 2)) = clamp(v) >> 1)
		const uint16_t *const pbx = c->fi[k].pbf;
		const int msf = c->fi[k].msf;
		if (!msf)
			for (int x = x0; x < x1; x++)
				out[x] = (uint16_t)((pbx[ln[x]] >> 1) & 0x3def);
		else
			for (int x = x0; x < x1; x++)
				out[x] = (uint16_t)((pbx[mix_index(ln[x], msf)] >> 1) & 0x3def);
	} else   // (no blend: shadow)
		for (int x = x0; x < x1; x++) {
			uint32_t v = mix_spread(pal[(pb + mix_index(ln[x], ms)) & 0x3fff]) + rgbf;
			if (sh)
				v = ((v >> 1) & MIXF(0x7f)) + MIXF(16);
			out[x] = (uint16_t)((s_clamp32[v & 0x7f] << 10) | (s_clamp32[(v >> 10) & 0x7f] << 5) | s_clamp32[(v >> 20) & 0x7f]);
		}
}

// one line (lmask: the layers' bits of each pixel); shift0: every mixshift 0 (the palette index is the pixel)
static inline __attribute__((always_inline)) void mix_line_k(mixctx_t *c, const uint16_t *sl, int W, const uint8_t *lmask,
                                                            uint16_t *out, const int shift0)
{
	const mixer_t *mx = c->mx;
	const int sh = mx->sprgroup_shift, smask = mx->sprgroup_mask;
	const unsigned pixmask = (unsigned)mx->sprpixmask, sm2 = pixmask & 0x7ffe, sshadow = (unsigned)mx->sprshadow;
	const unsigned shmask = (unsigned)mx->sprshadowmask;
	const uint8_t *const firstk = &mx->firstk[0][0];
	uint16_t *const sp = c->sp;
	uint16_t gms[SW];   // (the sprite group * 256 + the mask: a loop of its own, vectorised)
	int x0 = 0;
#if defined(__ARM_NEON)
	{   // (the same 8 at a time)
		const uint16x8_t k7fff = vdupq_n_u16(0x7fff), vsm2 = vdupq_n_u16((uint16_t)sm2), vssh = vdupq_n_u16((uint16_t)sshadow);
		const uint16x8_t vshm = vdupq_n_u16((uint16_t)shmask), vpm = vdupq_n_u16((uint16_t)pixmask);
		const uint16x8_t vsmask = vdupq_n_u16((uint16_t)smask), b6 = vdupq_n_u16(1 << L_SPRITES), b7 = vdupq_n_u16(0x80);
		const int16x8_t vsh = vdupq_n_s16((int16_t)-sh);
		for (; x0 + 8 <= W; x0 += 8) {
			const uint16x8_t p = vld1q_u16(sl + x0);
			const uint16x8_t op = vmvnq_u16(vceqq_u16(vandq_u16(p, k7fff), k7fff));
			const uint16x8_t shp = vandq_u16(op, vceqq_u16(vandq_u16(p, vsm2), vssh));
			const uint16x8_t s7 = vorrq_u16(shp, vtstq_u16(vmvnq_u16(p), vshm));
			uint16x8_t m = vmovl_u8(vld1_u8(lmask + x0));
			m = vorrq_u16(m, vandq_u16(vbicq_u16(op, shp), b6));
			m = vorrq_u16(m, vandq_u16(s7, b7));
			const uint16x8_t g = vandq_u16(vshlq_u16(p, vsh), vsmask);
			vst1q_u16(sp + x0, vandq_u16(p, vpm));
			vst1q_u16(gms + x0, vorrq_u16(vshlq_n_u16(g, 8), m));
		}
	}
#endif
	for (int x = x0; x < W; x++) {
		const unsigned p = sl[x];
		const unsigned op = (p & 0x7fff) != 0x7fff, shp = op & ((p & sm2) == sshadow);
		const unsigned m = lmask[x] | ((op & !shp) << L_SPRITES) | ((shp | ((~p & shmask) != 0)) << 7);
		sp[x] = (uint16_t)(p & pixmask);
		gms[x] = (uint16_t)((((p >> sh) & smask) << 8) | m);
	}
	// runs of pixels of the same gm (the same layers): their tables looked up once
	for (int x = 0; x < W;) {
		const unsigned gm = gms[x];
		int e = x + 1;
		const uint64_t g4 = gm * 0x0001000100010001ull;
		for (uint64_t w; e + 4 <= W && (memcpy(&w, gms + e, 8), w == g4);)   // (four at a time, then one)
			e += 4;
		while (e < W && gms[e] == gm)
			e++;
		const unsigned k = firstk[gm], k1 = k & 0x7f;
		const uint16_t *const ln = c->fi[k1].line;
		if (__builtin_expect(k & 0x80, 0)) {
			const unsigned t = mx->slow[0][gm];
			mix_slow_run(c, (int)k1, (int)t, x, e, out);
			x = e;
			continue;
		}
		const uint16_t *const pb = c->fi[k1].pbf;   // (no blend, no shadow: the clamped colour of the table)
		const int ms = c->fi[k1].msf;
		if (shift0 || !ms)
			for (; x < e; x++)
				out[x] = pb[ln[x]];
		else
			for (; x < e; x++)
				out[x] = pb[mix_index(ln[x], ms)];
	}
}
static void mix_line(mixctx_t *c, const struct s32_snap *sn, int y, const uint8_t *lmask, uint16_t *out)
{
	const mixer_t *mx = c->mx;
	const int W = sn->width;
	const uint16_t *spr = &sn->spr[(mx->spry_start + mx->sprdy * y) * SW];
	uint16_t slr[SW];   // the sprite line in screen order (when flipped)
	const uint16_t *sl = spr;
	if (mx->sprdx < 0) {
		for (int x = 0; x < W; x++)
			slr[x] = spr[W - 1 - x];
		sl = slr;
	}
	if (c->shift0)
		mix_line_k(c, sl, W, lmask, out, 1);
	else
		mix_line_k(c, sl, W, lmask, out, 0);
}

// -DS32_PROF: the render's parts timed (ms a frame, summed over the bands' threads), printed at exit
#ifdef S32_PROF
#include <time.h>
enum { PR_SETUP, PR_MSETUP, PR_PALX, PR_REMAP, PR_BEGIN, PR_TEXT, PR_NBG, PR_BMP, PR_MASK, PR_MIX0, PR_MIX1, PR_OUT, PR_RENDER, PR_N };
static double s_ph[PR_N];
static long s_pn[PR_N];   // (the timings each: one clock read's cost taken off each)
static int s_prof_n;
static double prof_now(void) { struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t); return t.tv_sec * 1e3 + t.tv_nsec / 1e6; }
static void prof_print(void)
{
	static const char *const nm[PR_N] = { "setup", "(mixer_setup", "palx_build", "remap)", "mix_begin", "text", "nbg", "bitmap", "bg+mask", "mix0", "mix1", "out", "render" };
	const int n = s_prof_n ? s_prof_n : 1;
	const double c0 = prof_now();
	for (int i = 0; i < 100000; i++)
		(void)prof_now();
	const double clk = (prof_now() - c0) / 100000;
	fprintf(stderr, "S32PROF %d frames (clock %.5f ms)", s_prof_n, clk);
	double all = 0;
	for (int i = 0; i < PR_N; i++)
		all += s_pn[i];
	for (int i = 0; i < PR_N; i++)
		fprintf(stderr, " %s %.3f", nm[i], (s_ph[i] - clk * (i == PR_RENDER ? all : s_pn[i])) / n);
	fprintf(stderr, "\n");
}
#define PROF_T(v) const double v = prof_now()
#define PROF_ADD(i, v0) prof_add(i, prof_now() - (v0))
static void prof_add(int i, double d)   // (bands on other threads: a lock-free double add)
{
	__atomic_fetch_add(&s_pn[i], 1, __ATOMIC_RELAXED);
	double o, nv;
	__atomic_load(&s_ph[i], &o, __ATOMIC_RELAXED);
	do
		nv = o + d;
	while (!__atomic_compare_exchange(&s_ph[i], &o, &nv, 0, __ATOMIC_RELAXED, __ATOMIC_RELAXED));
}
#else
#define PROF_T(v)
#define PROF_ADD(i, v0)
#endif

// the frame's output written by the bands, each line right after its mix (the core's video_out on the render thread
// alone was its largest serial part, ADR-073): the same pixels as video_out, by the frontend's colour function's table
// (s_olut: the fixed palette, made again when the function or format changes) or, when that table is RGB565's / the
// 0x00RRGGBB of 8-bit channels exactly, by the same arithmetic
enum { OUT_NONE, OUT_LUT, OUT_565, OUT_888 };
static uint32_t *s_olut;
static uint32_t (*s_olut_fn)(int, int, int);   // (no parameter names: b is the board macro)
static int s_olut_bpp, s_olut_kind;
static void out_table(void)
{
	uint32_t (*fn)(int, int, int) = rbx_video.out_color;
	if (s_olut_fn == fn && s_olut_bpp == rbx_video.out_bpp)
		return;
	int is565 = rbx_video.out_bpp == 2, is888 = rbx_video.out_bpp == 4;
	for (int i = 0; i < NPAL; i++) {
		const uint32_t c = rbx_video.palette[i];
		const uint32_t v = fn((int)(c >> 16) & 0xff, (int)(c >> 8) & 0xff, (int)c & 0xff);
		s_olut[i] = v;
		const uint32_t u = (uint32_t)i;
		if (is565 && (uint16_t)v != (((u & 0x7fe0) << 1) | (u & 0x1f)))
			is565 = 0;
		if (is888 && v != (((u & 0x7c00) << 9) | ((u & 0x3e0) << 6) | ((u & 0x1f) << 3)))
			is888 = 0;
	}
	s_olut_fn = fn;
	s_olut_bpp = rbx_video.out_bpp;
	s_olut_kind = is565 ? OUT_565 : is888 ? OUT_888 : OUT_LUT;
}
static int s_out_kind;   // (this frame's: OUT_NONE, the bitmap for the core's video_out)
// n pixels of a line (RGB555 indices below 0x8000) to the output line d
static void out_line(const uint16_t *s, void *d, int n)
{
	int x = 0;
	switch (s_out_kind) {
	case OUT_565: {
		uint16_t *o = (uint16_t *)d;
#if defined(__ARM_NEON)
		const uint16x8_t m1 = vdupq_n_u16(0x7fe0), m0 = vdupq_n_u16(0x1f);
		for (; x + 8 <= n; x += 8) {
			const uint16x8_t c = vld1q_u16(s + x);
			vst1q_u16(o + x, vorrq_u16(vshlq_n_u16(vandq_u16(c, m1), 1), vandq_u16(c, m0)));
		}
#endif
		for (; x < n; x++)
			o[x] = (uint16_t)(((s[x] & 0x7fe0) << 1) | (s[x] & 0x1f));
		break;
	}
	case OUT_888: {
		uint32_t *o = (uint32_t *)d;
		for (; x < n; x++) {
			const uint32_t c = s[x];
			o[x] = ((c & 0x7c00) << 9) | ((c & 0x3e0) << 6) | ((c & 0x1f) << 3);
		}
		break;
	}
	default:
		if (s_olut_bpp == 2) {
			uint16_t *o = (uint16_t *)d;
			for (; x < n; x++)
				o[x] = (uint16_t)s_olut[s[x] & (NPAL - 1)];
		} else {
			uint32_t *o = (uint32_t *)d;
			for (; x < n; x++)
				o[x] = s_olut[s[x] & (NPAL - 1)];
		}
	}
}

#define BANDS 28   // (8 lines each: the helpers' last bands leave the render thread less to wait for)
static void s32_band(int band)
{
	const struct s32_snap *sn = s_snap;
	const int y0 = SH * band / BANDS, y1 = SH * (band + 1) / BANDS;
	const int W = sn->width;
	const int on1 = s_multi && s_snap1->displayenable;   // (Multi 32's second monitor)
	const int ok = s_out_kind;
	if (!sn->displayenable && !on1) {
		static const uint16_t zero[SW * 2];
		for (int y = y0; y < y1; y++)
			if (ok)
				out_line(zero, (uint8_t *)rbx_video.out + (size_t)y * rbx_video.out_pitch, rbx_video.width);
			else
				memset(rbx_video.bitmap + y * rbx_video.pitch, 0, (size_t)SW * (s_multi ? 2 : 1) * 2);
		return;
	}
	uint16_t orow[SW * 2];   // (the mixed line when the bands write the output)
	uint16_t lines[8][SW];
	uint16_t *lb[8];
	for (int i = 0; i < 8; i++)
		lb[i] = lines[i];
	const uint16_t *vram = sn->videoram;
	mixctx_t mc[2];
	PROF_T(tb);
	if (sn->displayenable)
		mix_begin(&mc[0], sn, &s_mxs[0], lb);
	if (on1)
		mix_begin(&mc[1], s_snap1, &s_mxs[1], lb);
	PROF_ADD(PR_BEGIN, tb);
	const uint8_t en = sn->enablemask;
	for (int l = L_TEXT; l <= L_BITMAP; l++)   // (the lines of the layers off: 0 for the band, no pixel in the mask)
		if (!((en >> l) & 1))
			memset(lb[l], 0, (size_t)W * 2);
	for (int y = y0; y < y1; y++) {
		PROF_T(t0);
		if (en & 0x01) line_text(sn, y, lb[L_TEXT]);
		PROF_ADD(PR_TEXT, t0);
		PROF_T(t1);
		for (int n = 0; n < 4; n++)
			if (en & (2 << n)) {
				memset(lb[L_NBG0 + n], 0, (size_t)W * 2);
				if (n < 2) line_zoom(sn, n, y, lb[L_NBG0 + n]);
				else line_rowscroll(sn, n, y, lb[L_NBG0 + n]);
			}
		PROF_ADD(PR_NBG, t1);
		PROF_T(t2);
		if (en & 0x20) {
			memset(lb[L_BITMAP], 0, (size_t)W * 2);
			line_bitmap(sn, y, lb[L_BITMAP]);
		}
		PROF_ADD(PR_BMP, t2);
		PROF_T(t3);
		// the background colour
		int color;
		if ((vram[0x1ff5e / 2] >> 15) & 1) {
			const int yoffset = (vram[0x1ff5e / 2] + y) & 0x1ff;
			color = (vram[0x1ff5e / 2] & 0x1e00) + yoffset;
		} else
			color = vram[0x1ff5e / 2] & 0x1e00;
		for (int x = 0; x < W; x++)
			lb[L_BACKGROUND][x] = (uint16_t)color;
		uint8_t lmask[SW];   // (the layers with a pixel, for mix_line: all six lines in one pass)
		{
			const uint16_t *const p0 = lb[0], *const p1 = lb[1], *const p2 = lb[2], *const p3 = lb[3], *const p4 = lb[4],
			               *const p5 = lb[5];
			int x = 0;
#if defined(__ARM_NEON)
			for (; x + 8 <= W; x += 8) {
				uint16x8_t m = vandq_u16(vtstq_u16(vld1q_u16(p0 + x), vld1q_u16(p0 + x)), vdupq_n_u16(0x01));
				m = vbslq_u16(vtstq_u16(vld1q_u16(p1 + x), vld1q_u16(p1 + x)), vorrq_u16(m, vdupq_n_u16(0x02)), m);
				m = vbslq_u16(vtstq_u16(vld1q_u16(p2 + x), vld1q_u16(p2 + x)), vorrq_u16(m, vdupq_n_u16(0x04)), m);
				m = vbslq_u16(vtstq_u16(vld1q_u16(p3 + x), vld1q_u16(p3 + x)), vorrq_u16(m, vdupq_n_u16(0x08)), m);
				m = vbslq_u16(vtstq_u16(vld1q_u16(p4 + x), vld1q_u16(p4 + x)), vorrq_u16(m, vdupq_n_u16(0x10)), m);
				m = vbslq_u16(vtstq_u16(vld1q_u16(p5 + x), vld1q_u16(p5 + x)), vorrq_u16(m, vdupq_n_u16(0x20)), m);
				vst1_u8(lmask + x, vmovn_u16(m));
			}
#endif
			for (; x < W; x++)
				lmask[x] = (uint8_t)((p0[x] != 0) | ((p1[x] != 0) << 1) | ((p2[x] != 0) << 2) | ((p3[x] != 0) << 3) |
				                     ((p4[x] != 0) << 4) | ((p5[x] != 0) << 5));
		}
		PROF_ADD(PR_MASK, t3);
		PROF_T(t4);
		const int ry = s_screen_flipy ? SH - 1 - y : y;
		uint16_t *const row = ok ? orow : rbx_video.bitmap + ry * rbx_video.pitch;
		if (sn->displayenable)
			mix_line(&mc[0], sn, y, lmask, row);
		else
			memset(row, 0, (size_t)W * 2);
		PROF_ADD(PR_MIX0, t4);
		PROF_T(t5);
		if (s_multi) {
			if (on1)
				mix_line(&mc[1], s_snap1, y, lmask, row + W);
			else
				memset(row + W, 0, (size_t)W * 2);
		}
		PROF_ADD(PR_MIX1, t5);
		if (ok) {
			PROF_T(t6);
			out_line(row, (uint8_t *)rbx_video.out + (size_t)ry * rbx_video.out_pitch, rbx_video.width);
			PROF_ADD(PR_OUT, t6);
		}
	}
}
static decoband_t s_bands = { 0, 0, 0, 0, BANDS, s32_band };
static int s32_help(int helper) { return decoband_help(&s_bands, helper); }

static int s_pal_ready;
static void s32_render(void)
{
	if (!s_pal_ready) {
		nib_init();
		for (int v = 0; v < 128; v++)
			s_clamp32[v] = (uint8_t)clamp_ch(v - 32);
		for (int i = 0; i < NPAL; i++)
			rbx_video.palette[i] = ((uint32_t)((i >> 10) & 0x1f) << 19) | ((uint32_t)((i >> 5) & 0x1f) << 11) | ((uint32_t)(i & 0x1f) << 3);
		s_pal_ready = 1;
		s_olut_fn = NULL;   // (the output table made again from it)
	}
	PROF_T(tr0);
	s_snap = s_snapv[rbx_video_render_seq() & 1];
	s_snap1 = s_snap1v[rbx_video_render_seq() & 1];
	rbx_video.width = s_snap->width * (s_multi ? 2 : 1);
	zoom_setup(s_snap, 0, &s_zp[0]);
	zoom_setup(s_snap, 1, &s_zp[1]);
	s_out_kind = OUT_NONE;
	if (rbx_video.out && !rbx_video.blank && s_olut && (rbx_video.out_bpp == 2 || rbx_video.out_bpp == 4)) {
		out_table();
		s_out_kind = s_olut_kind;
	}
	for (int i = 0; i < (s_multi ? 2 : 1); i++) {
		struct s32_snap *const sn = i ? s_snap1 : s_snap;
		if (!sn->displayenable)
			continue;
		PROF_T(ts0);
		mixer_setup(sn, &s_mxs[i]);
		PROF_ADD(PR_MSETUP, ts0);
		PROF_T(ts1);
		palx_build(sn, &s_mxs[i]);
		PROF_ADD(PR_PALX, ts1);
		PROF_T(ts2);
		mixer_remap(&s_mxs[i]);
		PROF_ADD(PR_REMAP, ts2);
	}
	PROF_ADD(PR_SETUP, tr0);
	decoband_run(&s_bands);
	if (s_out_kind)
		rbx_video.out_done = 1;
#ifdef S32_PROF
	PROF_ADD(PR_RENDER, tr0);
	static int reg;
	if (!reg) {
		reg = 1;
		atexit(prof_print);
	}
	s_prof_n++;
#endif
}

// screen_update_system32 (at the vblank's start): the snapshot, update_tilemaps' enables and clips
static void vblank_draw(void)
{
	rbx_video_wait_slot();   // (a free snapshot: the frame before this one may still be being drawn)
	const unsigned slot = rbx_video_post_seq() & 1;
	struct s32_snap *sn = s_snapv[slot], *const sn1 = s_snap1v[slot];
	const uint16_t *vram = b.videoram;
	sn->width = ((vram[0x1ff00 / 2] >> 15) & 1) ? 52 * 8 : 40 * 8;
	sn->displayenable = b.displayenable;
	if (s_multi) {
		sn1->width = sn->width;
		sn1->displayenable = mb.displayenable;
	}
	for (int m = 0; m < (s_multi ? 2 : 1); m++) {   // (the blocks written since this slot's last snapshot: kept in it
		struct s32_snap *const t = m ? sn1 : sn;    // until its palx_build, which may not come for every snapshot)
		for (int i = 0; i < 0x4000 / 8 / 32; i++) {
			t->paldirty[i] |= s_paldirty[m][slot][i];
			s_paldirty[m][slot][i] = 0;
		}
	}
	if (sn->displayenable || (s_multi && mb.displayenable)) {
		memcpy(sn->videoram, b.videoram, sizeof(sn->videoram));
		memcpy(sn->paletteram, b.paletteram, sizeof(sn->paletteram));
		memcpy(sn->mixer, b.mixer, sizeof(sn->mixer));
		sn->spi = s_spmap[b.spr_front];
		sn->spr = s_sprp[0][sn->spi];
		memcpy(sn->latched, b.sprite_control_latched, sizeof(sn->latched));
		for (int n = 0; n < 4; n++)   // (get_tilemaps)
			sn->tilebank[n] = (uint8_t)(s_multi ? (b.tilebank_external >> (2 * n)) & 3
			                                    : ((b.tilebank_external & 1) << 1) | ((vram[0x1ff00 / 2] & 0x400) >> 10));
		const int bit02 = vram[0x1ff02 / 2], bit8e = vram[0x1ff8e / 2], bit00 = vram[0x1ff00 / 2];
		const int enable0 = !((bit02 >> 0) & 1) && !((bit8e >> 1) & 1);
		const int enable1 = !((bit02 >> 1) & 1) && !((bit8e >> 2) & 1);
		const int enable2 = !((bit02 >> 2) & 1) && !((bit8e >> 3) & 1) && !((bit00 >> 12) & 1);
		const int enable3 = !((bit02 >> 3) & 1) && !((bit8e >> 4) & 1) && !((bit00 >> 13) & 1);
		const int enablet = !((bit02 >> 4) & 1) && !((bit8e >> 0) & 1);
		const int enableb = !((bit02 >> 5) & 1) && !((bit8e >> 5) & 1);
		sn->enablemask = (uint8_t)((enablet << 0) | (enable0 << 1) | (enable1 << 2) | (enable2 << 3) | (enable3 << 4) | (enableb << 5));
		for (int n = 0; n < 4; n++)
			compute_clipping_extents(vram, sn->width, (bit02 >> (11 + n)) & 1, (bit02 >> (6 + n)) & 1,
			                         (vram[0x1ff06 / 2] >> (4 * n)) & 0x0f, &sn->ext[1 + n]);
		compute_clipping_extents(vram, sn->width, (bit02 >> 15) & 1, (bit02 >> 10) & 1, 0x10, &sn->ext[5]);
		if (s_multi) {   // the second monitor: its palette, mixer and sprite layer
			struct s32_snap *s1 = sn1;
			memcpy(s1->paletteram, mb.paletteram, sizeof(s1->paletteram));
			memcpy(s1->mixer, mb.mixer, sizeof(s1->mixer));
			s1->spi = sn->spi;
			s1->spr = s_sprp[1][sn->spi];
			memcpy(s1->latched, b.sprite_control_latched, sizeof(s1->latched));
			s1->enablemask = sn->enablemask;
		}
	}
	rbx_video_frame_done();
}

// =====================================================================================================================
// ---- timers, the frame ----
static void tm_vbstart(void)
{
	const int64_t t = b.texp[TM_VBSTART];
	t_adjust(TM_VBSTART, t + (int64_t)FRAME_AS);
	vblank_draw();
	s_frame_done = 1;
	// start_of_vblank_int
	signal_v60_irq(MAIN_IRQ_VBSTART);
	t_adjust(TM_VBEND, t + (int64_t)(LINES - VIS_LINES) * (int64_t)LINE_AS);
	if (s_prot_vblank == 1)
		darkedge_fd1149_vblank();
	else if (s_prot_vblank == 2)
		f1lap_fd1149_vblank();
	comm_tick();   // (m_s32comm->check_vint_irq)
}
static void tm_vbend(void)
{
	const int64_t t = b.texp[TM_VBEND];
	b.texp[TM_VBEND] = NEVER;
	signal_v60_irq(MAIN_IRQ_VBSTOP);
	t_adjust(TM_SPRITES, t + 50 * (int64_t)(ATTO / 1000000));   // system32_set_vblank(0)
}
static void tm_line(void)
{
	t_adjust(TM_LINE, b.texp[TM_LINE] + (int64_t)LINE_AS);
	sound_line();
}

static void execute_timers(void)
{
	for (;;) {
		int best = -1;
		for (int i = 0; i < NTIMERS; i++)
			if (b.texp[i] <= b.base && (best < 0 || b.texp[i] < b.texp[best] ||
			                            (b.texp[i] == b.texp[best] && (int32_t)(b.tseq[i] - b.tseq[best]) < 0)))
				best = i;
		if (best < 0)
			break;
		s_cbtime = b.texp[best];
		switch (best) {
		case TM_VBSTART: tm_vbstart(); break;
		case TM_VBEND: tm_vbend(); break;
		case TM_SPRITES: tm_sprites(); break;
		case TM_IRQ0: b.texp[TM_IRQ0] = NEVER; signal_v60_irq(MAIN_IRQ_TIMER0); break;
		case TM_IRQ1: b.texp[TM_IRQ1] = NEVER; signal_v60_irq(MAIN_IRQ_TIMER1); break;
		case TM_LINE: tm_line(); break;
		case TM_SYNC: tm_sync(); break;
		}
	}
	s_cbtime = b.base;
}

static int64_t next_timer(void)
{
	int64_t t = NEVER;
	for (int i = 0; i < NTIMERS; i++)
		if (b.texp[i] < t)
			t = b.texp[i];
	return t;
}

// ---- the sound program's table scan loops (ADR-067) ----
// "h: bit n,(hl) / jp z,X / ... / X: add hl,de / djnz h" (OutRunners' sound driver walks its 24 voice entries this way
// every main loop turn: a third of the Z80's time). The page with h is read through z_read, whose fetch of h ends
// z80_run after that BIT; zscan then does the turns whose BIT finds the bit clear (jp z taken, add, djnz, BIT) as
// z80_run would: each instruction only if it starts before the run's end, the same registers, flags, R, cycles and
// WZ. Those instructions only read mapped memory, so nothing else happens in between; an interrupt the Z80 would take
// (the line held and IFF1, a pending NMI or EI) ends it first.
#pragma push_macro("b")
#undef b   // (the Z80's B register below)
static void zscan(z80 *z, unsigned long end, int line)
{
	if (z->nmi_pending || z->iff_delay || (line && z->iff1) || z->halted)
		return;
	int k = 0;
	while (k < s_zscan_n && (uint16_t)(z->pc - 2) != s_zscan_h[k])
		k++;
	if (k == s_zscan_n)
		return;   // (an interrupt was taken after the BIT)
	const uint16_t h = s_zscan_h[k];
	const uint16_t x = (uint16_t)(s_zrom[h + 3] | s_zrom[h + 4] << 8);
	const uint8_t mask = (uint8_t)(1u << ((s_zrom[h + 1] >> 3) & 7));
	const uint16_t de = (uint16_t)(z->d << 8 | z->e);
	unsigned rinc = 0;
	while (z->zf && (long)(end - z->cyc) > 21) {   // jp z,X / add hl,de / djnz h each start before the end
		const uint16_t hl = (uint16_t)(z->h << 8 | z->l);
		const uint32_t sum = (uint32_t)hl + de;
		const uint8_t hi = (uint8_t)(sum >> 8);
		const int clo = ((hl & 0xff) + (de & 0xff)) > 0xff;
		z->hf = ((hl >> 8 & 0x0f) + (de >> 8 & 0x0f) + clo) > 0x0f;
		z->cf = sum > 0xffff;
		z->nf = 0;
		z->xf = (hi >> 3) & 1;
		z->yf = (hi >> 5) & 1;   // (sf, zf, pf: add hl keeps them)
		z->h = hi;
		z->l = (uint8_t)sum;
		z->mem_ptr = (uint16_t)(hl + 1);
		rinc += 3;
		if (--z->b == 0) {
			z->cyc += 10 + 11 + 8;
			z->pc = (uint16_t)(x + 3);
			break;
		}
		z->cyc += 10 + 11 + 13;
		z->pc = h;
		z->mem_ptr = h;
		const uint8_t *p = s_zmap[hi];
		if ((long)(end - z->cyc) <= 0 || !p)
			break;
		const uint8_t r = p[sum & 0xff] & mask;   // bit n,(hl)
		z->sf = r >> 7;
		z->zf = r == 0;
		z->pf = z->zf;
		z->hf = 1;
		z->nf = 0;
		z->yf = (h >> 13) & 1;   // (from WZ's high byte)
		z->xf = (h >> 11) & 1;
		z->cyc += 12;
		z->pc = (uint16_t)(h + 2);
		rinc += 2;
	}
	z->r = (uint8_t)((z->r & 0x80) | ((z->r + rinc) & 0x7f));
}
#pragma pop_macro("b")
// at init: the loops in the fixed ROM (up to two), their pages then read through z_read
static void zscan_find(void)
{
	s_zscan_n = 0;
	s_zscan_hit = 0;
	if (getenv("RBX_S32_NOZSCAN"))
		return;
	const uint32_t top = s_zrom_size < 0xa000 ? s_zrom_size : 0xa000;
	for (uint32_t h = 0; h + 5 <= top && s_zscan_n < 2; h++) {
		if (s_zrom[h] != 0xcb || (s_zrom[h + 1] & 0xc7) != 0x46 || s_zrom[h + 2] != 0xca)
			continue;
		const uint32_t x = (uint32_t)(s_zrom[h + 3] | s_zrom[h + 4] << 8);
		if (x + 3 > top || s_zrom[x] != 0x19 || s_zrom[x + 1] != 0x10 || (uint16_t)(x + 3 + (int8_t)s_zrom[x + 2]) != h)
			continue;
		s_zscan_h[s_zscan_n++] = (uint16_t)h;
		s_zmap[h >> 8] = NULL;
	}
	if (s_zscan_n == 1)
		s_zscan_h[1] = s_zscan_h[0];
	if (s_zscan_n && getenv("RBX_S32_ZSCANSTAT"))
		fprintf(stderr, "zscan: %d loop(s) at %04x %04x\n", s_zscan_n, s_zscan_h[0], s_zscan_h[1]);
}

static void timeslice(void)
{
	while (b.base < next_timer()) {
		int64_t target = next_timer();
		// the V60
		if (target - b.lt[CPU_V60] >= (int64_t)P_V60) {
			const int cycles = (int)udiv_p((uint64_t)(target - b.lt[CPU_V60]), P_V60, s_inv_v60);
			s_exec = CPU_V60;
			s_target = target;
			const int ran = v60_execute(cycles);
			s_exec = CPU_NONE;
			b.lt[CPU_V60] += (int64_t)ran * (int64_t)P_V60;
			b.v60_cycles += ran;
			if (b.lt[CPU_V60] < target)
				target = b.lt[CPU_V60] > b.base ? b.lt[CPU_V60] : b.base;
		}
		// the Z80
		if (target - b.lt[CPU_Z80] >= (int64_t)P_Z80) {
			const int cycles = (int)udiv_p((uint64_t)(target - b.lt[CPU_Z80]), P_Z80, s_inv_z80);
			unsigned long ran;
			if (b.z_reset)
				ran = (unsigned long)cycles;   // (held in reset: its time eaten)
			else {
				s_exec = CPU_Z80;
				s_target = target;
				s_zstart = s_z80.cyc;
				z80_break = 0;
				s_zscan_hit = 0;
				const unsigned long zend = s_zstart + (unsigned long)cycles;
				for (;;) {
					z80_run(&s_z80, zend, &s_zirq, 0);
					if (!s_zscan_hit)
						break;
					s_zscan_hit = z80_break = 0;
					zscan(&s_z80, zend, s_zirq);
					if ((long)(s_z80.cyc - zend) >= 0)
						break;
				}
				z80_break = 0;
				s_exec = CPU_NONE;
				ran = s_z80.cyc - s_zstart;
			}
			b.lt[CPU_Z80] += (int64_t)ran * (int64_t)P_Z80;
			if (b.lt[CPU_Z80] < target)
				target = b.lt[CPU_Z80] > b.base ? b.lt[CPU_Z80] : b.base;
		}
		// the V25 (in its input clocks)
		if (s_v25 && target - b.lt[CPU_V25] >= (int64_t)P_V25) {
			const int64_t clocks = (target - b.lt[CPU_V25]) / (int64_t)P_V25;
			const int64_t start = s_nec.total;
			s_exec = CPU_V25;
			s_target = target;
			nec_run_until(&s_nec, start + clocks);
			s_exec = CPU_NONE;
			b.lt[CPU_V25] += (s_nec.total - start) * (int64_t)P_V25;
		}
		b.base = target;
		s_cbtime = b.base;
	}
	execute_timers();
}

// the times rebased to the frame's start (attoseconds of a few frames fit in 64 bits)
static void rebase(void)
{
	const int64_t d = b.base;
	b.base -= d;
	b.lt[0] -= d;
	b.lt[1] -= d;
	b.lt[2] -= d;
	for (int i = 0; i < NTIMERS; i++)
		if (b.texp[i] != NEVER)
			b.texp[i] -= d;
	s_cbtime = b.base;
}

// ---- inputs ----
static void analog_frame(void)
{
	if (s_board == BD_ANALOG) {
		// steering / guns follow the stick or the d-pad (paddles back to the centre without input), pedals on
		// buttons 1 and 2
		int16_t *an_val = s_multi ? mb.an_val : b.an_val;
		for (int ch = 0; ch < (s_multi ? 8 : 4); ch++) {
			if (s_port_an[ch] < 0)
				continue;
			const int t = s_an[ch].type, pl = s_an[ch].player, mx = (int)s_an[ch].mask;
			int v = an_val[ch];
			if (t == RBX_IPT_PEDAL || t == RBX_IPT_PEDAL2 || t == RBX_IPT_PEDAL3) {
				const uint32_t in = rbx_input_bits(pl);
				const uint32_t btn = t == RBX_IPT_PEDAL ? RBX_IN_B1 : t == RBX_IPT_PEDAL2 ? RBX_IN_B2 : RBX_IN_B3;
				v = (in & btn) ? mx : (int)s_an[ch].def;
			} else {
				const int d = rbx_analog_dpad(pl, t, 8);
				if (d)
					v += d;
				else if (t == RBX_IPT_PADDLE)
					v += v < (int)s_an[ch].def ? 8 : v > (int)s_an[ch].def ? -8 : 0;
				if ((d == 0 && t == RBX_IPT_PADDLE) && ((v > (int)s_an[ch].def) != (an_val[ch] > (int)s_an[ch].def)))
					v = (int)s_an[ch].def;
				v = v < 0 ? 0 : v > mx ? mx : v;
			}
			an_val[ch] = (int16_t)v;
		}
		return;
	}
	if (s_board != BD_TRACK)
		return;
	for (int i = 0; i < 3; i++) {
		if (s_port_tr[2 * i] >= 0)
			b.upd_x[i] = (uint16_t)((b.upd_x[i] - rbx_analog_dpad(i + 1, RBX_IPT_TRACKBALL_X, 15)) & 0xfff);   // (TRACKX: PORT_REVERSE)
		if (s_port_tr[2 * i + 1] >= 0)
			b.upd_y[i] = (uint16_t)((b.upd_y[i] + rbx_analog_dpad(i + 1, RBX_IPT_TRACKBALL_Y, 15)) & 0xfff);
	}
}

static uint8_t ppi_in(void *u, int n) { (void)u; return port8(s_port_ex[n]); }
static uint8_t ppi_a(void *u) { return ppi_in(u, 0); }
static uint8_t ppi_b(void *u) { return ppi_in(u, 1); }
static uint8_t ppi_c(void *u) { return ppi_in(u, 2); }

// ---- init / reset ----
static void s32_reset(void)
{
	rbx_aux2_sync();
	memset(&b.texp, 0, offsetof(struct s32_state, state_end) - offsetof(struct s32_state, texp));
	for (int i = 0; i < NTIMERS; i++)
		b.texp[i] = NEVER;
	// device_start: video RAM cleared (with its control word), the mixer to 0xff, the sound RAM to 0xff
	memset(b.videoram, 0, sizeof(b.videoram));
	b.videoram[0x1ff00 / 2] = 0x8000;
	memset(b.mixer, 0xff, sizeof(b.mixer));
	memset(b.workram, 0, sizeof(b.workram));
	memset(b.spriteram, 0, sizeof(b.spriteram));
	memset(b.paletteram, 0, sizeof(b.paletteram));
	memset(b.zram, 0, sizeof(b.zram));
	s_spmap[0] = 0;   // (A, B cleared in b.sprbuf / mb.sprbuf: the render side is idle at a reset)
	s_spmap[1] = 1;
	memset(b.sprbuf, 0, sizeof(b.sprbuf));
	memset(b.protram, 0, sizeof(b.protram));
	memset(b.comm_shared, 0, sizeof(b.comm_shared));
	memset(b.dpram, 0, sizeof(b.dpram));
	rf5c68_init(&s_rf, RF_CLOCK);
	memset(s_rf.ram, 0xff, sizeof(s_rf.ram));
	if (s_multi) {
		memset(s_mp, 0, offsetof(struct m32_state, state_end));
		memset(mb.mixer, 0xff, sizeof(mb.mixer));
		for (int i = 0; i < 8; i++)
			mb.an_val[i] = (int16_t)s_an[i].def;
		uint32_t psz;
		const uint8_t *pcm = rbx_region("mainpcb:sega", &psz);
		if (s_pcm_async)
			rbx_aux_sync();   // (the worker done with the chip)
		multipcm_init(s_mpcm, 40000000u / 4, pcm, pcm ? psz : 0);
	}
	b.rng = 0x9d14abd7;
	for (int i = 0; i < 4; i++)
		b.an_val[i] = (int16_t)s_an[i].def;
	// device_reset: the interrupt controller to 0xff, the I/O chip's outputs to 0 (CNT2 low: the Z80 held in reset)
	memset(b.v60_irq_control, 0xff, sizeof(b.v60_irq_control));
	b.io_dir = 0;
	b.io_cnt = 0;
	memset(b.io_latch, 0, sizeof(b.io_latch));
	b.z_reset = b.z_reset_want = 1;
	b.displayenable = 0;
	b.texp[TM_VBSTART] = (int64_t)FRAME_AS;   // (the screen starts in its vblank: the first one a frame on)
	b.tseq[TM_VBSTART] = ++b.seq;
	b.texp[TM_LINE] = 0;
	b.tseq[TM_LINE] = ++b.seq;
	s_exec = CPU_NONE;
	s_cbtime = 0;
	s_zirq = 0;
	map_main();
	v60_init();
	if (s_multi)
		v60_set_pir(0x00007000);   // (the V70)
	v60_reset();
	z80_reset_cpu();
	z80_break = 0;
	sound_bank_update();
	rbx_ym_reset(0);
	if (!s_multi)
		rbx_ym_reset(1);
	if (s_board == BD_4P)
		i8255_reset(&s_ppi);
	if (s_v25) {
		v25_map();
		nec_reset(&s_nec);
		s_nec.total = 0;
	}
	s_mix_n = 0;
	s_pal_ready = 0;
	pal_dirty_all();
}

static void s32_exit(void);

static int s32_init(const rbx_game *g)
{
	char prefix[48];
	{
		const char *c = strchr(g->machine, ':');
		size_t n = c ? (size_t)(c - g->machine) : strlen(g->machine);
		if (n >= sizeof(prefix))
			n = sizeof(prefix) - 1;
		memcpy(prefix, g->machine, n);
		prefix[n] = 0;
	}
	s_multi = !strncmp(prefix, "sega_multi32", 12);
	s_board = (!strcmp(prefix, "sega_system32_analog") || !strcmp(prefix, "sega_multi32_analog")) ? BD_ANALOG
	        : !strcmp(prefix, "sega_system32_track") ? BD_TRACK
	        : (!strcmp(prefix, "sega_system32_4p") || !strcmp(prefix, "sega_system32_ga2") ||
	           !strcmp(prefix, "sega_system32_arf") || !strcmp(prefix, "sega_multi32_6p")) ? BD_4P : BD_REGULAR;
	s_main_clock = s_multi ? 32000000u : 32215900u;
	s_cpu_clock = s_multi ? 40000000u / 2 : 32215900u / 2;
	s_p_v60 = ATTO / V60_CLOCK;
	s_p_z80 = ATTO / Z80_CLOCK;
	s_inv_v60 = 1.0 / (double)s_p_v60;
	s_inv_z80 = 1.0 / (double)s_p_z80;
	const char *init = strchr(g->machine, ':') ? strchr(g->machine, ':') + 1 : "";
	s_prot = !strcmp(init, "sonic") ? PROT_SONIC : !strcmp(init, "brival") ? PROT_BRIVAL
	       : !strcmp(init, "darkedge") ? PROT_DARKEDGE : !strcmp(init, "dbzvrvs") ? PROT_DBZVRVS
	       : !strcmp(init, "jleague") ? PROT_JLEAGUE : PROT_NONE;
	s_prot_vblank = !strcmp(init, "darkedge") ? 1 : !strcmp(init, "f1lap") ? 2 : 0;
	s_screen_flipy = !s_multi && !strcmp(init, "holo");   // (Holosseum's ORIENTATION_FLIP_Y; AS-1 shares its init)
	s_scross = !strcmp(init, "scross");
	s_linktype = !strcmp(init, "radr") ? 14084 : (!strcmp(init, "f1lap") || !strcmp(init, "f1lapt")) ? 15612
	           : (!strcmp(init, "orunners") || s_scross) ? 15033 : 0;

	if (!s_bp && !(s_bp = calloc(1, sizeof(*s_bp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	s_rom = rbx_region("mainpcb:maincpu", &s_rom_size);
	s_zrom = rbx_region("mainpcb:soundcpu", &s_zrom_size);
	s_tiles = rbx_region("mainpcb:tiles", &s_tiles_size);
	s_sprites = rbx_region("mainpcb:sprites", &s_sprites_size);
	if (!s_rom || s_rom_size < 0x200000 || !s_zrom || !s_tiles || !s_sprites) {
		rbx_fail("%s: ROM regions missing", g->set);
		s32_exit();
		return -1;
	}
	// the NBG tiles: bytes 1 and 2 of each half row swapped (the layout's x order: 0, 4, 16, 20, 8, 12, 24, 28 bits)
	for (uint32_t i = 0; i + 4 <= s_tiles_size; i += 4) {
		const uint8_t t = s_tiles[i + 1];
		s_tiles[i + 1] = s_tiles[i + 2];
		s_tiles[i + 2] = t;
	}
	s_ntiles = s_tiles_size / 128;
	// the sprites: a big-endian 32-bit region (MAME swaps it to host order): swapped in place
	for (uint32_t i = 0; i + 4 <= s_sprites_size; i += 4) {
		uint8_t *p = s_sprites + i;
		const uint8_t t0 = p[0], t1 = p[1];
		p[0] = p[3]; p[1] = p[2]; p[2] = t1; p[3] = t0;
	}
	s_v25 = !strcmp(prefix, "sega_system32_ga2") || !strcmp(prefix, "sega_system32_arf");
	if (s_v25) {
		s_mcu = rbx_region("mainpcb:mcu", &s_mcu_size);
		if (!s_mcu || s_mcu_size < 0x10000) {
			rbx_fail("%s: mcu missing", g->set);
			s32_exit();
			return -1;
		}
		// decrypt_protrom: the address lines unscrambled
		uint8_t *temp = (uint8_t *)malloc(0x10000);
		if (!temp) {
			rbx_fail("%s: out of memory", g->set);
			s32_exit();
			return -1;
		}
		memcpy(temp, s_mcu, 0x10000);
		static const uint8_t bits[16] = { 14, 11, 15, 12, 13, 4, 3, 7, 5, 10, 2, 8, 9, 6, 1, 0 };   // (bitswap<16>)
		for (uint32_t i = 0; i < 0x10000; i++) {
			uint32_t src = 0;
			for (int k = 0; k < 16; k++)
				src |= ((i >> bits[k]) & 1u) << (15 - k);
			s_mcu[i] = temp[src];
		}
		free(temp);
		nec_init(&s_nec, NEC_V35, !strcmp(prefix, "sega_system32_ga2") ? s_ga2_opcode_table : s_arf_opcode_table);
		s_nec.read8 = v25_r8;
		s_nec.read16 = v25_r16;
		s_nec.write8 = v25_w8;
		s_nec.write16 = v25_w16;
		v25_map();
	}
	if (!strcmp(init, "jpark")) {   // (the drive board's check patched out, as MAME)
		s_rom[0xc15a8] = 0x70; s_rom[0xc15a9] = 0xcd;
		s_rom[0xc15aa] = 0xcd; s_rom[0xc15ab] = 0xd8;
	}
	for (int i = 0; i < 2; i++)
		s_snapv[i] = calloc(1, sizeof(*s_snap));
	s_snap = s_snapv[0];
	s_sj.ram = calloc(0x10000, sizeof(uint16_t));
	s_sj.pending = 0;
	s_rfp = calloc(1, sizeof(*s_rfp));
	s_mxs = calloc(2, sizeof(*s_mxs));
	s_olut = calloc(NPAL, sizeof(*s_olut));   // (the output table, ADR-073)
	s_olut_fn = NULL;
	if (s_multi) {
		for (int i = 0; i < 2; i++)
			s_snap1v[i] = calloc(1, sizeof(*s_snap1));
		s_snap1 = s_snap1v[0];
		s_mp = calloc(1, sizeof(*s_mp));
		s_mpcm = calloc(1, sizeof(*s_mpcm));
	}
	s_sprx = calloc((size_t)(s_multi ? 4 : 2) * SW * SH, sizeof(uint16_t));
	if (!s_snapv[0] || !s_snapv[1] || !s_sj.ram || !s_rfp || !s_mxs || !s_olut || !s_sprx || (s_multi && (!s_snap1v[0] || !s_snap1v[1] || !s_mp || !s_mpcm))) {
		rbx_fail("%s: out of memory", g->set);
		s32_exit();
		return -1;
	}
	s_port_p1 = rbx_port_find("mainpcb:P1_A");
	s_port_p2 = rbx_port_find("mainpcb:P2_A");
	s_port_pc = rbx_port_find("mainpcb:PORTC_A");
	s_port_s12 = rbx_port_find("mainpcb:SERVICE12_A");
	s_port_s34 = rbx_port_find("mainpcb:SERVICE34_A");
	{
		static const char *const tb[5] = { "mainpcb:P1_B", "mainpcb:P2_B", "mainpcb:PORTC_B", "mainpcb:SERVICE12_B",
		                                   "mainpcb:SERVICE34_B" };
		for (int i = 0; i < 5; i++)
			s_port_b[i] = s_multi ? rbx_port_find(tb[i]) : -1;
	}
	for (int i = 0; i < 8; i++) {
		char tag[24];
		snprintf(tag, sizeof(tag), "mainpcb:ANALOG%d", i + 1);
		s_port_an[i] = rbx_port_find(tag);
		s_an[i].type = RBX_IPT_UNUSED;
		s_an[i].player = 1;
		s_an[i].def = 0xff;
		s_an[i].mask = 0xff;
		for (int p = 0; g->ports && g->ports[p].tag; p++)
			if (!strcmp(g->ports[p].tag, tag) && g->ports[p].count) {
				const rbx_ipt_field *f = &g->fields[g->ports[p].first];
				s_an[i].type = f->type;
				s_an[i].player = f->player ? f->player : 1;
				s_an[i].def = f->def & f->mask;
				s_an[i].mask = f->mask;
			}
	}
	for (int i = 0; i < 3; i++) {
		char tag[24];
		snprintf(tag, sizeof(tag), "mainpcb:EXTRA%d", i + 1);
		s_port_ex[i] = rbx_port_find(tag);
		snprintf(tag, sizeof(tag), "mainpcb:TRACKX%d", i + 1);
		s_port_tr[2 * i] = rbx_port_find(tag);
		snprintf(tag, sizeof(tag), "mainpcb:TRACKY%d", i + 1);
		s_port_tr[2 * i + 1] = rbx_port_find(tag);
	}
	memset(&s_ppi, 0, sizeof(s_ppi));
	s_ppi.in_pa = ppi_a;
	s_ppi.in_pb = ppi_b;
	s_ppi.in_pc = ppi_c;
	s_ppi.tri_pa = s_ppi.tri_pb = s_ppi.tri_pc = 0xff;
	eepser_init(&s_eeprom, EEPSER_93CXX, 64, 16, V60_CLOCK);
	{
		uint32_t esz;
		const uint8_t *e = rbx_region_opt("mainpcb:eeprom", &esz);   // (the default contents of some sets)
		eepser_default(&s_eeprom, e, e ? esz : 0);
	}
	eepser_nv_load(&s_eeprom, g->set);
	// sound: the YM3438s at 8 MHz, 0.30 on each side; the RF5C68 0.40 (Multi 32: one YM3438 0.15, the MultiPCM 0.35)
	for (int c = 0; c < (s_multi ? 1 : 2); c++) {
		rbx_ym_config yc;
		memset(&yc, 0, sizeof(yc));
		yc.type = RBX_YM3438;
		yc.clock = YM_CLOCK;
		yc.gain[0] = yc.gain[1] = s_multi ? 38 : 77;
		yc.irq = c == 0 ? ym3438_irq_handler : NULL;
		if (rbx_ym_init(c, &yc) || rbx_ym_set_async(c, 2)) {
			rbx_fail("%s: out of memory", g->set);
			s32_exit();
			return -1;
		}
	}
	// (the two chips' last steps of a frame on two threads: the emulation thread makes one while the worker makes the
	// other, ADR-100)
	if (!s_multi && !getenv("RBX_S32_NOYMHELP"))
		rbx_ym_async_help(1);
	s_mix_cap = (int)(rbx_ym_rate(0) / 50) + 256;
	s_mix_l = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	s_mix_r = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	if (!s_mix_l || !s_mix_r || rbx_audio_setup_stereo(rbx_ym_rate(0)) || rbx_video_setup(SW * (s_multi ? 2 : 1), SH, NPAL)) {
		rbx_fail("%s: out of memory", g->set);
		s32_exit();
		return -1;
	}
	s_pcm_pos = 0;
	s_pcm_rate = rbx_ym_rate(0);
	if (s_multi && !getenv("RBX_S32_PCM_SYNC")) {
		s_pcm_l = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
		s_pcm_r = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
		s_pcm_async = s_pcm_l && s_pcm_r && !rbx_ym_async_set_user(0, pcm_user);
	}
	for (int i = 0; i < 0xa0; i++)
		s_zmap[i] = (uint32_t)(i + 1) * 256 <= s_zrom_size ? s_zrom + i * 256 : NULL;
	for (int i = 0; i < 0x20; i++)
		s_zmap[0xe0 + i] = b.zram + i * 256;
	for (int i = 0xc0; i < 0xe0; i++)
		s_zmap[i] = NULL;
	zscan_find();
	for (int m = 0; m < 2; m++)
		for (int i = 0; i < 4; i++)
			s_sprp[m][i] = m && !s_multi ? NULL : i < 2 ? (m ? mb.sprbuf[i] : b.sprbuf[i]) : s_sprx + (size_t)(m * 2 + i - 2) * SW * SH;
	s_spmap[0] = 0;
	s_spmap[1] = 1;
	for (int i = 0; i < 2; i++) {   // (a slot never taken: buffer A)
		s_snapv[i]->spi = 0;
		s_snapv[i]->spr = s_sprp[0][0];
		if (s_multi) {
			s_snap1v[i]->spi = 0;
			s_snap1v[i]->spr = s_sprp[1][0];
		}
	}
	s32_reset();
	s_sprite_async = getenv("RBX_S32_SPRITE_SYNC") == NULL;
	if (s_sprite_async) {
		rbx_aux2_set_job(sprite_job);
		// SegaSonic: CPU1's YM3438 worker is what the frame waits for (its sound and sprites the heaviest, its drawing
		// light): the sprites on CPU3 (above the drawing helper) where it is open (ADR-100)
		if (s_board == BD_TRACK && !getenv("RBX_S32_SPRCPU1"))
			rbx_aux2_set_alone(1);
	}
	// the next frame's snapshot taken while this one is drawn (ADR-058); RBX_S32_DEPTH=1: the render thread waited for
	if (!getenv("RBX_S32_DEPTH") || atoi(getenv("RBX_S32_DEPTH")) != 1)
		rbx_video_set_depth(2);
	return 0;
}

static void s32_exit(void)
{
	rbx_aux2_set_job(NULL);   // (waits for the sprites)
	free(s_sj.ram);
	s_sj.ram = NULL;
	eepser_nv_save(&s_eeprom);
	rbx_ym_exit(0);
	rbx_ym_exit(1);
	free(s_mix_l);
	free(s_mix_r);
	s_mix_l = s_mix_r = NULL;
	s_pcm_async = 0;   // (rbx_ym_exit above waited for the worker)
	free(s_pcm_l);
	free(s_pcm_r);
	s_pcm_l = s_pcm_r = NULL;
	for (int i = 0; i < 2; i++) {
		free(s_snapv[i]);
		free(s_snap1v[i]);
		s_snapv[i] = s_snap1v[i] = NULL;
	}
	s_snap = NULL;
	free(s_sprx);
	s_sprx = NULL;
	memset(s_sprp, 0, sizeof(s_sprp));
	free(s_rfp);
	s_rfp = NULL;
	free(s_mxs);
	free(s_olut);
	s_olut = NULL;
	s_olut_fn = NULL;
	s_mxs = NULL;
	s_snap1 = NULL;
	free(s_mp);
	s_mp = NULL;
	free(s_mpcm);
	s_mpcm = NULL;
	free(s_bp);
	s_bp = NULL;
	s_exec = CPU_NONE;
	v60_release();   // (the JIT's tables, ADR-200)
}

static void s32_frame(void)
{
	rbx_audio.frame_mhz = 60000;
	analog_frame();
	s_frame_done = 0;
	while (!s_frame_done)
		timeslice();
	rebase();
	sound_frame_end();
}

static int s32_state_io(void *ctx, int load)
{
	rbx_aux2_sync();   // (the sprites drawn into the saved / replaced frame buffer)
	if (s_pcm_async)
		rbx_aux_sync();   // (the MultiPCM's queued steps done)
	spr_canonical();   // (the render side is idle here: the snapshots' buffers may be written)
	v60_state_t vs;
	z80 zs = s_z80;
	if (!load)
		v60_get_state(&vs);
	int rc = 0;
	rc |= rbx_state_io(ctx, "v60", &vs, sizeof(vs));
	rc |= rbx_state_io(ctx, "z80", &zs, sizeof(zs));
	rc |= rbx_state_io(ctx, "s32", s_bp, offsetof(struct s32_state, state_end));
	rc |= rbx_state_io(ctx, "rf5c68", &s_rf, RF5C68_STATE_SIZE);
	rc |= eepser_state(&s_eeprom, ctx, load, "s32_eeprom");
	if (s_board == BD_4P)
		rc |= rbx_state_io(ctx, "ppi", (uint8_t *)&s_ppi + I8255_STATE_OFFSET, I8255_STATE_SIZE);
	if (s_v25)
		rc |= rbx_state_io(ctx, "v25", &s_nec, NEC_STATE_SIZE);
	if (s_multi) {
		rc |= rbx_state_io(ctx, "s32m", s_mp, offsetof(struct m32_state, state_end));
		rc |= rbx_state_io(ctx, "multipcm", s_mpcm, MULTIPCM_STATE_SIZE);
	}
	rc |= rbx_ym_state(0, ctx, load);
	if (!s_multi)
		rc |= rbx_ym_state(1, ctx, load);
	if (!rc && load) {
		v60_set_state(&vs);
		if (s_v25)
			nec_after_load(&s_nec);
		// (the Z80's registers; its callbacks and maps stay this run's)
		zs.read_byte = s_z80.read_byte;
		zs.write_byte = s_z80.write_byte;
		zs.port_in = s_z80.port_in;
		zs.port_out = s_z80.port_out;
		zs.userdata = s_z80.userdata;
		zs.read_map = s_zmap;
		zs.op_rom = NULL;
		s_z80 = zs;
		s_zirq = b.z_line;
		map_main();
		sound_bank_update();
		s_cbtime = b.base;
		s_pal_ready = 0;
		pal_dirty_all();
	}
	return rc ? -1 : 0;
}

static int s32_save(void *ctx) { return s32_state_io(ctx, 0); }
static int s32_load(void *ctx) { return s32_state_io(ctx, 1); }
static int s32_supported(const char *machine)
{
	static const char *const ok[] = { "sega_system32:", "sega_system32_analog:", "sega_system32_track:",
	                                  "sega_system32_4p:", "sega_system32_ga2:", "sega_system32_arf:", "sega_multi32:",
	                                  "sega_multi32_analog:", "sega_multi32_6p:", NULL };
	for (int i = 0; ok[i]; i++)
		if (!strncmp(machine, ok[i], strlen(ok[i])))
			return 1;
	return 0;
}

const rbx_driver rbx_drv_segas32 = { "Sega System 32", s32_init, s32_exit, s32_reset, s32_frame, s32_save, s32_load,
                                     s32_supported, s32_render, s32_help };
