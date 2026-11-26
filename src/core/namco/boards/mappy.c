/*  Namco Super Pac-Man / Pac & Pal / Grobda, Phozon and Mappy / The Tower of Druaga / Dig Dug II / Motos: two MC6809E
    (Phozon: three), the 56XX / 58XX / 59XX custom I/O chips, the 15XX wavetable sound and three video variants (the
    static 36 x 28 tilemap with 2bpp 16x16 sprites, Phozon's 8x8 sprites, Mappy's column-scrolled 36 x 60 tilemap with
    4bpp sprites).
    Ported from MAME 0.289 src/mame/namco/mappy.cpp, mappy.h, mappy_v.cpp (BSD-3-Clause, copyright-holders: Nicola
    Salmoria) and MAME's compute_resistor_weights (src/emu/video/resnet.cpp, BSD-3-Clause, copyright-holders:
    Couriersud).
    This file is part of ReBOX. GPL-2.0-or-later.

    Not done: the LEDs, the coin counters / lockout (out_lamps), the watchdog.
*/
#include "../../rbx.h"
#include "../../cpu/m6809/m6809.h"
#include "../../sound/namco_wsg.h"
#include "../machine/namcoio.h"

#include <stdlib.h>
#include <string.h>

#define VIS_W 288
#define VIS_H 224
#define FRAME_MHZ 60606         // 18.432 MHz / 3 / (384 x 264)
#define FRAME_CYCLES 25344      // (the CPUs' clocks a frame: 96 a line)
#define CPU_CLOCK 1536000       // 18.432 MHz / 12
#define SND_RATE 48000
#define QUANTUM 256             // (set_maximum_quantum 6000 Hz)
#define IO_DELAY 76             // (the custom I/O's run timer: 50 us after the vblank, 76.8 clocks)
#define LINE0_AT (40 * 96)      // (the screen's line 0 / vblank end timers: 40 lines after the vblank's start)

enum { VID_SUPERPAC, VID_PHOZON, VID_MAPPY };

// ---- the set's configuration ----
static int s_vid, s_ncpu;
static uint8_t *s_rom[3], *s_tilerom, *s_sprrom, *s_proms, *s_sndprom;
static uint32_t s_rom_size[3], s_tile_size, s_spr_size, s_proms_size, s_sndprom_size;
static uint8_t *s_chrpix, *s_sprpix;   // decoded, a byte a pixel
static uint32_t s_chr_count, s_spr_count;
static int s_spr_w, s_spr_bpp, s_spr_colors;   // (16 or 8 pixels, 2 or 4 bits, the colour count)
static int s_npens;
static uint32_t s_pal[64 * 4 + 64 * 16];
static uint8_t s_ind[64 * 4 + 64 * 16];   // (pen_indirect)
static uint8_t s_chr_trans[64];           // (configure_groups: a colour's pens drawn transparent in the second pass)
static uint16_t s_spr_trans[64];          // (transpen_mask)
static int s_port_p1, s_port_p2, s_port_buttons, s_port_coins, s_port_dsw0, s_port_dsw1, s_port_dsw2;

static m6809_t s_cpu[3];   // main, sub (sound), Phozon's sub2
static namcoio_t s_io[2];
static namco_wsg_t s_wsg;

// ---- the board's state (saved) ----
static struct {
	uint8_t latch;          // (the LS259's outputs)
	uint8_t main_irq_mask, sub_irq_mask, sub2_irq_mask;
	uint8_t flip, scroll, dipsel, pad;
	uint8_t io_armed[2];    // (namcoio_run_timer pending)
	uint8_t in_reset[3];    // (a CPU held in reset: its time runs on, it does not execute)
	int64_t t;              // (the scheduler's base time)
	uint64_t snd_frac;
	uint8_t vram[0x1000];
	uint8_t spr[0x1800];
	uint8_t ram2[0x800];    // (Phozon's sub2 work RAM)
} b;

// ---- the drawing's copy ----
typedef struct {
	uint8_t vram[0x1000];
	uint8_t spr[0x1800];
	uint8_t flip, scroll;
} snap_t;
static snap_t *s_snap;
static uint16_t *s_sbm;     // (Super Pac-Man's sprite bitmap)
static int32_t s_mix[16];   // (a timeslice's samples)

// ---- the synced input line changes (MAME's set_input_line: applied at the end of the timeslice) ----
static int s_sync;           // (the running CPU asked for a sync: its timeslice ends after this instruction)
static uint8_t s_pend_irq_clear[3];
static int8_t s_pend_reset[3][4];   // (the RESET line changes posted, in order; -1 ends)
static int s_pend_reset_n[3];

static void post_irq_clear(int cpu)
{
	s_pend_irq_clear[cpu] = 1;
	s_sync = 1;
}

static void post_reset(int cpu, int state)
{
	if (s_pend_reset_n[cpu] < 4)
		s_pend_reset[cpu][s_pend_reset_n[cpu]++] = (int8_t)state;
	s_sync = 1;
}

// ---- the LS259 (write_a0: the bit in A1-A3, the data in A0) ----
static void latch_w(int offset)
{
	const int bit = (offset >> 1) & 7, d = offset & 1;
	if (((b.latch >> bit) & 1) == d)
		return;
	b.latch = (uint8_t)(b.latch ^ (1 << bit));
	switch (bit) {
	case 0:   // sub_int_on_w
		b.sub_irq_mask = (uint8_t)d;
		if (!d)
			post_irq_clear(1);
		break;
	case 1:   // main_int_on_w
		b.main_irq_mask = (uint8_t)d;
		if (!d)
			post_irq_clear(0);
		break;
	case 2:
		if (s_vid == VID_MAPPY)   // flip_screen_set
			b.flip = (uint8_t)d;
		else if (s_vid == VID_PHOZON) {   // sub2_int_on_w
			b.sub2_irq_mask = (uint8_t)d;
			if (!d)
				post_irq_clear(2);
		}
		break;
	case 3:
		namco_wsg_sound_enable_w(&s_wsg, d);
		break;
	case 4:   // the I/O chips' reset (inverted)
		namcoio_set_reset_line(&s_io[0], !d);
		namcoio_set_reset_line(&s_io[1], !d);
		break;
	case 5:   // the sub CPU's reset (inverted)
		post_reset(1, !d);
		break;
	case 6:
		if (s_vid == VID_PHOZON)   // the sub2 CPU's reset (inverted)
			post_reset(2, !d);
		break;
	default:
		break;
	}
}

// ---- the I/O chips' pins ----
static uint8_t io_in(void *ctx, int n)
{
	const int chip = (int)(intptr_t)ctx;
	if (chip == 0) {
		static const int *const ports[4] = { &s_port_coins, &s_port_p1, &s_port_p2, &s_port_buttons };
		return (uint8_t)rbx_port_read(*ports[n]);
	}
	switch (n) {
	case 0: {   // the LS157 (dipmux): DSW2's low or high nibble
		const uint32_t v = rbx_port_read(s_port_dsw2);
		return (uint8_t)((b.dipsel ? v >> 4 : v) & 0x0f);
	}
	case 1: return (uint8_t)rbx_port_read(s_port_dsw1);
	case 2: return (uint8_t)(rbx_port_read(s_port_dsw1) >> 4);
	default: return (uint8_t)rbx_port_read(s_port_dsw0);
	}
}

static void io_out(void *ctx, int n, uint8_t data)
{
	if ((int)(intptr_t)ctx == 1 && n == 0)   // the dipmux's select
		b.dipsel = data & 1;
	// (chip 0: out_lamps on Pac & Pal and Motos, not done)
}

// ---- the main CPU (superpac_main_map, phozon main_map, mappy_main_map) ----
static uint8_t main_r(void *ctx, uint16_t a)
{
	(void)ctx;
	if (a >= 0x4000 && a < 0x4400) return namco_15xx_amap_r(&s_wsg, a & 0x3ff);
	if (a >= 0x4800 && a < 0x4820) return namcoio_read(&s_io[(a >> 4) & 1], a & 0x0f);
	if (a == 0x2000 && s_vid == VID_SUPERPAC) {   // flipscreen_r
		b.flip = 1;
		return 0xff;
	}
	return 0;
}

static void main_w(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	if (a >= 0x4000 && a < 0x4400)
		namco_15xx_amap_w(&s_wsg, a & 0x3ff, v);
	else if (a >= 0x4800 && a < 0x4820)
		namcoio_write(&s_io[(a >> 4) & 1], a & 0x0f, v);
	else if (a >= 0x5000 && a < 0x5010)
		latch_w(a & 0x0f);
	else if (a == 0x2000 && s_vid == VID_SUPERPAC)   // flipscreen_w
		b.flip = v & 1;
	else if (a >= 0x3800 && a < 0x4000 && s_vid == VID_MAPPY)   // mappy_scroll_w
		b.scroll = (uint8_t)((a - 0x3800) >> 3);
}

// ---- the sub CPU (superpac_sub_map, phozon sub_map) and Phozon's sub2 (sub2_map) ----
static uint8_t sub_r(void *ctx, uint16_t a)
{
	(void)ctx;
	if (a < 0x0400) return namco_15xx_amap_r(&s_wsg, a);
	return 0;
}

static void sub_w(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	if (a < 0x0400)
		namco_15xx_amap_w(&s_wsg, a, v);
	else if (a >= 0x2000 && a < 0x2010 && s_vid != VID_PHOZON)
		latch_w(a & 0x0f);
}

static uint8_t sub2_r(void *ctx, uint16_t a)
{
	(void)ctx;
	if (a >= 0x4000 && a < 0x4400) return namco_15xx_amap_r(&s_wsg, a & 0x3ff);
	return 0;
}

static void sub2_w(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	if (a >= 0x4000 && a < 0x4400)
		namco_15xx_amap_w(&s_wsg, a & 0x3ff, v);
}

// ---- drawing ----
static inline int superpac_scan(int col, int row)   // (36 x 28 from the 32 x 32 RAM)
{
	row += 2;
	col -= 2;
	return (col & 0x20) ? row + ((col & 0x1f) << 5) : col + (row << 5);
}

static inline int mappy_scan(int col, int row)   // (a 32 x 60 scrolling part and two fixed 2 x 28 parts)
{
	col -= 2;
	if (col & 0x20) {
		if (row & 0x20)
			return 0x7ff;   // outside visible area
		return ((row + 2) & 0x0f) + (row & 0x10) + ((col & 3) << 5) + 0x780;
	}
	return col + (row << 5);
}

// the tilemap; pass 0 opaque (all categories), pass 1 the category 1 tiles with the colour's transparent pens left
// out. The flipped screen is the tilemap mirrored (the sprites are not).
static void draw_bg(const snap_t *sn, int pass)
{
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch, flip = sn->flip;
	for (int y = 0; y < VIS_H; y++) {
		uint16_t *d = bm + y * pitch;
		const int ly = flip ? VIS_H - 1 - y : y;
		for (int x = 0; x < VIS_W; x++) {
			const int lx = flip ? VIS_W - 1 - x : x;
			const int col = lx >> 3;
			int idx, attr, code, ty = ly;
			if (s_vid == VID_MAPPY) {
				if (col >= 2 && col < 34)
					ty = (ly + sn->scroll) % 480;
				idx = mappy_scan(col, ty >> 3);
				code = sn->vram[idx];
				attr = sn->vram[idx + 0x800];
			} else {
				idx = superpac_scan(col, ty >> 3);
				attr = sn->vram[idx + 0x400];
				code = sn->vram[idx];
				if (s_vid == VID_PHOZON)
					code += (attr & 0x80) << 1;
			}
			const int color = attr & 0x3f;
			const int pen = s_chrpix[rbx_wrap((uint32_t)code, s_chr_count) * 64 + (uint32_t)((ty & 7) * 8 + (lx & 7))];
			if (pass) {
				if (!(attr & 0x40) || ((s_chr_trans[color] >> pen) & 1))
					continue;
			}
			d[x] = (uint16_t)(color * 4 + pen);
		}
	}
}

// drawgfx transmask of a w x w element (clipped to the screen)
static void blit(uint16_t *bm, int pitch, uint32_t code, int color, int fx, int fy, int sx, int sy)
{
	const int w = s_spr_w;
	const uint8_t *src = s_sprpix + rbx_wrap(code, s_spr_count) * (uint32_t)(w * w);
	const uint32_t trans = s_spr_trans[color];
	const uint16_t base = (uint16_t)(256 + color * (1 << s_spr_bpp));
	const int x0 = sx < 0 ? -sx : 0, x1 = sx + w > VIS_W ? VIS_W - sx : w;
	const int y0 = sy < 0 ? -sy : 0, y1 = sy + w > VIS_H ? VIS_H - sy : w;
	for (int j = y0; j < y1; j++) {
		const uint8_t *s = src + (fy ? w - 1 - j : j) * w;
		uint16_t *d = bm + (sy + j) * pitch + sx;
		for (int i = x0; i < x1; i++) {
			const uint8_t p = s[fx ? w - 1 - i : i];
			if (!((trans >> p) & 1))
				d[i] = (uint16_t)(base + p);
		}
	}
}

static void mappy_draw_sprites(const snap_t *sn, uint16_t *bm, int pitch)
{
	static const uint8_t gfx_offs[2][2] = { { 0, 1 }, { 2, 3 } };
	const uint8_t *s1 = sn->spr + 0x780, *s2 = s1 + 0x800, *s3 = s2 + 0x800;
	for (int offs = 0; offs < 0x80; offs += 2) {
		if (s3[offs + 1] & 2)   // (off)
			continue;
		int sprite = s1[offs];
		const int color = rbx_wrap(s1[offs + 1], s_spr_colors);
		const int sx = s2[offs + 1] + 0x100 * (s3[offs + 1] & 1) - 40;
		int sy = 256 - s2[offs] + 1;   // sprites are buffered and delayed by one scanline
		int flipx = s3[offs] & 1, flipy = (s3[offs] & 2) >> 1;
		const int sizex = (s3[offs] & 4) >> 2, sizey = (s3[offs] & 8) >> 3;
		sprite &= ~sizex;
		sprite &= ~(sizey << 1);
		sy -= 16 * sizey;
		sy = (sy & 0xff) - 32;   // fix wraparound
		if (sn->flip) {
			flipx ^= 1;
			flipy ^= 1;
		}
		for (int y = 0; y <= sizey; y++)
			for (int x = 0; x <= sizex; x++)
				blit(bm, pitch, (uint32_t)(sprite + gfx_offs[y ^ (sizey * flipy)][x ^ (sizex * flipx)]), color, flipx,
				     flipy, sx + 16 * x, sy + 16 * y);
	}
}

static void phozon_draw_sprites(const snap_t *sn, uint16_t *bm, int pitch)
{
	static const uint8_t size[4] = { 1, 0, 3, 0 };   // 16, 8, 32 pixels; fourth combination unused?
	static const uint8_t gfx_offs[4][4] = { { 0, 1, 4, 5 }, { 2, 3, 6, 7 }, { 8, 9, 12, 13 }, { 10, 11, 14, 15 } };
	const uint8_t *s1 = sn->spr + 0x780, *s2 = s1 + 0x800, *s3 = s2 + 0x800;
	for (int offs = 0; offs < 0x80; offs += 2) {
		if (s3[offs + 1] & 2)
			continue;
		const int sprite = (s1[offs] << 2) | ((s3[offs] & 0xc0) >> 6);
		const int color = s1[offs + 1] & 0x3f;
		const int sx = s2[offs + 1] + 0x100 * (s3[offs + 1] & 1) - 69;
		int sy = 256 - s2[offs];
		int flipx = s3[offs] & 1, flipy = (s3[offs] & 2) >> 1;
		const int sizex = size[(s3[offs] & 0x0c) >> 2], sizey = size[(s3[offs] & 0x30) >> 4];
		sy -= 8 * sizey;
		sy = (sy & 0xff) - 32;   // fix wraparound
		if (sn->flip) {
			flipx ^= 1;
			flipy ^= 1;
		}
		for (int y = 0; y <= sizey; y++)
			for (int x = 0; x <= sizex; x++)
				blit(bm, pitch, (uint32_t)(sprite + gfx_offs[y ^ (sizey * flipy)][x ^ (sizex * flipx)]), color, flipx,
				     flipy, sx + 8 * x, sy + 8 * y);
	}
}

static void mp_render(void)
{
	const snap_t *sn = s_snap;
	memcpy(rbx_video.palette, s_pal, (size_t)s_npens * sizeof(s_pal[0]));
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	draw_bg(sn, 0);
	if (s_vid == VID_SUPERPAC) {
		for (int i = 0; i < VIS_W * VIS_H; i++)
			s_sbm[i] = 15;
		mappy_draw_sprites(sn, s_sbm, VIS_W);
		for (int y = 0; y < VIS_H; y++) {   // copybitmap_trans(.., 15)
			const uint16_t *s = s_sbm + y * VIS_W;
			uint16_t *d = bm + y * pitch;
			for (int x = 0; x < VIS_W; x++)
				if (s[x] != 15)
					d[x] = s[x];
		}
		draw_bg(sn, 1);   // redraw the high priority characters
		for (int y = 0; y < VIS_H; y++) {   // sprite colour 0/1 still has priority over that (ghost eyes in Pac & Pal)
			const uint16_t *s = s_sbm + y * VIS_W;
			uint16_t *d = bm + y * pitch;
			for (int x = 0; x < VIS_W; x++)
				if (s_ind[s[x]] < 2)
					d[x] = s[x];
		}
	} else {
		if (s_vid == VID_PHOZON)
			phozon_draw_sprites(sn, bm, pitch);
		else
			mappy_draw_sprites(sn, bm, pitch);
		draw_bg(sn, 1);   // redraw the high priority characters
	}
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap->vram, b.vram, sizeof(b.vram));
	memcpy(s_snap->spr, b.spr, sizeof(b.spr));
	// (Phozon: the flip screen control is embedded in RAM)
	s_snap->flip = s_vid == VID_PHOZON ? (b.spr[0x1f7f - 0x800] & 1) : b.flip;
	s_snap->scroll = b.scroll;
	rbx_video_frame_done();
}

// ---- frame ----
// MAME's scheduling: timeslices of the 6 kHz quantum (256 clocks) from the last sync point, cut by the timers (the
// I/O chips' run 50 us after the vblank, the screen's line 0 and its vblank start); a synced input line change (the
// IRQ clears, the sub CPUs' RESET) ends the running CPU's timeslice where it happens, the later CPUs then run only up
// to that time, and the change applies at the end of the timeslice. The CPUs run in the order main, sub, sub2. A CPU
// held in reset eats its clocks. A frame here runs from the vblank's start to the next.
static void apply_pending(void)
{
	for (int i = 0; i < s_ncpu; i++) {
		if (s_pend_irq_clear[i]) {
			s_pend_irq_clear[i] = 0;
			m6809_set_irq(&s_cpu[i], 0);
		}
		for (int k = 0; k < s_pend_reset_n[i]; k++) {
			if (s_pend_reset[i][k])
				b.in_reset[i] = 1;
			else if (b.in_reset[i]) {   // (the line released: the device reset, then it runs again)
				const int64_t now = s_cpu[i].cycles;
				m6809_reset(&s_cpu[i]);
				s_cpu[i].cycles = now + 4;   // (the reset vector fetch takes 4 clocks)
				b.in_reset[i] = 0;
			}
		}
		s_pend_reset_n[i] = 0;
	}
}

static void timeslice(int64_t target)
{
	for (int i = 0; i < s_ncpu; i++) {
		m6809_t *c = &s_cpu[i];
		if (c->cycles >= target)
			continue;
		if (b.in_reset[i]) {
			c->cycles = target;
			continue;
		}
		s_sync = 0;
		while (c->cycles < target) {
			m6809_step(c);
			if (s_sync)
				break;
		}
		if (c->cycles < target)
			target = c->cycles > b.t ? c->cycles : b.t;
	}
	b.snd_frac += (uint64_t)(target - b.t);
	b.t = target;
	const int k = (int)(b.snd_frac / (CPU_CLOCK / SND_RATE));   // (48 kHz: a sample every 32 clocks)
	if (k > 0) {
		b.snd_frac -= (uint64_t)k * (CPU_CLOCK / SND_RATE);
		memset(s_mix, 0, (size_t)k * sizeof(s_mix[0]));
		namco_wsg_render(&s_wsg, s_mix, NULL, k, SND_RATE);
		for (int i = 0; i < k; i++)
			rbx_audio_put(s_mix[i]);
	}
	apply_pending();
}

static void mp_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	const int64_t start = b.t, end = b.t + FRAME_CYCLES;
	while (b.t < end) {
		int64_t target = b.t + QUANTUM;
		if (end < target)
			target = end;
		if (start + LINE0_AT > b.t && start + LINE0_AT < target)
			target = start + LINE0_AT;
		const int io_due = (b.io_armed[0] || b.io_armed[1]) && start + IO_DELAY > b.t;
		if (io_due && start + IO_DELAY < target)
			target = start + IO_DELAY;
		timeslice(target);
		if (io_due && b.t == start + IO_DELAY)   // namcoio_run_timer
			for (int k = 0; k < 2; k++)
				if (b.io_armed[k]) {
					b.io_armed[k] = 0;
					namcoio_run(&s_io[k]);
				}
	}
	// vblank: the frame drawn, then the IRQs and the I/O chips' timers
	vblank_capture();
	if (b.main_irq_mask)
		m6809_set_irq(&s_cpu[0], 1);
	for (int k = 0; k < 2; k++)
		if (!namcoio_read_reset_line(&s_io[k]))   // give the cpu a tiny bit of time to write the command
			b.io_armed[k] = 1;
	if (b.sub_irq_mask)
		m6809_set_irq(&s_cpu[1], 1);
	if (s_vid == VID_PHOZON && b.sub2_irq_mask)
		m6809_set_irq(&s_cpu[2], 1);
}

// ---- init / reset ----
static void mp_reset(void)
{
	b.latch = 0;
	b.main_irq_mask = b.sub_irq_mask = b.sub2_irq_mask = 0;
	b.flip = b.scroll = b.dipsel = 0;
	b.io_armed[0] = b.io_armed[1] = 0;
	b.t = 0;
	b.snd_frac = 0;
	s_sync = 0;
	memset(s_pend_irq_clear, 0, sizeof(s_pend_irq_clear));
	memset(s_pend_reset_n, 0, sizeof(s_pend_reset_n));
	for (int i = 0; i < s_ncpu; i++) {
		m6809_set_irq(&s_cpu[i], 0);
		m6809_reset(&s_cpu[i]);
		s_cpu[i].cycles = 4;   // (MAME: the reset vector fetch, INTERRUPT_VECTOR, takes 4 clocks)
		b.in_reset[i] = i > 0;   // (the latch cleared at reset: the sub CPUs held in reset)
	}
	namco_wsg_init(&s_wsg, NAMCO_WSG_TYPE_15XX, 18432000 / 768, 0, s_sndprom, 256);
	namco_wsg_sound_enable_w(&s_wsg, 0);
	for (int k = 0; k < 2; k++) {
		namcoio_set_reset_line(&s_io[k], 1);   // (the latch's Q4 cleared)
		namcoio_reset(&s_io[k]);
	}
}

static void mp_exit(void);

static void map_pages(m6809_t *c, int first, int n, uint8_t *mem)
{
	for (int p = 0; p < n; p++)
		c->rpage[first + p] = c->wpage[first + p] = mem + p * 256;
}

static void map_rom(m6809_t *c, int first, const uint8_t *rom)
{
	for (int p = first; p < 0x100; p++)
		c->rpage[p] = rom + p * 256;
}

static void map_15xx_ram(m6809_t *c, int first)   // (the 15XX's shared RAM past the register page)
{
	for (int p = 1; p < 4; p++)
		c->rpage[first + p] = c->wpage[first + p] = s_wsg.ram + p * 256;
}

// MAME compute_resistor_weights(0, 255, -1.0, ...): each resistor's output alone (no pulldown / pullup)
static double raw_weights(const int *res, int n, double *w)
{
	double sum = 0;
	for (int k = 0; k < n; k++) {
		double r0 = 1.0 / 1e12, r1 = 1.0 / 1e12;
		for (int j = 0; j < n; j++) {
			if (j == k)
				r1 += 1.0 / res[j];
			else
				r0 += 1.0 / res[j];
		}
		r0 = 1.0 / r0;
		r1 = 1.0 / r1;
		w[k] = 255.0 * r0 / (r1 + r0);
		sum += w[k];
	}
	return sum;
}

static int combine(const double *w, int n, int v)
{
	double s = 0;
	for (int k = 0; k < n; k++)
		s += w[k] * ((v >> k) & 1);
	return (int)(s + 0.5);
}

static void build_palette(void)
{
	uint32_t rgb[32];
	if (s_vid == VID_PHOZON) {
		static const int res[4] = { 2200, 1000, 470, 220 };
		double w[4];
		const double scale = 255.0 / raw_weights(res, 4, w);
		for (int k = 0; k < 4; k++)
			w[k] *= scale;
		for (int i = 0; i < 32; i++)
			rgb[i] = (uint32_t)combine(w, 4, s_proms[i]) << 16 | (uint32_t)combine(w, 4, s_proms[i | 0x100]) << 8 |
			         (uint32_t)combine(w, 4, s_proms[i | 0x200]);
	} else {
		static const int res[3] = { 1000, 470, 220 };
		double w3[3], w2[2];
		const double s3 = raw_weights(res, 3, w3), s2 = raw_weights(res + 1, 2, w2);
		const double scale = 255.0 / (s3 > s2 ? s3 : s2);
		for (int k = 0; k < 3; k++)
			w3[k] *= scale;
		for (int k = 0; k < 2; k++)
			w2[k] *= scale;
		for (int i = 0; i < 32; i++) {
			const uint8_t v = s_proms[i];
			rgb[i] = (uint32_t)combine(w3, 3, v) << 16 | (uint32_t)combine(w3, 3, v >> 3) << 8 | (uint32_t)combine(w2, 2, v >> 6);
		}
	}
	const uint8_t *lut = s_proms + (s_vid == VID_PHOZON ? 0x300 : 0x20);
	for (int i = 0; i < s_npens; i++) {
		const int ctab = lut[i] & 0x0f;
		int ind;
		if (i < 64 * 4)
			ind = s_vid == VID_SUPERPAC ? (ctab ^ 15) + 0x10 : s_vid == VID_MAPPY ? ctab + 0x10 : ctab;
		else
			ind = s_vid == VID_PHOZON ? ctab + 0x10 : ctab;
		s_ind[i] = (uint8_t)ind;
		s_pal[i] = rgb[ind];
	}
	// the tilemap's groups (configure_groups: 31, Phozon 15) and the sprites' transparent colour (15, Phozon 31)
	const int chr_tc = s_vid == VID_PHOZON ? 15 : 31, spr_tc = s_vid == VID_PHOZON ? 31 : 15;
	const int g = 1 << s_spr_bpp;
	for (int c = 0; c < 64; c++) {
		s_chr_trans[c] = 0;
		for (int p = 0; p < 4; p++)
			if (s_ind[c * 4 + p] == chr_tc)
				s_chr_trans[c] |= (uint8_t)(1 << p);
		s_spr_trans[c] = 0;
		if (c < s_spr_colors)
			for (int p = 0; p < g; p++)
				if (s_ind[256 + c * g + p] == spr_tc)
					s_spr_trans[c] |= (uint16_t)(1 << p);
	}
}

static int mp_init(const rbx_game *g)
{
	const char *m = g->machine;
	s_vid = !strcmp(m, "phozon") ? VID_PHOZON
	      : (!strcmp(m, "superpac") || !strcmp(m, "pacnpal") || !strcmp(m, "grobda")) ? VID_SUPERPAC : VID_MAPPY;
	s_ncpu = s_vid == VID_PHOZON ? 3 : 2;
	s_rom[0] = rbx_region("maincpu", &s_rom_size[0]);
	s_rom[1] = rbx_region("sub", &s_rom_size[1]);
	s_rom[2] = s_vid == VID_PHOZON ? rbx_region("sub2", &s_rom_size[2]) : NULL;
	s_tilerom = rbx_region("tiles", &s_tile_size);
	s_sprrom = rbx_region("sprites", &s_spr_size);
	s_proms = rbx_region("proms", &s_proms_size);
	s_sndprom = rbx_region("namco", &s_sndprom_size);
	const int todruaga = !strcmp(m, "todruaga");
	s_spr_w = s_vid == VID_PHOZON ? 8 : 16;
	s_spr_bpp = s_vid == VID_MAPPY ? 4 : 2;
	s_spr_colors = s_vid == VID_MAPPY && !todruaga ? 16 : 64;
	s_npens = 64 * 4 + s_spr_colors * (1 << s_spr_bpp);
	const uint32_t need_proms = s_vid == VID_PHOZON ? 0x500 : 0x20 + (uint32_t)s_npens;
	if (!s_rom[0] || s_rom_size[0] < 0x10000 || !s_rom[1] || s_rom_size[1] < 0x10000 ||
	    (s_vid == VID_PHOZON && (!s_rom[2] || s_rom_size[2] < 0x10000)) || !s_tilerom || s_tile_size < 16 || !s_sprrom ||
	    s_spr_size < 128 || !s_proms || s_proms_size < need_proms || !s_sndprom || s_sndprom_size < 0x100) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	s_chr_count = s_tile_size / 16;
	const uint32_t tile_bytes = (uint32_t)(s_spr_w * s_spr_w * s_spr_bpp / 8);
	s_spr_count = s_spr_size / tile_bytes;
	s_chrpix = (uint8_t *)malloc((size_t)s_chr_count * 64);
	s_sprpix = (uint8_t *)malloc((size_t)s_spr_count * (size_t)(s_spr_w * s_spr_w));
	s_snap = (snap_t *)calloc(1, sizeof(snap_t));
	s_sbm = (uint16_t *)calloc(1, VIS_W * VIS_H * sizeof(uint16_t));
	if (!s_chrpix || !s_sprpix || !s_snap || !s_sbm) {
		mp_exit();
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	{   // charlayout: 8x8, 2bpp, planes at bits 0 / 4 (MSB first), pixels 0-3 from byte 8, 4-7 from byte 0
		const uint8_t inv = s_vid == VID_MAPPY ? 0xff : 0x00;   // (ROMREGION_INVERT)
		for (uint32_t t = 0; t < s_chr_count; t++)
			for (int y = 0; y < 8; y++)
				for (int x = 0; x < 8; x++) {
					const uint8_t v = (uint8_t)(s_tilerom[t * 16 + (uint32_t)((x < 4 ? 8 : 0) + y)] ^ inv);
					const int k = x & 3;
					s_chrpix[t * 64 + (uint32_t)(y * 8 + x)] = (uint8_t)((((v >> (7 - k)) & 1) << 1) | ((v >> (3 - k)) & 1));
				}
	}
	for (uint32_t t = 0; t < s_spr_count; t++) {
		const int w = s_spr_w;
		uint8_t *d = s_sprpix + t * (uint32_t)(w * w);
		const uint8_t *src = s_sprrom + t * tile_bytes;
		for (int y = 0; y < w; y++)
			for (int x = 0; x < w; x++) {
				int p;
				if (s_spr_bpp == 4) {   // spritelayout_4bpp: planes at bits 0 / 4 / 8 / 12 of 16-bit rows
					const uint32_t bit = (uint32_t)((x >> 2) * 128 + (x & 3) + (y & 7) * 16 + (y >> 3) * 512);
					p = 0;
					for (int pl = 0; pl < 4; pl++) {
						const uint32_t o = bit + (uint32_t)pl * 4;
						p = (p << 1) | ((src[o >> 3] >> (7 - (o & 7))) & 1);
					}
				} else {   // spritelayout_2bpp / _8x8: 4-pixel columns of 8 bytes (16x16: rows 8-15 32 bytes on)
					const uint8_t v = src[(x >> 2) * 8 + (y & 7) + (y >> 3) * 32];
					const int k = x & 3;
					p = (((v >> (7 - k)) & 1) << 1) | ((v >> (3 - k)) & 1);
				}
				d[y * w + x] = (uint8_t)p;
			}
	}
	build_palette();
	s_port_p1 = rbx_port_find("P1");
	s_port_p2 = rbx_port_find("P2");
	s_port_buttons = rbx_port_find("BUTTONS");
	s_port_coins = rbx_port_find("COINS");
	s_port_dsw0 = rbx_port_find("DSW0");
	s_port_dsw1 = rbx_port_find("DSW1");
	s_port_dsw2 = rbx_port_find("DSW2");

	memset(&b, 0, sizeof(b));
	memset(&s_wsg, 0, sizeof(s_wsg));
	for (int i = 0; i < 3; i++)
		m6809_init(&s_cpu[i]);
	m6809_t *c = &s_cpu[0];
	c->read = main_r;
	c->write = main_w;
	if (s_vid == VID_MAPPY) {
		map_pages(c, 0x00, 0x10, b.vram);
		map_pages(c, 0x10, 0x18, b.spr);
		map_rom(c, 0x80, s_rom[0]);
	} else {
		map_pages(c, 0x00, 0x08, b.vram);
		map_pages(c, 0x08, 0x18, b.spr);
		map_rom(c, s_vid == VID_PHOZON ? 0x80 : 0xa0, s_rom[0]);
	}
	map_15xx_ram(c, 0x40);
	c = &s_cpu[1];
	c->read = sub_r;
	c->write = sub_w;
	map_15xx_ram(c, 0x00);
	map_rom(c, 0xe0, s_rom[1]);
	if (s_vid == VID_PHOZON) {
		c = &s_cpu[2];
		c->read = sub2_r;
		c->write = sub2_w;
		map_pages(c, 0x00, 0x08, b.vram);
		map_pages(c, 0x08, 0x18, b.spr);
		map_15xx_ram(c, 0x40);
		map_pages(c, 0xa0, 0x08, b.ram2);
		map_rom(c, 0xe0, s_rom[2]);
	}
	{   // the I/O chips of each machine
		int t0 = NAMCOIO_56XX, t1 = NAMCOIO_56XX;
		if (!strcmp(m, "pacnpal"))
			t1 = NAMCOIO_59XX;
		else if (!strcmp(m, "grobda") || !strcmp(m, "phozon") || !strcmp(m, "digdug2") || todruaga)
			t0 = NAMCOIO_58XX;
		else if (!strcmp(m, "mappy"))
			t0 = t1 = NAMCOIO_58XX;
		namcoio_init(&s_io[0], t0);
		namcoio_init(&s_io[1], t1);
		for (int k = 0; k < 2; k++) {
			s_io[k].in = io_in;
			s_io[k].out = io_out;
			s_io[k].ctx = (void *)(intptr_t)k;
		}
	}
	if (rbx_audio_setup(SND_RATE) || rbx_video_setup(VIS_W, VIS_H, s_npens)) {
		mp_exit();
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	mp_reset();
	return 0;
}

static void mp_exit(void)
{
	free(s_chrpix);
	free(s_sprpix);
	free(s_snap);
	free(s_sbm);
	s_chrpix = s_sprpix = NULL;
	s_snap = NULL;
	s_sbm = NULL;
}

static int mp_state(void *ctx, int load)
{
	(void)load;
	int rc = 0;
	rc |= rbx_state_io(ctx, "board", &b, sizeof(b));
	rc |= rbx_state_io(ctx, "cpu0", &s_cpu[0], M6809_STATE_SIZE);
	rc |= rbx_state_io(ctx, "cpu1", &s_cpu[1], M6809_STATE_SIZE);
	if (s_ncpu > 2)
		rc |= rbx_state_io(ctx, "cpu2", &s_cpu[2], M6809_STATE_SIZE);
	rc |= rbx_state_io(ctx, "io0", &s_io[0], (uint32_t)NAMCOIO_STATE_SIZE);
	rc |= rbx_state_io(ctx, "io1", &s_io[1], (uint32_t)NAMCOIO_STATE_SIZE);
	rc |= rbx_state_io(ctx, "wsg", &s_wsg, (uint32_t)NAMCO_WSG_STATE_SIZE);
	return rc;
}

static int mp_save(void *ctx) { return mp_state(ctx, 0); }
static int mp_load(void *ctx) { return mp_state(ctx, 1); }
static int mp_supported(const char *machine)
{
	static const char *const names[] = { "superpac", "pacnpal", "grobda", "phozon", "mappy", "digdug2", "todruaga", "motos" };
	for (size_t i = 0; i < sizeof(names) / sizeof(names[0]); i++)
		if (!strcmp(machine, names[i]))
			return 1;
	return 0;
}

const rbx_driver rbx_drv_mappy = { "Namco Mappy", mp_init, mp_exit, mp_reset, mp_frame, mp_save, mp_load,
                                   mp_supported, mp_render, NULL };
