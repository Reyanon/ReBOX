/*  Atari Toobin' hardware. Ported from MAME src/mame/atari/toobin.cpp (BSD-3-Clause, copyright-holders: Aaron Giles).
    68010 at 8 MHz; a 128x64 playfield of 8x8 tiles (4 priority categories), the Atari motion objects (atarimo.c,
    linked, one SLIP for the screen), a 64x48 alphanumerics layer; 1024 colours with a brightness register; a 2804
    parallel EEPROM. Sound: the JSA I without its TMS5220 (atarijsa.c). IRQ 1 at the line in interrupt_scan (a timer
    repeating every frame), IRQ 2 from the sound response; the interrupt mixer off (both lines: level 3).
    The screen is drawn in pieces up to the beam, as MAME's partial updates.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m68k_bus.h"
#include "../../cpu/m68k/m68k.h"
#include "../../video/gfxlayout.h"
#include "../video/atarimo.h"
#include "../machine/atarijsa.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAIN_CLOCK 8000000         // 32 MHz / 4
#define LINE_CYC 320               // set_raw(32 MHz / 2, 640, 0, 512, 416, 0, 384): 640 pixels at 2 per clock
#define HBLANK_CYC 256
#define LINES 416
#define VIS_W 512
#define VIS_H 384
#define FRAME_MHZ 60096
#define SLICES 4

static uint8_t *s_rom;
static uint32_t s_rom_size;
static rbx_gfx s_gfx_pf, s_gfx_mo, s_gfx_an;
static amo_t s_mo;

// main memory (host-order words)
static uint16_t s_pf[0x4000];        // c00000-c07fff: 128x64 entries of 2 words
static uint16_t s_vram[0x1000];      // c08000-c09fff: alpha (0xc00 words), MO at 0x1800
static uint16_t s_pal[0x400];        // c10000-c107ff
static uint16_t s_wram[0x2000];      // 82c000-82ffff
static uint8_t s_eeprom[0x200];
static int s_eep_unlocked;
static char s_nv_path[512];
static int s_port[3];
static uint16_t s_xscroll, s_yscroll, s_slip, s_intscan, s_intensity;
static int s_ipl;                    // the IPL0 (scanline) / IPL1 (sound) lines
static int64_t s_abs_line, s_irq_at; // lines since the reset; the scanline timer's line (-1 = not armed)
static int s_line, s_slice_pos, s_last_drawn;
static int64_t s_mdebt, s_sndacc;


// video: drawn by tb_render (the render thread with one), from a journal (ADR-061). The playfield and video RAM writes
// go through the handler, which logs each word written (index into pf + vram, value) to the journal of the frame being
// emulated; update_partial logs a band (lines, scroll, SLIP, MO bank) with the journal's length at that time. The
// drawing side keeps its own copy of that RAM (as it was at the last frame's vblank): for each band it applies the
// journal up to the band's position and draws the lines, as update_partial did on the emulation thread. A full
// journal is drawn up to there by the emulation thread (after waiting for the render), then started again.
// RBX_TB_NODEFER=1: every band drawn at once on the emulation thread (the same way, as before)
#define TB_RAMW 0x5000               // pf (0x4000 words) + vram (0x1000)
typedef struct { int16_t y0, y1; uint16_t xscroll, yscroll, slip; uint8_t bank; uint32_t jpos; } tb_band;
typedef struct {
	uint32_t *j;                     // (word index << 16 | value)
	uint32_t jn;
	int nb;
	tb_band b[VIS_H];
	uint16_t pal[0x400];
	uint16_t intensity;
	int pending;                     // posted, not drawn yet
} tb_slot;
static struct {
	uint16_t ram[TB_RAMW];           // the drawing side's pf + vram
	tb_slot s[2];
	amo_t mo;                        // (the drawing side's: its active list)
	uint32_t pal[0x400];             // (the palette of the last frame drawn)
} *s_dr;
static int s_cur;                    // the slot the emulation fills
static int s_rslot;                  // the slot posted for drawing
static uint32_t s_jcap;
static int s_nodefer;
static uint16_t *s_frame;            // VIS_W x VIS_H (the drawing side's)
static uint16_t *s_mobm;             // the MO bitmap
static uint8_t *s_pri;               // the playfield's categories

typedef uint8_t v8u8 __attribute__((vector_size(8)));
typedef uint16_t v8u16 __attribute__((vector_size(16)));

static int line_pos(void) { return s_slice_pos + m68k_cycles_run(); }

static void set_ipl(void)   // set_interrupt_mixer(false): the lines are the bits of the level
{
	for (int l = 1; l <= 3; l++)
		m68k_bus_set_line(l, l == s_ipl);
}

static void draw_lines(const tb_band *bd)   // screen_update on [y0, y1], from the drawing side's RAM
{
	const int y0 = bd->y0, y1 = bd->y1;
	const uint16_t *const pfram = s_dr->ram, *const vram = s_dr->ram + 0x4000;
	const int scrollx = bd->xscroll >> 6, scrolly = bd->yscroll >> 6;
	const uint32_t pfcount = s_gfx_pf.count;
	// (a tile row's 128 tiles decoded once per row: the RAM does not change within a call)
	int prow_n = -1;
	const uint8_t *tbase[128];
	uint16_t tlo[128], tcolor[128];
	uint8_t tcat[128];
	for (int y = y0; y <= y1; y++) {   // the playfield (128x64 tiles, by rows; the category to the priority bitmap)
		uint16_t *d = s_frame + y * VIS_W;
		uint8_t *pr = s_pri + y * VIS_W;
		const int ty = (y + scrolly) & 511;
		const int row = ty >> 3;
		if (row != prow_n) {
			prow_n = row;
			const uint16_t *prow = &pfram[row * 128 * 2];
			for (int c = 0; c < 128; c++) {
				const uint16_t hi = prow[c * 2], lo = prow[c * 2 + 1];
				tbase[c] = s_gfx_pf.pix + rbx_wrap(lo & 0x3fff, pfcount) * 64;
				tlo[c] = lo;
				tcolor[c] = (uint16_t)((hi & 0x0f) * 16);
				tcat[c] = (uint8_t)((hi >> 4) & 3);
			}
		}
		int x = 0, tx = scrollx & 1023;
		while (x < VIS_W) {
			const int c = tx >> 3;
			const uint16_t lo = tlo[c];
			const uint16_t color = tcolor[c];
			const uint8_t cat = tcat[c];
			const int py = (lo & 0x8000) ? 7 - (ty & 7) : ty & 7;
			const uint8_t *src = tbase[c] + py * 8;
			const int px0 = tx & 7;
			if (px0 == 0 && x + 8 <= VIS_W) {   // a whole tile (all but the edges), 8 pixels at once
				const uint64_t cat8 = cat * 0x0101010101010101ull;
				v8u8 s8;
				memcpy(&s8, src, 8);
				if (lo & 0x4000)
					s8 = __builtin_shuffle(s8, (v8u8){ 7, 6, 5, 4, 3, 2, 1, 0 });
				const v8u16 v = __builtin_convertvector(s8, v8u16) + color;
				memcpy(d + x, &v, 16);
				memcpy(pr + x, &cat8, 8);
				x += 8;
				tx = (tx + 8) & 1023;
				continue;
			}
			const int flipx = (lo >> 14) & 1;
			for (int px = px0; px < 8 && x < VIS_W; px++, x++, tx++) {
				d[x] = (uint16_t)(color + src[flipx ? 7 - px : px]);
				pr[x] = cat;
			}
			tx &= 1023;
		}
	}
	// the motion objects, merged: not over the high priority playfield pens (8-15) of categories 1-3
	amo_set_xscroll(&s_dr->mo, bd->xscroll >> 6);
	amo_set_yscroll(&s_dr->mo, (bd->yscroll >> 6) & 0x1ff);
	s_dr->mo.bank = bd->bank;
	amo_draw(&s_dr->mo, vram + 0x1800 / 2, &bd->slip, s_mobm, VIS_W, 0, VIS_W - 1, y0, y1);
	for (int y = y0; y <= y1; y++) {
		const uint16_t *mo = s_mobm + y * VIS_W;
		const uint8_t *pr = s_pri + y * VIS_W;
		uint16_t *pf = s_frame + y * VIS_W;
		for (int x0 = 0; x0 < VIS_W; x0 += 8) {   // (8 pixels at a time: most hold no MO pen)
			uint64_t q[2];
			memcpy(q, mo + x0, 16);
			if ((q[0] & q[1]) == ~(uint64_t)0)
				continue;
			for (int x = x0; x < x0 + 8; x++)
				if (mo[x] != 0xffff && (!pr[x] || !(pf[x] & 8)))
					pf[x] = mo[x];
		}
	}
	// the alphanumerics (64x48, by rows; pen 0 transparent)
	// (the row's non-empty tiles listed once per row: the RAM does not change within a call)
	int arow = -1, an = 0;
	uint8_t acol[VIS_W / 8];
	uint16_t adata[VIS_W / 8];
	uint32_t acode[VIS_W / 8];
	for (int y = y0; y <= y1; y++) {
		uint16_t *d = s_frame + y * VIS_W;
		const int row = y >> 3, py = y & 7;
		if (row != arow) {
			arow = row;
			an = 0;
			for (int col = 0; col < VIS_W / 8; col++) {
				const uint16_t data = vram[row * 64 + col];
				const uint32_t code = rbx_wrap(data & 0x3ff, s_gfx_an.count);
				if (!s_gfx_an.used[code])
					continue;
				acol[an] = (uint8_t)col;
				adata[an] = data;
				acode[an++] = code;
			}
		}
		for (int t = 0; t < an; t++) {
			const uint16_t data = adata[t];
			const uint16_t color = (uint16_t)(512 + ((data >> 12) & 0x0f) * 4);
			const uint8_t *src = s_gfx_an.pix + acode[t] * 64 + py * 8;
			uint16_t *dd = d + acol[t] * 8;
			if (data & 0x400) {
				for (int k = 0; k < 8; k++)
					if (src[7 - k])
						dd[k] = (uint16_t)(color + src[7 - k]);
			} else
				for (int k = 0; k < 8; k++)
					if (src[k])
						dd[k] = (uint16_t)(color + src[k]);
		}
	}
}

// the slot's bands drawn (each after the journal up to its position), then the rest of the journal applied: the
// drawing side's RAM is the emulation's of the slot's end
static void draw_slot(tb_slot *sl)
{
	uint16_t *const ram = s_dr->ram;
	uint32_t p = 0;
	for (int i = 0; i < sl->nb; i++) {
		const tb_band *b = &sl->b[i];
		for (; p < b->jpos; p++)
			ram[sl->j[p] >> 16] = (uint16_t)sl->j[p];
		draw_lines(b);
	}
	for (; p < sl->jn; p++)
		ram[sl->j[p] >> 16] = (uint16_t)sl->j[p];
	sl->nb = 0;
	sl->jn = 0;
}

// (the emulation thread) the slot being filled drawn now: the journal full, or RBX_TB_NODEFER
static void flush_bands(void)
{
	rbx_video_wait_idle();   // (the drawing side's RAM and bitmaps free: the last frame drawn)
	draw_slot(&s_dr->s[s_cur]);
}

static inline void jlog(uint32_t w, uint16_t v)   // a pf / vram word written (w: index into pf + vram)
{
	tb_slot *sl = &s_dr->s[s_cur];
	if (sl->jn == s_jcap)
		flush_bands();
	sl->j[sl->jn++] = w << 16 | v;
}

// the drawing side's RAM set to the emulation's, nothing pending (reset, state load: the render is idle)
static void draw_sync(void)
{
	memcpy(s_dr->ram, s_pf, sizeof(s_pf));
	memcpy(s_dr->ram + 0x4000, s_vram, sizeof(s_vram));
	for (int i = 0; i < 2; i++) {
		s_dr->s[i].nb = 0;
		s_dr->s[i].jn = 0;
		s_dr->s[i].pending = 0;
	}
}

static void update_partial(int vpos)   // screen().update_partial(vpos)
{
	if (vpos > VIS_H - 1)
		vpos = VIS_H - 1;
	if (vpos <= s_last_drawn)
		return;
	tb_slot *sl = &s_dr->s[s_cur];
	tb_band *b = &sl->b[sl->nb++];   // (at most VIS_H: each band one line at least)
	b->y0 = (int16_t)(s_last_drawn + 1);
	b->y1 = (int16_t)vpos;
	b->xscroll = s_xscroll;
	b->yscroll = s_yscroll;
	b->slip = s_slip;
	b->bank = (uint8_t)s_mo.bank;
	b->jpos = sl->jn;
	s_last_drawn = vpos;
	if (s_nodefer)
		flush_bands();
}

static inline uint16_t combine(uint16_t old, uint16_t v, uint16_t mask) { return (uint16_t)((old & ~mask) | (v & mask)); }

// ---- main bus ----
static void main_irq2(int state)
{
	s_ipl = state ? s_ipl | 2 : s_ipl & ~2;
	set_ipl();
}

static uint16_t io_read(uint32_t r)   // 0x82xxxx (mirror 0x450000)
{
	if (r >= 0xc000)
		return s_wram[(r & 0x3fff) >> 1];
	if (r >= 0xa000)
		return (uint16_t)(0xff00 | s_eeprom[(r & 0x3ff) >> 1]);
	if (r >= 0x9800)
		return (uint16_t)(0xff00 | jsa_main_response_r());
	if (r >= 0x9000) {
		uint16_t v = (uint16_t)(rbx_port_read(s_port[1]) & ~0xe000u);   // (the custom bits active low)
		if (line_pos() < HBLANK_CYC) v |= 0x2000;   // HBLANK
		if (s_line < VIS_H) v |= 0x4000;            // VBLANK
		if (!jsa_main_to_sound_ready()) v |= 0x8000;
		return v;
	}
	if (r >= 0x8800)
		return (uint16_t)rbx_port_read(s_port[0]);
	if ((r & 0xffc0) == 0x8380)
		return s_slip;
	return 0;
}

static uint16_t main_read16(uint32_t a)
{
	a &= 0xc7ffff;   // (global_mask)
	if (!(a & 0x800000))
		return a < 0x80000 ? *(const uint16_t *)(s_rom + a) : 0;
	if ((a & 0x420000) == 0x400000) {   // c00000-c1ffff (mirror 0x40000 for the alpha, MO and palette)
		const uint32_t o = a & 0x1ffff;
		if (o < 0x8000) return (a & 0x40000) ? 0 : s_pf[o >> 1];
		if (o < 0x10000) return s_vram[(o & 0x1fff) >> 1];
		return s_pal[(o & 0x7ff) >> 1];
	}
	if ((a & 0x820000) == 0x820000)
		return io_read(a & 0xffff);
	return 0;
}

static uint8_t main_read8(uint32_t a)
{
	const uint16_t v = main_read16(a & ~1u);
	return (uint8_t)(a & 1 ? v : v >> 8);
}

static void intscan_w(uint16_t v, uint16_t mask)
{
	const uint16_t n = combine(s_intscan, v, mask);
	if (n == s_intscan)
		return;
	s_intscan = n;
	// the timer at time_until_pos(n & 0x1ff): a line not after the beam is the next frame's
	const int vpos = n & 0x1ff;
	int64_t at = s_abs_line - s_line + vpos;
	if (vpos * LINE_CYC <= s_line * LINE_CYC + line_pos())
		at += LINES;
	s_irq_at = at;
}

static void io_write(uint32_t r, uint16_t v, uint16_t mask)
{
	if (r >= 0xc000) {
		uint16_t *p = &s_wram[(r & 0x3fff) >> 1];
		*p = combine(*p, v, mask);
		return;
	}
	if (r >= 0xa000) {   // the EEPROM (mirror 0x1c00)
		if ((mask & 0x00ff) && s_eep_unlocked) {
			s_eeprom[(r & 0x3ff) >> 1] = (uint8_t)v;
			s_eep_unlocked = 0;   // (lock_after_write)
		}
		return;
	}
	if (r >= 0x8800)
		return;
	switch (r & 0xff00) {
	case 0x8000: break;   // the watchdog
	case 0x8100:
		if (mask & 0x00ff) {
			jsa_main_command_w((uint8_t)v);
			m68k_end_timeslice();
		}
		break;
	case 0x8300:
		switch (r & 0xc0) {
		case 0x00:   // intensity_w
			if (mask & 0x00ff)
				s_intensity = (uint16_t)(v & 0x1f);
			break;
		case 0x40: intscan_w(v, mask); break;
		case 0x80: {   // slip_w
			const uint16_t n = combine(s_slip, v, mask);
			if (n != s_slip) {
				update_partial(s_line);
				s_slip = n;
			}
			break;
		}
		case 0xc0:   // scanline_int_ack_w
			s_ipl &= ~1;
			set_ipl();
			break;
		}
		break;
	case 0x8400: jsa_sound_reset_w(); m68k_end_timeslice(); break;
	case 0x8500: s_eep_unlocked = 1; break;
	case 0x8600: {   // xscroll_w
		const uint16_t n = combine(s_xscroll, v, mask);
		if (n != s_xscroll)
			update_partial(s_line);
		s_xscroll = n;
		amo_set_xscroll(&s_mo, n >> 6);
		break;
	}
	case 0x8700: {   // yscroll_w
		const uint16_t n = combine(s_yscroll, v, mask);
		if (n != s_yscroll)
			update_partial(s_line);
		s_yscroll = n;
		amo_set_yscroll(&s_mo, (n >> 6) & 0x1ff);
		break;
	}
	default: break;
	}
}

static void main_write16m(uint32_t a, uint16_t v, uint16_t mask)
{
	a &= 0xc7ffff;
	if (!(a & 0x800000))
		return;
	if ((a & 0x420000) == 0x400000) {
		const uint32_t o = a & 0x1ffff;
		if (o < 0x10000) {   // (the playfield and video RAM: logged for the drawing side)
			uint32_t w;
			if (o < 0x8000) {
				if (a & 0x40000)
					return;
				w = o >> 1;
			} else
				w = 0x4000 + ((o & 0x1fff) >> 1);
			uint16_t *p = w < 0x4000 ? &s_pf[w] : &s_vram[w - 0x4000];
			const uint16_t n = combine(*p, v, mask);
			*p = n;
			jlog(w, n);
			return;
		}
		uint16_t *p = &s_pal[(o & 0x7ff) >> 1];
		*p = combine(*p, v, mask);
		return;
	}
	if ((a & 0x820000) == 0x820000)
		io_write(a & 0xffff, v, mask);
}

static void main_write16(uint32_t a, uint16_t v) { main_write16m(a, v, 0xffff); }
static void main_write8(uint32_t a, uint8_t v)
{
	main_write16m(a & ~1u, (uint16_t)(v << 8 | v), a & 1 ? 0x00ff : 0xff00);
}

static void map_main(void)
{
	m68k_bus_reset_map();
	m68k_bus_map(0x000000, 0x07ffff, s_rom, 0x80000, 1, 0);
	m68k_bus_map(0xc00000, 0xc07fff, s_pf, sizeof(s_pf), 1, 0);   // (writes logged: main_write16m)
	m68k_bus_map(0xc08000, 0xc09fff, s_vram, sizeof(s_vram), 1, 0);
	m68k_bus_map(0xc10000, 0xc107ff, s_pal, sizeof(s_pal), 1, 1);
	m68k_bus_map(0x82c000, 0x82ffff, s_wram, sizeof(s_wram), 1, 1);
	m68k_bus.read8 = main_read8;
	m68k_bus.read16 = main_read16;
	m68k_bus.write8 = main_write8;
	m68k_bus.write16 = main_write16;
}

// ---- frame ----
static uint32_t pen_rgb(uint16_t w, float contrast)   // paletteram_w, with the pen's contrast (set_pen_contrast)
{
	int c[3] = { (w >> 10) & 31, (w >> 5) & 31, w & 31 };
	for (int i = 0; i < 3; i++) {
		c[i] = (c[i] * 224) >> 5;
		if (c[i])
			c[i] += 38;
		if (contrast != 1.0f) {   // (palette_t::adjust_palette_entry: the gamma map is the identity)
			int v = (int)((float)c[i] * contrast);
			c[i] = v < 0 ? 0 : v > 255 ? 255 : v;
		}
	}
	return (uint32_t)(c[0] << 16 | c[1] << 8 | c[2]);
}

static void capture(void)
{
	rbx_video_wait_idle();
	tb_slot *sl = &s_dr->s[s_cur];
	memcpy(sl->pal, s_pal, sizeof(s_pal));
	sl->intensity = s_intensity;
	sl->pending = 1;
	s_rslot = s_cur;
	s_cur ^= 1;   // (the other slot drawn and empty: the render is idle)
	rbx_video_frame_done();
}

static void tb_render(void)
{
	tb_slot *sl = &s_dr->s[s_rslot];
	if (sl->pending) {   // (drawn once)
		sl->pending = 0;
		draw_slot(sl);
		// intensity_w: m_brightness = (~data & 0x1f) / 31 for the pens with bit 15 clear (0 before the first write)
		const float bright = (float)((double)(~sl->intensity & 0x1f) / 31.0);
		for (int i = 0; i < 0x400; i++)
			s_dr->pal[i] = pen_rgb(sl->pal[i], (sl->pal[i] & 0x8000) ? 1.0f : bright);
	}
	for (int y = 0; y < VIS_H; y++)
		memcpy(rbx_video.bitmap + y * rbx_video.pitch, s_frame + y * VIS_W, VIS_W * 2);
	memcpy(rbx_video.palette, s_dr->pal, sizeof(s_dr->pal));
}

static void tb_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	jsa_set_test((rbx_port_read(s_port[1]) >> 12) & 1);
	jsa_frame_begin();
	s_last_drawn = -1;
	for (int line = 0; line < LINES; line++, s_abs_line++) {
		s_line = line;
		s_slice_pos = 0;
		if (s_abs_line == s_irq_at) {   // the scanline timer
			s_ipl |= 1;
			set_ipl();
			s_irq_at += LINES;
		}
		if (line && !(line & 63) && line < VIS_H - 1)   // the MO force_update timer (every 64 lines)
			update_partial(line - 1);
		if (line == VIS_H) {   // VIDEO_UPDATE_BEFORE_VBLANK
			update_partial(VIS_H - 1);
			capture();
		}
		// the CPUs in slices; after a sound command (the comm's quantum of 5 us) in steps of 40 clocks
		for (int left = LINE_CYC; left > 0;) {
			int step = jsa_boosting() ? 40 : LINE_CYC / SLICES;
			if (step > left)
				step = left;
			s_slice_pos = LINE_CYC - left - (int)s_mdebt;   // (the 68010 is ahead of the slice by its overrun)
			left -= step;
			s_mdebt += step;
			if (s_mdebt > 0)
				s_mdebt -= m68k_execute((int)s_mdebt);
			s_sndacc += (int64_t)step * (JSA_CPU_CLOCK * 2 + 1);   // (the 6502 at 3.579545 MHz / 2)
			const int sc = (int)(s_sndacc / (MAIN_CLOCK * 2));
			s_sndacc -= (int64_t)sc * (MAIN_CLOCK * 2);
			jsa_run(sc);
		}
		jsa_line();
	}
	jsa_frame_end();
}

// ---- init / reset ----
static void tb_reset(void)
{
	memset(s_pf, 0, sizeof(s_pf));
	memset(s_vram, 0, sizeof(s_vram));
	memset(s_pal, 0, sizeof(s_pal));
	memset(s_wram, 0, sizeof(s_wram));
	s_xscroll = s_yscroll = s_slip = s_intscan = 0;
	s_intensity = 0x1f;
	s_mo.bank = 0;
	amo_set_xscroll(&s_mo, 0);
	amo_set_yscroll(&s_mo, 0);
	s_eep_unlocked = 0;
	s_ipl = 0;
	s_abs_line = 0;
	s_irq_at = -1;
	m68k_bus_irq_reset();
	jsa_reset();
	s_mdebt = 0;
	s_sndacc = 0;
	s_line = s_slice_pos = 0;
	s_last_drawn = -1;
	draw_sync();
	s_cur = s_rslot = 0;
	m68k_pulse_reset();
}

static void tb_exit(void);

static int tb_init(const rbx_game *g)
{
	uint32_t ts, ss, cs, js;
	s_rom = rbx_region("maincpu", &s_rom_size);
	uint8_t *tiles = rbx_region("tiles", &ts), *spr = rbx_region("sprites", &ss), *chars = rbx_region("chars", &cs);
	uint8_t *jrom = rbx_region("jsa:cpu", &js);
	if (!s_rom || s_rom_size < 0x80000 || !tiles || !spr || !chars || !jrom || js < 0x10000) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	m68k_bus_swap16(s_rom, s_rom_size);
	{   // pflayout, molayout (RGN_FRAC(1,2)), anlayout
		const uint32_t th = ts / 2 * 8, sh = ss / 2 * 8;
		const uint32_t pp[4] = { th, th + 4, 0, 4 }, mp[4] = { sh, sh + 4, 0, 4 };
		static const uint32_t px[8] = { 0, 1, 2, 3, 8, 9, 10, 11 };
		static const uint32_t py[8] = { 0, 16, 32, 48, 64, 80, 96, 112 };
		static const uint32_t mx[16] = { 0, 1, 2, 3, 8, 9, 10, 11, 16, 17, 18, 19, 24, 25, 26, 27 };
		static const uint32_t my[16] = { 0, 32, 64, 96, 128, 160, 192, 224, 256, 288, 320, 352, 384, 416, 448, 480 };
		static const uint32_t ap[2] = { 0, 4 };
		const rbx_gfx_layout lp = { 8, 8, 4, pp, px, py, 128 };
		const rbx_gfx_layout lm = { 16, 16, 4, mp, mx, my, 512 };
		const rbx_gfx_layout la = { 8, 8, 2, ap, px, py, 128 };
		if (rbx_gfx_decode(&s_gfx_pf, &lp, tiles, ts / 2) || rbx_gfx_decode(&s_gfx_mo, &lm, spr, ss / 2) ||
		    rbx_gfx_decode(&s_gfx_an, &la, chars, cs)) {
			rbx_fail("%s: out of memory (graphics)", g->set);
			tb_exit();
			return -1;
		}
	}
	static const amo_config moc = {
		1, 1, 0, 0, 1, 0,      // banks, linked, split, reverse, swapxy, nextneighbor
		1024, 0, 0, 0x100, 0,  // slipheight, slipoffset, maxperline, palettebase, transpen
		{ 0, 0, 0x00ff, 0 }, { 0, 0x3fff, 0, 0 }, { 0, 0, 0, 0x000f }, { 0, 0, 0, 0xffc0 }, { 0x7fc0, 0, 0, 0 },
		{ 0x0007, 0, 0, 0 }, { 0x0038, 0, 0, 0 }, { 0, 0x4000, 0, 0 }, { 0, 0x8000, 0, 0 }, { 0 }, { 0 },
		{ 0x8000, 0, 0, 0 }, { 0 }, 0
	};
	if (amo_init(&s_mo, &moc, 16, 16)) {
		rbx_fail("%s: out of memory", g->set);
		tb_exit();
		return -1;
	}
	s_mo.gfx[0] = &s_gfx_mo;
	s_mo.granularity[0] = 16;
	s_frame = (uint16_t *)calloc(VIS_W * VIS_H, sizeof(uint16_t));
	s_mobm = (uint16_t *)calloc(VIS_W * VIS_H, sizeof(uint16_t));
	s_pri = (uint8_t *)calloc(VIS_W * VIS_H, 1);
	// (RBX_TB_JCAP: the journal's size in words; small to test the full journal's path)
	s_jcap = getenv("RBX_TB_JCAP") ? (uint32_t)atoi(getenv("RBX_TB_JCAP")) : 0x8000;
	if (s_jcap < 1)
		s_jcap = 1;
	s_nodefer = getenv("RBX_TB_NODEFER") != NULL;
	s_dr = calloc(1, sizeof(*s_dr));
	if (s_dr) {
		s_dr->mo = s_mo;   // (the lookups shared, read only)
		for (int i = 0; i < 2; i++)
			s_dr->s[i].j = (uint32_t *)malloc(s_jcap * sizeof(uint32_t));
	}
	static const char *const names[3] = { "FF8800", "FF9000", "JSA" };
	for (int i = 0; i < 3; i++)
		s_port[i] = rbx_port_find(names[i]);
	jsa_config jc;
	memset(&jc, 0, sizeof(jc));
	jc.type = 1;
	jc.pokey = 1;
	jc.out = JSA_OUT_STEREO;
	jc.gain = 256;
	jc.rom = jrom;
	jc.port = s_port[2];
	jc.main_int = main_irq2;
	if (!s_frame || !s_mobm || !s_pri || !s_dr || !s_dr->s[0].j || !s_dr->s[1].j || jsa_init(&jc) || rbx_video_setup(VIS_W, VIS_H, 0x400)) {
		rbx_fail("%s: out of memory", g->set);
		tb_exit();
		return -1;
	}
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
	tb_reset();
	return 0;
}

static void tb_exit(void)
{
	if (s_nv_path[0]) {
		FILE *f = fopen(s_nv_path, "wb");
		if (f) {
			fwrite(s_eeprom, 1, sizeof(s_eeprom), f);
			fclose(f);
		}
		s_nv_path[0] = 0;
	}
	m68k_set_mame_timing(0);
	jsa_exit();
	amo_exit(&s_mo);
	rbx_gfx_free(&s_gfx_pf);
	rbx_gfx_free(&s_gfx_mo);
	rbx_gfx_free(&s_gfx_an);
	free(s_frame);
	free(s_mobm);
	free(s_pri);
	if (s_dr) {
		free(s_dr->s[0].j);
		free(s_dr->s[1].j);
	}
	free(s_dr);
	s_frame = s_mobm = NULL;
	s_pri = NULL;
	s_dr = NULL;
	s_cur = s_rslot = 0;
}

// ---- save state ----
static int tb_state(void *ctx, int load)
{
	const uint32_t csize = m68k_context_size();
	uint8_t *cpu = (uint8_t *)malloc(csize);
	if (!cpu)
		return -1;
	if (!load)
		m68k_get_context(cpu);
	uint32_t irq[2] = { m68k_bus_irq_pending(), m68k_bus_cur->irq_asserted };
	int64_t r64[4] = { s_abs_line, s_irq_at, s_mdebt, s_sndacc };
	uint16_t r16[8] = { s_xscroll, s_yscroll, s_slip, s_intscan, s_intensity, (uint16_t)s_ipl, (uint16_t)s_eep_unlocked, 0 };
	int rc = 0;
	rc |= rbx_state_io(ctx, "m68k", cpu, csize);
	rc |= rbx_state_io(ctx, "irq", irq, sizeof(irq));
	rc |= rbx_state_io(ctx, "pf", s_pf, sizeof(s_pf));
	rc |= rbx_state_io(ctx, "vram", s_vram, sizeof(s_vram));
	rc |= rbx_state_io(ctx, "pal", s_pal, sizeof(s_pal));
	rc |= rbx_state_io(ctx, "wram", s_wram, sizeof(s_wram));
	rc |= rbx_state_io(ctx, "eeprom", s_eeprom, sizeof(s_eeprom));
	rc |= rbx_state_io(ctx, "r64", r64, sizeof(r64));
	rc |= rbx_state_io(ctx, "r16", r16, sizeof(r16));
	rc |= jsa_state(ctx, load);
	if (!rc && load) {
		m68k_set_context(cpu);
		m68k_bus_set_irq_pending(irq[0]);
		m68k_bus_cur->irq_asserted = irq[1];
		m68k_bus_update_irq();
		s_abs_line = r64[0];
		s_irq_at = r64[1];
		s_mdebt = r64[2];
		s_sndacc = r64[3];
		s_xscroll = r16[0];
		s_yscroll = r16[1];
		s_slip = r16[2];
		s_intscan = r16[3];
		s_intensity = r16[4];
		s_ipl = r16[5];
		s_eep_unlocked = r16[6];
		amo_set_xscroll(&s_mo, s_xscroll >> 6);
		amo_set_yscroll(&s_mo, (s_yscroll >> 6) & 0x1ff);
		draw_sync();
	}
	free(cpu);
	return rc ? -1 : 0;
}

static int tb_save(void *ctx) { return tb_state(ctx, 0); }
static int tb_load(void *ctx) { return tb_state(ctx, 1); }
static int tb_supported(const char *machine) { return !strcmp(machine, "toobin"); }

const rbx_driver rbx_drv_toobin = { "Atari Toobin'", tb_init, tb_exit, tb_reset, tb_frame, tb_save, tb_load,
                                    tb_supported, tb_render, NULL };
