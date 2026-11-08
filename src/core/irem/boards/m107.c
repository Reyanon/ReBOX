/*  Irem M107. Ported from MAME src/mame/irem/m107.cpp (BSD-3-Clause, copyright-holders: Bryan McPhail); the scroll
    arithmetic of src/emu/tilemap.cpp (BSD-3-Clause, copyright-holders: Nicola Salmoria, Aaron Giles).
    Close to the M92: a V33, a uPD71059C, four 64x64 tile layers, sprites through a ROM table on Fire Barrel, and the sound
    board of irem_snd.c (here in stereo). The layer registers per raster segment as on the M92 (m92.c).
    Not done: the EEPROM of World PK Soccer (MACHINE_NOT_WORKING in MAME, not mapped there).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/nec/nec.h"
#include "../../cpu/nec/irem_cpu.h"
#include "../machine/irem_snd.h"
#include "../machine/pic8259.h"
#include "../video/irem_gfx.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define LINES 256
#define VIS_X0 80
#define VIS_Y0 8
#define VIS_W 320
#define VIS_H 240
#define VBLANK_LINE 248
#define FRAME_MHZ 60000u
#define NPENS 0x800
#define MAX_SEG 244

static nec_t s_cpu;
static pic8259_t s_pic;
static uint8_t *s_rom, *s_sprtable;
static uint32_t s_rom_size, s_sprtable_size, s_clock;
static rbx_gfx s_chr, s_spr;
static uint8_t *s_chr_rows, *s_pri;
static int s_banked, s_wpk, s_spritesystem;
static int s_port_p12, s_port_coins, s_port_dsw, s_port_p34, s_port_wpk[5];
static int s_vbl_cycles;   // (the main CPU's cycles of the 2500 us vblank)

static struct {
	uint16_t ram[0x8000], vram[0x8000];
	uint16_t spriteram[0x800], spritebuf[0x800], palette[0x800];
	uint16_t control[0x10];
	int32_t raster;
	uint8_t bank, sprite_display, pad[2];
	int64_t cpu_end, vbl_at;
	uint64_t cyc_frac;
	uint32_t frame;
	uint8_t state_end;
} t;

typedef struct { uint16_t end, spr_gen, sprite_display, control[0x10]; } seg_t;
#define MAX_SPR 4   // (sprite buffers the segments of a frame may show)

static seg_t s_seg[MAX_SEG];
static int s_nseg;
static struct {
	seg_t seg[MAX_SEG + 1];
	int nseg;
	uint16_t vram[0x8000], spr[MAX_SPR][0x800], pal[0x800];
} s_vs;
static uint16_t s_spr_old[MAX_SPR][0x800];   // (the sprite buffer before a write in the frame being run)
static int s_spr_n;

static void seg_capture(int line)
{
	if (s_nseg >= MAX_SEG || line <= VIS_Y0 || line > VBLANK_LINE)
		return;
	seg_t *g = &s_seg[s_nseg++];
	g->end = (uint16_t)line;
	g->spr_gen = (uint16_t)s_spr_n;
	g->sprite_display = t.sprite_display;
	memcpy(g->control, t.control, sizeof(g->control));
}

static void capture(void)
{
	rbx_video_wait_idle();
	seg_capture(VBLANK_LINE);
	memcpy(s_vs.seg, s_seg, sizeof(seg_t) * (size_t)s_nseg);
	s_vs.nseg = s_nseg;
	s_nseg = 0;
	memcpy(s_vs.vram, t.vram, sizeof(s_vs.vram));
	// (the segments before a sprite buffer write show the old buffer: MAME drew them at the raster interrupt)
	memcpy(s_vs.spr, s_spr_old, sizeof(s_vs.spr[0]) * (size_t)s_spr_n);
	memcpy(s_vs.spr[s_spr_n], t.spritebuf, sizeof(s_vs.spr[0]));
	s_spr_n = 0;
	memcpy(s_vs.pal, t.palette, sizeof(s_vs.pal));
	rbx_video_frame_done();
}

// one layer's tiles of a category on a line (tilemap_draw + update_scroll_positions)
static void draw_layer_line(int lay, int category, int opaque, const seg_t *g, uint16_t *d, uint8_t *pri, int y)
{
	const uint16_t *vram = s_vs.vram;
	const uint16_t *ctl = g->control;
	const int layctl = ctl[0x08 + lay];
	int scrolly = ctl[0 + 2 * lay];
	const int scrollx = ctl[1 + 2 * lay];
	const int rowscroll_y = scrolly;
	if (layctl & 2)
		scrolly += vram[(0xe800 + 0x200 * lay) / 2 + y];
	const int sy = (y + 128 + scrolly) & 511;
	const int sx = (layctl & 1) ? vram[(0xe000 + 0x200 * lay) / 2 + (((sy + 0xff80) - rowscroll_y) & 0x1ff)] + scrollx : scrollx;
	const int value = (-3 + 2 * lay) - sx;
	const int px = (VIS_X0 - value) & 511;
	const uint32_t base = (uint32_t)((layctl >> 8) & 15) * 0x800;
	const int row = sy >> 3, ty = sy & 7, solid = lay == 3 || opaque;
	int col = px >> 3, off = px & 7, x = 0;
	while (x < VIS_W) {
		const uint32_t idx = (base + 2u * ((uint32_t)row * 64 + (uint32_t)col)) & 0x7fff;
		const uint32_t attr = vram[(idx + 1) & 0x7fff];
		const int n = 8 - off < VIS_W - x ? 8 - off : VIS_W - x;
		col = (col + 1) & 63;
		if ((int)((attr >> 9) & 1) == category) {
			const uint32_t code = rbx_wrap(vram[idx] + ((attr & 0x1000) << 4), s_chr.count);
			const int fx = (attr >> 10) & 1, fy = (attr >> 11) & 1, row8 = fy ? 7 - ty : ty;
			if (solid || (s_chr_rows[code] & (1 << row8))) {
				const uint8_t *s = s_chr.pix + (size_t)code * 64 + row8 * 8;
				const uint32_t pen = (attr & 0x7f) << 4;
				for (int i = 0; i < n; i++) {
					const uint32_t p = s[fx ? 7 - off - i : off + i];
					if (p || solid) {
						d[x + i] = (uint16_t)(pen + p);
						if (category)
							pri[x + i] = 1;
					}
				}
			}
		}
		x += n;
		off = 0;
	}
}

static int s_clip0, s_clip1;
static inline void spr_draw(uint32_t code, int colour, int fx, int fy, int x, int y, uint16_t *bm, int pitch, uint32_t pmask)
{
	rbx_gfx_prio_transpen(&s_spr, code, (uint16_t)(colour << 4), fx, fy, x - VIS_X0, y - VIS_Y0 - s_clip0, bm + s_clip0 * pitch,
	                      s_pri + s_clip0 * pitch, pitch, VIS_W, s_clip1 - s_clip0, pmask);
}

static void draw_sprites(const uint16_t *spriteram, uint16_t *bm, int pitch, int y0, int y1)
{
	const uint8_t *tb = s_sprtable;

	s_clip0 = y0;
	s_clip1 = y1;
	for (int offs = 0; offs < 0x800; offs += 4) {
		const uint32_t pri_mask = !(spriteram[offs + 2] & 0x80) ? 2 : 0;
		int y = spriteram[offs + 0] & 0x1ff;
		int x = spriteram[offs + 3] & 0x1ff;
		if (x == 0 || y == 0)
			continue;   // (offscreen)
		int sprite = spriteram[offs + 1] & 0x7fff;
		x = x - 16;
		y = 384 - 16 - y;
		const int colour = spriteram[offs + 2] & 0x7f;
		const int fx = (spriteram[offs + 2] >> 8) & 1;
		const int fy = (spriteram[offs + 2] >> 8) & 2;
		int y_multi = (spriteram[offs + 0] >> 11) & 3;

		if (s_spritesystem == 0) {
			y_multi = 1 << y_multi;
			int s_ptr = fy ? 0 : y_multi - 1;
			for (int i = 0; i < y_multi; i++) {
				spr_draw((uint32_t)(sprite + s_ptr), colour, fx, fy, x, y - i * 16, bm, pitch, pri_mask);
				spr_draw((uint32_t)(sprite + s_ptr), colour, fx, fy, x, (y - i * 16) - 0x200, bm, pitch, pri_mask);
				s_ptr += fy ? 1 : -1;
			}
		} else {
			uint32_t rom_offs = (uint32_t)sprite * 8;
			if (!tb)
				return;
			if (rom_offs + 7 < s_sprtable_size && (tb[rom_offs + 1] || tb[rom_offs + 3] || tb[rom_offs + 5] || tb[rom_offs + 7])) {
				while (rom_offs + 7 < s_sprtable_size) {
					int xdisp = tb[rom_offs + 6] + 256 * tb[rom_offs + 7];
					int ydisp = tb[rom_offs + 2] + 256 * tb[rom_offs + 3];
					const int ffx = fx ^ (tb[rom_offs + 1] & 1);
					const int ffy = fy ^ (tb[rom_offs + 1] & 2);
					sprite = tb[rom_offs + 4] + 256 * tb[rom_offs + 5];
					y_multi = 1 << ((tb[rom_offs + 3] >> 1) & 3);
					if (fx)
						xdisp = -xdisp - 16;
					if (fy)
						ydisp = -ydisp - (16 * y_multi - 1);
					if (!ffy)
						sprite += y_multi - 1;
					for (int i = 0; i < y_multi; i++) {
						const uint32_t code = (uint32_t)(sprite + (ffy ? i : -i));
						spr_draw(code, colour, ffx, ffy, (x + xdisp) & 0x1ff, (y - ydisp - 16 * i) & 0x1ff, bm, pitch, pri_mask);
						spr_draw(code, colour, ffx, ffy, (x + xdisp) & 0x1ff, ((y - ydisp - 16 * i) & 0x1ff) - 0x200, bm, pitch, pri_mask);
					}
					if (tb[rom_offs + 1] & 0x80)
						break;   // (end of block)
					rom_offs += 8;
				}
			}
		}
	}
}

static void m107_render(void)
{
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	uint32_t *pal = rbx_video.palette;
	for (int i = 0; i < NPENS; i++) {
		const uint32_t v = s_vs.pal[i];
		const uint32_t r = v & 0x1f, g = (v >> 5) & 0x1f, b = (v >> 10) & 0x1f;
		pal[i] = (r << 3 | r >> 2) << 16 | (g << 3 | g >> 2) << 8 | (b << 3 | b >> 2);
	}
	int sn = 0;
	for (int yy = 0; yy < VIS_H; yy++) {
		const int y = yy + VIS_Y0;
		while (sn < s_vs.nseg - 1 && y >= s_vs.seg[sn].end)
			sn++;
		const seg_t *g = &s_vs.seg[sn];
		const uint16_t *c = g->control;
		uint16_t *d = bm + yy * pitch;
		uint8_t *pri = s_pri + yy * pitch;
		const int off3 = (c[0x0b] >> 7) & 1;
		memset(d, 0, VIS_W * sizeof(uint16_t));
		memset(pri, 0, VIS_W);
		if (!off3) {
			draw_layer_line(3, 0, 0, g, d, pri, y);
			draw_layer_line(3, 1, 0, g, d, pri, y);
		}
		// (a layer switched off: bit 7 of its control)
		if (!(c[0x0a] & 0x80))
			draw_layer_line(2, 0, off3, g, d, pri, y);
		if (!(c[0x09] & 0x80))
			draw_layer_line(1, 0, 0, g, d, pri, y);
		if (!(c[0x08] & 0x80))
			draw_layer_line(0, 0, 0, g, d, pri, y);
		if (!(c[0x0a] & 0x80))
			draw_layer_line(2, 1, 0, g, d, pri, y);
		if (!(c[0x09] & 0x80))
			draw_layer_line(1, 1, 0, g, d, pri, y);
		if (!(c[0x08] & 0x80))
			draw_layer_line(0, 1, 0, g, d, pri, y);
	}
	// (the sprites of a run of segments with the same buffer)
	int y0 = 0;
	for (int i = 0; i < s_vs.nseg; i++) {
		const seg_t *g = &s_vs.seg[i];
		const int last = i == s_vs.nseg - 1;
		if (last || s_vs.seg[i + 1].spr_gen != g->spr_gen || s_vs.seg[i + 1].sprite_display != g->sprite_display) {
			const int y1 = last ? VIS_H : (int)g->end - VIS_Y0;
			if (g->sprite_display && y1 > y0)
				draw_sprites(s_vs.spr[g->spr_gen], bm, pitch, y0, y1);
			y0 = y1;
		}
	}
}

// ---- the main board ----
static void pic_out(int state) { nec_set_input(&s_cpu, NEC_LINE_IRQ, state); }
static int pic_ack(void) { return pic8259_ack(&s_pic); }
static void answer(int state) { pic8259_set_irq(&s_pic, 3, state); }
static inline void combine(uint16_t *r, uint16_t data, uint16_t mask) { *r = (uint16_t)((*r & ~mask) | (data & mask)); }

static void map_bank(void)
{
	nec_map(&s_cpu, 0xa0000, 0xc0000, s_rom + 0x80000 + (uint32_t)t.bank * 0x20000, NULL);
}

static uint16_t main_r16(uint32_t a)
{
	if (s_wpk && a >= 0xf0000 && a < 0xf0006)
		return (uint16_t)rbx_port_read(s_port_wpk[(a - 0xf0000) >> 1]);
	if (a >= 0xffff0)
		return *(const uint16_t *)(s_rom + 0x7fff0 + (a & 0xe));
	return 0;
}
static uint8_t main_r8(uint32_t a)
{
	const uint16_t v = main_r16(a & ~1u);
	return (a & 1) ? (uint8_t)(v >> 8) : (uint8_t)v;
}

static void control_w(uint32_t o, uint16_t d, uint16_t m)
{
	combine(&t.control[o], d, m);
	if (o == 0x0f) {
		t.raster = (int32_t)(uint16_t)(t.control[o] - 128);
		pic8259_set_irq(&s_pic, 2, 0);
	}
}

static void io_wm(uint32_t p, uint16_t v, uint16_t m)
{
	if (p >= 0x80 && p < 0xa0)
		control_w((p - 0x80) >> 1, v, m);
	else if (p == 0xb0) {
		if (m & 0xff) {
			if (s_nseg > 0 && s_spr_n < MAX_SPR - 1 && s_seg[s_nseg - 1].spr_gen == s_spr_n)   // (lines of this frame are out)
				memcpy(s_spr_old[s_spr_n++], t.spritebuf, sizeof(t.spritebuf));
			t.sprite_display = !(v & 0x1000);
			memcpy(t.spritebuf, t.spriteram, sizeof(t.spritebuf));
		}
	} else if (p == 0xc0)
		isnd_reset_line(v == 0);
	else if (p == 0 && (m & 0xff))
		isnd_latch_w((uint8_t)v);
	else if (p == 6 && (m & 0xff) && s_banked) {
		t.bank = (v & 6) >> 1;
		map_bank();
	} else if (p >= 0x40 && p < 0x44 && (m & 0xff))
		pic8259_write(&s_pic, (int)(p - 0x40) >> 1, (uint8_t)v);
}
static void main_out8(uint32_t p, uint8_t v)
{
	if (p & 1)
		io_wm(p & ~1u, (uint16_t)(v << 8), 0xff00);
	else
		io_wm(p, v, 0x00ff);
}
static void main_out16(uint32_t p, uint16_t v) { io_wm(p, v, 0xffff); }
static uint16_t main_in16(uint32_t p)
{
	switch (p) {
	case 0: return (uint16_t)rbx_port_read(s_port_p12);
	case 2:   // (bit 7 the screen's vblank)
		rbx_port_custom(s_port_coins, 0x80, nec_now(&s_cpu) - t.vbl_at < s_vbl_cycles ? 0x80 : 0);
		return (uint16_t)rbx_port_read(s_port_coins);
	case 4: return (uint16_t)rbx_port_read(s_port_dsw);
	case 6: return (uint16_t)rbx_port_read(s_port_p34);
	case 8: return isnd_latch2_r();
	case 0x40: return pic8259_read(&s_pic, 0);
	case 0x42: return pic8259_read(&s_pic, 1);
	case 0xc0: return s_wpk ? (uint16_t)rbx_port_read(s_port_wpk[3]) : 0;
	case 0xc2: return s_wpk ? (uint16_t)rbx_port_read(s_port_wpk[4]) : 0;
	}
	return 0;
}
static uint8_t main_in8(uint32_t p)
{
	if (p == 9 || p == 0x41 || p == 0x43)
		return 0;
	const uint16_t v = main_in16(p & ~1u);
	return (p & 1) ? (uint8_t)(v >> 8) : (uint8_t)v;
}

// ---- init / frame ----
static void m107_exit(void);
static void m107_reset(void);

static const uint8_t *dec_table(const char *m)
{
	if (!strcmp(m, "firebarr"))
		return rtypeleo_decryption_table;
	if (!strcmp(m, "airass"))
		return gunforce_decryption_table;
	if (!strcmp(m, "dsoccr94"))
		return dsoccr94_decryption_table;
	if (!strcmp(m, "wpksoc"))
		return leagueman_decryption_table;
	return NULL;
}

static int m107_init(const rbx_game *g)
{
	uint32_t cs = 0, ss = 0;
	s_rom = rbx_region("maincpu", &s_rom_size);
	const uint8_t *chr = rbx_region("tiles", &cs), *spr = rbx_region("sprites", &ss);
	const char *m = g->machine;
	const uint8_t *dec = dec_table(m);
	s_sprtable = rbx_region("sprtable", &s_sprtable_size);
	if (!s_rom || s_rom_size < 0x100000 || !chr || !spr || !dec) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	s_banked = !strcmp(m, "dsoccr94");
	s_wpk = !strcmp(m, "wpksoc");
	s_spritesystem = !strcmp(m, "firebarr") || !strcmp(m, "airass");
	s_clock = s_banked ? 10000000u : 14000000u;
	s_vbl_cycles = (int)(s_clock / 400);   // (2500 us)
	memset(&t, 0, sizeof(t));
	s_port_p12 = rbx_port_find("P1_P2");
	s_port_coins = rbx_port_find("COINS_DSW3");
	s_port_dsw = rbx_port_find("DSW");
	s_port_p34 = rbx_port_find("P3_P4");
	static const char *const wpk[5] = { "WPK_DSW0", "WPK_DSW1", "WPK_DSW2", "WPK_IN0", "WPK_IN1" };
	for (int i = 0; i < 5; i++)
		s_port_wpk[i] = rbx_port_find(wpk[i]);
	const int layout3 = !strcmp(m, "firebarr") || s_wpk;
	if (irem_gfx_decode(&s_chr, chr, cs, IGFX_M107_CHAR, &s_chr_rows) ||
	    irem_gfx_decode(&s_spr, spr, ss, layout3 ? IGFX_SPRITE3 : IGFX_SPRITE, NULL) ||
	    rbx_video_setup(VIS_W, VIS_H, NPENS) || isnd_init(dec, 1, FRAME_MHZ, LINES, answer)) {
		rbx_fail("%s: out of memory", g->set);
		m107_exit();
		return -1;
	}
	s_pri = (uint8_t *)malloc((size_t)rbx_video.pitch * VIS_H);
	if (!s_pri) {
		m107_exit();
		return -1;
	}
	rbx_audio.frame_mhz = FRAME_MHZ;
	s_pic.out_int = pic_out;
	m107_reset();
	return 0;
}

static void m107_exit(void)
{
	isnd_exit();
	irem_gfx_free(&s_chr);
	irem_gfx_free(&s_spr);
	free(s_chr_rows);
	free(s_pri);
	s_chr_rows = s_pri = NULL;
}

static void m107_reset(void)
{
	isnd_reset();
	memset(&t, 0, offsetof(__typeof__(t), state_end));
	t.vbl_at = -(int64_t)s_vbl_cycles;
	t.raster = 0;
	s_nseg = 0;
	s_spr_n = 0;
	memset(&s_vs, 0, sizeof(s_vs));
	pic8259_reset(&s_pic);
	nec_init(&s_cpu, NEC_V33, NULL);
	s_cpu.read8 = main_r8;
	s_cpu.read16 = main_r16;
	s_cpu.in8 = main_in8;
	s_cpu.in16 = main_in16;
	s_cpu.out8 = main_out8;
	s_cpu.out16 = main_out16;
	s_cpu.irq_ack = pic_ack;
	nec_map(&s_cpu, 0, s_wpk ? 0x80000 : s_banked ? 0xa0000 : 0xc0000, s_rom, NULL);
	if (s_banked)
		map_bank();
	nec_map(&s_cpu, 0xd0000, 0xe0000, (uint8_t *)t.vram, (uint8_t *)t.vram);
	nec_map(&s_cpu, 0xe0000, 0xf0000, (uint8_t *)t.ram, (uint8_t *)t.ram);
	nec_map(&s_cpu, 0xf8000, 0xf9000, (uint8_t *)t.spriteram, (uint8_t *)t.spriteram);
	nec_map(&s_cpu, 0xf9000, 0xfa000, (uint8_t *)t.palette, (uint8_t *)t.palette);
	s_cpu.omap[0xff] = s_rom + 0x7f000;
	nec_reset(&s_cpu);
}

static void m107_frame(void)
{
	for (int i = 0; i < LINES; i++) {
		const int vpos = (VBLANK_LINE + i) % LINES;
		if (vpos == t.raster) {   // (scanline_interrupt)
			seg_capture(vpos + 1);
			pic8259_set_irq(&s_pic, 2, 1);
		}
		if (vpos == VBLANK_LINE) {
			capture();
			t.vbl_at = nec_now(&s_cpu);
		}
		if (vpos != t.raster) {
			if (vpos == VBLANK_LINE) {
				pic8259_set_irq(&s_pic, 0, 1);
				pic8259_set_irq(&s_pic, 2, 0);
			} else
				pic8259_set_irq(&s_pic, 0, 0);
		}
		t.cpu_end += rbx_line_cycles(&t.cyc_frac, s_clock, FRAME_MHZ, LINES);
		nec_run_until(&s_cpu, t.cpu_end);
		isnd_line(i);
	}
	isnd_frame();
	t.frame++;
}

static int m107_state_io(void *ctx, int load)
{
	int rc = 0;
	rbx_video_wait_idle();
	static uint8_t cpu[sizeof(nec_t)];
	if (!load)
		memcpy(cpu, &s_cpu, NEC_STATE_SIZE);
	rc |= rbx_state_io(ctx, "m107_cpu", cpu, NEC_STATE_SIZE);
	rc |= rbx_state_io(ctx, "m107", &t, (uint32_t)offsetof(__typeof__(t), state_end));
	rc |= rbx_state_io(ctx, "m107_pic", &s_pic, PIC8259_STATE_SIZE);
	int32_t ns = s_nseg;
	rc |= rbx_state_io(ctx, "m107_ns", &ns, sizeof(ns));
	rc |= rbx_state_io(ctx, "m107_seg", s_seg, sizeof(s_seg));
	int32_t sn = s_spr_n;
	rc |= rbx_state_io(ctx, "m107_sn", &sn, sizeof(sn));
	rc |= rbx_state_io(ctx, "m107_so", s_spr_old, sizeof(s_spr_old));
	rc |= isnd_state(ctx, load);
	if (!rc && load) {
		memcpy(&s_cpu, cpu, NEC_STATE_SIZE);
		nec_after_load(&s_cpu);
		s_nseg = ns >= 0 && ns <= MAX_SEG ? ns : 0;
		s_spr_n = sn >= 0 && sn < MAX_SPR ? sn : 0;
		if (s_banked)
			map_bank();
	}
	return rc ? -1 : 0;
}

static int m107_save(void *ctx) { return m107_state_io(ctx, 0); }
static int m107_load(void *ctx) { return m107_state_io(ctx, 1); }
static int m107_supported(const char *machine) { return dec_table(machine) != NULL; }

const rbx_driver rbx_drv_m107 = { "Irem M107", m107_init, m107_exit, m107_reset, m107_frame, m107_save, m107_load,
                                  m107_supported, m107_render, NULL };
