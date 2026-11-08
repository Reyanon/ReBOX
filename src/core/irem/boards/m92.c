/*  Irem M92. Ported from MAME src/mame/irem/m92.cpp, m92.h, m92_v.cpp (BSD-3-Clause, copyright-holders: Bryan McPhail);
    the scroll arithmetic of src/emu/tilemap.cpp (BSD-3-Clause, copyright-holders: Nicola Salmoria, Aaron Giles).
    A V33 at 9 MHz, a uPD71059C, three 8x8 tile layers with rowscroll drawn line by line from the decoded pixels, two
    pens' categories around the sprites, and the sound board of irem_snd.c.
    Raster effects: MAME's update_partial at the raster interrupt is kept as a list of the layer registers per segment;
    VRAM, palette and sprites are the frame's at vblank.
    Not done: flip screen (MACHINE_NO_COCKTAIL), the bootlegs ppan and nbbatman2bl (other hardware).
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

#define CPU_CLOCK 9000000u     // (18 MHz / 2)
#define PIX_CLOCK 6666666u     // (26.666666 MHz / 4)
#define HTOTAL 424
#define LINES 262
#define VIS_X0 80
#define VIS_Y0 8
#define VIS_W 320
#define VIS_H 240
#define VBLANK_LINE 248
#define FRAME_MHZ 60012u       // (6666666 / 424 / 262)
#define NPENS 0x2000
#define MAX_SEG 244     // (nbbatman moves the raster interrupt on every line)

static nec_t s_cpu;
static pic8259_t s_pic;
static uint8_t *s_rom;
static uint32_t s_rom_size;
static rbx_gfx s_chr, s_spr;
static uint8_t *s_chr_rows, *s_pri;
static int s_banked, s_lethalth, s_majtitl;
static const uint8_t *s_eeprom;
static uint32_t s_eeprom_size;
static int s_port_p12, s_port_coins, s_port_dsw, s_port_p34;

static struct {
	uint16_t ram[0x8000], vram[0x8000];
	uint16_t spriteram[0x800], spritebuf[0x400];
	uint16_t paletteram[0x1000], palette[0x2000];
	uint16_t dmacontrol[8], videocontrol, master[4], control[3][4];
	uint8_t eeprom[0x2000];
	uint8_t pen_set[0x2000];   // (a pen never written keeps MAME's start palette: the rainbow of allocate_palette)
	int32_t raster;
	uint8_t bank, dma_busy, master_set, pad;   // (master_set: the layers whose control has been written)
	int64_t cpu_end, dma_end;
	uint32_t cyc_frac;
	uint32_t frame;
	uint8_t state_end;
} t;

// ---- the drawing's snapshot ----
typedef struct {
	uint16_t end;   // (lines up to this one)
	uint16_t videocontrol, master[3], control[3][4], master_set;
} seg_t;

static seg_t s_seg[MAX_SEG];
static int s_nseg;
static struct {
	seg_t seg[MAX_SEG + 1];
	int nseg;
	uint16_t vram[0x8000], spr[0x400], pal[0x2000], videocontrol;
	uint8_t pen_set[0x2000];
} s_vs;

static void seg_capture(int line)
{
	if (s_nseg >= MAX_SEG || line <= VIS_Y0 || line > VBLANK_LINE)
		return;
	seg_t *g = &s_seg[s_nseg++];
	g->end = (uint16_t)line;
	g->videocontrol = t.videocontrol;
	g->master_set = t.master_set;
	memcpy(g->master, t.master, sizeof(g->master));
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
	memcpy(s_vs.spr, t.spritebuf, sizeof(s_vs.spr));
	memcpy(s_vs.pal, t.palette, sizeof(s_vs.pal));
	memcpy(s_vs.pen_set, t.pen_set, sizeof(s_vs.pen_set));
	s_vs.videocontrol = t.videocontrol;
	rbx_video_frame_done();
}

static void draw_layer_line(int lay, const seg_t *g, uint16_t *d, uint8_t *pri, int y, int wide)
{
	const uint16_t *vram = s_vs.vram;
	const int mc = g->master[lay];
	const int width = wide ? 1024 : 512;
	const uint32_t base = (uint32_t)(mc & 3) * 0x2000;
	const int sy = (y + 128 + g->control[lay][0]) & 511;
	const int sx = (mc & 0x40) ? vram[(0xf400 + 0x400 * lay) / 2 + sy] : g->control[lay][2];
	const int value = (2 * lay - (wide ? 256 : 0)) - sx;
	const int px = (VIS_X0 - value) & (width - 1);
	const int row = sy >> 3, ty = sy & 7, cols = wide ? 128 : 64;
	const uint32_t chigh = (uint32_t)(g->videocontrol >> 7) & 0x180;
	int col = px >> 3, off = px & 7, x = 0;
	while (x < VIS_W) {
		const uint32_t idx = (base + 2u * ((uint32_t)row * (uint32_t)cols + (uint32_t)col)) & 0x7fff;
		const uint32_t attr = vram[(idx + 1) & 0x7fff];
		const uint32_t code = rbx_wrap(vram[idx] + ((attr & 0x8000) << 1), s_chr.count);
		const int fy = (attr >> 10) & 1, fx = (attr >> 9) & 1;
		const int n = 8 - off < VIS_W - x ? 8 - off : VIS_W - x;
		const int row8 = fy ? 7 - ty : ty;
		const int solid = lay == 2;
		col = (col + 1) & (cols - 1);
		if (!solid && !(s_chr_rows[code] & (1 << row8))) {
			x += n;
			off = 0;
			continue;
		}
		const uint8_t *s = s_chr.pix + (size_t)code * 64 + row8 * 8;
		const uint32_t pen = (((attr & 0x7f) | chigh) << 4);
		const int g_ = (attr & 0x100) ? 2 : (attr & 0x80) ? 1 : 0;
		if (g_ == 0) {   // (group 0: all behind the sprites)
			if (solid)
				for (int i = 0; i < n; i++)
					d[x + i] = (uint16_t)(pen + s[fx ? 7 - off - i : off + i]);
			else
				for (int i = 0; i < n; i++) {
					const uint32_t p = s[fx ? 7 - off - i : off + i];
					if (p)
						d[x + i] = (uint16_t)(pen + p);
				}
		} else if (g_ == 1) {   // (group 1: pens 8-15 over the sprites)
			for (int i = 0; i < n; i++) {
				const uint32_t p = s[fx ? 7 - off - i : off + i];
				if (p >= 8) {
					d[x + i] = (uint16_t)(pen + p);
					pri[x + i] = 1;
				} else if (p || solid)
					d[x + i] = (uint16_t)(pen + p);
			}
		} else {                // (group 2: all but pen 0 over the sprites)
			for (int i = 0; i < n; i++) {
				const uint32_t p = s[fx ? 7 - off - i : off + i];
				if (p) {
					d[x + i] = (uint16_t)(pen + p);
					pri[x + i] = 1;
				} else if (solid)
					d[x + i] = (uint16_t)pen;
			}
		}
		x += n;
		off = 0;
	}
}

static void draw_sprites(uint16_t *bm, int pitch)
{
	const uint16_t *source = s_vs.spr;
	const int vc = s_vs.videocontrol;
	const int pri_enabled = (vc >> 13) & 1;
	const int cmask = pri_enabled ? 0x7f : 0xff;
	const int chigh = (vc & 0x8000) ? 0x100 : 0;

	for (int offs = 0; offs < 0x400;) {
		int x = source[offs + 3] & 0x1ff;
		int y = source[offs + 0] & 0x1ff;
		const uint32_t code = source[offs + 1];
		const int color = (source[offs + 2] & cmask) | chigh;
		const uint32_t pri = pri_enabled ? (uint32_t)((~source[offs + 2] >> 6) & 2) : 2;
		const int flipx = (source[offs + 2] >> 8) & 1;
		const int flipy = (source[offs + 2] >> 9) & 1;
		const int numcols = 1 << ((source[offs + 0] >> 11) & 3);
		const int numrows = 1 << ((source[offs + 0] >> 9) & 3);

		offs += 4 * numcols;
		x = (x - 16) & 0x1ff;
		y = 384 - 16 - y;
		if (flipx)
			x += 16 * (numcols - 1);
		for (int col = 0; col < numcols; col++) {
			int s_ptr = 8 * col;
			if (!flipy)
				s_ptr += numrows - 1;
			for (int row = 0; row < numrows; row++) {
				const uint16_t cb = (uint16_t)(color << 4);
				const int sy = y - row * 16 - VIS_Y0;
				rbx_gfx_prio_transpen(&s_spr, code + (uint32_t)s_ptr, cb, flipx, flipy, x - VIS_X0, sy, bm, s_pri,
				                      pitch, VIS_W, VIS_H, pri);
				rbx_gfx_prio_transpen(&s_spr, code + (uint32_t)s_ptr, cb, flipx, flipy, x - 512 - VIS_X0, sy, bm, s_pri,
				                      pitch, VIS_W, VIS_H, pri);
				s_ptr += flipy ? 1 : -1;
			}
			x += flipx ? -16 : 16;
		}
	}
}

static void m92_render(void)
{
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	uint32_t *pal = rbx_video.palette;
	for (int i = 0; i < NPENS; i++) {
		const uint32_t v = s_vs.pal[i];
		const uint32_t r = v & 0x1f, g = (v >> 5) & 0x1f, b = (v >> 10) & 0x1f;
		pal[i] = s_vs.pen_set[i] ? (r << 3 | r >> 2) << 16 | (g << 3 | g >> 2) << 8 | (b << 3 | b >> 2)
		                         : ((i & 1) ? 0xff0000u : 0) | ((i & 2) ? 0xff00u : 0) | ((i & 4) ? 0xffu : 0);
	}
	int sn = 0;
	for (int y = 0; y < VIS_H; y++) {
		const int line = y + VIS_Y0;
		while (sn < s_vs.nseg - 1 && line >= s_vs.seg[sn].end)
			sn++;
		const seg_t *g = &s_vs.seg[sn];
		uint16_t *d = bm + y * pitch;
		uint8_t *pri = s_pri + y * pitch;
		memset(d, 0, VIS_W * sizeof(uint16_t));
		memset(pri, 0, VIS_W);
		for (int lay = 2; lay >= 0; lay--) {
			if (g->master[lay] & 0x10)
				continue;
			// (MAME's two tilemaps of a layer start enabled: both are drawn until its control register is written)
			if (!(g->master_set & (1 << lay)))
				draw_layer_line(lay, g, d, pri, line, 1);
			draw_layer_line(lay, g, d, pri, line, (g->master[lay] & 4) != 0);
		}
	}
	if (~s_vs.videocontrol & 0x80)
		draw_sprites(bm, pitch);
}

// ---- the main board ----
static void pic_out(int state) { nec_set_input(&s_cpu, NEC_LINE_IRQ, state); }
static int pic_ack(void) { return pic8259_ack(&s_pic); }
static void answer(int state) { pic8259_set_irq(&s_pic, 3, state); }

static void dma_set(int busy)
{
	t.dma_busy = (uint8_t)busy;
	rbx_port_custom(s_port_coins, 0x80, (uint32_t)(busy ? 0x80 : 0));
}

static void sprite_dma(int amount)
{
	const uint16_t *source = &t.spriteram[(t.videocontrol << 7) & 0x400];
	int dest_offs = 0;
	const int sort = t.dmacontrol[2] & 1;
	const int skip_layer7 = !((t.dmacontrol[2] >> 3) & 1);

	for (int layer = 0; layer < 8; layer++) {
		for (int offs = 0, numcols = 0; offs < 0x400; offs += 4 * numcols) {
			const int curlayer = (source[offs] >> 13) & 7;
			numcols = 1 << ((source[offs] >> 11) & 3);
			if ((!sort || layer == curlayer) && !(curlayer == 7 && skip_layer7)) {
				for (int i = 0; i < 4; i++)
					t.spritebuf[(dest_offs + i) & 0x3ff] = source[offs + i];
				dest_offs += 4 * numcols;
				if (dest_offs >= amount)
					return;
			}
		}
		if (layer == 6 && skip_layer7)
			return;
	}
}

static inline void combine(uint16_t *r, uint16_t data, uint16_t mask) { *r = (uint16_t)((*r & ~mask) | (data & mask)); }

static void dmacontrol_w(uint32_t offset, uint16_t data, uint16_t mask)
{
	combine(&t.dmacontrol[offset], data, mask);
	if (offset == 4) {   // (the data written doesn't matter)
		int clocks = 0x400;
		const int src_palbank = (t.videocontrol << 6) & 0xc00;
		const int dest_palbank = (t.dmacontrol[2] << 2) & 0x1c00;
		for (int i = 0; i < 0x400; i++)
			t.palette[i | dest_palbank] = t.paletteram[i | src_palbank];
		memset(t.pen_set + dest_palbank, 1, 0x400);
		memset(t.spritebuf, 0, sizeof(t.spritebuf));
		if (t.dmacontrol[0] & 0xff) {
			const int amount = (0x100 - (t.dmacontrol[0] & 0xff)) * 4;
			clocks += amount;
			sprite_dma(amount);
		}
		// (one word per 2 * pixel clock; always past this line's end)
		t.dma_end = nec_now(&s_cpu) + (int64_t)clocks * CPU_CLOCK / (2 * PIX_CLOCK);
		dma_set(0);
		pic8259_set_irq(&s_pic, 1, 0);
	}
}

static uint16_t spriteram_r(uint32_t o)
{
	return (t.dmacontrol[1] & 1) ? t.spritebuf[o] : t.spriteram[o | ((t.videocontrol << 10) & 0x400)];
}
static void spriteram_w(uint32_t o, uint16_t d, uint16_t m)
{
	if (t.dmacontrol[1] & 1)
		combine(&t.spritebuf[o], d, m);
	else
		combine(&t.spriteram[o | ((t.videocontrol << 10) & 0x400)], d, m);
}
static uint16_t paletteram_r(uint32_t o)
{
	if (t.dmacontrol[1] & 2)
		return t.palette[o | ((t.videocontrol << 2) & 0x1c00)];
	return t.paletteram[o | ((t.videocontrol << 9) & 0xc00)];
}
static void paletteram_w(uint32_t o, uint16_t d, uint16_t m)
{
	if (t.dmacontrol[1] & 2) {
		combine(&t.palette[o | ((t.videocontrol << 2) & 0x1c00)], d, m);
		t.pen_set[o | ((t.videocontrol << 2) & 0x1c00)] = 1;
	} else
		combine(&t.paletteram[o | ((t.videocontrol << 9) & 0xc00)], d, m);
}

static void map_bank(void)
{
	const uint8_t *p = s_rom + 0x80000 + (uint32_t)t.bank * 0x20000;
	nec_map(&s_cpu, 0xa0000, 0xc0000, p, NULL);
}

static uint16_t main_r16(uint32_t a)
{
	if (a >= 0xf8000 && a < 0xf8800)
		return spriteram_r((a - 0xf8000) >> 1);
	if (a >= 0xf8800 && a < 0xf9000)
		return paletteram_r((a - 0xf8800) >> 1);
	if (s_majtitl && a >= 0xf0000 && a < 0xf4000)
		return t.eeprom[((a - 0xf0000) >> 1) & 0x1fff];
	if (a >= 0xffff0)
		return *(const uint16_t *)(s_rom + 0x7fff0 + (a & 0xe));
	return 0;
}
static uint8_t main_r8(uint32_t a)
{
	const uint16_t v = main_r16(a & ~1u);
	return (a & 1) ? (uint8_t)(v >> 8) : (uint8_t)v;
}
static void main_wm(uint32_t a, uint16_t v, uint16_t m)
{
	if (a >= 0xf8000 && a < 0xf8800)
		spriteram_w((a - 0xf8000) >> 1, v, m);
	else if (a >= 0xf8800 && a < 0xf9000)
		paletteram_w((a - 0xf8800) >> 1, v, m);
	else if (a >= 0xf9000 && a < 0xf9010)
		dmacontrol_w((a - 0xf9000) >> 1, v, m);
	else if (a == 0xf9800)
		combine(&t.videocontrol, v, m);
	else if (s_majtitl && a >= 0xf0000 && a < 0xf4000 && (m & 0xff))
		t.eeprom[((a - 0xf0000) >> 1) & 0x1fff] = (uint8_t)v;
}
static void main_w8(uint32_t a, uint8_t v)
{
	if (a & 1)
		main_wm(a & ~1u, (uint16_t)(v << 8), 0xff00);
	else
		main_wm(a, v, 0x00ff);
}
static void main_w16(uint32_t a, uint16_t v) { main_wm(a, v, 0xffff); }

static void master_control_w(uint32_t o, uint16_t d, uint16_t m)
{
	combine(&t.master[o], d, m);
	if (o < 3)
		t.master_set |= (uint8_t)(1 << o);
	if (o == 3) {
		t.raster = (t.master[3] & 0x1ff) - 128;
		pic8259_set_irq(&s_pic, 2, 0);
	}
}

static void io_wm(uint32_t p, uint16_t v, uint16_t m)
{
	if (p >= 0x80 && p < 0x98)
		combine(&t.control[(p - 0x80) >> 3][(p >> 1) & 3], v, m);
	else if (p >= 0x98 && p < 0xa0)
		master_control_w((p - 0x98) >> 1, v, m);
	else if (p == 0xc0)
		isnd_reset_line(v == 0);
	else if (p == 0 && (m & 0xff))
		isnd_latch_w((uint8_t)v);
	else if (p == 0x20 && (m & 0xff) && s_banked) {
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
	case 2: return (uint16_t)rbx_port_read(s_port_coins);
	case 4: return (uint16_t)rbx_port_read(s_port_dsw);
	case 6: return (uint16_t)rbx_port_read(s_port_p34);
	case 8: return isnd_latch2_r();
	case 0x40: return pic8259_read(&s_pic, 0);
	case 0x42: return pic8259_read(&s_pic, 1);
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
static void m92_exit(void);
static void m92_reset(void);

static const uint8_t *dec_table(const char *m)
{
	static const struct { const char *name; const uint8_t *t; } tb[] = {
		{ "gunforce", gunforce_decryption_table }, { "bmaster", bomberman_decryption_table },
		{ "lethalth", lethalth_decryption_table }, { "uccops", dynablaster_decryption_table },
		{ "mysticri", mysticri_decryption_table }, { "majtitl2", majtitl2_decryption_table },
		{ "majtitl2a", mysticri_decryption_table }, { "hook", hook_decryption_table },
		{ "rtypeleo", rtypeleo_decryption_table }, { "inthunt", inthunt_decryption_table },
		{ "nbbatman", leagueman_decryption_table }, { "leaguemna", leagueman_decryption_table },
		{ "psoldier", psoldier_decryption_table }, { "dsoccr94j", dsoccr94_decryption_table },
		{ "gunforc2", lethalth_decryption_table }, { "geostorma", dsoccr94_decryption_table },
	};
	for (size_t i = 0; i < sizeof(tb) / sizeof(tb[0]); i++)
		if (!strcmp(m, tb[i].name))
			return tb[i].t;
	return NULL;
}

static int m92_init(const rbx_game *g)
{
	uint32_t cs = 0, ss = 0;
	s_rom = rbx_region("maincpu", &s_rom_size);
	const uint8_t *chr = rbx_region("gfx1", &cs), *spr = rbx_region("gfx2", &ss);
	const char *m = g->machine;
	const uint8_t *dec = dec_table(m);
	if (!s_rom || s_rom_size < 0x100000 || !chr || !spr || !dec) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	s_lethalth = !strcmp(m, "lethalth");
	s_majtitl = !strncmp(m, "majtitl2", 8);
	s_eeprom = rbx_region("eeprom", &s_eeprom_size);
	s_banked = s_majtitl || !strcmp(m, "nbbatman") || !strcmp(m, "leaguemna") || !strcmp(m, "dsoccr94j") ||
	           !strcmp(m, "gunforc2") || !strcmp(m, "geostorma");
	const int layout2 = !strcmp(m, "leaguemna") || !strcmp(m, "psoldier") || !strcmp(m, "dsoccr94j");
	memset(&t, 0, sizeof(t));
	s_port_p12 = rbx_port_find("P1_P2");
	s_port_coins = rbx_port_find("COINS_DSW3");
	s_port_dsw = rbx_port_find("DSW");
	s_port_p34 = rbx_port_find("P3_P4");
	if (irem_gfx_decode(&s_chr, chr, cs, IGFX_CHAR, &s_chr_rows) ||
	    irem_gfx_decode(&s_spr, spr, ss, layout2 ? IGFX_SPRITE2 : IGFX_SPRITE, NULL) ||
	    rbx_video_setup(VIS_W, VIS_H, NPENS) || isnd_init(dec, 0, FRAME_MHZ, LINES, answer)) {
		rbx_fail("%s: out of memory", g->set);
		m92_exit();
		return -1;
	}
	s_pri = (uint8_t *)malloc((size_t)rbx_video.pitch * VIS_H);
	if (!s_pri) {
		m92_exit();
		return -1;
	}
	rbx_audio.frame_mhz = FRAME_MHZ;
	s_pic.out_int = pic_out;
	m92_reset();
	return 0;
}

static void m92_exit(void)
{
	isnd_exit();
	irem_gfx_free(&s_chr);
	irem_gfx_free(&s_spr);
	free(s_chr_rows);
	free(s_pri);
	s_chr_rows = s_pri = NULL;
}

static void m92_reset(void)
{
	isnd_reset();
	memset(&t, 0, offsetof(__typeof__(t), eeprom));
	memset(t.eeprom, 0xff, sizeof(t.eeprom));
	if (s_eeprom)   // (majtitl2: the EEPROM's dump)
		memcpy(t.eeprom, s_eeprom, s_eeprom_size < sizeof(t.eeprom) ? s_eeprom_size : sizeof(t.eeprom));
	memset(&t.raster, 0, offsetof(__typeof__(t), state_end) - offsetof(__typeof__(t), raster));
	t.raster = -1;
	s_nseg = 0;
	s_vs.nseg = 0;
	memset(&s_vs, 0, sizeof(s_vs));
	pic8259_reset(&s_pic);
	nec_init(&s_cpu, NEC_V33, NULL);
	s_cpu.read8 = main_r8;
	s_cpu.read16 = main_r16;
	s_cpu.write8 = main_w8;
	s_cpu.write16 = main_w16;
	s_cpu.in8 = main_in8;
	s_cpu.in16 = main_in16;
	s_cpu.out8 = main_out8;
	s_cpu.out16 = main_out16;
	s_cpu.irq_ack = pic_ack;
	if (s_lethalth) {
		nec_map(&s_cpu, 0, 0x80000, s_rom, NULL);
		nec_map(&s_cpu, 0x80000, 0x90000, (uint8_t *)t.vram, (uint8_t *)t.vram);
	} else {
		nec_map(&s_cpu, 0, s_banked ? 0xa0000 : 0xc0000, s_rom, NULL);
		nec_map(&s_cpu, 0xc0000, 0xd0000, s_rom, NULL);
		nec_map(&s_cpu, 0xd0000, 0xe0000, (uint8_t *)t.vram, (uint8_t *)t.vram);
		if (s_banked)
			map_bank();
	}
	nec_map(&s_cpu, 0xe0000, 0xf0000, (uint8_t *)t.ram, (uint8_t *)t.ram);
	s_cpu.omap[0xff] = s_rom + 0x7f000;   // (ffff0-fffff: the reset vector)
	nec_reset(&s_cpu);
	dma_set(1);
}

static void m92_frame(void)
{
	for (int i = 0; i < LINES; i++) {
		const int vpos = (VBLANK_LINE + i) % LINES;
		if (vpos == t.raster) {   // (scanline_interrupt)
			seg_capture(vpos + 1);   // (update_partial draws the line itself too)
			pic8259_set_irq(&s_pic, 2, 1);
		} else
			pic8259_set_irq(&s_pic, 2, 0);
		if (vpos == VBLANK_LINE) {
			capture();
			pic8259_set_irq(&s_pic, 0, 1);
		} else
			pic8259_set_irq(&s_pic, 0, 0);
		t.cyc_frac += HTOTAL * CPU_CLOCK;
		const uint32_t c = t.cyc_frac / PIX_CLOCK;
		t.cyc_frac -= c * PIX_CLOCK;
		t.cpu_end += c;
		if (t.dma_end && t.dma_end <= t.cpu_end) {
			nec_run_until(&s_cpu, t.dma_end);
			t.dma_end = 0;
			dma_set(1);
			pic8259_set_irq(&s_pic, 1, 1);
		}
		nec_run_until(&s_cpu, t.cpu_end);
		isnd_line(i);
	}
	isnd_frame();
	t.frame++;
}

static int m92_state_io(void *ctx, int load)
{
	int rc = 0;
	rbx_video_wait_idle();
	static uint8_t cpu[sizeof(nec_t)];
	if (!load)
		memcpy(cpu, &s_cpu, NEC_STATE_SIZE);
	rc |= rbx_state_io(ctx, "m92_cpu", cpu, NEC_STATE_SIZE);
	rc |= rbx_state_io(ctx, "m92", &t, (uint32_t)offsetof(__typeof__(t), state_end));
	rc |= rbx_state_io(ctx, "m92_pic", &s_pic, PIC8259_STATE_SIZE);
	int32_t ns = s_nseg;
	rc |= rbx_state_io(ctx, "m92_ns", &ns, sizeof(ns));
	rc |= rbx_state_io(ctx, "m92_seg", s_seg, sizeof(s_seg));
	rc |= isnd_state(ctx, load);
	if (!rc && load) {
		memcpy(&s_cpu, cpu, NEC_STATE_SIZE);
		nec_after_load(&s_cpu);
		s_nseg = ns >= 0 && ns <= MAX_SEG ? ns : 0;
		if (s_banked)
			map_bank();
		rbx_port_custom(s_port_coins, 0x80, t.dma_busy ? 0x80 : 0);
	}
	return rc ? -1 : 0;
}

static int m92_save(void *ctx) { return m92_state_io(ctx, 0); }
static int m92_load(void *ctx) { return m92_state_io(ctx, 1); }
static int m92_supported(const char *machine) { return dec_table(machine) != NULL; }

const rbx_driver rbx_drv_m92 = { "Irem M92", m92_init, m92_exit, m92_reset, m92_frame, m92_save, m92_load, m92_supported,
                                 m92_render, NULL };
