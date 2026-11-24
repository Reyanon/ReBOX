/*  Irem M90 / M97 / M99 (the Bomberman family). Ported from MAME src/mame/irem/m90.cpp, m90.h, m90_v.cpp (BSD-3-Clause,
    copyright-holders: Bryan McPhail); the scroll arithmetic of src/emu/tilemap.cpp (BSD-3-Clause, copyright-holders:
    Nicola Salmoria, Aaron Giles).
    An encrypted V35 at 16 MHz (a V30 on the bootlegs), two tile layers of 64 or 128 x 64 with rowscroll and a line table
    of y scrolls, sprites in VRAM, the Z80 sound board of irem_zsnd.c. No raster interrupt: the frame is captured at vblank.
    Not done: dynablstb's interrupt of the Z80 runs on every 4th line (MAME: 64 * 60 Hz of 240 lines' 14400).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../cheat_cpu.h"
#include "../../rbx.h"
#include "../../cpu/nec/nec.h"
#include "../../cpu/nec/irem_cpu.h"
#include "../machine/irem_zsnd.h"
#include "../video/irem_gfx.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define FRAME_MHZ 60000u
#define NPENS 513     // (the last: black)

enum { V_M90, V_BOMBLORD, V_DYNABLSB };

static nec_t s_cpu;
static uint8_t *s_rom, *s_user1;
static uint32_t s_rom_size, s_user1_size, s_clock;
static rbx_gfx s_chr, s_spr;
static uint8_t *s_chr_rows, *s_pri;
static int s_kind, s_quiz, s_vis_x0, s_vis_y0, s_vis_w, s_vis_h, s_lines, s_v30;
static const uint8_t *s_dec;
static int s_port_p12, s_port_system, s_port_dsw, s_port_p34;

static struct {
	uint16_t ram[0x2000], vram[0x8000], spriteram[0x800], palette[0x800], control[8];   // (palette: a page of the map)
	uint8_t bank, pad[3];
	int64_t cpu_end;
	uint64_t cyc_frac;
	uint32_t frame;
	uint8_t state_end;
} t;

static struct {
	uint16_t vram[0x8000], spr[0x800], palette[0x200], control[8];
} s_vs;

static void capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_vs.vram, t.vram, sizeof(s_vs.vram));
	if (s_kind != V_M90)
		memcpy(s_vs.spr, t.spriteram, sizeof(s_vs.spr));
	memcpy(s_vs.palette, t.palette, sizeof(s_vs.palette));
	memcpy(s_vs.control, t.control, sizeof(s_vs.control));
	rbx_video_frame_done();
}

// a layer's line: category 0 then 1, as the two draws of the screen update
static void pf_line(const uint16_t *vram, int wide, int sx, int sy, int opaque, const uint16_t *rows, int bias, uint16_t *d, uint8_t *pri, int y)
{
	const int width = wide ? 1024 : 512;
	const int src_y = (y + sy) & 511;
	if (rows)
		sx = rows[src_y] + bias;
	const int cols = wide ? 128 : 64, row = src_y >> 3, ty = src_y & 7;
	for (int cat = 0; cat < 2; cat++) {
		const int px = (s_vis_x0 + sx) & (width - 1);
		int col = px >> 3, off = px & 7, x = 0;
		while (x < s_vis_w) {
			const uint32_t idx = (uint32_t)(row * cols + col) * 2;
			const uint32_t tile = vram[idx & 0x7fff], color = vram[(idx + 1) & 0x7fff];
			const int n = 8 - off < s_vis_w - x ? 8 - off : s_vis_w - x;
			col = (col + 1) & (cols - 1);
			if (((color & 0x30) ? 1 : 0) == cat) {
				const uint32_t code = rbx_wrap(tile, s_chr.count);
				const int fx = (color >> 6) & 1, fy = (color >> 7) & 1, r8 = fy ? 7 - ty : ty;
				if (opaque || (s_chr_rows[code] & (1 << r8))) {
					const uint8_t *s = s_chr.pix + (size_t)code * 64 + r8 * 8;
					const uint32_t pen = (color & 15) << 4;
					for (int i = 0; i < n; i++) {
						const uint32_t p = s[fx ? 7 - off - i : off + i];
						if (p || opaque) {
							d[x + i] = (uint16_t)(pen + p);
							if (cat)
								pri[x + i] = 1;
						}
					}
				}
			}
			x += n;
			off = 0;
		}
	}
}

static inline void spr(uint32_t code, int colour, int fx, int fy, int x, int y, uint16_t *bm, int pitch, uint32_t pmask)
{
	rbx_gfx_prio_transpen(&s_spr, code, (uint16_t)(256 + (colour << 4)), fx, fy, x - s_vis_x0, y - s_vis_y0, bm, s_pri, pitch, s_vis_w, s_vis_h, pmask);
}

static void draw_sprites(uint16_t *bm, int pitch)
{
	const uint16_t *sr = s_vs.vram + 0xee00 / 2;
	const int vc = s_vs.control[7];
	for (int offs = 0x1f2 / 2; offs >= 0; offs -= 3) {
		const uint32_t sprite = sr[offs + 1];
		const int colour = (sr[offs + 0] >> 9) & 0x0f;
		int y = sr[offs + 0] & 0x1ff, x = sr[offs + 2] & 0x1ff;
		x = x - 16;
		y = 512 - y;
		const int fx = (sr[offs + 2] >> 8) & 2;
		const int fy = (sr[offs + 0] >> 8) & 0x80;
		const int y_multi = 1 << ((sr[offs + 0] & 0x6000) >> 13);
		y -= 16 * y_multi;
		const uint32_t pm = (vc & 1) ? ((colour & 8) ? 0u : 2u) : (vc & 2) ? (((colour & 0xc) == 0xc) ? 0u : 2u) : 2u;
		for (int i = 0; i < y_multi; i++)
			spr(sprite + (uint32_t)(fy ? y_multi - 1 - i : i), colour, fx != 0, fy != 0, x, y + i * 16, bm, pitch, pm);
	}
}

static void boot_sprites(uint16_t *bm, int pitch)
{
	const uint16_t *sr = s_vs.spr;
	const int dyn = s_kind == V_DYNABLSB;
	const uint16_t end = dyn ? 0xffff : 0x8000;
	int offs = 7, last = 0;
	while (offs < 0x800 - 3 && sr[offs] != end) {
		last = offs;
		offs += 4;
	}
	for (offs = last; offs >= 7; offs -= 4) {
		const uint32_t sprite = sr[offs + 1];
		const int colour = (sr[offs + 2] >> 9) & 0x0f;
		int y = (sr[offs + 0] & 0x1ff) + (dyn ? 288 : 152);
		int x = (sr[offs + 3] & 0x1ff) + (dyn ? -64 : 16);
		x = x - 16;
		y = 512 - y;
		if (y < 0)
			y += 512;
		spr(sprite, colour, ((sr[offs + 3] >> 8) & 2) != 0, ((sr[offs + 2] >> 8) & 0x80) != 0, x, y, bm, pitch, (colour & 8) ? 0u : 2u);
	}
}

static void render_m90(uint16_t *bm, int pitch)
{
	const uint16_t *c = s_vs.control, *v = s_vs.vram;
	const int ctl[2] = { c[5], c[6] };
	static const int roff[2] = { 0xf000 >> 1, 0xf400 >> 1 }, bias[2] = { 2, -2 }, yt[2] = { 0xf800 / 2, 0xfc00 / 2 };
	if (c[7] & 4) {
		for (int y = 0; y < s_vis_h; y++)
			for (int x = 0; x < s_vis_w; x++)
				bm[y * pitch + x] = 512;
		return;
	}
	for (int yy = 0; yy < s_vis_h; yy++) {
		const int y = yy + s_vis_y0;
		uint16_t *d = bm + yy * pitch;
		uint8_t *pri = s_pri + yy * pitch;
		memset(pri, 0, (size_t)s_vis_w);
		if (ctl[1] & 0x10)
			memset(d, 0, sizeof(uint16_t) * (size_t)s_vis_w);
		for (int lay = 1; lay >= 0; lay--) {
			if (ctl[lay] & 0x10)
				continue;
			const int wide = (ctl[lay] & 4) != 0, base = ctl[lay] & 3;
			const uint16_t *vram = v + ((uint32_t)(wide ? base & ~1 : base) << 13);
			const int extra = wide ? 256 : 0;
			const uint16_t *rows = (ctl[lay] & 0x20) ? v + roff[lay] : NULL;
			const int sx = c[1 | (lay << 1)] + bias[lay] + extra;
			const int sy = (ctl[lay] & 0x40) ? 0x200 + v[yt[lay] + y] : c[lay << 1];
			pf_line(vram, wide, sx, sy, lay == 1, rows, bias[lay] + extra, d, pri, y);
		}
	}
	draw_sprites(bm, pitch);
}

static void render_boot(uint16_t *bm, int pitch)
{
	const uint16_t *c = s_vs.control, *v = s_vs.vram;
	for (int yy = 0; yy < s_vis_h; yy++) {
		const int y = yy + s_vis_y0;
		uint16_t *d = bm + yy * pitch;
		uint8_t *pri = s_pri + yy * pitch;
		memset(pri, 0, (size_t)s_vis_w);
		for (int x = 0; x < s_vis_w; x++)
			d[x] = 512;
		if (s_kind == V_BOMBLORD) {
			const int w1 = (c[6] & 2) != 0, w0 = (c[6] & 4) != 0;
			const int rs = (c[6] & 0x20) != 0;
			pf_line(v + 0x4000, w1, v[0xf000 / 2] - 16, v[0xf008 / 2] + (w1 ? 388 : -120), 0, NULL, 0, d, pri, y);
			pf_line(v, w0, v[0xf004 / 2] - 12, v[0xf00c / 2] + (w0 ? 392 : -116), 0, rs ? v + 0xf400 / 2 : NULL, -12 + (w0 ? 256 : 0), d, pri, y);
		} else {
			const int w0 = !(v[0xf008 / 2] & 0x4000), w1 = !(v[0xf008 / 2] & 0x8000);
			pf_line(v, w0, v[0xf004 / 2] + 64, v[0xf006 / 2] + (w0 ? 512 : 4), 1, NULL, 0, d, pri, y);
			pf_line(v + 0x4000, w1, v[0xf000 / 2] + 68, v[0xf002 / 2] + (w1 ? 512 : 4), 0, NULL, 0, d, pri, y);
		}
	}
	boot_sprites(bm, pitch);
}

static void m90_render(void)
{
	uint32_t *pal = rbx_video.palette;
	for (int i = 0; i < 512; i++) {
		const uint32_t v = s_vs.palette[i];
		const uint32_t r = v & 0x1f, g = (v >> 5) & 0x1f, b = (v >> 10) & 0x1f;
		pal[i] = (r << 3 | r >> 2) << 16 | (g << 3 | g >> 2) << 8 | (b << 3 | b >> 2);
	}
	pal[512] = 0;
	if (s_kind == V_M90)
		render_m90(rbx_video.bitmap, rbx_video.pitch);
	else
		render_boot(rbx_video.bitmap, rbx_video.pitch);
}

// ---- the main board ----
static inline void combine(uint16_t *r, uint16_t d, uint16_t m) { *r = (uint16_t)((*r & ~m) | (d & m)); }

static void map_bank(void)
{
	if (s_user1)
		nec_map(&s_cpu, 0x80000, 0x90000, s_user1 + (uint32_t)t.bank * 0x10000, NULL);
}

static void io_wm(uint32_t p, uint16_t v, uint16_t m)
{
	if (p == 0 && (m & 0xff))
		zsnd_latch_w((uint8_t)v);
	else if (p >= 0x80 && p < 0x90)
		combine(&t.control[(p - 0x80) >> 1], v, m);
	else if (p == 4 && s_quiz && (m & 0xff)) {
		t.bank = v & 0xf;
		map_bank();
	}
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
	case 2: return (uint16_t)rbx_port_read(s_port_system);
	case 4: return (uint16_t)rbx_port_read(s_port_dsw);
	case 6: return (uint16_t)rbx_port_read(s_port_p34);
	}
	return 0;
}
static uint8_t main_in8(uint32_t p)
{
	const uint16_t v = main_in16(p & ~1u);
	return (p & 1) ? (uint8_t)(v >> 8) : (uint8_t)v;
}
static int vec_dyna(void) { nec_set_input(&s_cpu, NEC_LINE_IRQ, 0); return 0x60 / 4; }
static int vec_lord(void) { nec_set_input(&s_cpu, NEC_LINE_IRQ, 0); return 0x50 / 4; }

// ---- init / frame ----
static void m90_exit(void);
static void m90_reset(void);

static const struct { const char *m; const uint8_t *dec; int ports, nmi; } s_cfgs[] = {
	{ "hasamu", gunforce_decryption_table, ZP_RTYPE2, ZN_PULSE },
	{ "quizf1", lethalth_decryption_table, ZP_RTYPE2, ZN_PULSE },
	{ "matchit2", matchit2_decryption_table, ZP_RTYPE2, ZN_PULSE },
	{ "riskchal", gussun_decryption_table, ZP_RTYPE2, ZN_PULSE },
	{ "bombrman", bomberman_decryption_table, ZP_RTYPE2, ZN_PULSE },
	{ "bbmanwj", dynablaster_decryption_table, ZP_RTYPE2, ZN_PULSE },
	{ "bbmanw", dynablaster_decryption_table, ZP_M99, ZN_FAKE },
	{ "dicegame", NULL, ZP_RTYPE2, ZN_PULSE },
	{ "dynablsb", NULL, ZP_DYNA, ZN_IRQHOLD },
	{ "bomblord", NULL, ZP_M99, ZN_FAKE80 },
};
#define NCFG ((int)(sizeof(s_cfgs) / sizeof(s_cfgs[0])))

static int cfg_index(const char *m)
{
	for (int i = 0; i < NCFG; i++)
		if (!strcmp(m, s_cfgs[i].m))
			return i;
	return -1;
}

static uint16_t bits16(uint16_t v, const int *b)
{
	uint16_t r = 0;
	for (int i = 0; i < 16; i++)
		r = (uint16_t)(r << 1 | ((v >> b[i]) & 1));
	return r;
}

static void bomblord_decrypt(void)   // (init_bomblord)
{
	static const int k[4][16] = {
		{ 0xc, 0x8, 0xd, 0xe, 0xf, 0xb, 0xa, 0x9, 0x6, 0x4, 0x7, 0x3, 0x1, 0x2, 0x0, 0x5 },
		{ 0xc, 0xb, 0xd, 0xa, 0xe, 0x9, 0xf, 0x8, 0x0, 0x6, 0x1, 0x5, 0x3, 0x4, 0x2, 0x7 },
		{ 0xd, 0x9, 0xc, 0x8, 0xe, 0xf, 0xa, 0xb, 0x4, 0x7, 0x3, 0x2, 0x5, 0x6, 0x1, 0x0 },
		{ 0xe, 0xd, 0xf, 0x8, 0xb, 0xa, 0x9, 0xc, 0x6, 0x3, 0x7, 0x5, 0x0, 0x1, 0x4, 0x2 },
	};
	uint16_t *rom = (uint16_t *)s_rom;
	for (int i = 0; i < 0x100000 / 2; i += 4)
		for (int j = 0; j < 4; j++)
			rom[i + j] = bits16(rom[i + j], k[j]);
}

static int m90_init(const rbx_game *g)
{
	rbx_cheat_prefer("nec");   // (the cheats' main CPU: another CPU is set up first)
	uint32_t gs = 0, ms = 0;
	const int ci = cfg_index(g->machine);
	s_rom = rbx_region("maincpu", &s_rom_size);
	const uint8_t *gfx = rbx_region("gfx1", &gs);
	uint8_t *samples = rbx_region("m72", &ms);
	if (ci < 0 || !s_rom || s_rom_size < 0x100000 || !gfx) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	const char *m = g->machine;
	s_kind = !strcmp(m, "bomblord") ? V_BOMBLORD : !strcmp(m, "dynablsb") ? V_DYNABLSB : V_M90;
	s_v30 = s_kind != V_M90;
	s_quiz = !strcmp(m, "quizf1");
	s_user1 = s_quiz ? rbx_region("user1", &s_user1_size) : NULL;
	s_clock = s_v30 ? 8000000u : 16000000u;
	s_lines = s_kind == V_DYNABLSB ? 240 : 512;
	const int tall = s_quiz || !strcmp(m, "matchit2");   // (17 * 8 - 8 .. 47 * 8 - 1 + 8)
	const int narrow = s_kind == V_BOMBLORD || !strcmp(m, "dicegame") || !strcmp(m, "riskchal") || !strcmp(m, "bombrman") ||
	                   !strcmp(m, "bbmanwj") || !strcmp(m, "bbmanw");
	s_vis_x0 = s_kind == V_DYNABLSB ? 0 : narrow ? 80 : 48;
	s_vis_w = s_kind == V_DYNABLSB || narrow ? 320 : 384;
	s_vis_y0 = s_kind == V_DYNABLSB ? 0 : tall ? 128 : 136;
	s_vis_h = tall ? 256 : 240;
	s_dec = s_cfgs[ci].dec;
	if (s_kind == V_BOMBLORD)
		bomblord_decrypt();
	memset(&t, 0, sizeof(t));
	s_port_p12 = rbx_port_find("P1_P2");
	s_port_system = rbx_port_find("SYSTEM");
	s_port_dsw = rbx_port_find("DSW");
	s_port_p34 = rbx_port_find("P3_P4");
	zsnd_cfg z;
	memset(&z, 0, sizeof(z));
	z.ports = s_cfgs[ci].ports;
	z.nmi = s_cfgs[ci].nmi;
	z.nmi_lines = 4;   // (128 * 60 Hz of 512 lines: every 4th; see the header for dynablstb)
	z.samples = samples;
	z.samples_size = ms;
	z.ym_gain = 38;    // (0.15)
	z.dac_gain = 26;   // (0.10)
	z.frame_mhz = FRAME_MHZ;
	z.lines = s_lines;
	if (irem_gfx_decode(&s_chr, gfx, gs, IGFX_CHAR, &s_chr_rows) || irem_gfx_decode(&s_spr, gfx, gs, IGFX_SPRITE, NULL) ||
	    rbx_video_setup(s_vis_w, s_vis_h, NPENS) || zsnd_init(&z)) {
		rbx_fail("%s: out of memory", g->set);
		m90_exit();
		return -1;
	}
	s_pri = (uint8_t *)malloc((size_t)rbx_video.pitch * s_vis_h);
	if (!s_pri) {
		m90_exit();
		return -1;
	}
	rbx_audio.frame_mhz = FRAME_MHZ;
	m90_reset();
	return 0;
}

static void m90_exit(void)
{
	zsnd_exit();
	irem_gfx_free(&s_chr);
	irem_gfx_free(&s_spr);
	free(s_chr_rows);
	free(s_pri);
	s_chr_rows = s_pri = NULL;
}

static void m90_reset(void)
{
	zsnd_reset(0);
	memset(&t, 0, offsetof(__typeof__(t), state_end));
	memset(&s_vs, 0, sizeof(s_vs));
	nec_init(&s_cpu, s_v30 ? NEC_V30 : NEC_V35, s_dec);
	s_cpu.in8 = main_in8;
	s_cpu.in16 = main_in16;
	s_cpu.out8 = main_out8;
	s_cpu.out16 = main_out16;
	nec_map(&s_cpu, 0, s_kind == V_DYNABLSB ? 0x40000 : 0x80000, s_rom, NULL);
	nec_map(&s_cpu, 0xa0000, 0xa4000, (uint8_t *)t.ram, (uint8_t *)t.ram);
	nec_map(&s_cpu, 0xd0000, 0xe0000, (uint8_t *)t.vram, (uint8_t *)t.vram);
	nec_map(&s_cpu, 0xe0000, 0xe1000, (uint8_t *)t.palette, (uint8_t *)t.palette);
	if (s_kind == V_DYNABLSB)
		nec_map(&s_cpu, 0x60000, 0x61000, (uint8_t *)t.spriteram, (uint8_t *)t.spriteram);
	else if (s_kind == V_BOMBLORD)
		nec_map(&s_cpu, 0xc0000, 0xc1000, (uint8_t *)t.spriteram, (uint8_t *)t.spriteram);
	if (s_quiz)
		map_bank();
	nec_map(&s_cpu, 0xff000, 0x100000, s_rom + 0xff000, NULL);
	if (s_kind == V_DYNABLSB)
		s_cpu.irq_ack = vec_dyna;
	else if (s_kind == V_BOMBLORD)
		s_cpu.irq_ack = vec_lord;
	nec_reset(&s_cpu);
}

static void m90_frame(void)
{
	for (int i = 0; i < s_lines; i++) {
		if (i == 0) {   // (the frame from vblank)
			capture();
			if (s_v30)
				nec_set_input(&s_cpu, NEC_LINE_IRQ, 1);
			else {
				nec_set_input(&s_cpu, NEC_LINE_INTP0, 1);
				nec_set_input(&s_cpu, NEC_LINE_INTP0, 0);
			}
		}
		t.cpu_end += rbx_line_cycles(&t.cyc_frac, s_clock, FRAME_MHZ, s_lines);
		nec_run_until(&s_cpu, t.cpu_end);
		zsnd_line(i);
	}
	zsnd_frame();
	t.frame++;
}

static int m90_state_io(void *ctx, int load)
{
	int rc = 0;
	rbx_video_wait_idle();
	rbx_aux2_sync();
	static uint8_t cpu[sizeof(nec_t)];
	if (!load)
		memcpy(cpu, &s_cpu, NEC_STATE_SIZE);
	rc |= rbx_state_io(ctx, "m90_cpu", cpu, NEC_STATE_SIZE);
	rc |= rbx_state_io(ctx, "m90", &t, (uint32_t)offsetof(__typeof__(t), state_end));
	rc |= zsnd_state(ctx, load);
	if (!rc && load) {
		memcpy(&s_cpu, cpu, NEC_STATE_SIZE);
		nec_after_load(&s_cpu);
		if (s_quiz)
			map_bank();
	}
	return rc ? -1 : 0;
}

static int m90_save(void *ctx) { return m90_state_io(ctx, 0); }
static int m90_load(void *ctx) { return m90_state_io(ctx, 1); }
static int m90_supported(const char *machine) { return cfg_index(machine) >= 0; }

const rbx_driver rbx_drv_m90 = { "Irem M90", m90_init, m90_exit, m90_reset, m90_frame, m90_save, m90_load, m90_supported,
                                 m90_render, NULL };
