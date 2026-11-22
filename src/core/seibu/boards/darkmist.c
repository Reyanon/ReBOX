/*  Seibu Kaihatsu The Lost Castle In Darkmist: a Z80 (opcodes and data encrypted apart), two ROM-mapped 16x16
    tilemaps, an 8x8 text layer and 16x16 sprites mixed through color lookup PROMs, the T5182 sound MCU with a YM2151
    (its external ROM's data lines scrambled); 256 colors (xRGB_444 RAM).
    Ported from MAME 0.289 src/mame/seibu/darkmist.cpp (BSD-3-Clause, copyright-holders: David Haywood, Nicola
    Salmoria, Tomasz Slanina).
    This file is part of ReBOX. GPL-2.0-or-later.

    The CPUs interleaved a scanline at a time. Not done: the sprites' random palette bit draws palette 0.
*/
#include "../../rbx.h"
#include "../../cpu/z80/z80.h"
#include "../../video/gfxlayout.h"
#include "../machine/smix.h"
#include "../machine/t5182.h"
#include "../video/seibu_draw.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define FRAME_MHZ 60000
#define LINES 256
#define VIS_Y0 16
#define VIS_W 256
#define VIS_H 224
#define CPU_CLOCK 4000000
#define PEN_BLACK 0x400

static uint8_t *s_rom, *s_op, *s_bgmap, *s_fgmap;
static uint8_t s_op_bank;   // (the bank whose opcodes s_op holds at 0x8000)
static uint32_t s_bgmap_size, s_fgmap_size;
static uint8_t s_clut[4][0x100];   // bg, fg, spr, tx
static rbx_gfx s_gtx, s_gbg, s_gfg, s_gspr;
static int s_port[6];
static z80 s_main;
static smix_t *s_mix;
static uint16_t *s_temp;   // (MAME's temp bitmap)

static struct {
	uint8_t pal[0x200], scroll[0x20], tx[0x800], ram[0x1000], spr[0x1000];
	uint8_t hw, spritebank;
	int main_irq;
	uint64_t main_frac, snd_frac, ym_frac;
	uint8_t state_end;
} t;

static struct {
	uint8_t scroll[0x20], tx[0x800], spr[0x1000];
	uint8_t hw, spritebank;
	uint32_t pens[0x401];
} *s_snap;

// ---- the main CPU ----
static uint8_t main_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x8000) return s_rom[a];
	if (a < 0xc000) return s_rom[0x10000 + (uint32_t)(t.hw >> 7) * 0x4000 + (a - 0x8000)];
	switch (a) {
	case 0xc801: return (uint8_t)rbx_port_read(s_port[0]);
	case 0xc802: return (uint8_t)rbx_port_read(s_port[1]);
	case 0xc803: return (uint8_t)rbx_port_read(s_port[2]);
	case 0xc806: return (uint8_t)rbx_port_read(s_port[3]);
	case 0xc807: return (uint8_t)rbx_port_read(s_port[4]);
	case 0xc808: return (uint8_t)rbx_port_read(s_port[5]);
	case 0xd681: return t5182_semaphore_snd_r();
	default: break;
	}
	if (a >= 0xd000 && a < 0xd100) return t.pal[a - 0xd000];
	if (a >= 0xd200 && a < 0xd300) return t.pal[0x100 + a - 0xd200];
	if (a >= 0xd400 && a < 0xd420) return t.scroll[a - 0xd400];
	if (a >= 0xd600 && a < 0xd680) return t5182_sharedram_r(a - 0xd600);
	if (a >= 0xd800 && a < 0xe000) return t.tx[a - 0xd800];
	if (a >= 0xe000 && a < 0xf000) return t.ram[a - 0xe000];
	if (a >= 0xf000) return t.spr[a - 0xf000];
	return 0xff;
}

static void main_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	switch (a) {
	case 0xc804: t.hw = v; return;
	case 0xc805: t.spritebank = v; return;
	case 0xd680: t5182_sound_irq_w(); return;
	case 0xd682: t5182_semaphore_main_acquire_w(); return;
	case 0xd683: t5182_semaphore_main_release_w(); return;
	default: break;
	}
	if (a >= 0xd000 && a < 0xd100) t.pal[a - 0xd000] = v;
	else if (a >= 0xd200 && a < 0xd300) t.pal[0x100 + a - 0xd200] = v;
	else if (a >= 0xd400 && a < 0xd420) t.scroll[a - 0xd400] = v;
	else if (a >= 0xd600 && a < 0xd680) t5182_sharedram_w(a - 0xd600, v);
	else if (a >= 0xd800 && a < 0xe000) t.tx[a - 0xd800] = v;
	else if (a >= 0xe000 && a < 0xf000) t.ram[a - 0xe000] = v;
	else if (a >= 0xf000) t.spr[a - 0xf000] = v;
}

static uint8_t z_in(z80 *z, uint8_t port) { (void)z; (void)port; return 0xff; }
static void z_out(z80 *z, uint8_t port, uint8_t v) { (void)z; (void)port; (void)v; }

// ---- the drawing ----
static inline int getscroll(const uint8_t *s, int n)
{
	return ((s[n] << 1) & 0xff) + ((s[n] & 0x80) ? 1 : 0) + (((s[n - 1] << 4) | (s[n - 1] << 12)) & 0xff00);
}

// mix_layer: the temp bitmap's pixels whose lookup is not transparent (bit 6) over the screen
static void mix_layer(const uint8_t *clut)
{
	for (int r = 0; r < VIS_H; r++) {
		uint16_t *d = rbx_video.bitmap + r * rbx_video.pitch;
		const uint16_t *s = s_temp + r * VIS_W;
		for (int x = 0; x < VIS_W; x++)
			if (!(clut[s[x] & 0xff] & 0x40))
				d[x] = s[x];
	}
}

// a ROM-mapped 16x16 tilemap (TILEMAP_SCAN_ROWS, cols x rows) into the temp bitmap
static void draw_map(const uint8_t *map, uint32_t map_size, const rbx_gfx *g, uint16_t base, int cols, int rows, int sx, int sy)
{
	const int mw = cols * 16, mh = rows * 16;
	for (int r = 0; r < VIS_H; r++) {
		uint16_t *d = s_temp + r * VIS_W;
		const int py = (r + VIS_Y0 + sy) & (mh - 1);
		for (int x = 0; x < VIS_W; x++) {
			const int px = (x + sx) & (mw - 1);
			const uint32_t idx = ((uint32_t)(py >> 4) * (uint32_t)cols + (uint32_t)(px >> 4)) * 2;
			const uint8_t code = idx < map_size ? map[idx] : 0, attr = idx + 1 < map_size ? map[idx + 1] : 0;
			const uint32_t c = code + (uint32_t)((attr & 3) << 8);
			d[x] = (uint16_t)(base + ((attr >> 4) & 0xf) * 16 + sd_elem(g, c)[(py & 15) * 16 + (px & 15)]);
		}
	}
}

static void dm_render(void)
{
	memcpy(rbx_video.palette, s_snap->pens, sizeof(s_snap->pens));
	const uint8_t *sc = s_snap->scroll;
	const int hw = s_snap->hw;
	for (int r = 0; r < VIS_H; r++) {
		uint16_t *d = rbx_video.bitmap + r * rbx_video.pitch;
		for (int x = 0; x < VIS_W; x++)
			d[x] = PEN_BLACK;
	}
	memset(s_temp, 0, sizeof(uint16_t) * VIS_W * VIS_H);
	if (hw & 4) {
		draw_map(s_bgmap, s_bgmap_size, &s_gbg, 0x000, 512, 64, getscroll(sc, 0x2), getscroll(sc, 0x6));
		mix_layer(s_clut[0]);
	}
	if (hw & 2) {
		draw_map(s_fgmap, s_fgmap_size, &s_gfg, 0x100, 64, 256, getscroll(sc, 0xa), getscroll(sc, 0xe));
		mix_layer(s_clut[1]);
	}
	if (hw & 1) {
		const sd_target tg = { s_temp, VIS_W, 0, VIS_Y0, VIS_W, VIS_H, NULL };
		for (int i = 0x1000 - 32; i >= 0; i -= 32) {
			const uint8_t *s = &s_snap->spr[i];
			uint32_t tile = s[0];
			if (s[1] & 0x20)
				tile += (uint32_t)s_snap->spritebank << 8;
			const int pal = (s[1] & 1) ? 0 : (s[1] >> 1) & 0xf;
			sd_transpen(&tg, &s_gspr, tile, 0x200 + (uint32_t)pal * 16, (s[1] >> 7) & 1, (s[1] >> 6) & 1, s[3], s[2], 0);
		}
		mix_layer(s_clut[2]);
	}
	if (hw & 16) {
		for (int r = 0; r < VIS_H; r++) {
			uint16_t *d = s_temp + r * VIS_W;
			const int y = r + VIS_Y0;
			for (int x = 0; x < VIS_W; x++) {
				const int idx = (y >> 3) * 32 + (x >> 3);
				const uint8_t attr = s_snap->tx[idx + 0x400];
				const uint32_t code = s_snap->tx[idx] + (uint32_t)((attr & 1) << 8);
				d[x] = (uint16_t)(0x300 + ((attr >> 1) & 0xf) * 16 + sd_elem(&s_gtx, code)[(y & 7) * 8 + (x & 7)]);
			}
		}
		mix_layer(s_clut[3]);
	}
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap->scroll, t.scroll, sizeof(t.scroll));
	memcpy(s_snap->tx, t.tx, sizeof(t.tx));
	memcpy(s_snap->spr, t.spr, sizeof(t.spr));
	s_snap->hw = t.hw;
	s_snap->spritebank = t.spritebank;
	uint32_t ind[256];
	for (int i = 0; i < 256; i++) {   // xRGB_444
		const uint16_t w = (uint16_t)(t.pal[i] | t.pal[0x100 + i] << 8);
		ind[i] = (uint32_t)((w >> 8) & 0xf) * 0x110000u | (uint32_t)((w >> 4) & 0xf) * 0x1100u | (uint32_t)(w & 0xf) * 0x11u;
	}
	static const uint8_t base[4] = { 0x80, 0x00, 0x40, 0xc0 };
	for (int p = 0; p < 4; p++)
		for (int i = 0; i < 256; i++)
			s_snap->pens[p << 8 | i] = ind[(s_clut[p][i] & 0x3f) | base[p]];
	s_snap->pens[PEN_BLACK] = 0;
	rbx_video_frame_done();
}

// ---- init / frame ----
static void dm_reset(void)
{
	rbx_video_wait_idle();
	memset(&t, 0, offsetof(__typeof__(t), state_end));
	z80_init(&s_main);
	s_main.read_byte = main_read;
	s_main.write_byte = main_write;
	s_main.port_in = z_in;
	s_main.port_out = z_out;
	s_main.op_rom = s_op;
	s_op_bank = 0;
	memcpy(s_op + 0x8000, s_rom + 0x10000, 0x4000);
	t5182_reset();
	s_mix->n = 0;
	s_mix->frac = 0;
}

static void dm_exit(void);

// the data lines (bitswap<16> of the two halves' bytes) and the address lines of a gfx region
static int unscramble(uint8_t *rom, uint32_t size, const int *a24)
{
	static const int d16[16] = { 9, 14, 7, 2, 6, 8, 3, 15, 10, 13, 5, 12, 0, 11, 4, 1 };
	uint8_t *buf = (uint8_t *)malloc(size);
	if (!buf)
		return -1;
	for (uint32_t i = 0; i < size / 2; i++) {
		const uint32_t w = sd_bitswap((uint32_t)(rom[i] << 8 | rom[i + size / 2]), 16, d16);
		buf[i] = (uint8_t)(w >> 8);
		buf[i + size / 2] = (uint8_t)w;
	}
	for (uint32_t i = 0; i < size; i++)
		rom[i] = buf[sd_bitswap(i, 24, a24) % size];
	free(buf);
	return 0;
}

static int remap(uint8_t *rom, uint32_t len, const int *a24)
{
	uint8_t *buf = (uint8_t *)malloc(len);
	if (!buf)
		return -1;
	memcpy(buf, rom, len);
	for (uint32_t i = 0; i < len; i++)
		rom[i] = buf[sd_bitswap(i, 24, a24) % len];
	free(buf);
	return 0;
}

static int dm_init(const rbx_game *g)
{
	uint32_t rs, txs, fgs, bgs, sps, cs, es, cl[4];
	s_rom = rbx_region("maincpu", &rs);
	uint8_t *tx = rbx_region("tx_gfx", &txs), *fg = rbx_region("fg_gfx", &fgs), *bg = rbx_region("bg_gfx", &bgs),
	        *sp = rbx_region("spr_gfx", &sps);
	s_bgmap = rbx_region("bg_map", &s_bgmap_size);
	s_fgmap = rbx_region("fg_map", &s_fgmap_size);
	const uint8_t *t5cpu = rbx_region("t5182:cpu", &cs);
	uint8_t *t5ext = rbx_region("t5182:external", &es);
	static const char *const cn[4] = { "bg_clut", "fg_clut", "spr_clut", "tx_clut" };
	const uint8_t *clut[4];
	for (int i = 0; i < 4; i++)
		clut[i] = rbx_region(cn[i], &cl[i]);
	if (!s_rom || rs < 0x18000 || !tx || !fg || !bg || !sp || !s_bgmap || !s_fgmap || !t5cpu || cs < 0x2000 || !t5ext ||
	    es < 0x8000 || !clut[0] || !clut[1] || !clut[2] || !clut[3]) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	for (int i = 0; i < 4; i++)
		memcpy(s_clut[i], clut[i], 0x100);
	s_op = calloc(1, 0x10000);
	s_snap = calloc(1, sizeof(*s_snap));
	s_mix = calloc(1, sizeof(*s_mix));
	s_temp = calloc(VIS_W * VIS_H, sizeof(uint16_t));
	if (!s_op || !s_snap || !s_mix || !s_temp) {
		rbx_fail("%s: out of memory", g->set);
		dm_exit();
		return -1;
	}
	{   // init_darkmist: the gfx, the sound ROM, the program, the maps
		static const int atx[24] = { 23, 22, 21, 20, 19, 18, 17, 16, 15, 14, 13, 12, 3, 2, 1, 11, 10, 9, 8, 0, 7, 6, 5, 4 };
		static const int atl[24] = { 23, 22, 21, 20, 19, 18, 17, 16, 15, 14, 13, 5, 4, 3, 2, 12, 11, 10, 9, 8, 1, 0, 7, 6 };
		static const int asp[24] = { 23, 22, 21, 20, 19, 18, 17, 16, 15, 14, 12, 11, 10, 9, 8, 5, 4, 3, 13, 7, 6, 1, 0, 2 };
		static const int abg[24] = { 23, 22, 21, 20, 19, 18, 17, 16, 7, 6, 5, 4, 3, 15, 14, 13, 12, 9, 8, 2, 1, 11, 10, 0 };
		static const int afg[24] = { 23, 22, 21, 20, 19, 18, 17, 16, 15, 6, 5, 4, 3, 12, 11, 10, 9, 14, 13, 2, 1, 8, 7, 0 };
		static const int snd8[8] = { 7, 1, 2, 3, 4, 5, 6, 0 };
		static const int p8[8] = { 7, 6, 5, 2, 3, 4, 1, 0 };
		if (unscramble(tx, txs, atx) || unscramble(bg, bgs, atl) || unscramble(fg, fgs, atl) || unscramble(sp, sps, asp) ||
		    remap(s_bgmap, s_bgmap_size, abg) || remap(s_fgmap, s_fgmap_size, afg)) {
			rbx_fail("%s: out of memory", g->set);
			dm_exit();
			return -1;
		}
		for (uint32_t i = 0; i < 0x8000; i++)
			t5ext[i] = (uint8_t)sd_bitswap(t5ext[i], 8, snd8);
		for (uint32_t i = 0; i < 0x8000; i++) {
			uint8_t p = s_rom[i], d = p;
			if (!(i & 0x20) && (i & 0x8)) p ^= 0x20;
			if (!(i & 0x20) && (i & 0xa)) d ^= 0x20;
			if ((i & 0x200) == 0x200 && (i & 0x408)) p ^= 0x10;
			if ((i & 0x220) != 0x200) {
				p = (uint8_t)sd_bitswap(p, 8, p8);
				d = (uint8_t)sd_bitswap(d, 8, p8);
			}
			s_rom[i] = d;
			s_op[i] = p;
		}
	}
	{   // charlayout, tilelayout (RGN_FRAC(1,2) planes)
		static const uint32_t cx[8] = { 0, 1, 2, 3, 8, 9, 10, 11 }, cy[8] = { 0, 16, 32, 48, 64, 80, 96, 112 };
		static const uint32_t tx16[16] = { 0, 1, 2, 3, 8, 9, 10, 11, 16, 17, 18, 19, 24, 25, 26, 27 };
		static uint32_t ty16[16];
		for (int i = 0; i < 16; i++)
			ty16[i] = (uint32_t)i * 32;
		const uint32_t hc = txs / 2 * 8, hb = bgs / 2 * 8, hf = fgs / 2 * 8, hs = sps / 2 * 8;
		const uint32_t pc[4] = { 0, 4, hc, hc + 4 }, pb[4] = { 0, 4, hb, hb + 4 }, pf[4] = { 0, 4, hf, hf + 4 },
		               ps[4] = { 0, 4, hs, hs + 4 };
		const rbx_gfx_layout lc = { 8, 8, 4, pc, cx, cy, 128 };
		const rbx_gfx_layout lb = { 16, 16, 4, pb, tx16, ty16, 512 }, lf = { 16, 16, 4, pf, tx16, ty16, 512 },
		                     ls = { 16, 16, 4, ps, tx16, ty16, 512 };
		if (rbx_gfx_decode(&s_gtx, &lc, tx, txs / 2) || rbx_gfx_decode(&s_gbg, &lb, bg, bgs / 2) ||
		    rbx_gfx_decode(&s_gfg, &lf, fg, fgs / 2) || rbx_gfx_decode(&s_gspr, &ls, sp, sps / 2)) {
			rbx_fail("%s: out of memory (graphics)", g->set);
			dm_exit();
			return -1;
		}
	}
	if (t5182_init(t5cpu, t5ext, es, rbx_port_find("T5182_COIN"), 256) || rbx_audio_setup(48000) ||
	    rbx_video_setup(VIS_W, VIS_H, 0x401)) {
		rbx_fail("%s: out of memory", g->set);
		dm_exit();
		return -1;
	}
	smix_init(s_mix, 48000, FRAME_MHZ, LINES);
	static const char *const names[6] = { "P1", "P2", "START", "DSW1", "DSW2", "UNK" };
	for (int i = 0; i < 6; i++)
		s_port[i] = rbx_port_find(names[i]);
	dm_reset();
	return 0;
}

static void dm_exit(void)
{
	rbx_video_wait_idle();
	t5182_exit();
	rbx_gfx_free(&s_gtx);
	rbx_gfx_free(&s_gbg);
	rbx_gfx_free(&s_gfg);
	rbx_gfx_free(&s_gspr);
	free(s_op);
	s_op = NULL;
	free(s_snap);
	s_snap = NULL;
	free(s_mix);
	s_mix = NULL;
	free(s_temp);
	s_temp = NULL;
}

static void dm_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	for (int line = 0; line < LINES; line++) {
		if (line == 240) {   // vblank-out: RST 10h
			vblank_capture();
			s_main.int_data = 0xd7;
			t.main_irq = 1;
		} else if (line == 0) {   // vblank-in: RST 08h
			s_main.int_data = 0xcf;
			t.main_irq = 1;
		}
		if ((t.hw >> 7) != s_op_bank) {   // (the bank's opcodes follow the data bank: plain ROM)
			s_op_bank = t.hw >> 7;
			memcpy(s_op + 0x8000, s_rom + 0x10000 + (uint32_t)s_op_bank * 0x4000, 0x4000);
		}
		const int c = rbx_line_cycles(&t.main_frac, CPU_CLOCK, FRAME_MHZ, LINES);
		z80_run(&s_main, s_main.cyc + (unsigned long)c, &t.main_irq, 1);
		t5182_run(rbx_line_cycles(&t.snd_frac, T5182_CLOCK, FRAME_MHZ, LINES));
		smix_begin(s_mix);
		int32_t *tmp = smix_tmp(s_mix);
		const int n = t5182_sound((uint32_t)rbx_line_cycles(&t.ym_frac, T5182_CLOCK, FRAME_MHZ, LINES), tmp, SMIX_TMP);
		smix_add(s_mix, tmp, n);
		smix_end(s_mix);
	}
	smix_flush(s_mix);
}

static int dm_state(void *ctx, int load)
{
	int rc = rbx_state_io(ctx, "darkmist", &t, (uint32_t)offsetof(__typeof__(t), state_end));
	z80 c = s_main;
	rc |= rbx_state_io(ctx, "darkmist_main", &c, sizeof(c));
	if (!rc && load) {
		c.read_byte = main_read;
		c.write_byte = main_write;
		c.port_in = z_in;
		c.port_out = z_out;
		c.userdata = NULL;
		c.read_map = NULL;
		c.op_rom = s_op;
		s_main = c;
		s_op_bank = 0xff;   // (refreshed at the next line)
	}
	rc |= t5182_state(ctx, load);
	return rc ? -1 : 0;
}

static int dm_save(void *ctx) { return dm_state(ctx, 0); }
static int dm_load(void *ctx) { return dm_state(ctx, 1); }
static int dm_supported(const char *m) { return !strcmp(m, "darkmist"); }

const rbx_driver rbx_drv_darkmist = { "Seibu Darkmist", dm_init, dm_exit, dm_reset, dm_frame, dm_save, dm_load,
                                      dm_supported, dm_render, NULL };
