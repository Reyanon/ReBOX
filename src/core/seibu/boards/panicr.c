/*  Seibu Kaihatsu Panic Road: a V20 with a ROM-mapped 16384x256 16x16 background (5 bpp: a priority plane), its
    collision bits read back by the CPU, an 8x8 text layer and 16x16 sprites, the T5182 sound MCU with a YM2151; 256
    colors from PROMs through lookup tables.
    Ported from MAME 0.289 src/mame/seibu/panicr.cpp (BSD-3-Clause, copyright-holders: David Haywood, Tomasz Slanina).
    This file is part of ReBOX. GPL-2.0-or-later.

    The CPUs interleaved a scanline at a time; the V20 runs as a V30 (the core has no 8-bit bus variant). MAME marks the
    game not working.
*/
#include "../../rbx.h"
#include "../../cpu/nec/nec.h"
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
#define CPU_CLOCK 8000000
#define NPENS 1024
#define PEN_BLACK 1024

static uint8_t *s_rom;
static uint32_t s_rom_size;
static const uint8_t *s_tilerom, *s_attrrom;
static uint8_t s_ind[NPENS];        // the pens' indirect colors
static uint32_t s_pens[NPENS + 1];
static rbx_gfx s_gtx, s_gbg, s_gspr, s_ginfo;
static int s_port[5];
static nec_t s_cpu;
static smix_t *s_mix;

typedef struct {
	uint8_t ram[0x4000];          // 0x00000-0x03fff (sprites 0x2000-0x3cff)
	uint8_t text[0x1000];
	int scrollx;
	uint8_t spritebank;
	int vector;
	int64_t main_target;
	uint64_t main_frac, snd_frac, ym_frac;
	uint8_t state_end;
} board_t;
static board_t *s_bp;   // (calloc at the init: ADR-040)
#define t (*s_bp)

static struct {
	uint8_t spr[0x1d00], text[0x1000];
	int scrollx;
	uint8_t spritebank;
} *s_snap;

// ---- the main CPU ----
static inline const uint8_t *tile_pix(const rbx_gfx *g, int tx, int ty)
{
	const uint32_t idx = (uint32_t)(ty >> 4) * 1024 + (uint32_t)(tx >> 4);
	const uint32_t code = s_tilerom[idx & 0x3fff] + ((uint32_t)(s_attrrom[idx & 0x3fff] & 7) << 8);
	return sd_elem(g, code) + (ty & 15) * 16 + (tx & 15);
}

static uint8_t collision_r(uint32_t offset)
{
	const int line = (int)(offset >> 6) & 0xff;
	const int col = ((int)(offset & 0x3f) * 4 - t.scrollx) & 0xff;
	uint8_t ret = 0;
	for (int k = 0; k < 4; k++) {
		const int tx = (((col + k) & 0xff) + t.scrollx) & 16383;
		ret |= (uint8_t)((*tile_pix(&s_ginfo, tx, line) & 3) << (6 - 2 * k));
	}
	return ret;
}

static uint8_t mem_r8(uint32_t a)
{
	a &= 0xfffff;
	if (a >= 0x8000 && a < 0xc000) return collision_r(a - 0x8000);
	if (a >= 0xc000 && a < 0xd000) return t.text[a - 0xc000];
	if (a >= 0xd200 && a < 0xd300) return (a & 1) ? 0 : t5182_sharedram_r((int)((a - 0xd200) >> 1));
	switch (a) {
	case 0xd004: return t5182_semaphore_snd_r();
	case 0xd400: return (uint8_t)rbx_port_read(s_port[0]);
	case 0xd402: return (uint8_t)rbx_port_read(s_port[1]);
	case 0xd404: return (uint8_t)rbx_port_read(s_port[2]);
	case 0xd406: return (uint8_t)rbx_port_read(s_port[3]);
	case 0xd407: return (uint8_t)rbx_port_read(s_port[4]);
	default: return 0xff;
	}
}

static uint16_t mem_r16(uint32_t a) { return (uint16_t)(mem_r8(a) | mem_r8(a + 1) << 8); }

static void mem_w8(uint32_t a, uint8_t v)
{
	a &= 0xfffff;
	if (a >= 0xc000 && a < 0xd000) { t.text[a - 0xc000] = v; return; }
	if (a >= 0xd200 && a < 0xd300) {
		if (!(a & 1))
			t5182_sharedram_w((int)((a - 0xd200) >> 1), v);
		return;
	}
	switch (a) {
	case 0xd000: t5182_sound_irq_w(); break;
	case 0xd002: t5182_semaphore_main_acquire_w(); break;
	case 0xd006: t5182_semaphore_main_release_w(); break;
	case 0xd802: t.scrollx = (t.scrollx & 0xff) | ((v & 0xf0) << 4) | ((v & 0x0f) << 12); break;
	case 0xd804: t.scrollx = (t.scrollx & 0xff00) | (v << 1 & 0xfe) | (v >> 7 & 0x01); break;
	case 0xd80c: t.spritebank = v; break;
	default: break;
	}
}

static void mem_w16(uint32_t a, uint16_t v)
{
	mem_w8(a, (uint8_t)v);
	mem_w8(a + 1, (uint8_t)(v >> 8));
}

static uint8_t io_in8(uint32_t p) { (void)p; return 0xff; }
static uint16_t io_in16(uint32_t p) { (void)p; return 0xffff; }
static void io_out8(uint32_t p, uint8_t v) { (void)p; (void)v; }
static void io_out16(uint32_t p, uint16_t v) { (void)p; (void)v; }

static int vector_r(void) { return t.vector; }

// ---- the drawing ----
static void pr_render(void)
{
	memcpy(rbx_video.palette, s_pens, sizeof(s_pens));
	const int sx = s_snap->scrollx;
	for (int r = 0; r < VIS_H; r++) {   // the background (the priority plane's pixels drawn again over the sprites)
		uint16_t *d = rbx_video.bitmap + r * rbx_video.pitch;
		const int ty = r + VIS_Y0;
		for (int x = 0; x < VIS_W; x++) {
			const int tx = (x + sx) & 16383;
			const uint32_t idx = (uint32_t)(ty >> 4) * 1024 + (uint32_t)(tx >> 4);
			const uint8_t attr = s_attrrom[idx & 0x3fff];
			const uint8_t pix = *tile_pix(&s_gbg, tx, ty);
			d[x] = (uint16_t)(0x200 + ((attr & 0xf0) >> 4) * 32 + (pix & 0x0f));
		}
	}
	const sd_target tg = { rbx_video.bitmap, rbx_video.pitch, 0, VIS_Y0, VIS_W, VIS_H, NULL };
	for (int offs = 0x1d00 - 16; offs >= 0; offs -= 16) {
		const uint8_t *s = &s_snap->spr[offs];
		int x = s[3];
		if (s[1] & 0x40)
			x -= 0x100;
		const uint32_t color = s[1] & 0x0f;
		uint32_t mask = 0;   // (transpen_mask: the pens whose color is the indirect 0)
		for (int p = 0; p < 16; p++)
			if (!s_ind[0x100 + color * 16 + (uint32_t)p])
				mask |= 1u << p;
		sd_transmask(&tg, &s_gspr, (uint32_t)(s[0] | s_snap->spritebank << 8), 0x100 + color * 16, 0, (s[1] >> 7) & 1, x,
		             s[2], mask);
	}
	for (int r = 0; r < VIS_H; r++) {
		uint16_t *d = rbx_video.bitmap + r * rbx_video.pitch;
		const int ty = r + VIS_Y0;
		for (int x = 0; x < VIS_W; x++) {
			const int tx = (x + sx) & 16383;
			const uint8_t pix = *tile_pix(&s_gbg, tx, ty);
			if (pix & 0x10) {
				const uint32_t idx = (uint32_t)(ty >> 4) * 1024 + (uint32_t)(tx >> 4);
				d[x] = (uint16_t)(0x200 + ((s_attrrom[idx & 0x3fff] & 0xf0) >> 4) * 32 + (pix & 0x0f));
			}
		}
		for (int x = 0; x < VIS_W; x++) {   // the text (pens of the indirect color 0 transparent)
			const int idx = (ty >> 3) * 32 + (x >> 3);
			const uint8_t attr = s_snap->text[idx * 4 + 2];
			const uint32_t code = s_snap->text[idx * 4] + (uint32_t)((attr & 8) << 5);
			const uint16_t pen = (uint16_t)((attr & 7) * 16 + sd_elem(&s_gtx, code)[(ty & 7) * 8 + (x & 7)]);
			if (s_ind[pen])
				d[x] = pen;
		}
	}
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap->spr, t.ram + 0x2000, sizeof(s_snap->spr));
	memcpy(s_snap->text, t.text, sizeof(s_snap->text));
	s_snap->scrollx = t.scrollx;
	s_snap->spritebank = t.spritebank;
	rbx_video_frame_done();
}

// ---- init / frame ----
static void pr_reset(void)
{
	rbx_video_wait_idle();
	memset(&t, 0, offsetof(__typeof__(t), state_end));
	nec_reset(&s_cpu);
	s_cpu.total = 0;
	t5182_reset();
	s_mix->n = 0;
	s_mix->frac = 0;
}

static void pr_exit(void)
{
	rbx_video_wait_idle();
	t5182_exit();
	rbx_gfx_free(&s_gtx);
	rbx_gfx_free(&s_gbg);
	rbx_gfx_free(&s_gspr);
	rbx_gfx_free(&s_ginfo);
	free(s_snap);
	s_snap = NULL;
	free(s_mix);
	s_mix = NULL;
	free(s_bp);
	s_bp = NULL;
}

// data lines (bitswap<16> of a byte from each of two regions' halves), then the address lines
static void unscramble(uint8_t *rom, uint8_t *buf, uint32_t size, const int *d16, const int *a24)
{
	for (uint32_t i = 0; i < size / 2; i++) {
		const uint32_t w = sd_bitswap((uint32_t)(rom[i] << 8 | rom[i + size / 2]), 16, d16);
		buf[i] = (uint8_t)(w >> 8);
		buf[i + size / 2] = (uint8_t)w;
	}
	for (uint32_t i = 0; i < size; i++)
		rom[i] = a24 ? buf[sd_bitswap(i, 24, a24) % size] : buf[i];
}

static void rearrange(uint8_t *rom, uint8_t *buf, uint32_t size)
{
	memcpy(buf, rom, size);
	for (uint32_t j = 0; j < 16; j++)
		for (uint32_t i = 0; i < size / 16; i += 8)
			memcpy(&rom[i + (size / 16) * j], &buf[i * 16 + 8 * j], 8);
}

static int pr_init(const rbx_game *g)
{
	uint32_t cs, ts, ss, trs, ars, ps, t5c, t5e;
	s_rom = rbx_region("maincpu", &s_rom_size);
	uint8_t *chars = rbx_region("chars", &cs), *tiles = rbx_region("tiles", &ts), *spr = rbx_region("sprites", &ss);
	uint8_t *tilerom = rbx_region("tilerom", &trs), *attrrom = rbx_region("attrrom", &ars);
	const uint8_t *prom = rbx_region("proms", &ps), *t5cpu = rbx_region("t5182:cpu", &t5c),
	              *t5ext = rbx_region("t5182:external", &t5e);
	if (!s_rom || s_rom_size < 0x100000 || !chars || !tiles || !spr || !tilerom || trs < 0x4000 || !attrrom ||
	    ars < 0x4000 || !prom || ps < 0x600 || !t5cpu || t5c < 0x2000 || !t5ext) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	s_tilerom = tilerom;
	s_attrrom = attrrom;
	{   // the palette: 256 colors, the text / tile / sprite lookup tables
		uint32_t rgb[256];
		for (int i = 0; i < 256; i++)
			rgb[i] = (uint32_t)((prom[i] & 15) * 0x11 << 16 | (prom[i + 0x100] & 15) * 0x11 << 8 | (prom[i + 0x200] & 15) * 0x11);
		memset(s_ind, 0, sizeof(s_ind));
		for (int i = 0; i < 256; i++) {
			const uint8_t tx = prom[0x300 + i], tl = prom[0x400 + i], sp = prom[0x500 + i];
			s_ind[i] = (tx & 0x40) ? 0 : (uint8_t)((tx & 0x3f) | 0x80);
			s_ind[0x200 + (i & 0x0f) + ((i & 0xf0) << 1)] = s_ind[0x210 + (i & 0x0f) + ((i & 0xf0) << 1)] = tl & 0x3f;
			s_ind[0x100 + i] = (sp & 0x40) ? 0 : (uint8_t)((sp & 0x3f) | 0x40);
		}
		for (int i = 0; i < NPENS; i++)
			s_pens[i] = rgb[s_ind[i]];
		s_pens[PEN_BLACK] = 0;
	}
	uint8_t *buf = (uint8_t *)malloc(0x80000);
	if (!buf) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	{   // init_panicr
		static const int dtx[16] = { 9, 12, 7, 3, 8, 13, 6, 2, 11, 14, 1, 5, 10, 15, 4, 0 };
		static const int atx[24] = { 23, 22, 21, 20, 19, 18, 17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 2, 3, 1, 0, 5, 4 };
		static const int d1[16] = { 14, 12, 11, 9, 3, 2, 1, 0, 10, 15, 13, 8, 7, 6, 5, 4 };
		static const int d2[16] = { 3, 13, 15, 4, 12, 2, 5, 11, 14, 6, 1, 10, 8, 7, 9, 0 };
		static const int atl[24] = { 23, 22, 21, 20, 19, 18, 17, 16, 15, 14, 13, 12, 5, 4, 3, 2, 11, 10, 9, 8, 7, 6, 0, 1 };
		static const int dsp[16] = { 11, 5, 7, 12, 4, 10, 13, 3, 6, 14, 9, 2, 0, 15, 1, 8 };
		unscramble(chars, buf, cs, dtx, atx);
		const uint32_t q = ts / 4;
		for (uint32_t i = 0; i < q; i++) {
			const uint32_t w1 = sd_bitswap((uint32_t)(tiles[i] << 8 | tiles[i + 3 * q]), 16, d1);
			const uint32_t w2 = sd_bitswap((uint32_t)(tiles[i + q] << 8 | tiles[i + 2 * q]), 16, d2);
			buf[i] = (uint8_t)(w1 >> 8);
			buf[i + q] = (uint8_t)w1;
			buf[i + 2 * q] = (uint8_t)(w2 >> 8);
			buf[i + 3 * q] = (uint8_t)w2;
		}
		for (uint32_t i = 0; i < ts; i++)
			tiles[i] = buf[sd_bitswap(i, 24, atl) % ts];
		unscramble(spr, buf, ss, dsp, NULL);
		rearrange(tilerom, buf, trs);
		rearrange(attrrom, buf, ars);
	}
	free(buf);
	{   // charlayout, bgtilelayout (5 planes), spritelayout, infotilelayout_2 (2 planes)
		static const uint32_t cx[8] = { 0, 1, 2, 3, 8, 9, 10, 11 }, cy[8] = { 0, 16, 32, 48, 64, 80, 96, 112 };
		static const uint32_t tx16[16] = { 0, 1, 2, 3, 8, 9, 10, 11, 16, 17, 18, 19, 24, 25, 26, 27 };
		static const uint32_t sx16[16] = { 0, 1, 2, 3, 32, 33, 34, 35, 64, 65, 66, 67, 96, 97, 98, 99 };
		static const uint32_t sy16[16] = { 0, 8, 16, 24, 128, 136, 144, 152, 256, 264, 272, 280, 384, 392, 400, 408 };
		static uint32_t ty16[16];
		for (int i = 0; i < 16; i++)
			ty16[i] = (uint32_t)i * 32;
		const uint32_t hc = cs / 2 * 8, qt = ts / 4 * 8, hs = ss / 2 * 8;
		const uint32_t pc[4] = { 0, 4, hc, hc + 4 };
		const uint32_t pt[5] = { qt, 2 * qt, 2 * qt + 4, 3 * qt, 3 * qt + 4 };
		const uint32_t pi[2] = { qt + 4, 4 };
		const uint32_t psp[4] = { 0, 4, hs, hs + 4 };
		const rbx_gfx_layout lc = { 8, 8, 4, pc, cx, cy, 128 };
		const rbx_gfx_layout lt = { 16, 16, 5, pt, tx16, ty16, 512 };
		const rbx_gfx_layout li = { 16, 16, 2, pi, tx16, ty16, 512 };
		const rbx_gfx_layout ls = { 16, 16, 4, psp, sx16, sy16, 512 };
		if (rbx_gfx_decode(&s_gtx, &lc, chars, cs / 2) || rbx_gfx_decode(&s_gbg, &lt, tiles, ts / 4) ||
		    rbx_gfx_decode(&s_ginfo, &li, tiles, ts / 4) || rbx_gfx_decode(&s_gspr, &ls, spr, ss / 2)) {
			rbx_fail("%s: out of memory (graphics)", g->set);
			pr_exit();
			return -1;
		}
	}
	s_snap = calloc(1, sizeof(*s_snap));
	s_mix = calloc(1, sizeof(*s_mix));
	s_bp = calloc(1, sizeof(board_t));
	if (!s_snap || !s_mix || !s_bp || t5182_init(t5cpu, t5ext, t5e, rbx_port_find("T5182_COIN"), 256) || rbx_audio_setup(48000) ||
	    rbx_video_setup(VIS_W, VIS_H, NPENS + 1)) {
		rbx_fail("%s: out of memory", g->set);
		pr_exit();
		return -1;
	}
	smix_init(s_mix, 48000, FRAME_MHZ, LINES);
	static const char *const names[5] = { "P1", "P2", "START", "DSW1", "DSW2" };
	for (int i = 0; i < 5; i++)
		s_port[i] = rbx_port_find(names[i]);
	nec_init(&s_cpu, NEC_V30, NULL);
	s_cpu.irq_ack = vector_r;
	nec_map(&s_cpu, 0x00000, 0x04000, t.ram, t.ram);
	nec_map(&s_cpu, 0xf0000, 0x100000, s_rom + 0xf0000, NULL);
	s_cpu.read8 = mem_r8;
	s_cpu.read16 = mem_r16;
	s_cpu.write8 = mem_w8;
	s_cpu.write16 = mem_w16;
	s_cpu.in8 = io_in8;
	s_cpu.in16 = io_in16;
	s_cpu.out8 = io_out8;
	s_cpu.out16 = io_out16;
	pr_reset();
	return 0;
}

static void pr_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	for (int line = 0; line < LINES; line++) {
		if (line == 240) {   // vblank-out
			vblank_capture();
			t.vector = 0xc4 / 4;
			nec_set_input(&s_cpu, NEC_LINE_IRQ, 1);
		} else if (line == 0) {
			t.vector = 0xc8 / 4;
			nec_set_input(&s_cpu, NEC_LINE_IRQ, 1);
		}
		t.main_target += rbx_line_cycles(&t.main_frac, CPU_CLOCK, FRAME_MHZ, LINES);
		nec_run_until(&s_cpu, t.main_target);
		t5182_run(rbx_line_cycles(&t.snd_frac, T5182_CLOCK, FRAME_MHZ, LINES));
		smix_begin(s_mix);
		int32_t *tmp = smix_tmp(s_mix);
		const int n = t5182_sound((uint32_t)rbx_line_cycles(&t.ym_frac, T5182_CLOCK, FRAME_MHZ, LINES), tmp, SMIX_TMP);
		smix_add(s_mix, tmp, n);
		smix_end(s_mix);
	}
	smix_flush(s_mix);
}

static int pr_state(void *ctx, int load)
{
	int rc = rbx_state_io(ctx, "panicr", &t, (uint32_t)offsetof(__typeof__(t), state_end));
	rc |= rbx_state_io(ctx, "panicr_cpu", &s_cpu, NEC_STATE_SIZE);
	rc |= t5182_state(ctx, load);
	if (load && !rc)
		nec_after_load(&s_cpu);
	return rc ? -1 : 0;
}

static int pr_save(void *ctx) { return pr_state(ctx, 0); }
static int pr_load(void *ctx) { return pr_state(ctx, 1); }
static int pr_supported(const char *m) { return !strcmp(m, "panicr"); }

const rbx_driver rbx_drv_panicr = { "Seibu Panic Road", pr_init, pr_exit, pr_reset, pr_frame, pr_save, pr_load,
                                    pr_supported, pr_render, NULL };
