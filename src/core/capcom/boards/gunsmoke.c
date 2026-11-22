/*  Capcom Gun.Smoke. Ported from MAME src/mame/capcom/gunsmoke.cpp (BSD-3-Clause, copyright-holders: Paul Leaman,
    Couriersud).
    Z80 at 3 MHz (IRQ held at vblank), Z80 sound CPU at 3 MHz (IRQ 4 times a frame) with two YM2203 at 1.5 MHz. Video: a
    65536x256 tilemap of 32x32 tiles from ROM, a 256x256 text layer (pens whose colour is 0x4f clear), 16x16 sprites
    from a buffered RAM; 256 colours (pal4bit PROMs) through lookup PROMs.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/z80/z80.h"
#include "../../sound/rbx_ym.h"
#include "../../video/gfxlayout.h"
#include "../../video/decoband.h"
#include "cap8.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define FRAME_MHZ 59637          // set_raw(12 MHz / 2, 384, 0, 256, 262, 16, 240)
#define LINES 262
#define VIS_Y0 16
#define VIS_H 224
#define VIS_W 256
#define CPU_CLOCK 3000000
#define YM_CLOCK 1500000
#define NPENS (32 * 4 + 16 * 16 + 16 * 16)
#define PEN_BLACK NPENS

static uint8_t *s_rom, *s_zrom, *s_tilerom;
static uint32_t s_rom_size, s_zrom_size, s_tilerom_size;
static uint32_t s_pens[NPENS + 1];
static uint8_t s_ctab[NPENS];
static rbx_gfx s_gchar, s_gtile, s_gspr;
static int s_port[5];
static z80 s_main, s_snd;
static const uint8_t *s_mmap[256], *s_zmap[256];
static int32_t *s_mix;
static int s_mix_n, s_mix_cap;

static struct {
	uint8_t ram[0x1000], spr[0x1000], sprbuf[0x1000], vram[0x400], cram[0x400];
	uint8_t zram[0x800];
	uint8_t scrollx[2], scrolly, bank, flip, chon, bgon, objon, spr3bank, latch;
	int main_irq, snd_irq, snd_acc;
	uint64_t main_frac, snd_frac, ym_frac;
	uint8_t state_end;
} *t;   // (calloc in init: ADR-040)

static struct {
	uint8_t spr[0x1000], vram[0x400], cram[0x400];
	uint16_t sx;
	uint8_t sy, flip, chon, bgon, objon, spr3bank;
} *s_snap;

static uint8_t z_in(z80 *z, uint8_t port) { (void)z; (void)port; return 0xff; }
static void z_out(z80 *z, uint8_t port, uint8_t v) { (void)z; (void)port; (void)v; }

static uint8_t snd_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x8000) return s_zrom[a];
	if (a >= 0xc000 && a < 0xc800) return t->zram[a - 0xc000];
	if (a == 0xc800) return t->latch;
	return 0;
}
static void snd_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0xc000 && a < 0xc800) t->zram[a - 0xc000] = v;
	else if (a == 0xe000 || a == 0xe001) rbx_ym_write(0, a & 1, v);
	else if (a == 0xe002 || a == 0xe003) rbx_ym_write(1, a & 1, v);
}

static void map_bank(void)
{
	for (int p = 0; p < 0x40; p++)
		s_mmap[0x80 + p] = s_rom + 0x8000 + ((uint32_t)t->bank * 0x4000 + p * 256) % (s_rom_size - 0x8000);
}

static uint8_t main_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x8000) return s_rom[a];
	if (a < 0xc000) return s_rom[0x8000 + ((uint32_t)t->bank * 0x4000 + (a - 0x8000)) % (s_rom_size - 0x8000)];
	if (a >= 0xc000 && a <= 0xc004) return (uint8_t)rbx_port_read(s_port[a - 0xc000]);
	if (a >= 0xc4c9 && a <= 0xc4cb) return a == 0xc4c9 ? 0xff : 0x00;   // (protection_r)
	if (a >= 0xd000 && a < 0xd400) return t->vram[a - 0xd000];
	if (a >= 0xd400 && a < 0xd800) return t->cram[a - 0xd400];
	if (a == 0xd800 || a == 0xd801) return t->scrollx[a & 1];
	if (a == 0xd802) return t->scrolly;
	if (a >= 0xe000 && a < 0xf000) return t->ram[a - 0xe000];
	if (a >= 0xf000) return t->spr[a - 0xf000];
	return 0;
}
static void main_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0xe000 && a < 0xf000) t->ram[a - 0xe000] = v;
	else if (a >= 0xf000) t->spr[a - 0xf000] = v;
	else if (a >= 0xd000 && a < 0xd400) t->vram[a - 0xd000] = v;
	else if (a >= 0xd400 && a < 0xd800) t->cram[a - 0xd400] = v;
	else switch (a) {
	case 0xc800: t->latch = v; break;
	case 0xc804:   // control_w
		t->bank = (v & 0x0c) >> 2;
		map_bank();
		t->flip = (v & 0x40) != 0;
		t->chon = v & 0x80;
		break;
	case 0xc806: memcpy(t->sprbuf, t->spr, sizeof(t->spr)); break;
	case 0xd800: case 0xd801: t->scrollx[a & 1] = v; break;
	case 0xd802: t->scrolly = v; break;
	case 0xd806: t->spr3bank = v & 7; t->bgon = v & 0x10; t->objon = v & 0x20; break;
	}
}

// (the tile's lookups once per tile: the pixels are the same as per-pixel lookups)
static void draw_bg(int y0, int y1)
{
	const int flip = s_snap->flip;
	for (int y = y0; y < y1; y++) {
		uint16_t *d = rbx_video.bitmap + (y - VIS_Y0) * rbx_video.pitch;
		if (!s_snap->bgon) {
			for (int x = 0; x < VIS_W; x++)
				d[x] = PEN_BLACK;
			continue;
		}
		const uint32_t ty = (uint32_t)(flip ? 255 - y + s_snap->sy : y + s_snap->sy) & 255;
		uint32_t last = ~0u, fx = 0;
		const uint8_t *src = NULL;
		uint16_t base = 0;
		for (int x = 0; x < VIS_W; x++) {   // TILEMAP_SCAN_COLS, 2048 x 8 of 32x32
			const uint32_t tx = (uint32_t)(flip ? 255 - x + s_snap->sx : x + s_snap->sx) & 0xffff;
			if ((tx >> 5) != last) {
				last = tx >> 5;
				const uint32_t offs = ((tx >> 5) * 8 + (ty >> 5)) * 2;
				const uint8_t attr = s_tilerom[(offs + 1) % s_tilerom_size];
				const uint32_t code = (uint32_t)(s_tilerom[offs % s_tilerom_size] + ((attr & 1) << 8)) % s_gtile.count;
				const uint32_t py = (attr & 0x80) ? 31 - (ty & 31) : ty & 31;
				src = s_gtile.pix + code * 1024 + py * 32;
				fx = (attr & 0x40) ? 31 : 0;
				base = (uint16_t)(32 * 4 + ((attr & 0x3c) >> 2) * 16);
			}
			d[x] = (uint16_t)(base + src[(tx & 31) ^ fx]);
		}
	}
}

static void draw_chars(int y0, int y1)
{
	const int flip = s_snap->flip;
	for (int y = y0; y < y1; y++) {
		uint16_t *d = rbx_video.bitmap + (y - VIS_Y0) * rbx_video.pitch;
		const uint32_t ty = (uint32_t)(flip ? 255 - y : y) & 255;
		uint32_t last = ~0u, cbase = 0;
		const uint8_t *src = NULL;
		for (int x = 0; x < VIS_W; x++) {
			const uint32_t tx = (uint32_t)(flip ? 255 - x : x) & 255;
			if ((tx >> 3) != last) {
				last = tx >> 3;
				const uint32_t idx = (ty >> 3) * 32 + (tx >> 3);
				const uint8_t c = s_snap->cram[idx];
				const uint32_t code = (uint32_t)(s_snap->vram[idx] + ((c & 0xe0) << 2)) % s_gchar.count;
				src = s_gchar.pix + code * 64 + (ty & 7) * 8;
				cbase = (c & 0x1f) * 4u;
			}
			const uint32_t pen = cbase + src[tx & 7];
			if (s_ctab[pen] != 0x4f)   // (configure_groups(gfx, 0x4f))
				d[x] = (uint16_t)pen;
		}
	}
}

// the frame in bands of lines drawn in parallel (decoband.h): the background, the sprites once, the text
#define BANDS 8
static int s_phase;
static void gs_band(int band)
{
	const int y0 = VIS_Y0 + VIS_H * band / BANDS, y1 = VIS_Y0 + VIS_H * (band + 1) / BANDS;
	if (s_phase == 0)
		draw_bg(y0, y1);
	else if (s_snap->chon)
		draw_chars(y0, y1);
}
static decoband_t s_bands = { 0, 0, 0, 0, BANDS, gs_band };
static int gs_help(int helper) { return decoband_help(&s_bands, helper); }

static void gs_render(void)
{
	memcpy(rbx_video.palette, s_pens, sizeof(s_pens));
	const int flip = s_snap->flip;
	s_phase = 0;
	decoband_run(&s_bands);
	if (s_snap->objon)
		for (int o = 0x1000 - 32; o >= 0; o -= 32) {
			const uint8_t *s = &s_snap->spr[o];
			const int attr = s[1];
			int bank = (attr & 0xc0) >> 6;
			if (bank == 3)
				bank += s_snap->spr3bank;
			int fx = 0, fy = (attr & 0x10) != 0;
			int sx = s[3] - ((attr & 0x20) << 3), sy = s[2];
			if (flip) {
				sx = 240 - sx;
				sy = 240 - sy;
				fx = !fx;
				fy = !fy;
			}
			cap8_transpen(&s_gspr, (uint32_t)(s[0] + 256 * bank), (uint16_t)(32 * 4 + 16 * 16 + (attr & 0x0f) * 16), fx, fy, sx, sy,
			              0, VIS_Y0, VIS_Y0 + VIS_H, VIS_Y0, 0, VIS_W);
		}
	s_phase = 1;
	decoband_run(&s_bands);
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap->spr, t->sprbuf, sizeof(t->sprbuf));
	memcpy(s_snap->vram, t->vram, sizeof(t->vram));
	memcpy(s_snap->cram, t->cram, sizeof(t->cram));
	s_snap->sx = (uint16_t)(t->scrollx[0] + 256 * t->scrollx[1]);
	s_snap->sy = t->scrolly;
	s_snap->flip = t->flip;
	s_snap->chon = t->chon;
	s_snap->bgon = t->bgon;
	s_snap->objon = t->objon;
	s_snap->spr3bank = t->spr3bank;
	rbx_video_frame_done();
}

static void z_setup(z80 *z, uint8_t (*rd)(void *, uint16_t), void (*wr)(void *, uint16_t, uint8_t), const uint8_t **map)
{
	z80_init(z);
	z->read_byte = rd;
	z->write_byte = wr;
	z->port_in = z_in;
	z->port_out = z_out;
	z->read_map = map;
}

static void gs_reset(void)
{
	memset(t, 0, offsetof(__typeof__(*t), state_end));
	map_bank();
	z_setup(&s_main, main_read, main_write, s_mmap);
	z_setup(&s_snd, snd_read, snd_write, s_zmap);
	rbx_ym_reset(0);
	rbx_ym_reset(1);
	s_mix_n = 0;
}

static void gs_exit(void);

static int gs_init(const rbx_game *g)
{
	free(t);
	t = calloc(1, sizeof(*t));
	if (!t) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	uint32_t s1, s2, s3, sp;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_zrom = rbx_region("audiocpu", &s_zrom_size);
	s_tilerom = rbx_region("bgtiles", &s_tilerom_size);
	uint8_t *chars = rbx_region("chars", &s1), *tiles = rbx_region("tiles", &s2), *spr = rbx_region("sprites", &s3);
	const uint8_t *prom = rbx_region("proms", &sp);
	if (!s_rom || s_rom_size < 0x18000 || !s_zrom || s_zrom_size < 0x8000 || !s_tilerom || !chars || s1 < 0x4000 || !tiles ||
	    s2 < 0x40000 || !spr || s3 < 0x40000 || !prom || sp < 0x800) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	{
		uint32_t rgb[256];
		for (int i = 0; i < 256; i++)
			rgb[i] = (uint32_t)((prom[i] & 15) * 17) << 16 | (uint32_t)((prom[256 + i] & 15) * 17) << 8 | (uint32_t)((prom[512 + i] & 15) * 17);
		const uint8_t *cp = prom + 0x300;
		for (int i = 0; i < 0x80; i++)
			s_ctab[i] = (uint8_t)(cp[i] | 0x40);
		for (int i = 0x100; i < 0x200; i++)
			s_ctab[i - 0x80] = (uint8_t)(cp[i] | ((cp[i + 0x100] & 3) << 4));
		for (int i = 0x300; i < 0x400; i++)
			s_ctab[i - 0x180] = (uint8_t)(cp[i] | ((cp[i + 0x100] & 7) << 4) | 0x80);
		for (int i = 0; i < NPENS; i++)
			s_pens[i] = rgb[s_ctab[i]];
		s_pens[PEN_BLACK] = 0;
	}
	{	// charlayout (reversed), tilelayout (32x32, 512), spritelayout (2048)
		static const uint32_t pc[2] = { 4, 0 }, xc[8] = { 11, 10, 9, 8, 3, 2, 1, 0 };
		static const uint32_t yc[8] = { 112, 96, 80, 64, 48, 32, 16, 0 };
		static uint32_t xt[32], yt[32], xs[16], ys[16];
		for (int i = 0; i < 32; i++) {
			xt[i] = (uint32_t)((i >> 3) * 512 + ((i >> 2) & 1) * 8 + (i & 3));
			yt[i] = (uint32_t)i * 16;
		}
		for (int i = 0; i < 16; i++) {
			xs[i] = (uint32_t)((i >> 3) * 256 + ((i >> 2) & 1) * 8 + (i & 3));
			ys[i] = (uint32_t)i * 16;
		}
		static const uint32_t pt[4] = { 512 * 256 * 8 + 4, 512 * 256 * 8 + 0, 4, 0 };
		static const uint32_t ps[4] = { 2048 * 64 * 8 + 4, 2048 * 64 * 8 + 0, 4, 0 };
		const rbx_gfx_layout lc = { 8, 8, 2, pc, xc, yc, 128 };
		const rbx_gfx_layout lt = { 32, 32, 4, pt, xt, yt, 2048 }, ls = { 16, 16, 4, ps, xs, ys, 512 };
		if (rbx_gfx_decode(&s_gchar, &lc, chars, 1024 * 16) || rbx_gfx_decode(&s_gtile, &lt, tiles, 512 * 256) ||
		    rbx_gfx_decode(&s_gspr, &ls, spr, 2048 * 64)) {
			rbx_fail("%s: out of memory (graphics)", g->set);
			gs_exit();
			return -1;
		}
	}
	s_snap = calloc(1, sizeof(*s_snap));
	rbx_ym_config yc;
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM2203;
	yc.clock = YM_CLOCK;
	yc.gain[0] = yc.gain[1] = yc.gain[2] = 56;   // (SSG 0.22, FM 0.14)
	yc.gain[3] = 36;
	if (!s_snap || rbx_ym_init(0, &yc) || rbx_ym_set_async(0, 1) || rbx_ym_init(1, &yc) || rbx_ym_set_async(1, 1)) {
		rbx_fail("%s: out of memory", g->set);
		gs_exit();
		return -1;
	}
	s_mix_cap = (int)(rbx_ym_rate(0) / 50) + 256;
	s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	if (!s_mix || rbx_audio_setup(rbx_ym_rate(0)) || rbx_video_setup(VIS_W, VIS_H, NPENS + 1)) {
		rbx_fail("%s: out of memory", g->set);
		gs_exit();
		return -1;
	}
	memset(s_mmap, 0, sizeof(s_mmap));
	memset(s_zmap, 0, sizeof(s_zmap));
	for (int p = 0; p < 0x80; p++) {
		s_mmap[p] = s_rom + p * 256;
		s_zmap[p] = s_zrom + p * 256;
	}
	static const char *const names[5] = { "SYSTEM", "P1", "P2", "DSW1", "DSW2" };
	for (int i = 0; i < 5; i++)
		s_port[i] = rbx_port_find(names[i]);
	gs_reset();
	return 0;
}

static void gs_exit(void)
{
	rbx_video_wait_idle();
	rbx_ym_exit(0);
	rbx_ym_exit(1);
	free(s_mix);
	s_mix = NULL;
	rbx_gfx_free(&s_gchar);
	rbx_gfx_free(&s_gtile);
	rbx_gfx_free(&s_gspr);
	free(s_snap);
	s_snap = NULL;
	free(t);
	t = NULL;
}

static void gs_frame(void)
{
	static int32_t tmp[1024];
	rbx_audio.frame_mhz = FRAME_MHZ;
	for (int line = 0; line < LINES; line++) {
		if (line == VIS_Y0 + VIS_H) {
			vblank_capture();
			s_main.int_data = 0xff;
			t->main_irq = 1;
		}
		t->snd_acc += 4;   // (set_periodic_int: 384 * 262 / 4 pixel clocks)
		if (t->snd_acc >= LINES) {
			t->snd_acc -= LINES;
			s_snd.int_data = 0xff;
			t->snd_irq = 1;
		}
		z80_run(&s_main, s_main.cyc + (unsigned long)rbx_line_cycles(&t->main_frac, CPU_CLOCK, FRAME_MHZ, LINES), &t->main_irq, 1);
		z80_run(&s_snd, s_snd.cyc + (unsigned long)rbx_line_cycles(&t->snd_frac, CPU_CLOCK, FRAME_MHZ, LINES), &t->snd_irq, 1);
		const uint32_t clocks = (uint32_t)rbx_line_cycles(&t->ym_frac, YM_CLOCK, FRAME_MHZ, LINES);
		s_mix_n += rbx_ym_advance(0, clocks, s_mix + s_mix_n, s_mix_cap - s_mix_n);
		rbx_ym_advance(1, clocks, tmp, 1024);
	}
	rbx_ym_async_take(0, s_mix, s_mix_n);
	rbx_ym_async_take(1, s_mix, s_mix_n);
	for (int i = 0; i < s_mix_n; i++)
		rbx_audio_put(s_mix[i]);
	memset(s_mix, 0, sizeof(int32_t) * (size_t)s_mix_n);
	s_mix_n = 0;
}

static int z_state(void *ctx, const char *tag, z80 *z, int load, uint8_t (*rd)(void *, uint16_t),
                   void (*wr)(void *, uint16_t, uint8_t), const uint8_t **map)
{
	z80 c = *z;
	const int rc = rbx_state_io(ctx, tag, &c, sizeof(c));
	if (!rc && load) {
		c.read_byte = rd;
		c.write_byte = wr;
		c.port_in = z_in;
		c.port_out = z_out;
		c.userdata = NULL;
		c.read_map = map;
		*z = c;
	}
	return rc;
}

static int gs_state(void *ctx, int load)
{
	int rc = 0;
	rc |= rbx_state_io(ctx, "gs", t, (uint32_t)offsetof(__typeof__(*t), state_end));
	rc |= z_state(ctx, "gs_main", &s_main, load, main_read, main_write, s_mmap);
	rc |= z_state(ctx, "gs_snd", &s_snd, load, snd_read, snd_write, s_zmap);
	rc |= rbx_ym_state(0, ctx, load);
	rc |= rbx_ym_state(1, ctx, load);
	if (!rc && load)
		map_bank();
	return rc ? -1 : 0;
}

static int gs_save(void *ctx) { return gs_state(ctx, 0); }
static int gs_load(void *ctx) { return gs_state(ctx, 1); }
static int gs_supported(const char *machine) { return !strcmp(machine, "gunsmoke"); }

const rbx_driver rbx_drv_gunsmoke = { "Capcom Gun.Smoke", gs_init, gs_exit, gs_reset, gs_frame, gs_save, gs_load,
                                      gs_supported, gs_render, gs_help };
