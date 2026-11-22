/*  Capcom 1943: The Battle of Midway (and 1943 Kai, Midway Kaisen). Ported from MAME src/mame/capcom/1943.cpp,
    1943_v.cpp, 1943.h (BSD-3-Clause, copyright-holders: Paul Leaman).
    Z80 at 6 MHz (IRQ at lines 144 and 240), Z80 sound CPU at 3 MHz (IRQ 4 times a frame) with two YM2203 at 1.5 MHz,
    an i8751 MCU at 3 MHz between the two (its INT1 on the vblank; not on the bootlegs). Video: two 65536x256 tilemaps
    of 32x32 tiles from ROM (the front one's pens drawn where their colour is not 0x0f), a 256x256 text layer, 16x16
    sprites from a buffered RAM, behind the front tiles where a PROM says; 256 colours through lookup PROMs.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/z80/z80.h"
#include "../../cpu/mcs51/mcs51.h"
#include "../../sound/rbx_ym.h"
#include "../../video/gfxlayout.h"
#include "cap8.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define FRAME_MHZ 59637          // set_raw(24 MHz / 4, 384, 0, 256, 262, 16, 240)
#define LINES 262
#define VIS_Y0 16
#define VIS_H 224
#define VIS_W 256
#define MAIN_CLOCK 6000000
#define SND_CLOCK 3000000
#define MCU_CYCLES 250000        // (3 MHz / 12)
#define YM_CLOCK 1500000
#define NPENS (32 * 4 + 16 * 16 * 3)
#define PEN_BLACK NPENS

static int s_has_mcu;
static uint8_t *s_rom, *s_zrom, *s_mrom, *s_tilerom, s_prom[0xc00];
static uint32_t s_rom_size, s_zrom_size, s_mrom_size, s_tilerom_size;
static uint32_t s_pens[NPENS + 1];
static uint8_t s_ctab[NPENS];
static rbx_gfx s_gchar, s_gbg, s_gbg2, s_gspr;
static int s_port[5];
static z80 s_main, s_snd;
static mcs51_t s_mcu;
static const uint8_t *s_mmap[256], *s_zmap[256];
static int32_t *s_mix;
static int s_mix_n, s_mix_cap;
static int s_line;

static struct {
	uint8_t ram[0x1000], spr[0x1000], sprbuf[0x1000], vram[0x400], cram[0x400];
	uint8_t zram[0x800];
	uint8_t scrollx[2], scrolly, bgscrollx[2];
	uint8_t bank, snd_reset, flip, char_on, bg1_on, bg2_on, obj_on, latch;
	uint8_t cpu_to_mcu, mcu_to_cpu, audiocpu_to_mcu, mcu_to_audiocpu, mcu_p0, mcu_p2, mcu_p3;
	int main_irq, snd_irq;
	uint64_t main_frac, snd_frac, mcu_frac, ym_frac;
	uint8_t state_end;
} *t;   // (calloc in init: ADR-040)

static struct {
	uint8_t spr[0x1000], vram[0x400], cram[0x400];
	uint16_t sx, sx2;
	uint8_t sy, flip, char_on, bg1_on, bg2_on, obj_on;
} *s_snap;

static uint8_t z_in(z80 *z, uint8_t port) { (void)z; (void)port; return 0xff; }
static void z_out(z80 *z, uint8_t port, uint8_t v) { (void)z; (void)port; (void)v; }

// ---- sound ----
static uint8_t snd_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x8000) return s_zrom[a];
	if (a >= 0xc000 && a < 0xc800) return t->zram[a - 0xc000];
	if (a == 0xc800) return t->latch;
	if (a == 0xd800) return t->mcu_to_audiocpu;
	return 0;
}
static void snd_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0xc000 && a < 0xc800) t->zram[a - 0xc000] = v;
	else if (a == 0xd800) t->audiocpu_to_mcu = v;
	else if (a == 0xe000 || a == 0xe001) rbx_ym_write(0, a & 1, v);
	else if (a == 0xe002 || a == 0xe003) rbx_ym_write(1, a & 1, v);
}

// ---- MCU ----
static uint8_t mcu_in(void *u, int port)
{
	(void)u;
	switch (port) {
	case 0: return t->cpu_to_mcu;
	case 1: return (uint8_t)s_line;   // (vpos)
	case 2: return t->audiocpu_to_mcu;
	}
	return 0xff;
}
static void mcu_out(void *u, int port, uint8_t v)
{
	(void)u;
	if (port == 0) t->mcu_p0 = v;
	else if (port == 2) t->mcu_p2 = v;
	else if (port == 3) {
		if ((t->mcu_p3 & 0x40) && !(v & 0x40)) {
			t->mcu_to_cpu = t->mcu_p0;
			t->mcu_to_audiocpu = t->mcu_p2;
		}
		t->mcu_p3 = v;
	}
}

// ---- main CPU ----
static void map_bank(void)
{
	for (int p = 0; p < 0x40; p++)
		s_mmap[0x80 + p] = s_rom + 0x10000 + ((uint32_t)t->bank * 0x4000 + p * 256) % (s_rom_size - 0x10000);
}

static void control_w(uint8_t v)
{
	t->bank = (v & 0x1c) >> 2;
	map_bank();
	if ((v & 0x20) && !t->snd_reset) {
		z80_init(&s_snd);
		s_snd.read_byte = snd_read;
		s_snd.write_byte = snd_write;
		s_snd.port_in = z_in;
		s_snd.port_out = z_out;
		s_snd.read_map = s_zmap;
		t->snd_irq = 0;
	}
	t->snd_reset = (v & 0x20) != 0;
	t->flip = (v & 0x40) != 0;
	t->char_on = (v & 0x80) != 0;
}

static uint8_t main_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x8000) return s_rom[a];
	if (a < 0xc000) return s_rom[0x10000 + ((uint32_t)t->bank * 0x4000 + (a - 0x8000)) % (s_rom_size - 0x10000)];
	if (a == 0xc000)   // (SYSTEM bit 3: vblank)
		rbx_port_custom(s_port[0], 0x08, (s_line < VIS_Y0 || s_line >= VIS_Y0 + VIS_H) ? 0x08u : 0);
	if (a >= 0xc000 && a <= 0xc004) return (uint8_t)rbx_port_read(s_port[a - 0xc000]);
	if (a == 0xc007) return s_has_mcu ? t->mcu_to_cpu : 0;
	if (a >= 0xd000 && a < 0xd400) return t->vram[a - 0xd000];
	if (a >= 0xd400 && a < 0xd800) return t->cram[a - 0xd400];
	if (a == 0xd800 || a == 0xd801) return t->scrollx[a & 1];
	if (a == 0xd802) return t->scrolly;
	if (a == 0xd803 || a == 0xd804) return t->bgscrollx[a - 0xd803];
	if (a >= 0xe000 && a < 0xf000) return t->ram[a - 0xe000];
	if (a >= 0xf000) return t->spr[a - 0xf000];
	return 0;
}
static void main_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0xe000 && a < 0xf000) { t->ram[a - 0xe000] = v; return; }
	if (a >= 0xf000) { t->spr[a - 0xf000] = v; return; }
	if (a >= 0xd000 && a < 0xd400) { t->vram[a - 0xd000] = v; return; }
	if (a >= 0xd400 && a < 0xd800) { t->cram[a - 0xd400] = v; return; }
	switch (a) {
	case 0xc800: t->latch = v; break;
	case 0xc804: control_w(v); break;
	case 0xc806: memcpy(t->sprbuf, t->spr, sizeof(t->spr)); break;   // (86S105 DMA)
	case 0xc807: if (s_has_mcu) t->cpu_to_mcu = v; break;
	case 0xd800: case 0xd801: t->scrollx[a & 1] = v; break;
	case 0xd802: t->scrolly = v; break;
	case 0xd803: case 0xd804: t->bgscrollx[a - 0xd803] = v; break;
	case 0xd806: t->bg1_on = (v & 0x10) != 0; t->bg2_on = (v & 0x20) != 0; t->obj_on = (v & 0x40) != 0; break;
	}
}

// ---- video ----
static uint8_t (*s_pri)[VIS_W];   // (calloc in init: ADR-040)

static void draw_bg(int which, int y, uint16_t *d, uint8_t *pri)
{
	const rbx_gfx *g = which ? &s_gbg2 : &s_gbg;
	const uint32_t base = which ? 0x180 : 0x80;
	const int flip = s_snap->flip;
	const uint32_t sx = which ? s_snap->sx2 : s_snap->sx, sy = which ? 0 : s_snap->sy;
	const uint32_t ty = (uint32_t)(flip ? 255 - y + (int)sy : y + (int)sy) & 255;
	for (int x = 0; x < VIS_W; x++) {
		const uint32_t tx = (uint32_t)(flip ? 255 - x + (int)sx : x + (int)sx) & 0xffff;
		const uint32_t idx = (tx >> 5) * 8 + (ty >> 5);   // (TILEMAP_SCAN_COLS, 2048 x 8)
		const uint32_t offs = (which ? 0x8000 : 0) + idx * 2;
		const uint8_t attr = s_tilerom[(offs + 1) % s_tilerom_size];
		const uint32_t code = (uint32_t)(s_tilerom[offs % s_tilerom_size] + (which ? 0 : (attr & 1) << 8)) % g->count;
		const uint32_t color = (attr & 0x3c) >> 2;
		const uint32_t px = (attr & 0x40) ? 31 - (tx & 31) : tx & 31, py = (attr & 0x80) ? 31 - (ty & 31) : ty & 31;
		const uint8_t p = g->pix[code * 1024 + py * 32 + px];
		const uint32_t pen = base + color * 16 + p;
		if (which) {
			d[x] = (uint16_t)pen;
			pri[x] = 1;
		} else if (s_ctab[pen] != 0x0f) {   // (configure_groups(gfx, 0x0f))
			d[x] = (uint16_t)pen;
			pri[x] = 2;
		}
	}
}

static void c43_render(void)
{
	memcpy(rbx_video.palette, s_pens, sizeof(s_pens));
	const int flip = s_snap->flip;
	for (int y = VIS_Y0; y < VIS_Y0 + VIS_H; y++) {
		uint16_t *d = rbx_video.bitmap + (y - VIS_Y0) * rbx_video.pitch;
		uint8_t *pri = s_pri[y - VIS_Y0];
		memset(pri, 0, VIS_W);
		if (s_snap->bg2_on)
			draw_bg(1, y, d, pri);
		else
			for (int x = 0; x < VIS_W; x++)
				d[x] = PEN_BLACK;
		if (s_snap->bg1_on)
			draw_bg(0, y, d, pri);
	}
	if (s_snap->obj_on)   // (_1943_drawgfx: the first sprite drawn at a pixel wins)
		for (int o = 0; o < 0x1000; o += 32) {
			const uint8_t *s = &s_snap->spr[o];
			const uint8_t attr = s[1];
			const uint32_t code = (uint32_t)(s[0] + ((attr & 0xe0) << 3)) % s_gspr.count;
			const uint32_t color = attr & 0x0f;
			int sx = s[3] - ((attr & 0x10) << 4), sy = s[2];
			if (flip) {
				sx = 240 - sx;
				sy = 240 - sy;
			}
			const uint8_t *src = s_gspr.pix + code * 256;
			for (int j = 0; j < 16; j++) {
				const int yy = sy + j;
				if (yy < VIS_Y0 || yy >= VIS_Y0 + VIS_H)
					continue;
				const uint8_t *row = src + (flip ? 15 - j : j) * 16;
				uint16_t *d = rbx_video.bitmap + (yy - VIS_Y0) * rbx_video.pitch;
				uint8_t *pri = s_pri[yy - VIS_Y0];
				for (int i = 0; i < 16; i++) {
					const int xx = sx + i;
					if ((unsigned)xx >= VIS_W || (pri[xx] & 0x80))
						continue;
					const uint8_t c = row[flip ? 15 - i : i];
					if (!c)
						continue;
					if (!(pri[xx] & 2) || !(s_prom[0x900 + c + (color << 4)] & 0x08))
						d[xx] = (uint16_t)(0x280 + color * 16 + c);
					pri[xx] = 0xff;
				}
			}
		}
	if (s_snap->char_on)
		for (int y = VIS_Y0; y < VIS_Y0 + VIS_H; y++) {
			uint16_t *d = rbx_video.bitmap + (y - VIS_Y0) * rbx_video.pitch;
			const uint32_t ty = (uint32_t)(flip ? 255 - y : y) & 255;
			for (int x = 0; x < VIS_W; x++) {
				const uint32_t tx = (uint32_t)(flip ? 255 - x : x) & 255;
				const uint32_t idx = (ty >> 3) * 32 + (tx >> 3);
				const uint8_t c = s_snap->cram[idx];
				const uint32_t code = (uint32_t)(s_snap->vram[idx] + ((c & 0xe0) << 3)) % s_gchar.count;
				const uint8_t p = s_gchar.pix[code * 64 + (ty & 7) * 8 + (tx & 7)];
				if (p)
					d[x] = (uint16_t)((c & 0x1f) * 4 + p);
			}
		}
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap->spr, t->sprbuf, sizeof(t->sprbuf));
	memcpy(s_snap->vram, t->vram, sizeof(t->vram));
	memcpy(s_snap->cram, t->cram, sizeof(t->cram));
	s_snap->sx = (uint16_t)(t->scrollx[0] + 256 * t->scrollx[1]);
	s_snap->sx2 = (uint16_t)(t->bgscrollx[0] + 256 * t->bgscrollx[1]);
	s_snap->sy = t->scrolly;
	s_snap->flip = t->flip;
	s_snap->char_on = t->char_on;
	s_snap->bg1_on = t->bg1_on;
	s_snap->bg2_on = t->bg2_on;
	s_snap->obj_on = t->obj_on;
	rbx_video_frame_done();
}

// ---- init / frame ----
static void z_setup(z80 *z, uint8_t (*rd)(void *, uint16_t), void (*wr)(void *, uint16_t, uint8_t), const uint8_t **map)
{
	z80_init(z);
	z->read_byte = rd;
	z->write_byte = wr;
	z->port_in = z_in;
	z->port_out = z_out;
	z->read_map = map;
}

static void c43_reset(void)
{
	memset(t, 0, offsetof(__typeof__(*t), state_end));
	z_setup(&s_main, main_read, main_write, s_mmap);
	z_setup(&s_snd, snd_read, snd_write, s_zmap);
	control_w(0);   // (machine_reset: control_w(0), layer_w(0))
	if (s_has_mcu) {
		mcs51_init(&s_mcu, s_mrom, s_mrom_size);
		s_mcu.port_in = mcu_in;
		s_mcu.port_out = mcu_out;
		mcs51_reset(&s_mcu);
	}
	rbx_ym_reset(0);
	rbx_ym_reset(1);
	s_mix_n = 0;
	s_line = 0;
}

static void c43_exit(void);

static int c43_init(const rbx_game *g)
{
	free(s_pri);
	s_pri = (uint8_t (*)[VIS_W])calloc(VIS_H, VIS_W);
	if (!s_pri) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	free(t);
	t = calloc(1, sizeof(*t));
	if (!t) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	const char *m = g->machine;
	s_has_mcu = !strncmp(m, "_1943:", 6) || !strcmp(m, "_1943");
	uint32_t s1, s2, s3, s4, sp;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_zrom = rbx_region("audiocpu", &s_zrom_size);
	s_mrom = s_has_mcu ? rbx_region("mcu", &s_mrom_size) : NULL;
	s_tilerom = rbx_region("tilerom", &s_tilerom_size);
	uint8_t *chars = rbx_region("gfx1", &s1), *bg = rbx_region("gfx2", &s2), *bg2 = rbx_region("gfx3", &s3), *spr = rbx_region("gfx4", &s4);
	const uint8_t *prom = rbx_region("proms", &sp);
	if (!s_rom || s_rom_size < 0x30000 || !s_zrom || s_zrom_size < 0x8000 || (s_has_mcu && !s_mrom) || !s_tilerom ||
	    s_tilerom_size < 0x10000 || !chars || !bg || !bg2 || !spr || !prom || sp < 0xc00) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	memcpy(s_prom, prom, 0xc00);
	{	// _1943_palette
		uint32_t rgb[256];
		for (int i = 0; i < 256; i++)
			rgb[i] = (uint32_t)(cap8_res4(prom[i]) << 16 | cap8_res4(prom[256 + i]) << 8 | cap8_res4(prom[512 + i]));
		const uint8_t *cp = prom + 0x300;
		for (int i = 0; i < 0x80; i++)
			s_ctab[i] = (uint8_t)((cp[i] & 0x0f) | 0x40);
		for (int i = 0x80; i < 0x180; i++)
			s_ctab[i] = (uint8_t)(((cp[0x200 + (i - 0x80)] & 3) << 4) | (cp[0x100 + (i - 0x80)] & 0x0f));
		for (int i = 0x180; i < 0x280; i++)
			s_ctab[i] = (uint8_t)(((cp[0x400 + (i - 0x180)] & 3) << 4) | (cp[0x300 + (i - 0x180)] & 0x0f));
		for (int i = 0x280; i < 0x380; i++)
			s_ctab[i] = (uint8_t)(((cp[0x600 + (i - 0x280)] & 7) << 4) | (cp[0x500 + (i - 0x280)] & 0x0f) | 0x80);
		for (int i = 0; i < NPENS; i++)
			s_pens[i] = rgb[s_ctab[i]];
		s_pens[PEN_BLACK] = 0;
	}
	{	// charlayout, tilelayout (32x32, RGN_FRAC(1,2)), spritelayout (RGN_FRAC(1,2))
		static const uint32_t pc[2] = { 4, 0 }, xc[8] = { 0, 1, 2, 3, 8, 9, 10, 11 };
		static const uint32_t yc[8] = { 0, 16, 32, 48, 64, 80, 96, 112 };
		static uint32_t xt[32], yt[32], xs[16], ys[16];
		for (int i = 0; i < 32; i++) {
			xt[i] = (uint32_t)((i >> 3) * 512 + ((i >> 2) & 1) * 8 + (i & 3));
			yt[i] = (uint32_t)i * 16;
		}
		for (int i = 0; i < 16; i++) {
			xs[i] = (uint32_t)((i >> 3) * 256 + ((i >> 2) & 1) * 8 + (i & 3));
			ys[i] = (uint32_t)i * 16;
		}
		const uint32_t h2 = s2 / 2 * 8, h3 = s3 / 2 * 8, h4 = s4 / 2 * 8;
		const uint32_t p2[4] = { h2 + 4, h2, 4, 0 }, p3[4] = { h3 + 4, h3, 4, 0 }, p4[4] = { h4 + 4, h4, 4, 0 };
		const rbx_gfx_layout lc = { 8, 8, 2, pc, xc, yc, 128 };
		const rbx_gfx_layout l2 = { 32, 32, 4, p2, xt, yt, 2048 }, l3 = { 32, 32, 4, p3, xt, yt, 2048 };
		const rbx_gfx_layout l4 = { 16, 16, 4, p4, xs, ys, 512 };
		if (rbx_gfx_decode(&s_gchar, &lc, chars, s1) || rbx_gfx_decode(&s_gbg, &l2, bg, s2 / 2) ||
		    rbx_gfx_decode(&s_gbg2, &l3, bg2, s3 / 2) || rbx_gfx_decode(&s_gspr, &l4, spr, s4 / 2)) {
			rbx_fail("%s: out of memory (graphics)", g->set);
			c43_exit();
			return -1;
		}
	}
	s_snap = calloc(1, sizeof(*s_snap));
	rbx_ym_config yc;
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM2203;
	yc.clock = YM_CLOCK;
	yc.gain[0] = yc.gain[1] = yc.gain[2] = 38;   // (SSG 0.15, FM 0.10)
	yc.gain[3] = 26;
	int rc = !s_snap || rbx_ym_init(0, &yc) || rbx_ym_set_async(0, 1) || rbx_ym_init(1, &yc) || rbx_ym_set_async(1, 1);
	if (rc) {
		rbx_fail("%s: out of memory", g->set);
		c43_exit();
		return -1;
	}
	s_mix_cap = (int)(rbx_ym_rate(0) / 50) + 256;
	s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	if (!s_mix || rbx_audio_setup(rbx_ym_rate(0)) || rbx_video_setup(VIS_W, VIS_H, NPENS + 1)) {
		rbx_fail("%s: out of memory", g->set);
		c43_exit();
		return -1;
	}
	memset(s_mmap, 0, sizeof(s_mmap));
	memset(s_zmap, 0, sizeof(s_zmap));
	for (int p = 0; p < 0x80; p++) {
		s_mmap[p] = s_rom + p * 256;
		s_zmap[p] = s_zrom + p * 256;
	}
	static const char *const names[5] = { "SYSTEM", "P1", "P2", "DSWA", "DSWB" };
	for (int i = 0; i < 5; i++)
		s_port[i] = rbx_port_find(names[i]);
	c43_reset();
	return 0;
}

static void c43_exit(void)
{
	rbx_video_wait_idle();
	rbx_ym_exit(0);
	rbx_ym_exit(1);
	free(s_mix);
	s_mix = NULL;
	rbx_gfx_free(&s_gchar);
	rbx_gfx_free(&s_gbg);
	rbx_gfx_free(&s_gbg2);
	rbx_gfx_free(&s_gspr);
	free(s_snap);
	s_snap = NULL;
	free(s_pri);
	s_pri = NULL;
	free(t);
	t = NULL;
}

static void c43_frame(void)
{
	static int32_t tmp[1024];
	rbx_audio.frame_mhz = FRAME_MHZ;
	for (int line = 0; line < LINES; line++) {
		s_line = line;
		if (line == VIS_Y0 + VIS_H) {
			vblank_capture();
			if (s_has_mcu)
				mcs51_set_input(&s_mcu, MCS51_INT1_LINE, 1);
		}
		if (line == VIS_Y0 && s_has_mcu)
			mcs51_set_input(&s_mcu, MCS51_INT1_LINE, 0);
		if (line % 16 == 0) {   // (scanline: every 16 lines)
			if (line == 144 || line == 240) {
				s_main.int_data = 0xff;
				t->main_irq = 1;
			}
			if (line % 64 == 32 && !t->snd_reset) {
				s_snd.int_data = 0xff;
				t->snd_irq = 1;
			}
		}
		z80_run(&s_main, s_main.cyc + (unsigned long)rbx_line_cycles(&t->main_frac, MAIN_CLOCK, FRAME_MHZ, LINES), &t->main_irq, 1);
		const int cs = rbx_line_cycles(&t->snd_frac, SND_CLOCK, FRAME_MHZ, LINES);
		if (t->snd_reset)
			s_snd.cyc += (unsigned long)cs;
		else
			z80_run(&s_snd, s_snd.cyc + (unsigned long)cs, &t->snd_irq, 1);
		if (s_has_mcu)
			mcs51_execute(&s_mcu, rbx_line_cycles(&t->mcu_frac, MCU_CYCLES, FRAME_MHZ, LINES));
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

static int c43_state(void *ctx, int load)
{
	int rc = 0;
	rc |= rbx_state_io(ctx, "c43", t, (uint32_t)offsetof(__typeof__(*t), state_end));
	rc |= z_state(ctx, "c43_main", &s_main, load, main_read, main_write, s_mmap);
	rc |= z_state(ctx, "c43_snd", &s_snd, load, snd_read, snd_write, s_zmap);
	if (s_has_mcu)
		rc |= rbx_state_io(ctx, "c43_mcu", &s_mcu.st, sizeof(s_mcu.st));
	rc |= rbx_ym_state(0, ctx, load);
	rc |= rbx_ym_state(1, ctx, load);
	if (!rc && load)
		map_bank();
	return rc ? -1 : 0;
}

static int c43_save(void *ctx) { return c43_state(ctx, 0); }
static int c43_load(void *ctx) { return c43_state(ctx, 1); }
static int c43_supported(const char *machine) { return !strncmp(machine, "_1943", 5); }

const rbx_driver rbx_drv_c1943 = { "Capcom 1943", c43_init, c43_exit, c43_reset, c43_frame, c43_save, c43_load,
                                   c43_supported, c43_render, NULL };
