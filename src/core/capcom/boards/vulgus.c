/*  Capcom Vulgus (and Mach-9). Ported from MAME src/mame/capcom/vulgus.cpp (BSD-3-Clause, copyright-holders:
    Mirko Buffoni).
    Z80 at 3 MHz and a Z80 sound CPU at 3 MHz with two AY-8910 at 1.5 MHz; the interrupts from a PROM read by the
    scanline counter. Video: a 512x512 tilemap of 16x16 (3 bpp, four palette banks), a 256x256 text layer (pens whose
    colour is 47 clear), 16x16 sprites in columns of 1, 2, 4 or 16; 256 colours through lookup PROMs.
    1942iti (MAME: not working) is left out.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/z80/z80.h"
#include "../../sound/ay8910.h"
#include "../../video/gfxlayout.h"
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
#define AY_CLOCK 1500000
#define NPENS (64 * 4 + 16 * 16 + 4 * 32 * 8)

static uint8_t *s_rom, *s_zrom, s_irqprom[0x100];
static uint32_t s_rom_size, s_zrom_size;
static uint32_t s_pens[NPENS];
static uint8_t s_ctab[NPENS];
static rbx_gfx s_gchar, s_gtile, s_gspr;
static int s_port[5];
static z80 s_main, s_snd;
static ay8910_t s_ay[2];
static const uint8_t *s_mmap[256], *s_zmap[256];
static int32_t s_mix[2048];
static int s_mix_n;

static struct {
	uint8_t ram[0x1000], spr[0x80], fg[0x800], bg[0x800];
	uint8_t zram[0x800];
	uint8_t latch[2], scroll_low[2], scroll_high[2], palbank, flip;
	int main_irq, snd_irq;
	uint64_t main_frac, snd_frac, ay_frac;
	uint8_t state_end;
} t;

static struct {
	uint8_t spr[0x80], fg[0x800], bg[0x800];
	uint16_t sx, sy;
	uint8_t palbank, flip;
} *s_snap;

static uint8_t z_in(z80 *z, uint8_t port) { (void)z; (void)port; return 0xff; }
static void z_out(z80 *z, uint8_t port, uint8_t v) { (void)z; (void)port; (void)v; }

static uint8_t snd_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x4000) return s_zrom[a];
	if (a >= 0x4000 && a < 0x4800) return t.zram[a - 0x4000];
	if (a == 0x6000 || a == 0x6001) return t.latch[a & 1];
	return 0;
}
static void snd_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0x4000 && a < 0x4800) t.zram[a - 0x4000] = v;
	else if (a == 0x8000 || a == 0x8001) ay8910_write(&s_ay[0], a & 1, v);
	else if (a == 0xc000 || a == 0xc001) ay8910_write(&s_ay[1], a & 1, v);
}

static uint8_t main_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0xa000) return a < s_rom_size ? s_rom[a] : 0xff;
	if (a >= 0xc000 && a <= 0xc004) return (uint8_t)rbx_port_read(s_port[a - 0xc000]);
	if (a == 0xc802 || a == 0xc803) return t.scroll_low[a & 1];
	if (a == 0xc902 || a == 0xc903) return t.scroll_high[a & 1];
	if (a >= 0xd000 && a < 0xd800) return t.fg[a - 0xd000];
	if (a >= 0xd800 && a < 0xe000) return t.bg[a - 0xd800];
	if (a >= 0xe000 && a < 0xf000) return t.ram[a - 0xe000];
	return 0xff;   // (unmap_value_high)
}
static void main_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0xe000 && a < 0xf000) t.ram[a - 0xe000] = v;
	else if (a >= 0xd000 && a < 0xd800) t.fg[a - 0xd000] = v;
	else if (a >= 0xd800 && a < 0xe000) t.bg[a - 0xd800] = v;
	else if (a >= 0xcc00 && a < 0xcc80) t.spr[a - 0xcc00] = v;
	else switch (a) {
	case 0xc800: case 0xc801: t.latch[a & 1] = v; break;
	case 0xc802: case 0xc803: t.scroll_low[a & 1] = v; break;
	case 0xc902: case 0xc903: t.scroll_high[a & 1] = v; break;
	case 0xc804: t.flip = (v & 0x80) != 0; break;
	case 0xc805: t.palbank = v & 3; break;
	}
}

static void vg_render(void)
{
	memcpy(rbx_video.palette, s_pens, sizeof(s_pens));
	const int flip = s_snap->flip;
	for (int y = VIS_Y0; y < VIS_Y0 + VIS_H; y++) {
		uint16_t *d = rbx_video.bitmap + (y - VIS_Y0) * rbx_video.pitch;
		const uint32_t ty = (uint32_t)(flip ? 255 - y + s_snap->sy : y + s_snap->sy) & 511;
		uint32_t last = 0xffffffffu, cbase = 0, hflip = 0;
		const uint8_t *trow = NULL;
		for (int x = 0; x < VIS_W; x++) {   // bg: TILEMAP_SCAN_COLS 32 x 32 of 16x16
			const uint32_t tx = (uint32_t)(flip ? 255 - x + s_snap->sx : x + s_snap->sx) & 511;
			if ((tx >> 4) != last) {   // (a tile looked up once for its 16 pixels)
				last = tx >> 4;
				const uint32_t idx = (tx >> 4) * 32 + (ty >> 4);
				const uint8_t c = s_snap->bg[idx + 0x400];
				const uint32_t code = rbx_wrap((uint32_t)(s_snap->bg[idx] + ((c & 0x80) << 1)), s_gtile.count);
				const uint32_t py = (c & 0x40) ? 15 - (ty & 15) : ty & 15;
				trow = s_gtile.pix + code * 256 + py * 16;
				hflip = c & 0x20;
				cbase = (uint32_t)(64 * 4 + 16 * 16 + ((c & 0x1f) + 0x20 * s_snap->palbank) * 8);
			}
			d[x] = (uint16_t)(cbase + trow[hflip ? 15 - (tx & 15) : tx & 15]);
		}
	}
	for (int o = 0x80 - 4; o >= 0; o -= 4) {
		const uint8_t *s = &s_snap->spr[o];
		int code = (s[0] & 0x7f) | (((s[1] >> 5) & 1) << 7) | (((s[0] >> 7) & 1) << 8);
		const int color = s[1] & 0x0f;
		int sy = s[2], sx = s[3] - (((s[1] >> 4) & 1) << 8), dir = 1;
		if (flip) {
			sx = 240 - sx;
			sy = 240 - sy;
			dir = -1;
		}
		const int size = (s[1] & 0xc0) >> 6, row = size == 3 ? 16 : (1 << size);
		code &= ~(row - 1);
		for (int i = 0; i < row; i++)
			cap8_transpen(&s_gspr, (uint32_t)(code + i), (uint16_t)(64 * 4 + color * 16), flip, flip, sx, sy + 16 * i * dir, 15,
			              VIS_Y0, VIS_Y0 + VIS_H, VIS_Y0, 0, VIS_W);
	}
	for (int y = VIS_Y0; y < VIS_Y0 + VIS_H; y++) {
		uint16_t *d = rbx_video.bitmap + (y - VIS_Y0) * rbx_video.pitch;
		const uint32_t ty = (uint32_t)(flip ? 255 - y : y) & 255;
		uint32_t last = 0xffffffffu, cbase = 0;
		const uint8_t *trow = NULL;
		for (int x = 0; x < VIS_W; x++) {
			const uint32_t tx = (uint32_t)(flip ? 255 - x : x) & 255;
			if ((tx >> 3) != last) {   // (a character looked up once for its 8 pixels)
				last = tx >> 3;
				const uint32_t idx = (ty >> 3) * 32 + (tx >> 3);
				const uint8_t c = s_snap->fg[idx + 0x400];
				const uint32_t code = rbx_wrap((uint32_t)(s_snap->fg[idx] + ((c & 0x80) << 1)), s_gchar.count);
				trow = s_gchar.pix + code * 64 + (ty & 7) * 8;
				cbase = (uint32_t)(c & 0x3f) * 4;
			}
			const uint32_t pen = cbase + trow[tx & 7];
			if (s_ctab[pen] != 47)   // (configure_groups(gfx, 47))
				d[x] = (uint16_t)pen;
		}
	}
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap->spr, t.spr, sizeof(t.spr));
	memcpy(s_snap->fg, t.fg, sizeof(t.fg));
	memcpy(s_snap->bg, t.bg, sizeof(t.bg));
	s_snap->sx = (uint16_t)(t.scroll_low[1] + 256 * t.scroll_high[1]);
	s_snap->sy = (uint16_t)(t.scroll_low[0] + 256 * t.scroll_high[0]);
	s_snap->palbank = t.palbank;
	s_snap->flip = t.flip;
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

static void vg_reset(void)
{
	memset(&t, 0, offsetof(__typeof__(t), state_end));
	z_setup(&s_main, main_read, main_write, s_mmap);
	z_setup(&s_snd, snd_read, snd_write, s_zmap);
	for (int i = 0; i < 2; i++)
		ay8910_reset(&s_ay[i]);
	s_mix_n = 0;
}

static void vg_exit(void);

static int vg_init(const rbx_game *g)
{
	uint32_t s1, s2, s3, sp, si;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_zrom = rbx_region("audiocpu", &s_zrom_size);
	uint8_t *chars = rbx_region("chars", &s1), *tiles = rbx_region("tiles", &s2), *spr = rbx_region("sprites", &s3);
	const uint8_t *prom = rbx_region("proms", &sp), *irq = rbx_region("irqprom", &si);
	if (!s_rom || s_rom_size < 0xa000 || !s_zrom || s_zrom_size < 0x4000 || !chars || !tiles || !spr || !prom || sp < 0x600 ||
	    !irq || si < 0x100) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	memcpy(s_irqprom, irq, 0x100);
	{	// palette: 256 colours, the lookup PROMs (chars 32 + n, sprites 16 + n, tiles n + 64 x bank)
		uint32_t rgb[256];
		for (int i = 0; i < 256; i++)
			rgb[i] = (uint32_t)(cap8_res4(prom[i]) << 16 | cap8_res4(prom[256 + i]) << 8 | cap8_res4(prom[512 + i]));
		const uint8_t *cp = prom + 0x300;
		for (int i = 0; i < 64 * 4; i++)
			s_ctab[i] = (uint8_t)(32 + *cp++);
		for (int i = 0; i < 16 * 16; i++)
			s_ctab[64 * 4 + i] = (uint8_t)(16 + *cp++);
		for (int i = 0; i < 32 * 8; i++, cp++)
			for (int b = 0; b < 4; b++)
				s_ctab[64 * 4 + 16 * 16 + b * 32 * 8 + i] = (uint8_t)(*cp + 64 * b);
		for (int i = 0; i < NPENS; i++)
			s_pens[i] = rgb[s_ctab[i]];
	}
	{	// charlayout, tilelayout (RGN_FRAC(1,3)), spritelayout (RGN_FRAC(1,2))
		static const uint32_t pc[2] = { 4, 0 }, xc[8] = { 0, 1, 2, 3, 8, 9, 10, 11 };
		static const uint32_t yc[8] = { 0, 16, 32, 48, 64, 80, 96, 112 };
		static const uint32_t xt[16] = { 0, 1, 2, 3, 4, 5, 6, 7, 128, 129, 130, 131, 132, 133, 134, 135 };
		static const uint32_t xs[16] = { 0, 1, 2, 3, 8, 9, 10, 11, 256, 257, 258, 259, 264, 265, 266, 267 };
		static uint32_t yt[16], ys[16];
		for (int i = 0; i < 16; i++) {
			yt[i] = (uint32_t)i * 8;
			ys[i] = (uint32_t)i * 16;
		}
		const uint32_t f3 = s2 / 3 * 8, f2 = s3 / 2 * 8;
		const uint32_t pt[3] = { 0, f3, 2 * f3 }, ps[4] = { f2 + 4, f2, 4, 0 };
		const rbx_gfx_layout lc = { 8, 8, 2, pc, xc, yc, 128 };
		const rbx_gfx_layout lt = { 16, 16, 3, pt, xt, yt, 256 }, ls = { 16, 16, 4, ps, xs, ys, 512 };
		if (rbx_gfx_decode(&s_gchar, &lc, chars, s1) || rbx_gfx_decode(&s_gtile, &lt, tiles, s2 / 3) ||
		    rbx_gfx_decode(&s_gspr, &ls, spr, s3 / 2)) {
			rbx_fail("%s: out of memory (graphics)", g->set);
			vg_exit();
			return -1;
		}
	}
	s_snap = calloc(1, sizeof(*s_snap));
	for (int i = 0; i < 2; i++)
		ay8910_init(&s_ay[i], AY8910_TYPE_AY8910, AY_CLOCK, AY8910_LEGACY_OUTPUT, 64, 4, 0);   // (0.25)
	if (!s_snap || rbx_audio_setup(ay8910_rate(&s_ay[0])) || rbx_video_setup(VIS_W, VIS_H, NPENS)) {
		rbx_fail("%s: out of memory", g->set);
		vg_exit();
		return -1;
	}
	memset(s_mmap, 0, sizeof(s_mmap));
	memset(s_zmap, 0, sizeof(s_zmap));
	for (int p = 0; p < 0xa0 && (uint32_t)p * 256 + 256 <= s_rom_size; p++)
		s_mmap[p] = s_rom + p * 256;
	for (int p = 0; p < 0x40; p++)
		s_zmap[p] = s_zrom + p * 256;
	static const char *const names[5] = { "SYSTEM", "P1", "P2", "DSW1", "DSW2" };
	for (int i = 0; i < 5; i++)
		s_port[i] = rbx_port_find(names[i]);
	vg_reset();
	return 0;
}

static void vg_exit(void)
{
	rbx_video_wait_idle();
	rbx_gfx_free(&s_gchar);
	rbx_gfx_free(&s_gtile);
	rbx_gfx_free(&s_gspr);
	free(s_snap);
	s_snap = NULL;
}

static void vg_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	for (int line = 0; line < LINES; line++) {
		if (line == VIS_Y0 + VIS_H)
			vblank_capture();
		const uint8_t irq = s_irqprom[line & 0xff];
		if (irq & 8) {
			s_main.int_data = (uint8_t)(0xc7 | ((irq << 3) & 0x18));
			t.main_irq = 1;
		}
		if (irq & 4) {
			s_snd.int_data = 0xff;
			t.snd_irq = 1;
		}
		z80_run(&s_main, s_main.cyc + (unsigned long)rbx_line_cycles(&t.main_frac, CPU_CLOCK, FRAME_MHZ, LINES), &t.main_irq, 1);
		z80_run(&s_snd, s_snd.cyc + (unsigned long)rbx_line_cycles(&t.snd_frac, CPU_CLOCK, FRAME_MHZ, LINES), &t.snd_irq, 1);
		const uint32_t ac = (uint32_t)rbx_line_cycles(&t.ay_frac, AY_CLOCK, FRAME_MHZ, LINES);
		const int room = 2048 - s_mix_n;
		ay8910_render(&s_ay[0], ac, s_mix + s_mix_n, room);
		s_mix_n += ay8910_render(&s_ay[1], ac, s_mix + s_mix_n, room);
	}
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

static int vg_state(void *ctx, int load)
{
	int rc = 0;
	rc |= rbx_state_io(ctx, "vg", &t, (uint32_t)offsetof(__typeof__(t), state_end));
	rc |= z_state(ctx, "vg_main", &s_main, load, main_read, main_write, s_mmap);
	rc |= z_state(ctx, "vg_snd", &s_snd, load, snd_read, snd_write, s_zmap);
	rc |= rbx_state_io(ctx, "vg_ay0", &s_ay[0], ay8910_state_size());
	rc |= rbx_state_io(ctx, "vg_ay1", &s_ay[1], ay8910_state_size());
	return rc ? -1 : 0;
}

static int vg_save(void *ctx) { return vg_state(ctx, 0); }
static int vg_load(void *ctx) { return vg_state(ctx, 1); }
static int vg_supported(const char *machine) { return !strcmp(machine, "vulgus"); }

const rbx_driver rbx_drv_vulgus = { "Capcom Vulgus", vg_init, vg_exit, vg_reset, vg_frame, vg_save, vg_load,
                                    vg_supported, vg_render, NULL };
