/*  Technos U.S. Championship V'ball. Ported from MAME src/mame/technos/vball.cpp (BSD-3-Clause,
    copyright-holders: Paul Hampson).
    M6502 at 2 MHz (NMI at vcount F8, IRQ every 16 lines), Z80 sound CPU at 3.58 MHz with a YM2151 and an OKI6295.
    Video: a 512x512 tilemap of 8x8 with 32 rows of horizontal scroll (taken from the scanline counter, MAME's
    m_scrollx[255 - scanline]), 16x16 sprites; colours from PROM banks. Drawn line by line (update_partial on every
    scanline): scroll latched per line, the video RAM by pieces.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m6502/m6502.h"
#include "../../cpu/z80/z80.h"
#include "../../sound/okim6295.h"
#include "../../sound/rbx_ym.h"
#include "../../video/gfxlayout.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define FRAME_MHZ 57445          // set_raw(12 MHz / 2, 384, 0, 256, 272, 8, 248)
#define LINES 272
#define VIS_Y0 8
#define VIS_H 240
#define VIS_W 256
#define MAIN_CLOCK 2000000
#define Z80_CLOCK 3579545
#define YM_CLOCK 3579545

typedef struct {
	uint16_t sx, sy;
	uint8_t flip, gfxset;
} line_regs;

#define MAXSEG 32
typedef struct {
	int y0;
	uint8_t vram[0x1000], attr[0x1000], spr[0x100];
} vseg;
typedef struct {
	int n;
	vseg seg[MAXSEG];
} vbuf;

static uint8_t *s_rom, *s_zrom;
static uint32_t s_rom_size, s_zrom_size;
static uint8_t s_prom[0x1000];
static rbx_gfx s_gchar, s_gspr;
static int s_port[7];
static m6502_t s_main;
static z80 s_z80;
static okim6295_t s_oki;
static const uint8_t *s_zmap[256];
static int32_t *s_mix;
static int s_mix_n, s_mix_cap;
static int s_line;
static vbuf *s_vb[2];
static int s_vb_cur, s_vdirty;

static struct {
	uint8_t ram[0x800], spr[0x100], vram[0x1000], attr[0x1000];
	uint8_t zram[0x800];
	uint16_t scrollx_hi, scrolly_hi;
	uint8_t scrollx_lo, scrolly_lo, gfxset, flip, bank, bgbank, spbank;
	uint16_t scrollx[256];
	uint8_t latch, latch_pending;
	int32_t ym_irq;
	int64_t main_t;
	uint64_t main_frac, z80_frac, ym_frac;
	uint32_t oki_acc;
	int32_t oki_hold;
	line_regs lines[VIS_H];
	uint8_t state_end;
} t;

static struct {
	line_regs lines[VIS_H];
	uint8_t bgbank, spbank;
	const vbuf *vb;
} *s_snap;

// ---- sound ----
static void ym_irq(int state) { t.ym_irq = state; }

static uint8_t z_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x8000) return s_zrom[rbx_wrap(a, s_zrom_size)];
	if (a < 0x8800) return t.zram[a - 0x8000];
	if (a == 0x8800 || a == 0x8801) return rbx_ym_read(0, a & 1);
	if (a >= 0x9800 && a < 0x9804) return oki_read(&s_oki);
	if (a == 0xa000) {
		t.latch_pending = 0;
		return t.latch;
	}
	return 0;
}
static void z_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0x8000 && a < 0x8800) t.zram[a - 0x8000] = v;
	else if (a == 0x8800 || a == 0x8801) rbx_ym_write(0, a & 1, v);
	else if (a >= 0x9800 && a < 0x9804) oki_write(&s_oki, v);
}
static uint8_t z_in(z80 *z, uint8_t port) { (void)z; (void)port; return 0xff; }
static void z_out(z80 *z, uint8_t port, uint8_t v) { (void)z; (void)port; (void)v; }

static void z_start(void)
{
	z80_init(&s_z80);
	s_z80.read_byte = z_read;
	s_z80.write_byte = z_write;
	s_z80.port_in = z_in;
	s_z80.port_out = z_out;
	s_z80.read_map = s_zmap;
}

static void sound_line(void)
{
	z80_run(&s_z80, s_z80.cyc + (unsigned long)rbx_line_cycles(&t.z80_frac, Z80_CLOCK, FRAME_MHZ, LINES), &t.ym_irq, 0);
	const uint32_t clocks = (uint32_t)rbx_line_cycles(&t.ym_frac, YM_CLOCK, FRAME_MHZ, LINES);
	const int n = rbx_ym_advance(0, clocks, s_mix + s_mix_n, s_mix_cap - s_mix_n);
	const uint32_t yr = rbx_ym_rate(0);
	for (int i = 0; i < n; i++) {
		t.oki_acc += s_oki.rate;
		while (t.oki_acc >= yr) {
			t.oki_acc -= yr;
			t.oki_hold = oki_sample(&s_oki);
		}
		s_mix[s_mix_n + i] += t.oki_hold;
	}
	s_mix_n += n;
}

// ---- main CPU ----
static void map_bank(void)
{
	for (int p = 0; p < 0x40; p++)
		s_main.rpage[0x40 + p] = s_rom + (uint32_t)t.bank * 0x4000 + p * 256;
}

static uint8_t main_read(void *u, uint16_t a)
{
	(void)u;
	if (a >= 0x8000) return s_rom[a];
	if (a >= 0x4000) return s_rom[(uint32_t)t.bank * 0x4000 + (a - 0x4000)];
	if (a < 0x800) return t.ram[a];
	if (a < 0x900) return t.spr[a - 0x800];
	if (a >= 0x2000 && a < 0x3000) return t.vram[a - 0x2000];
	if (a >= 0x3000) return t.attr[a - 0x3000];
	if (a >= 0x1000 && a <= 0x1006) {
		if (a == 0x1002)
			rbx_port_custom(s_port[2], 0x08, (s_line < VIS_Y0 || s_line >= VIS_Y0 + VIS_H) ? 8u : 0);
		return (uint8_t)rbx_port_read(s_port[a - 0x1000]);
	}
	return 0;
}
static void main_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0x4000) return;
	if (a < 0x800) t.ram[a] = v;
	else if (a < 0x900) { t.spr[a - 0x800] = v; s_vdirty = 1; }
	else if (a >= 0x2000 && a < 0x3000) { t.vram[a - 0x2000] = v; s_vdirty = 1; }
	else if (a >= 0x3000) { t.attr[a - 0x3000] = v; s_vdirty = 1; }
	else switch (a) {
	case 0x1008:   // scrollx_hi_w
		t.flip = ~v & 1;
		t.scrollx_hi = (uint16_t)((v & 2) << 7);
		t.bgbank = (v >> 2) & 7;
		t.spbank = (v >> 5) & 7;
		break;
	case 0x1009:   // bankswitch_w
		t.bank = v & 1;
		map_bank();
		t.gfxset = (v & 0x20) ^ 0x20;
		t.scrolly_hi = (uint16_t)((v & 0x40) << 2);
		break;
	case 0x100a: m6502_set_nmi(&s_main, 0); break;
	case 0x100b: m6502_set_irq(&s_main, 0); break;
	case 0x100c: t.scrollx_lo = v; break;
	case 0x100d:
		t.latch = v;
		if (!t.latch_pending)
			z80_gen_nmi(&s_z80);
		t.latch_pending = 1;
		break;
	case 0x100e: t.scrolly_lo = v; break;
	}
}

// ---- video ----
static int s_clip0, s_clip1;

static void draw_spr16(uint32_t code, int color, int fx, int fy, int sx, int sy)
{
	code %= s_gspr.count;
	if (!s_gspr.used[code] || sy >= s_clip1 || sy + 16 <= s_clip0)
		return;
	const uint8_t *src = s_gspr.pix + code * 256;
	for (int j = 0; j < 16; j++) {
		const int yy = sy + j;
		if (yy < s_clip0 || yy >= s_clip1)
			continue;
		const uint8_t *s = src + (fy ? 15 - j : j) * 16;
		uint16_t *d = rbx_video.bitmap + (yy - VIS_Y0) * rbx_video.pitch;
		for (int i = 0; i < 16; i++) {
			const int xx = sx + i;
			if ((unsigned)xx >= VIS_W)
				continue;
			const uint8_t p = s[fx ? 15 - i : i];
			if (p)
				d[xx] = (uint16_t)(128 + color * 16 + p);
		}
	}
}

static void draw_piece(const vseg *sg, int y0, int y1)
{
	const line_regs *L = s_snap->lines - VIS_Y0;
	for (int y = y0; y < y1; y++) {
		uint16_t *d = rbx_video.bitmap + (y - VIS_Y0) * rbx_video.pitch;
		const line_regs *l = &L[y];
		const uint32_t ty = (uint32_t)(l->flip ? 255 - y + l->sy : y + l->sy) & 511;
		for (int x = 0; x < VIS_W; x++) {
			const uint32_t tx = (uint32_t)(l->flip ? 255 - x + l->sx : x + l->sx) & 511;
			const uint32_t col = tx >> 3, row = ty >> 3;
			const uint32_t idx = (col & 0x1f) + ((row & 0x1f) << 5) + ((col & 0x20) << 5) + ((row & 0x20) << 6);
			const uint8_t attr = sg->attr[idx];
			const uint32_t code = rbx_wrap((uint32_t)(sg->vram[idx] + ((attr & 0x1f) << 8) + (l->gfxset << 8)), s_gchar.count);
			d[x] = (uint16_t)(((attr >> 5) & 7) * 16 + s_gchar.pix[code * 64 + (ty & 7) * 8 + (tx & 7)]);
		}
	}
	s_clip0 = y0;
	s_clip1 = y1;
	const int flip = L[y0].flip;
	for (int i = 0; i < 0x100; i += 4) {
		const uint8_t *s = &sg->spr[i];
		const int attr = s[1];
		const uint32_t which = (uint32_t)(s[2] + ((attr & 7) << 8));
		int sx = ((s[3] + 8) & 0xff) - 7, sy = 240 - s[0];
		const int size = (attr & 0x80) >> 7, color = (attr & 0x38) >> 3;
		int fx = (~attr & 0x40) != 0, fy = 0, dy = -16;
		if (flip) {
			sx = 240 - sx;
			sy = 240 - sy;
			fx = !fx;
			fy = !fy;
			dy = -dy;
		}
		if (!size)
			draw_spr16(which, color, fx, fy, sx, sy);
		else {
			draw_spr16(which, color, fx, fy, sx, sy + dy);
			draw_spr16(which + 1, color, fx, fy, sx, sy);
		}
	}
}

static void vb_render(void)
{
	for (int i = 0; i < 256; i++) {   // (bgprombank_w / spprombank_w: 0xff = not set yet, black)
		const int bank = i < 128 ? s_snap->bgbank : s_snap->spbank;
		if (bank == 0xff) {
			rbx_video.palette[i] = 0;
			continue;
		}
		const uint8_t *c = &s_prom[(i < 128 ? 0 : 0x400) + bank * 0x80 + (i & 0x7f)];
		rbx_video.palette[i] = (uint32_t)((c[0] & 15) * 17) << 16 | (uint32_t)((c[0] >> 4) * 17) << 8 | (uint32_t)((c[0x800] & 15) * 17);
	}
	const vbuf *vb = s_snap->vb;
	for (int k = 0; k < vb->n; k++) {
		const int y0 = vb->seg[k].y0, y1 = k + 1 < vb->n ? vb->seg[k + 1].y0 : VIS_Y0 + VIS_H;
		draw_piece(&vb->seg[k], y0, y1);
	}
}

static void line_done(int line)   // (update_partial at the next line's start)
{
	line_regs *r = &t.lines[line - VIS_Y0];
	const uint16_t sy = (uint16_t)(t.scrolly_hi + t.scrolly_lo);
	r->sy = sy;
	r->flip = t.flip;
	r->gfxset = t.gfxset;
	// (32 scroll rows: the tilemap row of this line, rows 2..31 from m_scrollx[row - 2], rows 0 and 1 never set)
	const uint32_t ty = (uint32_t)(t.flip ? 255 - line + sy : line + sy) & 511;
	const int row = (int)(ty >> 4);
	r->sx = row >= 2 ? t.scrollx[row - 2] : 0;
	vbuf *b = s_vb[s_vb_cur];
	if (line == VIS_Y0)
		b->n = 0;
	else if (!s_vdirty)
		return;
	vseg *sg;
	if (b->n < MAXSEG) {
		sg = &b->seg[b->n++];
		sg->y0 = line;
	} else
		sg = &b->seg[MAXSEG - 1];
	memcpy(sg->vram, t.vram, sizeof(t.vram));
	memcpy(sg->attr, t.attr, sizeof(t.attr));
	memcpy(sg->spr, t.spr, sizeof(t.spr));
	s_vdirty = 0;
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap->lines, t.lines, sizeof(t.lines));
	s_snap->bgbank = t.bgbank;
	s_snap->spbank = t.spbank;
	s_snap->vb = s_vb[s_vb_cur];
	s_vb_cur ^= 1;
	rbx_video_frame_done();
}

// ---- init / frame ----
static void vb_reset(void)
{
	memset(&t, 0, offsetof(__typeof__(t), state_end));
	t.bgbank = t.spbank = 0xff;
	m6502_init(&s_main, M6502_NMOS);
	s_main.read = main_read;
	s_main.write = main_write;
	for (int p = 0; p < 8; p++)
		s_main.rpage[p] = s_main.wpage[p] = t.ram + p * 256;
	for (int p = 0x80; p < 0x100; p++)
		s_main.rpage[p] = s_rom + p * 256;
	map_bank();
	m6502_reset(&s_main);
	z_start();
	rbx_ym_reset(0);
	oki_reset(&s_oki);
	s_oki.bank = 0;
	s_mix_n = 0;
	s_line = 0;
	s_vdirty = 0;
}

static void vb_exit(void);

static int vb_init(const rbx_game *g)
{
	uint32_t csize, ssize, osize, psize, lsize, hsize;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_zrom = rbx_region("audiocpu", &s_zrom_size);
	uint8_t *chars = rbx_region("fg_tiles", &csize), *spr = rbx_region("sprites", &ssize), *oki = rbx_region("oki", &osize);
	const uint8_t *prom = rbx_region("color_proms", &psize);
	const uint8_t *lo = rbx_region("color_proms:nib_low", &lsize), *hi = rbx_region("color_proms:nib_high", &hsize);
	if (!s_rom || s_rom_size < 0x10000 || !s_zrom || !chars || !spr || !oki || !prom || psize < 0x1000 || !lo || !hi ||
	    lsize < 0x800 || hsize < 0x800) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	memcpy(s_prom, prom, 0x1000);
	for (int i = 0; i < 0x800; i++)   // (ROM_LOAD_NIB_LOW / _HIGH)
		s_prom[i] = (uint8_t)((lo[i] & 0x0f) | (hi[i] & 0x0f) << 4);
	{	// charlayout, spritelayout (RGN_FRAC(1,2))
		static const uint32_t pc[4] = { 0, 2, 4, 6 };
		static const uint32_t xc[8] = { 1, 0, 64 + 1, 64 + 0, 128 + 1, 128 + 0, 192 + 1, 192 + 0 };
		static const uint32_t yc[8] = { 0, 8, 16, 24, 32, 40, 48, 56 };
		static const uint32_t xt[16] = { 3, 2, 1, 0, 16 * 8 + 3, 16 * 8 + 2, 16 * 8 + 1, 16 * 8 + 0,
		                                 32 * 8 + 3, 32 * 8 + 2, 32 * 8 + 1, 32 * 8 + 0, 48 * 8 + 3, 48 * 8 + 2, 48 * 8 + 1, 48 * 8 + 0 };
		static uint32_t yt[16];
		for (int i = 0; i < 16; i++)
			yt[i] = (uint32_t)i * 8;
		const uint32_t ps[4] = { ssize * 4, ssize * 4 + 4, 0, 4 };
		const rbx_gfx_layout lc = { 8, 8, 4, pc, xc, yc, 32 * 8 };
		const rbx_gfx_layout ls = { 16, 16, 4, ps, xt, yt, 64 * 8 };
		if (rbx_gfx_decode(&s_gchar, &lc, chars, csize) || rbx_gfx_decode(&s_gspr, &ls, spr, ssize / 2)) {
			rbx_fail("%s: out of memory (graphics)", g->set);
			vb_exit();
			return -1;
		}
	}
	s_snap = calloc(1, sizeof(*s_snap));
	s_vb[0] = calloc(1, sizeof(vbuf));
	s_vb[1] = calloc(1, sizeof(vbuf));
	s_vb_cur = 0;
	rbx_ym_config yc;
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM2151;
	yc.clock = YM_CLOCK;
	yc.gain[0] = yc.gain[1] = 77;   // (0.60 to each side of a stereo speaker: (L + R) / 2 each)
	yc.irq = ym_irq;
	if (!s_snap || !s_vb[0] || !s_vb[1] || rbx_ym_init(0, &yc) || rbx_ym_set_async(0, 1)) {
		rbx_fail("%s: out of memory", g->set);
		vb_exit();
		return -1;
	}
	s_mix_cap = (int)(rbx_ym_rate(0) / 50) + 256;
	s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	if (!s_mix || rbx_audio_setup(rbx_ym_rate(0)) || rbx_video_setup(VIS_W, VIS_H, 256)) {
		rbx_fail("%s: out of memory", g->set);
		vb_exit();
		return -1;
	}
	oki_init(&s_oki, 1056000, 1, oki, osize, 256);
	memset(s_zmap, 0, sizeof(s_zmap));
	for (int p = 0; p < 0x80 && (uint32_t)p * 256 + 256 <= s_zrom_size; p++)
		s_zmap[p] = s_zrom + p * 256;
	static const char *const names[7] = { "P1", "P2", "SYSTEM", "DSW1", "DSW2", "P3", "P4" };
	for (int i = 0; i < 7; i++)
		s_port[i] = rbx_port_find(names[i]);
	vb_reset();
	return 0;
}

static void vb_exit(void)
{
	rbx_video_wait_idle();
	rbx_ym_exit(0);
	free(s_mix);
	s_mix = NULL;
	rbx_gfx_free(&s_gchar);
	rbx_gfx_free(&s_gspr);
	free(s_snap);
	s_snap = NULL;
	free(s_vb[0]);
	free(s_vb[1]);
	s_vb[0] = s_vb[1] = NULL;
}

static inline int scanline_to_vcount(int scanline)
{
	const int vcount = scanline + 8;
	return vcount < 0x100 ? vcount : ((vcount - 0x18) | 0x100);
}

static void vb_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	for (int line = 0; line < LINES; line++) {
		s_line = line;
		if (line == VIS_Y0 + VIS_H)
			vblank_capture();
		const int vold = scanline_to_vcount(line == 0 ? LINES - 1 : line - 1), vc = scanline_to_vcount(line);
		if (!(vold & 8) && (vc & 8))
			m6502_set_irq(&s_main, 1);
		if (vc == 0xf8)
			m6502_set_nmi(&s_main, 1);
		if (line < 256)
			t.scrollx[255 - line] = (uint16_t)(t.scrollx_hi + t.scrollx_lo + 4);
		m6502_run_until(&s_main, t.main_t += rbx_line_cycles(&t.main_frac, MAIN_CLOCK, FRAME_MHZ, LINES));
		sound_line();
		if (line >= VIS_Y0 && line < VIS_Y0 + VIS_H)
			line_done(line);
	}
	rbx_ym_async_take(0, s_mix, s_mix_n);
	for (int i = 0; i < s_mix_n; i++)
		rbx_audio_put(s_mix[i]);
	memset(s_mix, 0, sizeof(int32_t) * (size_t)s_mix_n);
	s_mix_n = 0;
}

static int vb_state(void *ctx, int load)
{
	int rc = 0;
	z80 z = s_z80;
	rc |= rbx_state_io(ctx, "vb", &t, (uint32_t)offsetof(__typeof__(t), state_end));
	rc |= rbx_state_io(ctx, "vb_main", &s_main, (uint32_t)offsetof(m6502_t, rpage));
	rc |= rbx_state_io(ctx, "vb_z80", &z, sizeof(z));
	rc |= rbx_ym_state(0, ctx, load);
	rc |= rbx_state_io(ctx, "vb_oki", s_oki.v, sizeof(s_oki.v) + sizeof(int));
	if (!rc && load) {
		z.read_byte = z_read;
		z.write_byte = z_write;
		z.port_in = z_in;
		z.port_out = z_out;
		z.userdata = NULL;
		z.read_map = s_zmap;
		s_z80 = z;
		map_bank();
		s_vdirty = 1;
	}
	return rc ? -1 : 0;
}

static int vb_save(void *ctx) { return vb_state(ctx, 0); }
static int vb_load(void *ctx) { return vb_state(ctx, 1); }
static int vb_supported(const char *machine) { return !strcmp(machine, "vball"); }

const rbx_driver rbx_drv_vball = { "Technos U.S. Championship V'ball", vb_init, vb_exit, vb_reset, vb_frame, vb_save,
                                   vb_load, vb_supported, vb_render, NULL };
