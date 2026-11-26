/*  Technos WWF Superstars. Ported from MAME src/mame/technos/wwfsstar.cpp (BSD-3-Clause, copyright-holders: David
    Haywood).
    68000 at 10 MHz with an IRQ every 16 lines (level 5) and one at line 240 (6), both held until acknowledged; a 16x16
    BG tilemap of 512x512 (opaque), 16x16 sprites (one or two high), an 8x8 FG tilemap; the screen is drawn in 16-line
    pieces (update_partial at each raster IRQ), so the scroll registers are latched per piece. Sound: Z80 at 3.58 MHz
    with a YM2151 (the bootleg: two YM2203s) and an OKI6295 at 1.056 MHz, the latch on the NMI.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m68k_bus.h"
#include "../../cpu/m68k/m68k.h"
#include "../../cpu/z80/z80.h"
#include "../../sound/okim6295.h"
#include "../../sound/rbx_ym.h"
#include "../../video/decoband.h"
#include "../../video/gfxlayout.h"

#include <stdlib.h>
#include <string.h>

#define CPU_CLOCK 10000000
#define Z80_CLOCK 3579545
#define YM_CLOCK 3579545
#define OKI_CLOCK 1056000
#define FRAME_MHZ 57445          // set_raw(20 MHz / 4, 320, 0, 256, 272, 8, 248)
#define LINES 272
#define VIS_Y0 8
#define VIS_H 240
#define VIS_W 256
#define VBL_LINE 240

typedef struct { uint16_t sx, sy; uint8_t flip; } line_regs;
// a piece of the screen as MAME's update_partial draws it: the registers and the (unbuffered) RAMs at its end
typedef struct {
	int y0, y1;
	line_regs r;
	uint16_t fgram[0x800], bgram[0x800], spr[0x200];
} piece;
#define MAX_PIECES 18

static int s_boot;   // wwfsstarb2: two YM2203s
static uint8_t *s_rom, *s_zrom;
static uint32_t s_rom_size, s_zrom_size;
static uint16_t (*s_ram_hp)[0x2000];          // 0x1c0000 .. 0x1c3fff
#define s_ram (*s_ram_hp)
static uint16_t s_fgram[0x800];         // 0x080000 .. 0x080fff
static uint16_t s_bgram[0x800];         // 0x0c0000 .. 0x0c0fff
static uint16_t s_spriteram[0x200];     // 0x100000 .. 0x1003ff
static uint16_t s_pal[0x180];           // 0x140000 .. 0x1402ff (write only)
static uint16_t s_scroll[2];
static uint8_t s_flip, s_vblank;
static piece (*s_pieces_hp)[MAX_PIECES];
#define s_pieces (*s_pieces_hp)
static int s_npieces;
static int s_seg_start;
static rbx_gfx s_gfxfg, s_gfxspr, s_gfxbg;
static int s_port[5];
static int s_line;
static uint64_t s_cpu_frac, s_z80_frac, s_ym_frac;
static struct {
	piece pieces[MAX_PIECES];
	int npieces;
	uint16_t pal[0x180];
} *s_snap;
static const piece *s_cur;   // (the piece being drawn)

// ---- sound ----
static z80 s_z80;
static uint8_t s_zram[0x800];
static uint8_t s_latch;
static int s_latch_pending;
static okim6295_t s_oki;
static int s_ym_irq;
static uint32_t s_oki_acc;
static int32_t s_oki_hold;
static int32_t *s_mix;
static int s_mix_n, s_mix_cap;
static const uint8_t *s_zmap[256];

static void ym_irq(int state) { s_ym_irq = state; }

static uint8_t z80_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x8000) return a < s_zrom_size ? s_zrom[a] : 0;
	if (a < 0x8800) return s_zram[a - 0x8000];
	if (a == 0x8800 || a == 0x8801) return rbx_ym_read(0, a & 1);
	if (s_boot && (a == 0x8802 || a == 0x8803)) return rbx_ym_read(1, a & 1);
	if (a == 0x9800) return oki_read(&s_oki);
	if (a == 0xa000) {
		s_latch_pending = 0;
		return s_latch;
	}
	return 0;
}
static void z80_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0x8000 && a < 0x8800) s_zram[a - 0x8000] = v;
	else if (a == 0x8800 || a == 0x8801) rbx_ym_write(0, a & 1, v);
	else if (s_boot && (a == 0x8802 || a == 0x8803)) rbx_ym_write(1, a & 1, v);
	else if (a == 0x9800) oki_write(&s_oki, v);
}
static uint8_t z80_in(z80 *z, uint8_t port) { (void)z; (void)port; return 0xff; }
static void z80_out(z80 *z, uint8_t port, uint8_t v) { (void)z; (void)port; (void)v; }

static void z80_start(void)
{
	z80_init(&s_z80);
	s_z80.read_byte = z80_read;
	s_z80.write_byte = z80_write;
	s_z80.port_in = z80_in;
	s_z80.port_out = z80_out;
	s_z80.read_map = s_zmap;
}

static void sound_line(void)
{
	const int cycles = rbx_line_cycles(&s_z80_frac, Z80_CLOCK, FRAME_MHZ, LINES);
	z80_run(&s_z80, s_z80.cyc + (unsigned long)cycles, &s_ym_irq, 0);
	const uint32_t clocks = (uint32_t)rbx_line_cycles(&s_ym_frac, YM_CLOCK, FRAME_MHZ, LINES);
	const int n = rbx_ym_advance(0, clocks, s_mix + s_mix_n, s_mix_cap - s_mix_n);
	if (s_boot)
		rbx_ym_advance(1, clocks, s_mix + s_mix_n, s_mix_cap - s_mix_n);
	const uint32_t yr = rbx_ym_rate(0);
	for (int i = 0; i < n; i++) {
		s_oki_acc += s_oki.rate;
		while (s_oki_acc >= yr) {
			s_oki_acc -= yr;
			s_oki_hold = oki_sample(&s_oki);
		}
		s_mix[s_mix_n + i] += s_oki_hold;
	}
	s_mix_n += n;
}

static void sound_frame_end(void)
{
	rbx_ym_async_take(0, s_mix, s_mix_n);
	if (s_boot)
		rbx_ym_async_take(1, s_mix, s_mix_n);
	for (int i = 0; i < s_mix_n; i++)
		rbx_audio_put(s_mix[i]);
	memset(s_mix, 0, sizeof(int32_t) * (size_t)s_mix_n);
	s_mix_n = 0;
}

// ---- bus ----
static uint16_t main_read16(uint32_t a)
{
	a &= 0xffffff;
	if (a >= 0x180000 && a < 0x18000a) {   // DSW1, DSW2, P1, P2, SYSTEM
		const int i = (int)((a - 0x180000) >> 1);
		if (i == 4)
			rbx_port_custom(s_port[4], 0x0001, s_vblank);
		return (uint16_t)rbx_port_read(s_port[i]);
	}
	return 0;
}
static uint8_t main_read8(uint32_t a)
{
	const uint16_t v = main_read16(a & ~1u);
	return (uint8_t)(a & 1 ? v : v >> 8);
}
static void main_write16m(uint32_t a, uint16_t v, uint16_t mask)
{
	a &= 0xffffff;
	if (a >= 0x140000 && a < 0x140300) {
		uint16_t *w = &s_pal[(a - 0x140000) >> 1];
		*w = (uint16_t)((*w & ~mask) | (v & mask));
	} else if (a >= 0x180000 && a < 0x180004)
		m68k_bus_set_line(a & 2 ? 5 : 6, 0);
	else if (a >= 0x180004 && a < 0x180008) {
		uint16_t *w = &s_scroll[(a >> 1) & 1];
		*w = (uint16_t)((*w & ~mask) | (v & mask));
	} else if (a == 0x180008 && (mask & 0x00ff)) {
		s_latch = (uint8_t)v;
		if (!s_latch_pending)
			z80_gen_nmi(&s_z80);
		s_latch_pending = 1;
	} else if (a == 0x18000a)
		s_flip = v & 1;
}
static void main_write16(uint32_t a, uint16_t v) { main_write16m(a, v, 0xffff); }
static void main_write8(uint32_t a, uint8_t v)
{
	const int sh = a & 1 ? 0 : 8;
	main_write16m(a & ~1u, (uint16_t)(v << sh), (uint16_t)(0xff << sh));
}

// ---- video ----
static void latch_lines(int end)
{
	if (end <= s_seg_start || s_npieces >= MAX_PIECES)
		return;
	piece *p = &s_pieces[s_npieces++];
	memset(&p->r, 0, sizeof(p->r));
	p->y0 = s_seg_start;
	p->y1 = end;
	p->r.sx = s_scroll[0];
	p->r.sy = s_scroll[1];
	p->r.flip = s_flip;
	memcpy(p->fgram, s_fgram, sizeof(s_fgram));
	memcpy(p->bgram, s_bgram, sizeof(s_bgram));
	memcpy(p->spr, s_spriteram, sizeof(s_spriteram));
	s_seg_start = end;
}

static void draw_bg_line(const piece *pc, int y, uint16_t *dst)   // opaque (no transparent pen)
{
	const line_regs *r = &pc->r;
	const uint32_t ty = (uint32_t)(r->flip ? 255 - y + r->sy : y + r->sy) & 511;
	const uint32_t row = ty >> 4, fy = ty & 15;
	for (int x = 0; x < VIS_W;) {
		const uint32_t tx = (uint32_t)(r->flip ? VIS_W - 1 - x + r->sx : x + r->sx) & 511;
		const uint32_t col = tx >> 4;
		const uint32_t idx = (col & 0x0f) + ((row & 0x0f) << 4) + ((col & 0x10) << 4) + ((row & 0x10) << 5);   // bg0_scan
		const uint16_t *t = &pc->bgram[idx * 2];
		const uint32_t code = rbx_wrap(((t[1] & 0x00ffu) | (t[0] & 0x000fu) << 8), s_gfxbg.count);
		const uint16_t c0 = (uint16_t)(256 + ((t[0] & 0x0070) >> 4) * 16);
		const int fx = (t[0] >> 7) & 1;
		const uint8_t *src = s_gfxbg.pix + code * 256 + fy * 16;
		int px = (int)(tx & 15);
		if (!r->flip)
			for (; px < 16 && x < VIS_W; px++, x++)
				dst[x] = (uint16_t)(c0 + src[fx ? 15 - px : px]);
		else
			for (; px >= 0 && x < VIS_W; px--, x++)
				dst[x] = (uint16_t)(c0 + src[fx ? 15 - px : px]);
	}
}

static void draw_fg_line(const piece *pc, int y, uint16_t *dst)
{
	const int flip = pc->r.flip;
	const int ty = (flip ? 255 - y : y) & 255;
	int last = -1;
	uint32_t cbase = 0;
	const uint8_t *trow = NULL;
	for (int x = 0; x < VIS_W; x++) {
		const int tx = (flip ? VIS_W - 1 - x : x) & 255;
		if ((tx >> 3) != last) {   // (a character looked up once for its 8 pixels)
			last = tx >> 3;
			const uint16_t *t = &pc->fgram[((ty >> 3) * 32 + (tx >> 3)) * 2];
			const uint32_t code = rbx_wrap((uint32_t)((t[1] & 0x00ff) | (t[0] & 0x000f) << 8), s_gfxfg.count);
			trow = s_gfxfg.pix + code * 64 + (ty & 7) * 8;
			cbase = (uint32_t)((t[0] & 0x00f0) >> 4) * 16;
		}
		const uint8_t p = trow[tx & 7];
		if (p)
			dst[x] = (uint16_t)(cbase + p);
	}
}

static void draw_tile16(uint32_t code, uint16_t c0, int flipx, int flipy, int sx, int sy, int y0, int y1)
{
	code %= s_gfxspr.count;
	if (!s_gfxspr.used[code])
		return;
	sy -= VIS_Y0;
	y0 -= VIS_Y0;
	y1 -= VIS_Y0;
	if (sy >= y1 || sy + 16 <= y0 || sx >= VIS_W || sx + 16 <= 0)
		return;
	const uint8_t *src = s_gfxspr.pix + code * 256;
	for (int j = 0; j < 16; j++) {
		const int yy = sy + j;
		if (yy < y0 || yy >= y1)
			continue;
		const uint8_t *s = src + (flipy ? 15 - j : j) * 16;
		uint16_t *d = rbx_video.bitmap + yy * rbx_video.pitch;
		for (int i = 0; i < 16; i++) {
			const int xx = sx + i;
			if ((unsigned)xx >= VIS_W)
				continue;
			const uint8_t p = s[flipx ? 15 - i : i];
			if (p)
				d[xx] = (uint16_t)(c0 + p);
		}
	}
}

static void draw_sprites(int y0, int y1, int flip)
{
	for (int o = 0; o <= 0x200 - 5; o += 5) {
		const uint16_t *s = s_cur->spr + o;
		if (!(s[1] & 0x0001))
			continue;
		int ypos = (s[0] & 0x00ff) | (s[1] & 0x0004) << 6;
		ypos = ((256 - ypos) & 0x1ff) - 16;
		int xpos = (s[4] & 0x00ff) | (s[1] & 0x0008) << 5;
		xpos = ((256 - xpos) & 0x1ff) - 16;
		int flipx = (s[2] & 0x0080) >> 7, flipy = (s[2] & 0x0040) >> 6;
		const int chain = ((s[1] & 0x0002) >> 1) + 1;
		uint32_t number = (uint32_t)((s[3] & 0x00ff) | (s[2] & 0x003f) << 8);
		number &= ~(uint32_t)(chain - 1);
		const uint16_t c0 = (uint16_t)(128 + ((s[1] & 0x00f0) >> 4) * 16);
		if (flip) {
			flipy = !flipy;
			flipx = !flipx;
			ypos = 240 - ypos;
			xpos = 240 - xpos;
		}
		for (int count = 0; count < chain; count++) {
			int yy;
			if (flip)
				yy = !flipy ? ypos + 16 * count : ypos + 16 * (chain - 1) - 16 * count;
			else
				yy = !flipy ? ypos - 16 * (chain - 1) + 16 * count : ypos - 16 * count;
			draw_tile16(number + (uint32_t)count, c0, flipx, flipy, xpos, yy, y0, y1);
		}
	}
}

// (the pieces are lines apart: the BG of every line in bands, each piece's sprites, then the FG in bands, as MAME's
// pieces in turn)
#define BANDS 8
static const piece *s_line_piece[VIS_H];
static void bg_band(int band)
{
	for (int y = VIS_H * band / BANDS; y < VIS_H * (band + 1) / BANDS; y++)
		if (s_line_piece[y])
			draw_bg_line(s_line_piece[y], y + VIS_Y0, rbx_video.bitmap + y * rbx_video.pitch);
}
static void fg_band(int band)
{
	for (int y = VIS_H * band / BANDS; y < VIS_H * (band + 1) / BANDS; y++)
		if (s_line_piece[y])
			draw_fg_line(s_line_piece[y], y + VIS_Y0, rbx_video.bitmap + y * rbx_video.pitch);
}
static decoband_t s_bg_bands = { 0, 0, 0, 0, BANDS, bg_band }, s_fg_bands = { 0, 0, 0, 0, BANDS, fg_band };
static int ws_help(int helper) { return decoband_help(&s_bg_bands, helper) || decoband_help(&s_fg_bands, helper); }

static void ws_render(void)
{
	for (int i = 0; i < 0x180; i++) {   // xBGR_444
		const uint16_t p = s_snap->pal[i];
		rbx_video.palette[i] = (uint32_t)(((p & 15) * 17) << 16 | (((p >> 4) & 15) * 17) << 8 | (((p >> 8) & 15) * 17));
	}
	memset(s_line_piece, 0, sizeof(s_line_piece));
	for (int k = 0; k < s_snap->npieces; k++) {
		const piece *pc = &s_snap->pieces[k];
		for (int y = pc->y0; y < pc->y1; y++)
			if (y >= VIS_Y0 && y < VIS_Y0 + VIS_H)
				s_line_piece[y - VIS_Y0] = pc;
	}
	decoband_run(&s_bg_bands);
	for (int k = 0; k < s_snap->npieces; k++) {
		const piece *pc = &s_snap->pieces[k];
		const int y = pc->y0 > VIS_Y0 ? pc->y0 : VIS_Y0, y1 = pc->y1 < VIS_Y0 + VIS_H ? pc->y1 : VIS_Y0 + VIS_H;
		if (y >= y1)
			continue;
		s_cur = pc;
		draw_sprites(y, y1, pc->r.flip);
	}
	decoband_run(&s_fg_bands);
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap->pieces, s_pieces, sizeof(piece) * (size_t)s_npieces);
	s_snap->npieces = s_npieces;
	memcpy(s_snap->pal, s_pal, sizeof(s_pal));
	rbx_video_frame_done();
}

// ---- init / frame ----
static void ws_reset(void)
{
	memset(s_ram, 0, sizeof(s_ram));
	memset(s_fgram, 0, sizeof(s_fgram));
	memset(s_bgram, 0, sizeof(s_bgram));
	memset(s_spriteram, 0, sizeof(s_spriteram));
	memset(s_pal, 0, sizeof(s_pal));
	s_npieces = 0;
	s_scroll[0] = s_scroll[1] = 0;
	s_flip = s_vblank = 0;
	s_seg_start = 0;
	s_latch = 0;
	s_latch_pending = 0;
	s_ym_irq = 0;
	memset(s_zram, 0, sizeof(s_zram));
	z80_start();
	rbx_ym_reset(0);
	if (s_boot)
		rbx_ym_reset(1);
	oki_reset(&s_oki);
	s_oki_acc = 0;
	s_oki_hold = 0;
	s_mix_n = 0;
	m68k_bus_irq_reset();
	m68k_pulse_reset();
	s_line = 0;
	s_cpu_frac = s_z80_frac = s_ym_frac = 0;
}

static void ws_exit(void);

static int ws_init(const rbx_game *g)
{
	if (!s_pieces_hp && !(s_pieces_hp = calloc(1, sizeof(*s_pieces_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	if (!s_ram_hp && !(s_ram_hp = calloc(1, sizeof(*s_ram_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	s_boot = !strcmp(g->machine, "wwfsstarb2");
	uint32_t fsize, ssize, bsize, osize;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_zrom = rbx_region("audiocpu", &s_zrom_size);
	uint8_t *fg = rbx_region("fgtiles", &fsize), *spr = rbx_region("sprites", &ssize), *bg = rbx_region("bgtiles", &bsize);
	uint8_t *oki = rbx_region("oki", &osize);
	if (!s_rom || s_rom_size < 0x40000 || !s_zrom || !fg || !spr || !bg || !oki) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	m68k_bus_swap16(s_rom, s_rom_size);
	{	// tiles8x8_layout, tiles16x16_layout (RGN_FRAC(1,2))
		static const uint32_t x8[8] = { 1, 0, 8 * 8 + 1, 8 * 8 + 0, 16 * 8 + 1, 16 * 8 + 0, 24 * 8 + 1, 24 * 8 + 0 };
		static uint32_t y8[8], x16[16], y16[16];
		for (int i = 0; i < 8; i++)
			y8[i] = (uint32_t)(i * 8);
		for (int i = 0; i < 16; i++) {
			x16[i] = (uint32_t)((i >> 2) * 16 * 8 + (3 - (i & 3)));
			y16[i] = (uint32_t)(i * 8);
		}
		const uint32_t p8[4] = { 0, 2, 4, 6 };
		const uint32_t hs = ssize * 4, hb = bsize * 4;
		const uint32_t ps[4] = { hs, hs + 4, 0, 4 }, pb[4] = { hb, hb + 4, 0, 4 };
		const rbx_gfx_layout l8 = { 8, 8, 4, p8, x8, y8, 32 * 8 };
		const rbx_gfx_layout ls = { 16, 16, 4, ps, x16, y16, 64 * 8 };
		const rbx_gfx_layout lb = { 16, 16, 4, pb, x16, y16, 64 * 8 };
		if (rbx_gfx_decode(&s_gfxfg, &l8, fg, fsize) || rbx_gfx_decode(&s_gfxspr, &ls, spr, ssize / 2) ||
		    rbx_gfx_decode(&s_gfxbg, &lb, bg, bsize / 2)) {
			rbx_fail("%s: out of memory (graphics)", g->set);
			ws_exit();
			return -1;
		}
	}
	s_snap = calloc(1, sizeof(*s_snap));
	rbx_ym_config yc;
	memset(&yc, 0, sizeof(yc));
	yc.clock = YM_CLOCK;
	if (s_boot) {   // two YM2203s, all outputs 0.45 (mono)
		yc.type = RBX_YM2203;
		yc.gain[0] = yc.gain[1] = yc.gain[2] = yc.gain[3] = 115;
	} else {        // YM2151 0.45 to both sides: (L + R) / 2
		yc.type = RBX_YM2151;
		yc.gain[0] = yc.gain[1] = 58;
	}
	yc.irq = ym_irq;
	int bad = !s_snap || rbx_ym_init(0, &yc) || rbx_ym_set_async(0, 1);
	if (!bad && s_boot) {
		yc.irq = NULL;
		bad = rbx_ym_init(1, &yc) || rbx_ym_set_async(1, 1);
	}
	if (bad) {
		rbx_fail("%s: out of memory", g->set);
		ws_exit();
		return -1;
	}
	s_mix_cap = (int)(rbx_ym_rate(0) / 50) + 256;
	s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	if (!s_mix || rbx_audio_setup(rbx_ym_rate(0)) || rbx_video_setup(VIS_W, VIS_H, 0x180)) {
		rbx_fail("%s: out of memory", g->set);
		ws_exit();
		return -1;
	}
	oki_init(&s_oki, OKI_CLOCK, 1, oki, osize, 120);
	memset(s_zmap, 0, sizeof(s_zmap));
	for (uint32_t p = 0; p < 0x80 && p * 256 + 256 <= s_zrom_size; p++)
		s_zmap[p] = s_zrom + p * 256;
	static const char *const names[5] = { "DSW1", "DSW2", "P1", "P2", "SYSTEM" };
	for (int i = 0; i < 5; i++)
		s_port[i] = rbx_port_find(names[i]);
	m68k_init();
	m68k_set_cpu_type(M68K_CPU_TYPE_68000);
	m68k_set_mame_timing(1);   // (MAME's DIVx / MULx / ADDQ.W An times)
	m68k_bus_reset_map();
	m68k_bus_map(0, 0x3ffff, s_rom, 0x40000, 1, 0);
	m68k_bus_map(0x080000, 0x080fff, s_fgram, sizeof(s_fgram), 1, 1);
	m68k_bus_map(0x0c0000, 0x0c0fff, s_bgram, sizeof(s_bgram), 1, 1);
	m68k_bus_map(0x100000, 0x1003ff, s_spriteram, sizeof(s_spriteram), 1, 1);
	m68k_bus_map(0x1c0000, 0x1c3fff, s_ram, sizeof(s_ram), 1, 1);
	m68k_bus.read8 = main_read8;
	m68k_bus.read16 = main_read16;
	m68k_bus.write8 = main_write8;
	m68k_bus.write16 = main_write16;
	ws_reset();
	return 0;
}

static void ws_exit(void)
{
	m68k_set_mame_timing(0);
	rbx_ym_exit(0);
	rbx_ym_exit(1);
	free(s_mix);
	s_mix = NULL;
	rbx_gfx_free(&s_gfxfg);
	rbx_gfx_free(&s_gfxspr);
	rbx_gfx_free(&s_gfxbg);
	free(s_snap);
	s_snap = NULL;
	free(s_pieces_hp);
	s_pieces_hp = NULL;
	free(s_ram_hp);
	s_ram_hp = NULL;
}

static void ws_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	for (int line = 0; line < LINES; line++) {
		s_line = line;
		if (line == 0)
			s_vblank = 0;
		if (line % 16 == 0) {   // scanline: the raster IRQ every 16 lines
			if (line > 0)
				latch_lines(line);
			else
				s_seg_start = s_npieces = 0;
			m68k_bus_set_line(5, 1);
		}
		if (line == VBL_LINE) {
			s_vblank = 1;
			latch_lines(line);
			m68k_bus_set_line(6, 1);
		}
		if (line == VIS_Y0 + VIS_H) {   // (the screen's vblank start draws lines 240 .. 247: update_now)
			latch_lines(line);
			vblank_capture();
		}
		m68k_execute(rbx_line_cycles(&s_cpu_frac, CPU_CLOCK, FRAME_MHZ, LINES));
		sound_line();
	}
	sound_frame_end();
}

static int ws_state(void *ctx, int load)
{
	const uint32_t csize = m68k_context_size();
	uint8_t *cpu = (uint8_t *)malloc(csize);
	if (!cpu)
		return -1;
	if (!load)
		m68k_get_context(cpu);
	uint32_t irq[2] = { m68k_bus_irq_pending(), m68k_bus_cur->irq_asserted };
	uint64_t fr[3] = { s_cpu_frac, s_z80_frac, s_ym_frac };
	int32_t misc[8] = { s_latch, s_latch_pending, s_ym_irq, (int32_t)s_oki_acc, s_oki_hold, s_flip, s_vblank, 0 };
	z80 z = s_z80;
	int rc = 0;
	rc |= rbx_state_io(ctx, "m68k", cpu, csize);
	rc |= rbx_state_io(ctx, "irq", irq, sizeof(irq));
	rc |= rbx_state_io(ctx, "ram", s_ram, sizeof(s_ram));
	rc |= rbx_state_io(ctx, "fg", s_fgram, sizeof(s_fgram));
	rc |= rbx_state_io(ctx, "bg", s_bgram, sizeof(s_bgram));
	rc |= rbx_state_io(ctx, "spr", s_spriteram, sizeof(s_spriteram));
	rc |= rbx_state_io(ctx, "pal", s_pal, sizeof(s_pal));
	rc |= rbx_state_io(ctx, "scrl", s_scroll, sizeof(s_scroll));
	rc |= rbx_state_io(ctx, "z80", &z, sizeof(z));
	rc |= rbx_state_io(ctx, "zram", s_zram, sizeof(s_zram));
	rc |= rbx_state_io(ctx, "oki", s_oki.v, sizeof(s_oki.v) + sizeof(int));
	rc |= rbx_ym_state(0, ctx, load);
	if (s_boot)
		rc |= rbx_ym_state(1, ctx, load);
	rc |= rbx_state_io(ctx, "frac", fr, sizeof(fr));
	rc |= rbx_state_io(ctx, "misc", misc, sizeof(misc));
	if (!rc && load) {
		m68k_set_context(cpu);
		m68k_bus_set_irq_pending(irq[0]);
		m68k_bus_cur->irq_asserted = irq[1];
		m68k_bus_update_irq();
		z.read_byte = z80_read;
		z.write_byte = z80_write;
		z.port_in = z80_in;
		z.port_out = z80_out;
		z.userdata = NULL;
		z.read_map = s_zmap;
		s_z80 = z;
		s_cpu_frac = fr[0];
		s_z80_frac = fr[1];
		s_ym_frac = fr[2];
		s_latch = (uint8_t)misc[0];
		s_latch_pending = misc[1];
		s_ym_irq = misc[2];
		s_oki_acc = (uint32_t)misc[3];
		s_oki_hold = misc[4];
		s_flip = (uint8_t)misc[5];
		s_vblank = (uint8_t)misc[6];
		s_seg_start = s_npieces = 0;
	}
	free(cpu);
	return rc ? -1 : 0;
}

static int ws_save(void *ctx) { return ws_state(ctx, 0); }
static int ws_load(void *ctx) { return ws_state(ctx, 1); }
static int ws_supported(const char *machine) { return !strcmp(machine, "wwfsstar") || !strcmp(machine, "wwfsstarb2"); }

const rbx_driver rbx_drv_wwfsstar = { "Technos WWF Superstars", ws_init, ws_exit, ws_reset, ws_frame, ws_save, ws_load,
                                      ws_supported, ws_render, ws_help };
