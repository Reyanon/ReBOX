/*  Irem M62: Kung-Fu Master, The Battle-Road, the Lode Runners, Lot Lot, Kid Niki, the Spelunkers, Youjyuden and Horizon.
    Ported from MAME src/mame/irem/m62.cpp, m62_v.cpp, m62.h (BSD-3-Clause, copyright-holders: smf, David Haywood)
    and the palette's src/emu/video/resnet.cpp (BSD-3-Clause, copyright-holders: Couriersud). A Z80, a
    tilemap split front and back around 16x16 sprites a game shapes its own way, sometimes a text layer over them; the
    sound board of machine/irem_audio.c.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../cheat_cpu.h"
#include "../../rbx.h"
#include "../../cpu/z80/z80.h"
#include "../../video/gfxlayout.h"
#include "../machine/irem_audio.h"
#include "../video/irem_draw.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define LINES 284
#define PEN_SP 512             // (the tiles' 512, the sprites' 256, the text's 256, black)
#define PEN_FG 768
#define PEN_BLACK 1024
#define NPENS 1025
#define TTL_VOL 0.05           // (resnet.h)
#define TTL_VOH 4.0

enum { K_KUNGFUM, K_BATTROAD, K_LDRUN, K_LDRUN2, K_LDRUN3, K_LDRUN4, K_LOTLOT, K_KIDNIKI, K_SPELUNKR, K_SPELUNK2, K_YOUJYUDN,
       K_HORIZON };
enum { DRAW_OPAQUE, DRAW_FRONT, DRAW_CAT0, DRAW_CAT1, DRAW_TEXT };

#define T_FX (1u << 24)
#define T_GROUP (1u << 25)

static uint8_t *s_rom;
static uint32_t s_rom_size;
static const uint8_t *s_hprom;
static rbx_gfx s_bg, s_fg, s_spr;
static z80 s_z;
static const uint8_t *s_map[256];
static int s_kind, s_port[5];
static int s_vx0, s_vw, s_htotal, s_line_cyc, s_has_fg;
static int s_bcols, s_brows, s_btw, s_bth, s_fcols, s_frows, s_ftw, s_fth, s_fbits, s_spr_bytes;
static uint32_t s_fg_base;

static struct {
	uint8_t spr[0x200], tile[0x2000], text[0x1000], ram[0x1000], scroll[0x40];
	int32_t hscroll, vscroll, text_vscroll;
	uint8_t flip, bgbank, palbank, topbottom, bank1, bank2, ldrun2_swap, bankctl[2], irq;
	unsigned long main_end;
	uint32_t frame;
	uint8_t state_end;
} t;

// ---- the drawing's snapshot ----
static struct {
	uint8_t spr[0x200], tile[0x2000], text[0x1000], scroll[0x40];
	int32_t hscroll, vscroll, text_vscroll;
	uint8_t flip, bgbank, palbank, topbottom;
} s_vs;

static void capture(void)
{
	memcpy(s_vs.spr, t.spr, sizeof(s_vs.spr));
	memcpy(s_vs.tile, t.tile, sizeof(s_vs.tile));
	if (s_has_fg)
		memcpy(s_vs.text, t.text, sizeof(s_vs.text));
	memcpy(s_vs.scroll, t.scroll, sizeof(s_vs.scroll));
	s_vs.hscroll = t.hscroll;
	s_vs.vscroll = t.vscroll;
	s_vs.text_vscroll = t.text_vscroll;
	s_vs.flip = t.flip;
	s_vs.bgbank = t.bgbank;
	s_vs.palbank = t.palbank;
	s_vs.topbottom = t.topbottom;
}

static inline uint32_t mk(uint32_t code, uint32_t color, uint32_t flags) { return code | color << 16 | flags; }

// (the get_*_tile_info: code | color << 16 | T_FX | T_GROUP, T_GROUP also the Kung-Fu Master's category)
static inline uint32_t bg_tile(int idx, int row)
{
	const uint8_t *r = s_vs.tile;
	if (s_kind == K_KUNGFUM) {
		const uint32_t code = r[idx], c = r[idx + 0x800];
		return mk(code | (c & 0xc0) << 2, c & 0x1f, ((c & 0x20) ? T_FX : 0) | (row < 6 || ((c & 0x1f) >> 1) > 0x0c ? T_GROUP : 0));
	}
	const uint32_t code = r[idx * 2], c = r[idx * 2 + 1];
	switch (s_kind) {
	case K_BATTROAD:
		return mk(code | (c & 0x40) << 3 | (c & 0x10) << 4, c & 0x0f, ((c & 0x20) ? T_FX : 0) | (((c & 0x1f) >> 1) >= 4 ? T_GROUP : 0));
	case K_LDRUN:
		return mk(code | (c & 0xc0) << 2, c & 0x1f, ((c & 0x20) ? T_FX : 0) | (((c & 0x1f) >> 1) >= 0x0c ? T_GROUP : 0));
	case K_LDRUN2:
	case K_LDRUN3:
		return mk(code | (c & 0xc0) << 2, c & 0x1f, ((c & 0x20) ? T_FX : 0) | (((c & 0x1f) >> 1) >= 4 ? T_GROUP : 0));
	case K_LDRUN4:
		return mk(code | (c & 0xc0) << 2 | (c & 0x20) << 5, c & 0x1f, 0);
	case K_LOTLOT:
		return mk(code | (c & 0xc0) << 2, c & 0x1f, (c & 0x20) ? T_FX : 0);
	case K_KIDNIKI:
		return mk(code | (c & 0xe0) << 3 | (uint32_t)s_vs.bgbank << 11, c & 0x1f, (c & 0xe0) == 0xe0 ? T_GROUP : 0);
	case K_SPELUNKR:
		return mk(code | (c & 0x10) << 4 | (c & 0x20) << 6 | (c & 0xc0) << 3, (c & 0x0f) | (uint32_t)s_vs.palbank << 4, 0);
	case K_SPELUNK2:
		return mk(code | (c & 0xf0) << 4, (c & 0x0f) | (uint32_t)s_vs.palbank << 4, 0);
	case K_YOUJYUDN:
		return mk(code | (c & 0x60) << 3, c & 0x1f, ((c & 0x1f) >> 1) >= 8 ? T_GROUP : 0);
	default:   // (K_HORIZON)
		return mk(code | (c & 0xc0) << 2 | (c & 0x20) << 5, c & 0x1f, ((c & 0x1f) >> 1) >= 8 ? T_GROUP : 0);
	}
}

static inline uint32_t fg_tile(int idx)
{
	const uint32_t code = s_vs.text[idx * 2], c = s_vs.text[idx * 2 + 1];
	switch (s_kind) {
	case K_BATTROAD: return mk(code | (c & 0x40) << 3 | (c & 0x10) << 4, c & 0x0f, 0);
	case K_YOUJYUDN: return mk(code | (c & 0xc0) << 2, c & 0x0f, 0);
	case K_SPELUNKR:
	case K_SPELUNK2: return mk(code | (c & 0x10) << 4, (c & 0x0f) | (uint32_t)s_vs.palbank << 4, 0);
	default: return mk(code | (c & 0xc0) << 2, c & 0x1f, 0);   // (lotlot, kidniki)
	}
}

static inline int mod(int v, int n) { v %= n; return v < 0 ? v + n : v; }

// (tilemap draw: x on screen = x in the map - scroll; flipped, MAME's effective scroll: screen size - map size + scroll)
static inline int row_scroll(int row, int base)
{
	if (s_kind == K_KUNGFUM)
		return row < 6 ? 0 : s_vs.hscroll;
	if (s_kind == K_HORIZON)
		return s_vs.scroll[row << 1] | s_vs.scroll[(row << 1) | 1] << 8;
	return base;
}

static void draw_layer(int fg, int mode, int scrollx, int scrolly)
{
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch, flip = s_vs.flip;
	const rbx_gfx *g = fg ? &s_fg : &s_bg;
	const int cols = fg ? s_fcols : s_bcols, rows = fg ? s_frows : s_brows;
	const int tw = fg ? s_ftw : s_btw, th = fg ? s_fth : s_bth;
	const int W = cols * tw, H = rows * th, n = tw * th, vw = s_vw;
	const int vy = flip ? LINES - H + scrolly : -scrolly;
	const uint32_t mul = fg ? (s_fbits == 2 ? 4 : 8) : 8;
	const uint32_t base0 = fg ? s_fg_base : 0;
	for (int pr = 0; pr < rows; pr++) {
		const int r = flip ? rows - 1 - pr : pr;
		const int yrel = mod(pr * th + vy, H);
		if (yrel >= 256 && yrel + th <= H)
			continue;
		const int rs = fg ? scrollx : row_scroll(r, scrollx);
		const int vx = flip ? s_htotal - W + rs : -rs;
		for (int pc = 0; pc < cols; pc++) {
			const int c = flip ? cols - 1 - pc : pc;
			const int xrel = mod(pc * tw + vx - s_vx0, W);
			if (xrel >= vw && xrel + tw <= W)
				continue;
			const uint32_t ti = fg ? fg_tile(r * cols + c) : bg_tile(r * cols + c, r);
			const int group = (ti & T_GROUP) != 0;
			if ((mode == DRAW_FRONT && !group) || (mode == DRAW_CAT0 && group) || (mode == DRAW_CAT1 && !group))
				continue;
			const uint32_t code = rbx_wrap(ti & 0xffff, g->count);
			if ((mode == DRAW_FRONT || mode == DRAW_TEXT) && !g->used[code])
				continue;
			const uint32_t tm = mode == DRAW_FRONT || mode == DRAW_TEXT ? 1 : 0;
			const uint32_t base = base0 + ((ti >> 16) & 0xff) * mul;
			const uint8_t *src = g->pix + code * (uint32_t)n;
			const int fx = ((ti & T_FX) != 0) ^ flip;
			for (int k = 0; k < 2; k++) {   // (a tile across the map's wrap comes twice)
				const int y = k ? yrel - H : yrel;
				if (y >= 256 || y + th <= 0)
					continue;
				irem_blit(bm, pitch, 0, 0, vw, 256, src, tw, th, base, tm, fx, flip, xrel, y);
				if (xrel + tw > W)
					irem_blit(bm, pitch, 0, 0, vw, 256, src, tw, th, base, tm, fx, flip, xrel - W, y);
			}
		}
	}
}

static void draw_sprites(int colormask, int prioritymask, int priority)
{
	uint16_t *bm = rbx_video.bitmap;
	const uint8_t *sr = s_vs.spr;
	for (int offs = 0; offs < s_spr_bytes; offs += 8, sr += 8) {
		if ((sr[0] & prioritymask) != priority)
			continue;
		int code = sr[4] + ((sr[5] & 0x07) << 8);
		const int col = sr[0] & colormask;
		int sx = 256 * (sr[7] & 1) + sr[6];
		int sy = 256 + 128 - 15 - (256 * (sr[3] & 1) + sr[2]);
		int flipx = sr[5] & 0x40, flipy = sr[5] & 0x80;
		int i = s_hprom[(code >> 5) & 0x1f], incr;
		if (i == 1) {   // (double height)
			code &= ~1;
			sy -= 16;
		} else if (i == 2) {   // (quadruple height)
			i = 3;
			code &= ~3;
			sy -= 3 * 16;
		}
		if (s_vs.flip) {
			sx = 496 - sx;
			sy = 242 - i * 16 - sy;   // (sprites are slightly misplaced by the hardware)
			flipx = !flipx;
			flipy = !flipy;
		}
		if (flipy) {
			incr = -1;
			code += i;
		} else
			incr = 1;
		do {
			const uint32_t c = rbx_wrap((uint32_t)(code + i * incr), s_spr.count);
			if (s_spr.used[c])
				irem_blit(bm, rbx_video.pitch, 0, 0, s_vw, 256, s_spr.pix + c * 256, 16, 16, PEN_SP + (uint32_t)(col & 31) * 8, 1, flipx,
				          flipy, sx - s_vx0, sy + 16 * i);
			i--;
		} while (i >= 0);
	}
}

static void m62_render(void)
{
	const int hs = s_vs.hscroll, vs = s_vs.vscroll;
	switch (s_kind) {
	case K_KUNGFUM:
		draw_layer(0, DRAW_CAT0, 0, 0);
		draw_sprites(0x1f, 0x00, 0x00);
		draw_layer(0, DRAW_CAT1, 0, 0);
		break;
	case K_BATTROAD:
	case K_LDRUN:
	case K_LDRUN2:
	case K_LDRUN3:
		draw_layer(0, DRAW_OPAQUE, hs, vs);
		draw_sprites(0x0f, 0x10, 0x00);
		draw_layer(0, DRAW_FRONT, hs, vs);
		draw_sprites(0x0f, 0x10, 0x10);
		if (s_kind == K_BATTROAD)
			draw_layer(1, DRAW_TEXT, 128, 0);
		if (s_kind == K_LDRUN3 && s_vs.topbottom) {
			for (int y = 0; y < 256; y = y == 7 ? 248 : y + 1) {
				uint16_t *d = rbx_video.bitmap + y * rbx_video.pitch;
				for (int x = 0; x < s_vw; x++)
					d[x] = PEN_BLACK;
			}
		}
		break;
	case K_LDRUN4:
		draw_layer(0, DRAW_OPAQUE, hs - 2, 0);
		draw_sprites(0x1f, 0x00, 0x00);
		break;
	case K_LOTLOT:
		draw_layer(0, DRAW_OPAQUE, hs - 64, vs + 32);
		draw_layer(1, DRAW_TEXT, -64, 32);
		draw_sprites(0x1f, 0x00, 0x00);
		break;
	case K_KIDNIKI:
		draw_layer(0, DRAW_OPAQUE, hs, 0);
		draw_sprites(0x1f, 0x00, 0x00);
		draw_layer(0, DRAW_FRONT, hs, 0);
		draw_layer(1, DRAW_TEXT, -64, s_vs.text_vscroll + 128);
		break;
	case K_SPELUNKR:
		draw_layer(0, DRAW_OPAQUE, hs, vs + 128);
		draw_sprites(0x1f, 0x00, 0x00);
		draw_layer(1, DRAW_TEXT, -64, 0);
		break;
	case K_SPELUNK2:
		draw_layer(0, DRAW_OPAQUE, hs - 1, vs + 128);
		draw_sprites(0x1f, 0x00, 0x00);
		draw_layer(1, DRAW_TEXT, -65, 0);
		break;
	case K_YOUJYUDN:
		draw_layer(0, DRAW_OPAQUE, hs, 0);
		draw_sprites(0x1f, 0x00, 0x00);
		draw_layer(0, DRAW_FRONT, hs, 0);
		draw_layer(1, DRAW_TEXT, -64, 0);
		break;
	case K_HORIZON:
		draw_layer(0, DRAW_OPAQUE, 0, 0);
		draw_sprites(0x1f, 0x00, 0x00);
		draw_layer(0, DRAW_FRONT, 0, 0);
		break;
	}
}

// ---- the Z80's bus ----
static void set_bank(uint8_t *field, uint8_t v)
{
	*field = v;
	if (s_kind == K_SPELUNK2) {
		for (int p = 0; p < 0x10; p++) {
			s_map[0x80 + p] = s_rom + 0x20000 + (uint32_t)t.bank1 * 0x1000 + (uint32_t)p * 256;
			s_map[0x90 + p] = s_rom + 0x10000 + (uint32_t)t.bank2 * 0x1000 + (uint32_t)p * 256;
		}
		return;
	}
	const int base = s_kind == K_BATTROAD ? 0xa0 : 0x80;
	const int size = s_kind == K_LDRUN4 || s_kind == K_YOUJYUDN ? 0x4000 : 0x2000;
	for (int p = 0; p < size / 256; p++)
		s_map[base + p] = s_rom + 0x10000 + (uint32_t)t.bank1 * (uint32_t)size + (uint32_t)p * 256;
}

static uint8_t rd(void *u, uint16_t a)
{
	(void)u;
	const int k = s_kind;
	if (a < 0x8000)
		return s_rom[a];
	if (a >= 0xe000)
		return a < 0xf000 ? t.ram[a - 0xe000] : 0;
	if (k == K_LDRUN3 || k == K_HORIZON) {
		if (a < 0xc000)
			return s_rom[a];
	}
	if (a >= 0xa000 && a < 0xc000) {
		if (k == K_KIDNIKI || k == K_LOTLOT) {
			if (a < 0xb000)
				return k == K_KIDNIKI ? t.tile[a - 0xa000] : t.text[a - 0xa000];
			return 0;
		}
		if (k == K_SPELUNKR || k == K_SPELUNK2)
			return t.tile[a - 0xa000];
	}
	if (a < 0xc000) {
		const uint8_t *m = s_map[a >> 8];
		return m ? m[a & 0xff] : 0;
	}
	if (a >= 0xd000) {
		if (k == K_KIDNIKI)
			return t.text[a - 0xd000];
		if (k == K_SPELUNKR || k == K_SPELUNK2)
			return 0;
		if (k == K_YOUJYUDN)
			return a < 0xd800 ? t.tile[a - 0xd000] : 0;
		return t.tile[a - 0xd000];
	}
	if (a >= 0xc800) {
		if (k == K_LDRUN3)
			return a == 0xc800 ? 5 : a == 0xcc00 || a == 0xcfff ? 7 : 0;
		if (k == K_BATTROAD || k == K_SPELUNKR || k == K_SPELUNK2 || k == K_YOUJYUDN)
			return t.text[a - 0xc800];
		if (k == K_HORIZON && a < 0xc840)
			return t.scroll[a - 0xc800];
		return 0;
	}
	if (k == K_HORIZON && a < 0xc200)
		return t.spr[a - 0xc000];
	return 0;
}

static void wr(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	const int k = s_kind;
	if (a >= 0xe000) {
		if (a < 0xf000)
			t.ram[a - 0xe000] = v;
		return;
	}
	if (a >= 0xc000 && a < 0xc100) {
		t.spr[a - 0xc000] = v;
		return;
	}
	switch (k) {
	case K_KUNGFUM:
		if (a == 0xa000)
			t.hscroll = (t.hscroll & 0xff00) | v;
		else if (a == 0xb000)
			t.hscroll = (t.hscroll & 0xff) | v << 8;
		else if (a >= 0xd000)
			t.tile[a - 0xd000] = v;
		break;
	case K_LOTLOT:
		if (a >= 0xa000 && a < 0xb000)
			t.text[a - 0xa000] = v;
		else if (a >= 0xd000)
			t.tile[a - 0xd000] = v;
		break;
	case K_KIDNIKI:
		if (a >= 0xa000 && a < 0xb000)
			t.tile[a - 0xa000] = v;
		else if (a >= 0xd000)
			t.text[a - 0xd000] = v;
		break;
	case K_SPELUNKR:
		if (a >= 0xa000 && a < 0xc000)
			t.tile[a - 0xa000] = v;
		else if (a >= 0xc800 && a < 0xd000)
			t.text[a - 0xc800] = v;
		else if (a == 0xd000)
			t.vscroll = (t.vscroll & 0xff00) | v;
		else if (a == 0xd001)
			t.vscroll = (t.vscroll & 0xff) | v << 8;
		else if (a == 0xd002)
			t.hscroll = (t.hscroll & 0xff00) | v;
		else if (a == 0xd003)
			t.hscroll = (t.hscroll & 0xff) | v << 8;
		else if (a == 0xd004)
			set_bank(&t.bank1, v & 3);
		else if (a == 0xd005)
			t.palbank = v & 1;
		break;
	case K_SPELUNK2:
		if (a >= 0xa000 && a < 0xc000)
			t.tile[a - 0xa000] = v;
		else if (a >= 0xc800 && a < 0xd000)
			t.text[a - 0xc800] = v;
		else if (a == 0xd000)
			t.vscroll = (t.vscroll & 0xff00) | v;
		else if (a == 0xd001)
			t.hscroll = (t.hscroll & 0xff00) | v;
		else if (a == 0xd002) {
			t.hscroll = (t.hscroll & 0xff) | ((v & 2) >> 1) << 8;
			t.vscroll = (t.vscroll & 0xff) | (v & 1) << 8;
			t.palbank = (v & 0x0c) >> 2;
		} else if (a == 0xd003) {
			t.bank1 = (v & 0xc0) >> 6;
			set_bank(&t.bank2, (v & 0x3c) >> 2);
		}
		break;
	case K_YOUJYUDN:
		if (a >= 0xc800 && a < 0xd000)
			t.text[a - 0xc800] = v;
		else if (a >= 0xd000 && a < 0xd800)
			t.tile[a - 0xd000] = v;
		break;
	case K_BATTROAD:
		if (a >= 0xc800 && a < 0xd000)
			t.text[a - 0xc800] = v;
		else if (a >= 0xd000)
			t.tile[a - 0xd000] = v;
		break;
	case K_LDRUN4:
		if (a == 0xc800)
			set_bank(&t.bank1, v & 1);
		else if (a >= 0xd000)
			t.tile[a - 0xd000] = v;
		break;
	case K_HORIZON:
		if (a >= 0xc000 && a < 0xc200)
			t.spr[a - 0xc000] = v;
		else if (a >= 0xc800 && a < 0xc840)
			t.scroll[a - 0xc800] = v;
		else if (a >= 0xd000)
			t.tile[a - 0xd000] = v;
		break;
	default:
		if (a >= 0xd000)
			t.tile[a - 0xd000] = v;
		break;
	}
}

static uint8_t pin(z80 *z, uint8_t p)
{
	(void)z;
	if (p < 5)
		return (uint8_t)rbx_port_read(s_port[p]);
	if (p == 0x80 && s_kind == K_LDRUN2) {   // (ldrun2_bankswitch_r: the service mode's ROM test)
		if (t.ldrun2_swap && !--t.ldrun2_swap)
			set_bank(&t.bank1, 1);
		return 0;
	}
	return 0;
}

static void pout(z80 *z, uint8_t p, uint8_t v)
{
	(void)z;
	static const uint8_t banks[30] = { 0,0,0,0,0,1,0,1,0,0, 0,1,1,1,1,1,0,0,0,0, 1,0,1,1,1,1,1,1,1,1 };
	if (p == 0)
		irem_audio_cmd_w(v);
	else if (p == 1) {   // (m62_flipscreen_w: the flip is both software and hardware)
		v ^= (uint8_t)(~rbx_port_read(s_port[4]) & 1);
		t.flip = v & 1;
	} else if (p & 0x80) {
		switch (s_kind) {
		case K_BATTROAD:
			if (p == 0x80)
				t.vscroll = (t.vscroll & 0xff00) | v;
			else if (p == 0x81)
				t.hscroll = (t.hscroll & 0xff) | v << 8;
			else if (p == 0x82)
				t.hscroll = (t.hscroll & 0xff00) | v;
			else if (p == 0x83)
				set_bank(&t.bank1, v & 0x0f);
			break;
		case K_LDRUN2:
			if (p == 0x80 || p == 0x81) {
				t.bankctl[p & 1] = v;
				if (p == 0x80) {
					if (v >= 1 && v <= 30)
						set_bank(&t.bank1, banks[v - 1]);
				} else
					t.ldrun2_swap = t.bankctl[0] == 0x01 && v == 0x0d ? 2 : 0;
			}
			break;
		case K_LDRUN3:
			if (p == 0x80)
				t.vscroll = (t.vscroll & 0xff00) | v;
			else if (p == 0x81)
				t.topbottom = v & 1;
			break;
		case K_LDRUN4:
			if (p == 0x82)
				t.hscroll = (t.hscroll & 0xff) | v << 8;
			else if (p == 0x83)
				t.hscroll = (t.hscroll & 0xff00) | v;
			break;
		case K_KIDNIKI:
			if (p == 0x80)
				t.hscroll = (t.hscroll & 0xff00) | v;
			else if (p == 0x81)
				t.hscroll = (t.hscroll & 0xff) | v << 8;
			else if (p == 0x82)
				t.text_vscroll = (t.text_vscroll & 0xff00) | v;
			else if (p == 0x83)
				t.text_vscroll = (t.text_vscroll & 0xff) | v << 8;
			else if (p == 0x84)
				t.bgbank = v & 1;
			else if (p == 0x85)
				set_bank(&t.bank1, v & 0x0f);
			break;
		case K_YOUJYUDN:
			if (p == 0x80)
				t.hscroll = (t.hscroll & 0xff) | v << 8;
			else if (p == 0x81)
				t.hscroll = (t.hscroll & 0xff00) | v;
			else if (p == 0x83)
				set_bank(&t.bank1, v & 1);
			break;
		}
	}
}

static void z_fix(z80 *z)
{
	z->read_byte = rd;
	z->write_byte = wr;
	z->port_in = pin;
	z->port_out = pout;
	z->userdata = NULL;
	z->read_map = s_map;
	z->op_rom = NULL;
}

// ---- init / frame ----
static void m62_exit(void);
static void m62_reset(void);

// (compute_res_net, RES_NET_VCC_5V | RES_NET_VIN_TTL_OUT, no amplifier)
static int res_net(int inputs, int n, const double *r, double rgnd)
{
	double rtotal = 0, v = 0;
	for (int i = 0; i < n; i++)
		if (!((inputs >> i) & 1)) {
			rtotal += 1.0 / r[i];
			v += TTL_VOL / r[i];
		}
	if (rgnd != 0.0)
		rtotal += 1.0 / rgnd;
	const int open = rtotal != 0 && v / rtotal > TTL_VOH;
	for (int i = 0; i < n; i++)
		if (((inputs >> i) & 1) && !open) {
			rtotal += 1.0 / (r[i] + 50);
			v += TTL_VOH / (r[i] + 50);
		}
	rtotal = 1.0 / rtotal;
	v *= rtotal;
	if (v < 0)
		v = 0;
	return (int)(v * 255 / 5.0 + 0.4);
}

static inline uint32_t net4(int r, int g, int b)
{
	static const double res[4] = { 2200, 1000, 470, 220 };
	return (uint32_t)(res_net(r, 4, res, 470) << 16 | res_net(g, 4, res, 470) << 8 | res_net(b, 4, res, 470));
}

static void palette(const uint8_t *spr, const uint8_t *chr, const uint8_t *fgp)
{
	uint32_t *pal = rbx_video.palette;
	static uint32_t c[PEN_BLACK];
	memset(c, 0, sizeof(c));
	const int nchr = s_kind == K_SPELUNK2 ? 512 : 256;
	for (int i = 0; i < 256; i++)
		c[PEN_SP + i] = net4(spr[i] & 15, spr[0x100 + i] & 15, spr[0x200 + i] & 15);
	for (int i = 0; i < nchr; i++)
		c[i] = s_kind == K_SPELUNK2 ? net4(chr[i] & 15, chr[i] >> 4, chr[0x200 + i] & 15)
		                            : net4(chr[i] & 15, chr[0x100 + i] & 15, chr[0x200 + i] & 15);
	int nfg = 0;
	if (s_kind == K_LOTLOT) {
		nfg = 256;
		for (int i = 0; i < 256; i++)
			c[PEN_FG + i] = net4(fgp[i] & 15, fgp[0x100 + i] & 15, fgp[0x200 + i] & 15);
	} else if (s_kind == K_BATTROAD) {
		static const double r2[2] = { 470, 220 }, r3[3] = { 1000, 470, 220 };
		for (int i = 0; i < 256; i++) {   // (32 entries; the colours past them wrap)
			const int v = fgp[i & 31];
			c[PEN_FG + i] = (uint32_t)(res_net((v >> 6) & 3, 2, r2, 0) << 16 | res_net((v >> 3) & 7, 3, r3, 0) << 8 | res_net(v & 7, 3, r3, 0));
		}
	}
	// (m62_amplify_contrast: the palette is very dark; the Battle-Road's text left out)
	uint32_t ymax = 1;
	for (int i = 0; i < PEN_BLACK; i++) {
		if ((i >= nchr && i < PEN_SP) || (i >= PEN_FG && i < PEN_FG + nfg ? 0 : i >= PEN_FG))
			continue;
		const uint32_t rgb = c[i], y = 299 * (rgb >> 16) + 587 * ((rgb >> 8) & 255) + 114 * (rgb & 255);
		if (y > ymax)
			ymax = y;
	}
	const float contrast = (float)(255000.0 / ymax);
	for (int i = 0; i < PEN_BLACK; i++) {
		if (i >= PEN_FG && s_kind == K_BATTROAD) {
			pal[i] = c[i];
			continue;
		}
		const uint32_t rgb = c[i];
		int r = (int)((float)(rgb >> 16) * contrast), g = (int)((float)((rgb >> 8) & 255) * contrast);
		int b = (int)((float)(rgb & 255) * contrast);
		r = r > 255 ? 255 : r;
		g = g > 255 ? 255 : g;
		b = b > 255 ? 255 : b;
		pal[i] = (uint32_t)(r << 16 | g << 8 | b);
	}
	pal[PEN_BLACK] = 0;
}

static int m62_init(const rbx_game *g)
{
	rbx_cheat_prefer("z80");   // (the cheats' main CPU: another CPU is set up first)
	static const struct { const char *m; int k; } kinds[] = {
		{ "kungfum", K_KUNGFUM }, { "battroad", K_BATTROAD }, { "ldrun", K_LDRUN }, { "ldrun2", K_LDRUN2 }, { "ldrun3", K_LDRUN3 },
		{ "ldrun4", K_LDRUN4 }, { "lotlot", K_LOTLOT }, { "kidniki", K_KIDNIKI }, { "spelunkr", K_SPELUNKR },
		{ "spelunk2", K_SPELUNK2 }, { "youjyudn", K_YOUJYUDN }, { "horizon", K_HORIZON },
	};
	s_kind = 0;
	for (size_t i = 0; i < sizeof(kinds) / sizeof(kinds[0]); i++)
		if (!strcmp(g->machine, kinds[i].m))
			s_kind = kinds[i].k;
	const int k = s_kind;
	uint32_t s1 = 0, s2 = 0, s3 = 0, hs = 0, ss = 0, cs = 0, fs = 0;
	s_rom = rbx_region("maincpu", &s_rom_size);
	const uint8_t *gfx1 = rbx_region("gfx1", &s1), *gfx2 = rbx_region("gfx2", &s2), *gfx3 = rbx_region("gfx3", &s3);
	s_hprom = rbx_region("spr_height_prom", &hs);
	const uint8_t *spr = rbx_region("spr_color_proms", &ss), *chr = rbx_region("chr_color_proms", &cs);
	const uint8_t *fgp = rbx_region("fg_color_proms", &fs);
	s_has_fg = k == K_BATTROAD || k == K_LOTLOT || k == K_KIDNIKI || k == K_SPELUNKR || k == K_SPELUNK2 || k == K_YOUJYUDN;
	if (!s_rom || !gfx1 || !gfx2 || !s_hprom || hs < 0x20 || !spr || ss < 0x300 || !chr || cs < 0x300 ||
	    (s_has_fg && !gfx3) || ((k == K_LOTLOT || k == K_BATTROAD) && !fgp)) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	memset(&t, 0, sizeof(t));
	const int k_6m = k == K_KUNGFUM || k == K_BATTROAD || k == K_YOUJYUDN || k == K_HORIZON;
	s_vx0 = k_6m ? 128 : 64;
	s_vw = k_6m ? 256 : 384;
	s_htotal = k_6m ? 384 : 512;
	s_line_cyc = k == K_HORIZON ? 250 : s_htotal / 2;
	const uint32_t pix = k_6m ? 6144000u : 8000000u;
	s_bcols = 64;
	s_brows = 32;
	s_btw = s_bth = 8;
	s_spr_bytes = k == K_HORIZON ? 0x200 : 0x100;
	s_fbits = 3;
	s_fg_base = 0;
	if (k == K_LOTLOT) {
		s_bcols = 32;
		s_brows = 64;
		s_btw = 12;
		s_bth = 10;
	} else if (k == K_SPELUNKR || k == K_SPELUNK2)
		s_brows = 64;
	else if (k == K_YOUJYUDN) {
		s_brows = 16;
		s_bth = 16;
	}
	if (k == K_BATTROAD) {
		s_fcols = s_frows = 32;
		s_ftw = s_fth = 8;
		s_fbits = 2;
		s_fg_base = PEN_FG;
	} else if (k == K_LOTLOT) {
		s_fcols = 32;
		s_frows = 64;
		s_ftw = 12;
		s_fth = 10;
		s_fg_base = PEN_FG;
	} else if (k == K_KIDNIKI) {
		s_fcols = 32;
		s_frows = 64;
		s_ftw = 12;
		s_fth = 8;
	} else {
		s_fcols = s_frows = 32;
		s_ftw = 12;
		s_fth = 8;
		if (k == K_YOUJYUDN)
			s_fg_base = 128;
	}
	// ---- the layouts ----
	static const uint32_t ix[8] = { 0, 1, 2, 3, 4, 5, 6, 7 };
	static const uint32_t iy[16] = { 0, 8, 16, 24, 32, 40, 48, 56, 64, 72, 80, 88, 96, 104, 112, 120 };
	const uint32_t pf[3] = { s1 / 3 * 16, s1 / 3 * 8, 0 };   // (gfx_8x8x3_planar over gfx1)
	const rbx_gfx_layout tl = { 8, 8, 3, pf, ix, iy, 64 };
	const rbx_gfx_layout yl = { 8, 16, 3, pf, ix, iy, 16 * 8 };   // (youjyudn_tilelayout)
	static const uint32_t lot_p[3] = { 0, 256 * 32 * 8, 2 * 256 * 32 * 8 };
	static const uint32_t lot_x[12] = { 0, 1, 2, 3, 16 * 8 + 0, 16 * 8 + 1, 16 * 8 + 2, 16 * 8 + 3, 16 * 8 + 4, 16 * 8 + 5, 16 * 8 + 6, 16 * 8 + 7 };
	const rbx_gfx_layout ll = { 12, 10, 3, lot_p, lot_x, iy, 32 * 8 };
	static const uint32_t kid_p[3] = { 0, 0x4000 * 8, 2 * 0x4000 * 8 };
	static const uint32_t kid_x[12] = { 0, 1, 2, 3, 64 + 0, 64 + 1, 64 + 2, 64 + 3, 64 + 4, 64 + 5, 64 + 6, 64 + 7 };
	const rbx_gfx_layout kl = { 12, 8, 3, kid_p, kid_x, iy, 16 * 8 };
	static const uint32_t sp2_x[12] = { 0, 1, 2, 3, 0x2000 * 8 + 0, 0x2000 * 8 + 1, 0x2000 * 8 + 2, 0x2000 * 8 + 3, 0x2000 * 8 + 4, 0x2000 * 8 + 5,
	                                    0x2000 * 8 + 6, 0x2000 * 8 + 7 };
	const rbx_gfx_layout s2l = { 12, 8, 3, kid_p, sp2_x, iy, 8 * 8 };
	static const uint32_t bat_p[2] = { 0, 1024 * 8 * 8 };
	const rbx_gfx_layout bl = { 8, 8, 2, bat_p, ix, iy, 8 * 8 };
	const uint32_t spf[3] = { s2 / 3 * 16, s2 / 3 * 8, 0 };
	uint32_t x16[16];
	for (int i = 0; i < 16; i++)
		x16[i] = (uint32_t)(i < 8 ? i : 16 * 8 + i - 8);
	const rbx_gfx_layout sl = { 16, 16, 3, spf, x16, iy, 32 * 8 };
	int bad = rbx_gfx_decode(&s_spr, &sl, gfx2, s2 / 3);
	if (k == K_LOTLOT)
		bad |= rbx_gfx_decode(&s_bg, &ll, gfx1, 256 * 32);
	else if (k == K_YOUJYUDN)
		bad |= rbx_gfx_decode(&s_bg, &yl, gfx1, s1 / 3);
	else
		bad |= rbx_gfx_decode(&s_bg, &tl, gfx1, s1 / 3);
	if (k == K_BATTROAD)
		bad |= rbx_gfx_decode(&s_fg, &bl, gfx3, 1024 * 8);
	else if (k == K_LOTLOT)
		bad |= rbx_gfx_decode(&s_fg, &ll, gfx3, 256 * 32);
	else if (k == K_KIDNIKI || k == K_YOUJYUDN)
		bad |= rbx_gfx_decode(&s_fg, &kl, gfx3, 1024 * 16);
	else if (k == K_SPELUNKR || k == K_SPELUNK2)
		bad |= rbx_gfx_decode(&s_fg, &s2l, gfx3, 512 * 8);
	static const char *const pn[5] = { "SYSTEM", "P1", "P2", "DSW1", "DSW2" };
	for (int i = 0; i < 5; i++)
		s_port[i] = rbx_port_find(pn[i]);
	if (bad || rbx_video_setup(s_vw, 256, NPENS) || irem_audio_init(IREM_AUDIO_M62, (uint32_t)s_htotal, pix)) {
		rbx_fail("%s: out of memory", g->set);
		m62_exit();
		return -1;
	}
	rbx_audio.frame_mhz = (uint32_t)((uint64_t)pix * 1000 / ((uint32_t)s_htotal * LINES));
	palette(spr, chr, fgp);
	m62_reset();
	return 0;
}

static void m62_exit(void)
{
	irem_audio_exit();
	rbx_gfx_free(&s_bg);
	rbx_gfx_free(&s_spr);
	if (s_has_fg)
		rbx_gfx_free(&s_fg);
	s_has_fg = 0;
}

static void map_static(void)
{
	memset(s_map, 0, sizeof(s_map));
	const int top = s_kind == K_LDRUN3 || s_kind == K_HORIZON ? 0xc0 : 0x80;
	for (int p = 0; p < top; p++)
		s_map[p] = s_rom + p * 256;
	for (int p = 0; p < 0x10; p++)
		s_map[0xe0 + p] = t.ram + p * 256;
	switch (s_kind) {
	case K_BATTROAD: case K_LDRUN2: case K_LDRUN4: case K_KIDNIKI: case K_SPELUNKR: case K_SPELUNK2: case K_YOUJYUDN:
		set_bank(&t.bank1, t.bank1);
		break;
	}
}

static void m62_reset(void)
{
	irem_audio_reset();
	memset(&t, 0, offsetof(__typeof__(t), state_end));
	memset(&s_vs, 0, sizeof(s_vs));
	z80_init(&s_z);
	z_fix(&s_z);
	s_z.int_data = 0xff;
	map_static();
}

static void m62_frame(void)
{
	for (int line = 0; line < LINES; line++) {
		if (line == 256) {   // (the update; irq0_line_hold)
			rbx_video_wait_idle();
			capture();
			rbx_video_frame_done();
			t.irq = 1;
		}
		t.main_end += (unsigned long)s_line_cyc;
		int irq = t.irq;
		z80_run(&s_z, t.main_end, &irq, 1);
		t.irq = (uint8_t)irq;
		irem_audio_line();
	}
	irem_audio_frame();
	t.frame++;
}

static int m62_state(void *ctx, int load)
{
	int rc = 0;
	rc |= irem_audio_state(ctx, load);
	rc |= rbx_state_io(ctx, "m62", &t, (uint32_t)offsetof(__typeof__(t), state_end));
	z80 z = s_z;
	rc |= rbx_state_io(ctx, "m62_z80", &z, sizeof(z));
	if (!rc && load) {
		z_fix(&z);
		s_z = z;
		map_static();
	}
	return rc ? -1 : 0;
}

static int m62_save(void *ctx) { return m62_state(ctx, 0); }
static int m62_load(void *ctx) { return m62_state(ctx, 1); }
static int m62_supported(const char *machine)
{
	static const char *const m[] = { "kungfum", "battroad", "ldrun", "ldrun2", "ldrun3", "ldrun4", "lotlot", "kidniki", "spelunkr",
	                                 "spelunk2", "youjyudn", "horizon" };
	for (size_t i = 0; i < sizeof(m) / sizeof(m[0]); i++)
		if (!strcmp(machine, m[i]))
			return 1;
	return 0;
}

const rbx_driver rbx_drv_m62 = { "Irem M62", m62_init, m62_exit, m62_reset, m62_frame, m62_save, m62_load, m62_supported,
                                 m62_render, NULL };
