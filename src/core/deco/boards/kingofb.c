/*  Wood Place / Data East King of Boxer and Ring King. Ported from MAME src/mame/dataeast/kingofb.cpp (BSD-3-Clause,
    copyright-holders: Ernesto Corvi).
    Four Z80 at 4 MHz: the main CPU, a video CPU and a sprite CPU (each sharing 2 KB with the main one, their IRQs held
    by the main CPU's writes, the three NMIs from vblank gated by a control bit), and the sound CPU (an NMI at 6 kHz,
    the latch on its IRQ and on the AY's port A, an AY-3-8910 and an 8-bit DAC). The video: a 16x16 map of 16x16 tiles
    (scrolled in y), a 32x32 text layer in 1 bpp of the primary colours, sprites; 256 colours from three 4-bit PROMs
    through resistor weights.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/z80/z80.h"
#include "../../sound/ay8910.h"
#include "../../video/gfxlayout.h"

#include <stdlib.h>
#include <string.h>

#define CLOCK 4000000
#define FRAME_MHZ 60000          // set_refresh_hz(60), set_size(256, 256), visarea y 16-239
#define LINES 256
#define VIS_Y0 16
#define VIS_H 224
#define VBL_LINES 39             // vblank 2500 us

static int s_rk;                            // Ring King's maps and graphics
static uint8_t *s_rom[4];
static uint32_t s_rom_size[4];
static uint8_t s_ram_main[0x400], s_ram_f0[0x800], s_share_spr[0x800], s_share_vid[0x800];
static uint8_t s_ram_vid[0x800], s_ram_spr[0x800], s_ram_snd[0x400];
static uint8_t s_vram[2][0x400], s_cram[2][0x400], s_spriteram[0x400], s_spr_extra[0x40];
static uint8_t s_scroll_y, s_control, s_slatch;
static int s_nmi_line, s_vid_irq, s_spr_irq, s_snd_irq;
static int32_t s_dac;
static int s_port_dsw[2], s_port_p1, s_port_p2, s_port_sys, s_port_extra;
static int s_line;
static uint32_t s_frame;
static uint64_t s_frac[4], s_ay_frac, s_nmi_frac;
static z80 s_cpu[4];                        // main, video, sprite, sound
static ay8910_t s_ay;
static int32_t *s_mix;
static int s_mix_cap;
static uint32_t s_rgb[0x110];
static rbx_gfx s_gfx[5];
static const uint8_t *s_map[4][256];
static struct {
	uint8_t vram[2][0x400], cram[2][0x400], spriteram[0x400], scroll_y, control;
} *s_snap;

// s_line counts from the vblank start (line 240), as MAME's screen begins there at time 0 with no vblank edge yet
static int vblank(void) { return s_frame && s_line < VBL_LINES; }

// the NMI gate (INPUT_MERGER_ALL_HIGH of vblank and the control's bit 5) to the three CPUs
static void update_nmi(void)
{
	const int n = vblank() && ((s_control >> 5) & 1);
	if (n && !s_nmi_line)
		for (int c = 0; c < 3; c++)
			z80_gen_nmi(&s_cpu[c]);
	s_nmi_line = n;
}

static uint8_t port_sys(void)
{
	if (s_rk)   // (Ring King: bit 5 the vblank, active low)
		rbx_port_custom(s_port_sys, 0x20, vblank() ? 0x20 : 0);
	return (uint8_t)rbx_port_read(s_port_sys);
}

// ---- main ----
static uint8_t main_rd(void *u, uint16_t a)
{
	(void)u;
	if (a < 0xc000) return s_rom[0][a];
	if (a < 0xc400) return s_ram_main[a - 0xc000];
	if (s_rk) {
		if (a >= 0xc800 && a < 0xd000) return s_share_spr[a - 0xc800];
		if (a >= 0xd000 && a < 0xd800) return s_share_vid[a - 0xd000];
		if (a >= 0xe000 && a < 0xe006) a = (uint16_t)(0xfc00 + (a - 0xe000));
		else if (a >= 0xf000 && a < 0xf800) return s_ram_f0[a - 0xf000];
		else return 0;
	} else {
		if (a >= 0xe000 && a < 0xe800) return s_share_spr[a - 0xe000];
		if (a >= 0xe800 && a < 0xf000) return s_share_vid[a - 0xe800];
		if (a >= 0xf000 && a < 0xf800) return s_ram_f0[a - 0xf000];
	}
	switch (a) {
	case 0xfc00: return (uint8_t)rbx_port_read(s_port_dsw[0]);
	case 0xfc01: return (uint8_t)rbx_port_read(s_port_dsw[1]);
	case 0xfc02: return (uint8_t)rbx_port_read(s_port_p1);
	case 0xfc03: return (uint8_t)rbx_port_read(s_port_p2);
	case 0xfc04: return port_sys();
	case 0xfc05: return (uint8_t)rbx_port_read(s_port_extra);
	}
	return 0;
}
static void control_w(uint8_t v)
{
	s_control = v;   // (bit 5 the NMI gate, 3-4 the palette bank, 7 the flip)
	update_nmi();
}
static void main_wr(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0xc000 && a < 0xc400) { s_ram_main[a - 0xc000] = v; return; }
	if (s_rk) {
		if (a >= 0xc800 && a < 0xd000) s_share_spr[a - 0xc800] = v;
		else if (a >= 0xd000 && a < 0xd800) s_share_vid[a - 0xd000] = v;
		else if (a == 0xd800) control_w(v);
		else if (a == 0xd801) s_spr_irq = 1;
		else if (a == 0xd802) s_vid_irq = 1;
		else if (a == 0xd803) { s_slatch = v; s_snd_irq = 1; }
		else if (a == 0xe800) s_scroll_y = v;
		else if (a >= 0xf000 && a < 0xf800) s_ram_f0[a - 0xf000] = v;
		return;
	}
	if (a >= 0xe000 && a < 0xe800) s_share_spr[a - 0xe000] = v;
	else if (a >= 0xe800 && a < 0xf000) s_share_vid[a - 0xe800] = v;
	else if (a >= 0xf000 && a < 0xf800) s_ram_f0[a - 0xf000] = v;
	else if (a == 0xf800) control_w(v);
	else if (a == 0xf802) s_scroll_y = v;
	else if (a == 0xf803) { s_spr_irq = 1; s_scroll_y = v; }   // scroll_interrupt_w
	else if (a == 0xf804) s_vid_irq = 1;
	else if (a == 0xf807) { s_slatch = v; s_snd_irq = 1; }
}

// ---- video CPU ----
static uint8_t *vid_ptr(uint16_t a)
{
	if (a >= 0x8000 && a < 0x8800) return &s_ram_vid[a - 0x8000];
	if (s_rk) {
		if (a >= 0xa000 && a < 0xa400) return &s_vram[1][a - 0xa000];
		if (a >= 0xa400 && a < 0xa800) return &s_cram[1][a - 0xa400];
		if (a >= 0xa800 && a < 0xa900) return &s_vram[0][a - 0xa800];
		if (a >= 0xac00 && a < 0xad00) return &s_cram[0][a - 0xac00];
		if (a >= 0xc000 && a < 0xc800) return &s_share_vid[a - 0xc000];
		return NULL;
	}
	if (a >= 0xa000 && a < 0xa800) return &s_share_vid[a - 0xa000];
	if (a >= 0xc000 && a < 0xc100) return &s_vram[0][a - 0xc000];
	if (a >= 0xc400 && a < 0xc500) return &s_cram[0][a - 0xc400];
	if (a >= 0xc800 && a < 0xcc00) return &s_vram[1][a - 0xc800];
	if (a >= 0xcc00 && a < 0xd000) return &s_cram[1][a - 0xcc00];
	return NULL;
}
static uint8_t vid_rd(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x4000) return s_rom[1][a];
	uint8_t *p = vid_ptr(a);
	return p ? *p : 0;
}
static void vid_wr(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	uint8_t *p = vid_ptr(a);
	if (p)
		*p = v;
}

// ---- sprite CPU ----
static uint8_t *spr_ptr(uint16_t a)
{
	if (a >= 0x8000 && a < 0x8800) return &s_ram_spr[a - 0x8000];
	if (s_rk) {
		if (a >= 0xc800 && a < 0xd000) return &s_share_spr[a - 0xc800];
		if (a >= 0xa000 && a < 0xa400) return &s_spriteram[a - 0xa000];
		if (a >= 0xa400 && a < 0xa440) return &s_spr_extra[a - 0xa400];
		return NULL;
	}
	if (a >= 0xa000 && a < 0xa800) return &s_share_spr[a - 0xa000];
	if (a >= 0xc000 && a < 0xc400) return &s_spriteram[a - 0xc000];
	if (a >= 0xc400 && a < 0xc440) return &s_spr_extra[a - 0xc400];
	return NULL;
}
static uint8_t spr_rd(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x2000) return s_rom[2][a];
	uint8_t *p = spr_ptr(a);
	return p ? *p : 0;
}
static void spr_wr(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	uint8_t *p = spr_ptr(a);
	if (p)
		*p = v;
}

// ---- sound CPU ----
static uint8_t snd_rd(void *u, uint16_t a)
{
	(void)u;
	if (a < 0xc000) return s_rom[3][a];
	return a < 0xc400 ? s_ram_snd[a - 0xc000] : 0;
}
static void snd_wr(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0xc000 && a < 0xc400)
		s_ram_snd[a - 0xc000] = v;
}
static uint8_t ay_porta(void *u) { (void)u; return s_slatch; }   // (the AY's port A: the latch)
static uint8_t snd_in(z80 *z, uint8_t p)
{
	(void)z;
	if ((!s_rk && p == 0x08) || (s_rk && p == 0x02))
		return ay8910_data_r(&s_ay);
	return 0xff;
}
static void snd_out(z80 *z, uint8_t p, uint8_t v)
{
	(void)z;
	if (p == 0x00)
		s_dac = (int32_t)v * 256 - 32768;   // DAC_8BIT_R2R
	else if (!s_rk) {
		if (p == 0x08) ay8910_data_w(&s_ay, v);
		else if (p == 0x0c) ay8910_address_w(&s_ay, v);
	} else if (p == 0x02) ay8910_data_w(&s_ay, v);
	else if (p == 0x03) ay8910_address_w(&s_ay, v);
}
static uint8_t no_in(z80 *z, uint8_t p) { (void)z; (void)p; return 0xff; }
static void no_out(z80 *z, uint8_t p, uint8_t v) { (void)z; (void)p; (void)v; }

static void hook(int c)
{
	static uint8_t (*const rd[4])(void *, uint16_t) = { main_rd, vid_rd, spr_rd, snd_rd };
	static void (*const wr[4])(void *, uint16_t, uint8_t) = { main_wr, vid_wr, spr_wr, snd_wr };
	z80 *z = &s_cpu[c];
	z->read_byte = rd[c];
	z->write_byte = wr[c];
	z->port_in = c == 3 ? snd_in : no_in;
	z->port_out = c == 3 ? snd_out : no_out;
	z->userdata = NULL;
	z->read_map = s_map[c];
	z->int_data = 0xff;
}

// ---- video ----
static uint16_t s_scr[256 * 256];

static const uint8_t *elem(const rbx_gfx *g, uint32_t code) { return g->pix + (size_t)rbx_wrap(code, g->count) * g->width * g->height; }

static void kb_render(void)
{
	for (int i = 0; i < 0x110; i++)
		rbx_video.palette[i] = s_rgb[i];
	const int flip = (s_snap->control >> 7) & 1, pbank = (s_snap->control >> 3) & 3;
	const int scrolly = -(int)s_snap->scroll_y;
	// the map: 16x16 tiles, TILEMAP_SCAN_COLS_FLIP_Y (16 rows), the first column tile 0; opaque
	for (int y = 0; y < 256; y++) {   // (a tile's row at a time: 16 pixels from x0, one tile)
		const int ty = flip ? 255 - ((y - scrolly) & 255) : (y + scrolly) & 255;
		for (int x0 = 0; x0 < 256; x0 += 16) {
			const int tx0 = flip ? 255 - x0 : x0;
			const int col = tx0 >> 4, row = ty >> 4, idx = col * 16 + (15 - row);
			const int attr = s_snap->cram[0][idx];
			int code;
			const rbx_gfx *g;
			if (s_rk) {
				code = idx / 16 ? s_snap->vram[0][idx] : 0;
				g = &s_gfx[4];
			} else {
				code = idx / 16 ? s_snap->vram[0][idx] + ((attr & 3) << 8) : 0;
				g = &s_gfx[2 + ((attr & 4) >> 2)];
			}
			const int color = ((attr & 0x70) >> 4) + 8 * pbank;
			const uint8_t *src = elem(g, (uint32_t)code) + (ty & 15) * 16;
			uint16_t *d = s_scr + y * 256 + x0;
			for (int i = 0; i < 16; i++)
				d[i] = (uint16_t)(color * 8 + src[flip ? 15 - i : i]);
		}
	}
	const uint8_t *sp = s_snap->spriteram;   // draw_sprites
	for (int offs = 0; offs < 0x400; offs += 4) {
		int bank, code, color, fx = 0, fy, sx, sy;
		if (s_rk) {
			bank = (sp[offs + 1] & 4) >> 2;
			code = sp[offs + 3] + ((sp[offs + 1] & 3) << 8);
			color = ((sp[offs + 1] & 0x70) >> 4) + 8 * pbank;
			fy = (sp[offs + 1] & 0x80) ? 0 : 1;
			sx = sp[offs + 2];
			sy = sp[offs];
		} else {   // (the offset scrambled: bitswap<16>(offs, 15..10, 4, 7, 6, 5, 9, 8, 3, 2, 1, 0) ^ 0x3c)
			int roffs = (offs & 0xfc0f) | ((offs >> 4) & 1) << 9 | ((offs >> 7) & 1) << 8 | ((offs >> 6) & 1) << 7 |
			            ((offs >> 5) & 1) << 6 | ((offs >> 9) & 1) << 5 | ((offs >> 8) & 1) << 4;
			roffs ^= 0x3c;
			if (roffs & 0x200)
				roffs ^= 0x1c0;
			roffs &= 0x3ff;
			bank = (sp[roffs + 3] & 4) >> 2;
			code = sp[roffs + 2] + ((sp[roffs + 3] & 3) << 8);
			color = ((sp[roffs + 3] & 0x70) >> 4) + 8 * pbank;
			fy = (sp[roffs + 3] & 0x80) != 0;
			sx = sp[roffs + 1];
			sy = sp[roffs];
		}
		if (flip) {
			sx = 240 - sx;
			sy = 240 - sy;
			fx = !fx;
			fy = !fy;
		}
		const uint8_t *src = elem(&s_gfx[2 + bank], (uint32_t)code);
		for (int yy = 0; yy < 16; yy++) {
			const int dy = sy + yy;
			if (dy < 0 || dy > 255)
				continue;
			for (int xx = 0; xx < 16; xx++) {
				const int dx = sx + xx;
				if (dx < 0 || dx > 255)
					continue;
				const uint8_t p = src[(fy ? 15 - yy : yy) * 16 + (fx ? 15 - xx : xx)];
				if (p)
					s_scr[dy * 256 + dx] = (uint16_t)(color * 8 + p);
			}
		}
	}
	for (int y = 0; y < 256; y++)   // the text: 8x8, SCAN_COLS_FLIP_Y (32 rows), 1 bpp: pen 256 + colour * 2 + 1
		for (int x0 = 0; x0 < 256; x0 += 8) {   // (a tile's row at a time)
			const int tx0 = flip ? 255 - x0 : x0, ty = flip ? 255 - y : y;
			const int idx = (tx0 >> 3) * 32 + (31 - (ty >> 3));
			const int attr = s_snap->cram[1][idx];
			const int code = s_snap->vram[1][idx] + ((attr & 1) << 8), color = (attr & 0x38) >> 3;
			const uint8_t *src = elem(&s_gfx[(attr & 2) >> 1], (uint32_t)code) + (ty & 7) * 8;
			const uint16_t pen = (uint16_t)(0x100 + color * 2 + 1);
			uint16_t *d = s_scr + y * 256 + x0;
			for (int i = 0; i < 8; i++)
				if (src[flip ? 7 - i : i])
					d[i] = pen;
		}
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	for (int y = 0; y < VIS_H; y++)
		memcpy(bm + y * pitch, s_scr + (VIS_Y0 + y) * 256, 256 * 2);
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap->vram, s_vram, sizeof(s_vram));
	memcpy(s_snap->cram, s_cram, sizeof(s_cram));
	memcpy(s_snap->spriteram, s_spriteram, sizeof(s_spriteram));
	s_snap->scroll_y = s_scroll_y;
	s_snap->control = s_control;
	rbx_video_frame_done();
}

// MAME compute_resistor_weights (resnet.cpp) for one network: each resistor's output alone, scaled
static void weights(const double *r, int n, double pulldown, double scale, double *w)
{
	for (int i = 0; i < n; i++) {
		double r0 = pulldown > 0 ? 1.0 / pulldown : 1e-12, r1 = 0;
		for (int j = 0; j < n; j++) {
			if (j == i)
				r1 += 1.0 / r[j];
			else
				r0 += 1.0 / r[j];
		}
		r0 = 1.0 / r0;
		r1 = 1.0 / r1;
		w[i] = 255.0 * r0 / (r1 + r0) * scale;
	}
}

static void palette_init(const uint8_t *prom, uint32_t ps)
{
	static const double res[4] = { 1500, 750, 360, 180 }, res_fg[1] = { 51 };
	double wfg[1], w[4];
	weights(res_fg, 1, 0, 1.0, wfg);
	const double scale = 255.0 / wfg[0];   // (scale -1: the largest network to 255)
	weights(res_fg, 1, 0, scale, wfg);
	weights(res, 4, 470, scale, w);
	uint32_t ind[0x108];
	for (int i = 0; i < 0x100; i++) {
		int c[3], o[3];
		if (s_rk) {   // ringking_state::get_rgb_data
			c[0] = (prom[i] >> 4) & 15;
			c[1] = prom[i] & 15;
			c[2] = (uint32_t)(i + 0x100) < ps ? prom[i + 0x100] & 15 : 0;
		} else {
			c[0] = prom[i] & 15;
			c[1] = prom[0x100 + i] & 15;
			c[2] = prom[0x200 + i] & 15;
		}
		for (int k = 0; k < 3; k++)
			o[k] = (int)(w[0] * (c[k] & 1) + w[1] * ((c[k] >> 1) & 1) + w[2] * ((c[k] >> 2) & 1) + w[3] * ((c[k] >> 3) & 1) + 0.5);
		ind[i] = (uint32_t)(o[0] << 16 | o[1] << 8 | o[2]);
	}
	for (int i = 0x100; i < 0x108; i++) {   // the text's primary colours
		const int r = (int)((((i - 0x100) >> 2) & 1) * wfg[0]), g = (int)((((i - 0x100) >> 1) & 1) * wfg[0]),
		          b = (int)(((i - 0x100) & 1) * wfg[0]);
		ind[i] = (uint32_t)(r << 16 | g << 8 | b);
	}
	for (int i = 0; i < 0x100; i++)
		s_rgb[i] = ind[i];
	for (int i = 0x100; i < 0x110; i++)   // (the even pens: indirect entry 0)
		s_rgb[i] = (i & 1) ? ind[((i - 0x101) >> 1) | 0x100] : ind[0];
}

// ---- init / frame ----
static void kb_reset(void)
{
	memset(s_ram_main, 0, sizeof(s_ram_main));
	memset(s_ram_f0, 0, sizeof(s_ram_f0));
	memset(s_share_spr, 0, sizeof(s_share_spr));
	memset(s_share_vid, 0, sizeof(s_share_vid));
	memset(s_ram_vid, 0, sizeof(s_ram_vid));
	memset(s_ram_spr, 0, sizeof(s_ram_spr));
	memset(s_ram_snd, 0, sizeof(s_ram_snd));
	memset(s_vram, 0, sizeof(s_vram));
	memset(s_cram, 0, sizeof(s_cram));
	memset(s_spriteram, 0, sizeof(s_spriteram));
	memset(s_spr_extra, 0, sizeof(s_spr_extra));
	s_scroll_y = s_slatch = 0;
	s_control = 0;   // (control_w(0): the LS174 reset)
	s_nmi_line = s_vid_irq = s_spr_irq = s_snd_irq = 0;
	s_dac = 0;
	ay8910_reset(&s_ay);
	for (int c = 0; c < 4; c++) {
		z80_init(&s_cpu[c]);
		hook(c);
	}
	s_line = 0;
	s_frame = 0;
	memset(s_frac, 0, sizeof(s_frac));
	s_ay_frac = s_nmi_frac = 0;
}

static void kb_exit(void);

static int kb_init(const rbx_game *g)
{
	s_rk = !strcmp(g->machine, "ringking");
	static const char *const cpus[4] = { "maincpu", "video", "sprite", "audiocpu" };
	for (int c = 0; c < 4; c++) {
		s_rom[c] = rbx_region(cpus[c], &s_rom_size[c]);
		if (!s_rom[c] || s_rom_size[c] < 0x10000) {
			rbx_fail("%s: ROM regions missing", g->set);
			return -1;
		}
	}
	uint32_t cs, ss, s2s = 0, bs, ps, us = 0;
	uint8_t *chars = rbx_region("chars", &cs), *spr = rbx_region("sprites", &ss), *bg = rbx_region("bgtiles", &bs);
	uint8_t *spr2 = s_rk ? rbx_region("sprites2", &s2s) : NULL, *prom = rbx_region("proms", &ps);
	const uint8_t *user1 = rbx_region("user1", &us);
	if (!chars || !spr || !bg || !prom || (s_rk && !spr2)) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	if (!strcmp(g->set, "ringking3"))   // init_ringking3: the first PROM expanded
		for (int i = 0; i < 0x100; i++)
			prom[i] = prom[i + 0x100] >> 4;
	if (!strcmp(g->set, "ringkingw") && user1 && us >= 0xc00)   // init_ringkingw: the PROMs re-encoded
		for (int i = 0, j = 0; j < 0x40; i++, j++) {
			if ((i & 0xf) == 8)
				i += 8;
			for (int k = 0; k <= 3; k++) {
				prom[j + 0x000 + 0x40 * k] = user1[i + 0x000 + 0x100 * k];
				prom[j + 0x100 + 0x40 * k] = user1[i + 0x400 + 0x100 * k];
				prom[j + 0x200 + 0x40 * k] = user1[i + 0x800 + 0x100 * k];
			}
		}
	palette_init(prom, ps);
	{
		static uint32_t xc[8], xc2[8], yc[8], xs[16], ys[16], xt[16], xb[16], yb[16];
		static const uint32_t p1[1] = { 0 };
		int rc;
		for (int i = 0; i < 16; i++) {
			ys[i] = (uint32_t)(i * 8);
			yb[i] = (uint32_t)((15 - i) * 8);
		}
		if (!s_rk) {
			for (int i = 0; i < 8; i++) {
				xc[i] = (uint32_t)i;
				yc[i] = (uint32_t)(i * 8);
				xs[i] = (uint32_t)(3 * 0x4000 * 8 + i);
				xs[8 + i] = (uint32_t)i;
				xt[i] = (uint32_t)(3 * 0x2000 * 8 + i);
				xt[8 + i] = (uint32_t)i;
			}
			const uint32_t psp[3] = { 2 * 0x4000 * 8, 0x4000 * 8, 0 }, pt[3] = { 2 * 0x2000 * 8, 0x2000 * 8, 0 };
			const rbx_gfx_layout lc = { 8, 8, 1, p1, xc, yc, 8 * 8 }, ls = { 16, 16, 3, psp, xs, ys, 16 * 8 },
			                     lt = { 16, 16, 3, pt, xt, ys, 16 * 8 };
			rc = rbx_gfx_decode(&s_gfx[0], &lc, chars, 512 * 8) || rbx_gfx_decode(&s_gfx[1], &lc, chars + 0x1000, 512 * 8) ||
			     rbx_gfx_decode(&s_gfx[2], &ls, spr, 1024 * 16) || rbx_gfx_decode(&s_gfx[3], &lt, bg, 512 * 16);
		} else {
			for (int i = 0; i < 4; i++) {
				xc[i] = (uint32_t)(7 - i);
				xc[4 + i] = (uint32_t)(0x1000 * 8 + 7 - i);
				xc2[i] = (uint32_t)(3 - i);
				xc2[4 + i] = (uint32_t)(0x1000 * 8 + 3 - i);
			}
			for (int i = 0; i < 8; i++) {
				yc[i] = (uint32_t)((7 - i) * 8);
				xs[i] = (uint32_t)(7 - i);
				xs[8 + i] = (uint32_t)(16 * 8 + 7 - i);
			}
			static const uint32_t xbb[16] = { 16 * 8 + 3, 16 * 8 + 2, 16 * 8 + 1, 16 * 8 + 0, 0x2000 * 8 + 3, 0x2000 * 8 + 2,
			                                  0x2000 * 8 + 1, 0x2000 * 8 + 0, 3, 2, 1, 0, 0x2010 * 8 + 3, 0x2010 * 8 + 2,
			                                  0x2010 * 8 + 1, 0x2010 * 8 + 0 };
			for (int i = 0; i < 16; i++)
				xb[i] = xbb[i];
			const uint32_t psp[3] = { 0, 0x8000 * 8, 2 * 0x8000 * 8 }, pt[3] = { 0, 0x4000 * 8, 2 * 0x4000 * 8 },
			               pb[3] = { 0x4000 * 8 + 4, 0, 4 };
			const rbx_gfx_layout lc1 = { 8, 8, 1, p1, xc, yc, 8 * 8 }, lc2 = { 8, 8, 1, p1, xc2, yc, 8 * 8 },
			                     ls = { 16, 16, 3, psp, xs, ys, 32 * 8 }, lt = { 16, 16, 3, pt, xs, ys, 32 * 8 },
			                     lb = { 16, 16, 3, pb, xb, yb, 32 * 8 };
			rc = rbx_gfx_decode(&s_gfx[0], &lc1, chars, 512 * 8) || rbx_gfx_decode(&s_gfx[1], &lc2, chars, 512 * 8) ||
			     rbx_gfx_decode(&s_gfx[2], &ls, spr, 1024 * 32) || rbx_gfx_decode(&s_gfx[3], &lt, spr2, 512 * 32) ||
			     rbx_gfx_decode(&s_gfx[4], &lb, bg, 256 * 32);
		}
		if (rc) {
			rbx_fail("%s: out of memory (graphics)", g->set);
			kb_exit();
			return -1;
		}
	}
	s_snap = calloc(1, sizeof(*s_snap));
	ay8910_init(&s_ay, AY8910_TYPE_AY8910, 1500000, AY8910_LEGACY_OUTPUT, 64, 4, 0);   // (0.25)
	s_ay.port_a_read = ay_porta;
	s_mix_cap = (int)(ay8910_rate(&s_ay) / 50) + 64;
	s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	if (!s_snap || !s_mix || rbx_audio_setup(ay8910_rate(&s_ay)) || rbx_video_setup(256, VIS_H, 0x110)) {
		rbx_fail("%s: out of memory", g->set);
		kb_exit();
		return -1;
	}
	memset(s_map, 0, sizeof(s_map));
	static const int top[4] = { 0xc0, 0x40, 0x20, 0xc0 };
	for (int c = 0; c < 4; c++)
		for (int p = 0; p < top[c]; p++)
			s_map[c][p] = s_rom[c] + p * 256;
	s_port_dsw[0] = rbx_port_find("DSW1");
	s_port_dsw[1] = rbx_port_find("DSW2");
	s_port_p1 = rbx_port_find("P1");
	s_port_p2 = rbx_port_find("P2");
	s_port_sys = rbx_port_find("SYSTEM");
	s_port_extra = rbx_port_find("EXTRA");
	kb_reset();
	return 0;
}

static void kb_exit(void)
{
	for (int i = 0; i < 5; i++)
		rbx_gfx_free(&s_gfx[i]);
	free(s_snap);
	s_snap = NULL;
	free(s_mix);
	s_mix = NULL;
}

static void kb_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	for (int line = 0; line < LINES; line++) {   // (from line 240)
		s_line = line;
		update_nmi();
		int none = 0;
		for (int half = 0; half < 2; half++) {   // (heavy sync: two slices a line)
			for (int c = 0; c < 4; c++) {
				const int cyc = rbx_line_cycles(&s_frac[c], CLOCK, FRAME_MHZ, LINES * 2);
				int *irq = c == 1 ? &s_vid_irq : c == 2 ? &s_spr_irq : c == 3 ? &s_snd_irq : &none;
				z80_run(&s_cpu[c], s_cpu[c].cyc + (unsigned long)cyc, irq, 1);   // (HOLD_LINE)
			}
			s_nmi_frac += (uint64_t)6000 * 1000;   // the sound NMI clock: 6 kHz
			while (s_nmi_frac >= (uint64_t)FRAME_MHZ * LINES * 2) {
				s_nmi_frac -= (uint64_t)FRAME_MHZ * LINES * 2;
				z80_gen_nmi(&s_cpu[3]);
			}
		}
		const uint32_t clocks = (uint32_t)rbx_line_cycles(&s_ay_frac, 1500000, FRAME_MHZ, LINES);
		const int n = ay8910_render(&s_ay, clocks, s_mix, s_mix_cap);
		for (int k = 0; k < n; k++) {
			rbx_audio_put(s_mix[k] + s_dac / 8);   // (the DAC 0.125)
			s_mix[k] = 0;
		}
	}
	vblank_capture();   // (the screen update at the next vblank start)
	s_frame++;
}

static int kb_state(void *ctx, int load)
{
	z80 c[4];
	memcpy(c, s_cpu, sizeof(c));
	uint64_t fr[6] = { s_frac[0], s_frac[1], s_frac[2], s_frac[3], s_ay_frac, s_nmi_frac };
	int32_t misc[12] = { (int32_t)s_frame, s_scroll_y, s_control, s_slatch, s_nmi_line, s_vid_irq, s_spr_irq, s_snd_irq,
	                     s_dac, 0, 0, 0 };
	uint8_t ay[256];
	const uint32_t ays = ay8910_state_size();
	if (!load)
		memcpy(ay, &s_ay, ays);
	int rc = 0;
	rc |= rbx_state_io(ctx, "cpus", c, sizeof(c));
	rc |= rbx_state_io(ctx, "rmain", s_ram_main, sizeof(s_ram_main));
	rc |= rbx_state_io(ctx, "rf0", s_ram_f0, sizeof(s_ram_f0));
	rc |= rbx_state_io(ctx, "shspr", s_share_spr, sizeof(s_share_spr));
	rc |= rbx_state_io(ctx, "shvid", s_share_vid, sizeof(s_share_vid));
	rc |= rbx_state_io(ctx, "rvid", s_ram_vid, sizeof(s_ram_vid));
	rc |= rbx_state_io(ctx, "rspr", s_ram_spr, sizeof(s_ram_spr));
	rc |= rbx_state_io(ctx, "rsnd", s_ram_snd, sizeof(s_ram_snd));
	rc |= rbx_state_io(ctx, "vram", s_vram, sizeof(s_vram));
	rc |= rbx_state_io(ctx, "cram", s_cram, sizeof(s_cram));
	rc |= rbx_state_io(ctx, "spr", s_spriteram, sizeof(s_spriteram));
	rc |= rbx_state_io(ctx, "sprx", s_spr_extra, sizeof(s_spr_extra));
	rc |= rbx_state_io(ctx, "ay", ay, sizeof(ay));
	rc |= rbx_state_io(ctx, "frac", fr, sizeof(fr));
	rc |= rbx_state_io(ctx, "misc", misc, sizeof(misc));
	if (!rc && load) {
		memcpy(s_cpu, c, sizeof(c));
		for (int i = 0; i < 4; i++)
			hook(i);
		memcpy(&s_ay, ay, ays);
		for (int i = 0; i < 4; i++)
			s_frac[i] = fr[i];
		s_ay_frac = fr[4];
		s_nmi_frac = fr[5];
		s_frame = (uint32_t)misc[0];
		s_scroll_y = (uint8_t)misc[1];
		s_control = (uint8_t)misc[2];
		s_slatch = (uint8_t)misc[3];
		s_nmi_line = misc[4];
		s_vid_irq = misc[5];
		s_spr_irq = misc[6];
		s_snd_irq = misc[7];
		s_dac = misc[8];
	}
	return rc ? -1 : 0;
}

static int kb_save(void *ctx) { return kb_state(ctx, 0); }
static int kb_load(void *ctx) { return kb_state(ctx, 1); }
static int kb_supported(const char *m) { return !strcmp(m, "kingofb") || !strcmp(m, "ringking"); }

const rbx_driver rbx_drv_kingofb = { "DECO King of Boxer", kb_init, kb_exit, kb_reset, kb_frame, kb_save, kb_load,
                                     kb_supported, kb_render, NULL };
