/*  Cycle Maabou / Sky Destroyer hardware, ported from MAME src/mame/taito/cyclemb.cpp
    (BSD-3-Clause, copyright-holders: Angelo Salese).
    Main Z80 at 6 MHz (IRQ at vblank), sound Z80 at 3 MHz (60 Hz IRQ), YM2203 at 1.5 MHz (route 0.50; its IRQ is not
    connected). The two i8741 MCUs (AP-002 / AP-003) are not dumped: as in MAME their host interface is simulated
    (input packets with parity, DIP switch reads including MAME's PC hack, sound latch), and MAME's sound CPU ROM
    patches (crash + checksum) are applied. Cycle Maabou's dial follows MAME's dial_r (direction bit + 5-bit count).
    Video: 64x32 opaque tilemap with per-row scroll (cyclemb) or per-column scroll and colour cycling (skydest),
    16x16 / 32x32 sprites from two sprite pages; drawn as in MAME on a 512x256 frame and cropped to the visible area.
    The main CPU I/O map decodes 16-bit port addresses (0xc0xx); the Z80 core passes the low byte, which is unique
    for every port of these maps.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/z80/z80.h"
#include "../../sound/rbx_ym.h"

#include <stdlib.h>
#include <string.h>

#define MAIN_CLOCK 6000000
#define AUDIO_CLOCK 3000000
#define YM_CLOCK 1500000
#define LINES 256
#define VIS_Y0 16
#define VIS_Y1 239
#define SCREEN_W 256

typedef struct {
	z80 z;
	int irq;
	unsigned long start;
	int len;
} zcpu;

typedef struct {
	uint8_t rxd, txd, rst, state, packet_type;
} mcu_sim_t;

static int s_sky;                      // skydest
static int s_vis_x0;                   // first visible column of the 512 wide frame
static uint8_t *s_main_rom, *s_audio_rom;
static uint8_t s_ram[0x3000];          // 0x9000-0xbfff: vram, cram, obj1, obj2, obj3, work RAM
static uint8_t s_audio_ram[0x400];
static zcpu s_main, s_audio;
static int s_bank, s_sprite_page, s_screen_display, s_flip;
static mcu_sim_t s_mcu[2];
static uint8_t s_latch;
static uint16_t s_dsw_pc_hack;
static uint8_t s_dial_value[2], s_dial_reverse[2];
static int s_dial_delta[2];
// sound
static int32_t *s_mix;
static int s_mix_cap, s_mix_n;
static uint32_t s_ym_done;             // YM clocks of the frame done
static int s_line_pos;                 // audio CPU clocks at the start of the current line
// video
static uint8_t *s_chr, *s_spr16, *s_spr32;
static uint32_t s_pal[256];
static uint16_t s_frame[512 * 256];
static uint8_t s_snap[0x2800];
static int s_snap_page, s_snap_display, s_snap_flip;
// ports
static int s_system, s_p1_0, s_p1_1, s_p2_0, s_p2_1, s_dsw1, s_dsw2, s_dsw3;
static uint64_t s_frac_main, s_frac_audio;

static uint8_t port8(int p) { return p < 0 ? 0 : (uint8_t)rbx_port_read(p); }

// ---- sound ----
static void ym_sync(void)
{
	int done = (int)(s_audio.z.cyc - s_audio.start);
	if (done > s_audio.len) done = s_audio.len;
	if (done < 0) done = 0;
	uint32_t pos = (uint32_t)(s_line_pos + done) / 2;   // YM at half the sound CPU clock
	if (pos <= s_ym_done)
		return;
	s_mix_n += rbx_ym_advance(0, pos - s_ym_done, s_mix + s_mix_n, s_mix_cap - s_mix_n);
	s_ym_done = pos;
}

// ---- simulated i8741 (MAME skydest_i8741_*) ----
static void update_dial(int p)
{
	if (s_sky)
		return;
	int delta = s_dial_delta[p];
	if (delta > 0x1f) delta = 0x1f;
	if (delta < -0x1f) delta = -0x1f;
	if (delta) {
		s_dial_reverse[p] = delta < 0;
		s_dial_value[p] = (uint8_t)((s_dial_value[p] + (delta < 0 ? -delta : delta)) & 0x1f);
	}
}

static uint8_t dial_r(int p) { return (uint8_t)(s_dial_value[p] | (s_dial_reverse[p] ? 0x80 : 0)); }

static uint8_t mcu0_r(int offset)
{
	mcu_sim_t *m = &s_mcu[0];
	if (offset == 1)
		return 1;
	if (s_main.z.pc == s_dsw_pc_hack)
		m->rxd = (uint8_t)((port8(s_dsw1) & 0x1f) << 2);
	else if (m->rst) {
		switch (m->state) {
		case 1:
			m->packet_type ^= 0x20;
			m->rxd = (uint8_t)((port8((m->packet_type & 0x20) ? s_dsw3 : s_system) & 0x9f) | m->packet_type);
			break;
		case 2:
		case 3: {
			int p = m->state - 2;
			m->packet_type ^= 0x20;
			if (m->packet_type & 0x20) {
				update_dial(p);
				uint8_t v = s_sky ? port8(p ? s_p2_1 : s_p1_1) : dial_r(p);
				m->rxd = (uint8_t)((v & 0x9f) | m->packet_type);
			} else {
				m->rxd = (uint8_t)((port8(p ? s_p2_0 : s_p1_0) & 0x1f) | m->packet_type);
				m->rxd |= port8(s_system) & 0x80;
			}
			break;
		}
		default:
			m->rxd = 0;
			break;
		}
		int pt = 0;
		for (int i = 0; i < 8; i++)
			pt += (m->rxd >> i) & 1;
		if (pt & 1)
			m->rxd |= 0x40;
	}
	return m->rxd;
}

static void mcu0_w(int offset, uint8_t data)
{
	mcu_sim_t *m = &s_mcu[0];
	if (offset == 1) {
		switch (data) {
		case 0: m->rxd = 0x40; m->rst = 0; m->state = 0; break;
		case 1: m->rxd = 0x40; m->rst = 0; break;
		case 2: m->rxd = (uint8_t)((port8(s_dsw2) & 0x1f) << 2); m->rst = 0; break;
		case 3: m->rst = 1; m->txd = 0; break;
		}
		return;
	}
	m->txd = data;
	s_mcu[1].rst = 0;
	if (data == 0x41) m->state = 1;
	else if (data == 0x42) m->state = 2;
	else if (data == 0x44) m->state = 3;
	else s_latch = data;
}

static uint8_t mcu1_r(int offset)
{
	if (offset == 1)
		return 1;
	if (s_mcu[1].rst == 1)
		return 0x40;
	uint8_t v = s_latch;
	s_latch = 0;   // clear_w
	return v;
}

// ---- main CPU ----
static uint8_t main_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x8000) return s_main_rom[a];
	if (a < 0x9000) return s_main_rom[0x10000 + s_bank * 0x1000 + (a - 0x8000)];
	if (a < 0xc000) return s_ram[a - 0x9000];
	return 0xff;
}
static void main_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0x9000 && a < 0xc000) s_ram[a - 0x9000] = v;
}
static uint8_t main_in(z80 *z, uint8_t p)
{
	(void)z;
	if (s_sky ? (p == 0x80 || p == 0x81) : (p == 0x9e || p == 0x9f))
		return mcu0_r(p & 1);
	return 0xff;
}
static void main_out(z80 *z, uint8_t p, uint8_t v)
{
	(void)z;
	if (p == 0x00) {   // bankswitch
		s_bank = v & 3;
		s_sprite_page = (v & 4) >> 2;
		if (s_sky) s_flip = (v & 0x40) == 0;
	} else if (p == 0x20)
		s_screen_display = v & 1;
	else if (s_sky ? (p == 0x80 || p == 0x81) : (p == 0x9e || p == 0x9f))
		mcu0_w(p & 1, v);
	else if (!s_sky && p == 0xbf)
		s_flip = v & 1;
}

// ---- sound CPU ----
static uint8_t audio_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x4000) return s_audio_rom[a];
	if (a >= 0x6000 && a < 0x6400) return s_audio_ram[a - 0x6000];
	return 0xff;
}
static void audio_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0x6000 && a < 0x6400) s_audio_ram[a - 0x6000] = v;
}
static uint8_t audio_in(z80 *z, uint8_t p)
{
	(void)z;
	if (p < 2) { ym_sync(); return rbx_ym_read(0, p & 1); }
	if (p == 0x40 || p == 0x41) return mcu1_r(p & 1);
	return 0xff;
}
static void audio_out(z80 *z, uint8_t p, uint8_t v)
{
	(void)z;
	if (p < 2) { ym_sync(); rbx_ym_write(0, p & 1, v); }
	else if (p == 0x41 && v == 0xf0) s_mcu[1].rst = 1;
}

static void zcpu_init(zcpu *c, uint8_t (*rd)(void *, uint16_t), void (*wr)(void *, uint16_t, uint8_t),
                      uint8_t (*in)(z80 *, uint8_t), void (*out)(z80 *, uint8_t, uint8_t))
{
	z80_init(&c->z);
	c->z.read_byte = rd;
	c->z.write_byte = wr;
	c->z.port_in = in;
	c->z.port_out = out;
	c->z.int_data = 0xff;
	c->irq = 0;
}

static void zcpu_run(zcpu *c, int cycles)
{
	c->start = c->z.cyc;
	c->len = cycles;
	unsigned long end = c->z.cyc + (unsigned long)cycles;
	while ((long)(c->z.cyc - end) < 0) {
		c->z.int_pending = c->irq ? 1 : 0;
		z80_step(&c->z);
		if (c->irq && !c->z.int_pending)
			c->irq = 0;
	}
}

// ---- video ----
static int gbit(const uint8_t *p, uint32_t b) { return (p[b >> 3] >> (7 - (b & 7))) & 1; }

// 2 bits per pixel, planes { 0, 4 }; xg = bit offsets of the 4-pixel groups, yo = row offsets
static uint8_t *decode2(const uint8_t *src, int n, int w, const uint32_t *xg, const uint32_t *yo, uint32_t inc)
{
	uint8_t *out = (uint8_t *)malloc((size_t)n * (size_t)(w * w));
	if (!out)
		return NULL;
	for (int t = 0; t < n; t++)
		for (int y = 0; y < w; y++)
			for (int x = 0; x < w; x++) {
				uint32_t b = (uint32_t)t * inc + yo[y] + xg[x >> 2] + (uint32_t)(x & 3);
				out[(size_t)t * (size_t)(w * w) + (size_t)(y * w + x)] = (uint8_t)(gbit(src, b) << 1 | gbit(src, b + 4));
			}
	return out;
}

// draw w x w pixels at (sx, sy) clipped to the visible area of the 512x256 frame; opaque or pen 0 transparent
static void draw_gfx(const uint8_t *g, int w, int color, int fx, int fy, int sx, int sy, int opaque)
{
	const int x0 = s_vis_x0, x1 = s_vis_x0 + SCREEN_W;
	if (sx >= x1 || sx + w <= x0 || sy > VIS_Y1 || sy + w <= VIS_Y0)
		return;
	uint16_t base = (uint16_t)(color * 4);
	for (int y = 0; y < w; y++) {
		int dy = sy + y;
		if (dy < VIS_Y0 || dy > VIS_Y1)
			continue;
		const uint8_t *row = g + (fy ? w - 1 - y : y) * w;
		uint16_t *d = s_frame + dy * 512;
		for (int x = 0; x < w; x++) {
			int dx = sx + x;
			if (dx < x0 || dx >= x1)
				continue;
			uint8_t c = row[fx ? w - 1 - x : x];
			if (opaque || c)
				d[dx] = (uint16_t)(base + c);
		}
	}
}

static void cyclemb_tilemap(const uint8_t *vram, const uint8_t *cram)
{
	int count = 0;
	for (int y = 0; y < 32; y++)
		for (int x = 0; x < 64; x++, count++) {
			int attr = cram[count];
			int tile = vram[count] | ((attr & 3) << 8);
			int color = ((attr & 0xf8) >> 3) ^ 0x1f;
			int odd_line = (y & 1) ? 0x40 : 0x00;
			int scrollx = (vram[y / 2 + odd_line] + (cram[y / 2 + odd_line] << 8) + 48) & 0x1ff;
			if (!(attr & 4))
				color += 0x20;
			const uint8_t *g = s_chr + (tile & 1023) * 64;
			if (s_snap_flip) {
				draw_gfx(g, 8, color, 1, 1, 504 - x * 8 - scrollx, 248 - y * 8, 1);
				draw_gfx(g, 8, color, 1, 1, 504 - x * 8 - scrollx + 512, 248 - y * 8, 1);
			} else {
				draw_gfx(g, 8, color, 0, 0, x * 8 - scrollx, y * 8, 1);
				draw_gfx(g, 8, color, 0, 0, x * 8 - scrollx + 512, y * 8, 1);
			}
		}
}

static void skydest_tilemap(const uint8_t *vram, const uint8_t *cram)
{
	for (int y = 0; y < 32; y++)
		for (int x = 2; x < 62; x++) {
			int attr = cram[x + y * 64];
			int tile = vram[x + y * 64] | ((attr & 3) << 8);
			int color = (attr & 0xfc) >> 2;
			int scrollx = vram[0] | ((cram[0] & 1) << 8);
			int cycle = (cram[0] & 0xf0) >> 4;
			color ^= 0x3f;
			if (attr & 0x40)
				color ^= cycle;
			scrollx -= 0xc0;
			int scrolly = x < 32 ? vram[x * 64] : vram[(x - 32) * 64 + 1];
			const uint8_t *g = s_chr + (tile & 1023) * 64;
			if (s_snap_flip) {
				int sy = 248 - (((y * 8) - scrolly) & 0xff);
				draw_gfx(g, 8, color, 1, 1, 504 - x * 8 + scrollx, sy, 1);
				draw_gfx(g, 8, color, 1, 1, 504 - x * 8 + scrollx - 480, sy, 1);
				draw_gfx(g, 8, color, 1, 1, 504 - x * 8 + scrollx + 480, sy, 1);
			} else {
				int sy = ((y * 8) - scrolly) & 0xff;
				draw_gfx(g, 8, color, 0, 0, x * 8 + scrollx, sy, 1);
				draw_gfx(g, 8, color, 0, 0, x * 8 + scrollx - 480, sy, 1);
				draw_gfx(g, 8, color, 0, 0, x * 8 + scrollx + 480, sy, 1);
			}
		}
}

static void draw_sprites(const uint8_t *obj1, const uint8_t *obj2, const uint8_t *obj3)
{
	int page_start = s_snap_page * 0x80;
	for (int i = page_start; i < page_start + 0x80; i += 2) {
		int x, y;
		int spr_offs = obj1[i] + ((obj3[i] & 3) << 8);
		int col = obj1[i + 1] & 0x3f;
		int region = ((obj3[i] & 0x10) >> 4) + 1;
		if (s_sky) {
			y = obj2[i] - 1;
			x = obj2[i + 1];
			if (obj3[i + 1] & 1)
				x |= 0x100;
			x = 0x138 - x;
			if (region == 2) {
				spr_offs >>= 2;
				x -= 16;
			}
		} else {
			y = 0xf1 - obj2[i];
			x = obj2[i + 1] - 56;
			if (region == 2) {
				spr_offs >>= 2;
				y -= 16;
			}
			if (obj3[i + 1] & 1)
				x += 256;
		}
		int fx = (obj3[i] & 4) >> 2, fy = (obj3[i] & 8) >> 3;
		if (s_snap_flip) {
			fx = !fx;
			fy = !fy;
		}
		if (region == 1)
			draw_gfx(s_spr16 + (spr_offs & 1023) * 256, 16, col, fx, fy, x, y, 0);
		else
			draw_gfx(s_spr32 + (spr_offs & 255) * 1024, 32, col, fx, fy, x, y, 0);
	}
}

static void cm_render(void)
{
	memcpy(rbx_video.palette, s_pal, sizeof(s_pal));
	for (int y = VIS_Y0; y <= VIS_Y1; y++)
		memset(s_frame + y * 512 + s_vis_x0, 0, SCREEN_W * sizeof(uint16_t));
	if (s_snap_display) {
		const uint8_t *vram = s_snap, *cram = s_snap + 0x800;
		if (s_sky)
			skydest_tilemap(vram, cram);
		else
			cyclemb_tilemap(vram, cram);
		draw_sprites(s_snap + 0x1000, s_snap + 0x1800, s_snap + 0x2000);
	}
	uint16_t *bm = rbx_video.bitmap;
	for (int y = VIS_Y0; y <= VIS_Y1; y++)
		memcpy(bm + (y - VIS_Y0) * rbx_video.pitch, s_frame + y * 512 + s_vis_x0, SCREEN_W * sizeof(uint16_t));
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap, s_ram, sizeof(s_snap));
	s_snap_page = s_sprite_page;
	s_snap_display = s_screen_display;
	s_snap_flip = s_flip;
	rbx_video_frame_done();
}

// ---- init / frame ----
static void cm_reset(void)
{
	memset(s_ram, 0, sizeof(s_ram));
	memset(s_audio_ram, 0, sizeof(s_audio_ram));
	memset(&s_main, 0, sizeof(s_main));
	memset(&s_audio, 0, sizeof(s_audio));
	zcpu_init(&s_main, main_read, main_write, main_in, main_out);
	zcpu_init(&s_audio, audio_read, audio_write, audio_in, audio_out);
	memset(s_mcu, 0, sizeof(s_mcu));
	s_latch = 0;
	s_bank = s_sprite_page = 0;
	s_screen_display = 1;
	s_flip = 0;
	memset(s_dial_value, 0, sizeof(s_dial_value));
	memset(s_dial_reverse, 0, sizeof(s_dial_reverse));
	rbx_ym_reset(0);
	s_ym_done = 0;
	s_mix_n = 0;
	s_frac_main = s_frac_audio = 0;
}

static void cm_exit(void)
{
	free(s_chr); free(s_spr16); free(s_spr32);
	s_chr = s_spr16 = s_spr32 = NULL;
	free(s_mix);
	s_mix = NULL;
	rbx_ym_exit(0);
}

static int cm_init(const rbx_game *g)
{
	if (strcmp(g->machine, "cyclemb") && strcmp(g->machine, "skydest")) {
		rbx_fail("%s: machine %s is not emulated", g->set, g->machine);
		return -1;
	}
	s_sky = !strcmp(g->machine, "skydest");
	s_vis_x0 = s_sky ? 16 : 0;
	uint32_t ms, as, ts, ss, ps;
	s_main_rom = rbx_region("maincpu", &ms);
	s_audio_rom = rbx_region("audiocpu", &as);
	uint8_t *tiles = rbx_region("tilemap_data", &ts);
	uint8_t *spr = rbx_region("sprite_data", &ss);
	uint8_t *prom = rbx_region("proms", &ps);
	if (!s_main_rom || ms < 0x18000 || !s_audio_rom || as < 0x4000 || !tiles || ts < 0x4000 || !spr || ss < 0x10000 ||
	    !prom || ps < 0x200) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	// MAME init_cyclemb / init_skydest: DIP switch PC hack, sound CPU crash and checksum patches
	s_dsw_pc_hack = s_sky ? 0x554 : 0x760;
	static const uint16_t patch_cm[6] = { 0x282, 0x283, 0x284, 0xa36, 0xa37, 0xa38 };
	static const uint16_t patch_sd[6] = { 0x286, 0x287, 0x288, 0xa36, 0xa37, 0xa38 };
	for (int i = 0; i < 6; i++)
		s_audio_rom[s_sky ? patch_sd[i] : patch_cm[i]] = 0;
	if (s_sky)   // ROMREGION_INVERT (not applied by the loader)
		for (uint32_t i = 0; i < ts; i++)
			tiles[i] ^= 0xff;
	static const uint32_t cx[2] = { 0, 64 }, cy[8] = { 0, 8, 16, 24, 32, 40, 48, 56 };
	static const uint32_t s1x[4] = { 0, 64, 128, 192 };
	static const uint32_t s1y[16] = { 0, 8, 16, 24, 32, 40, 48, 56, 256, 264, 272, 280, 288, 296, 304, 312 };
	static const uint32_t s2x[8] = { 0, 64, 128, 192, 512, 576, 640, 704 };
	uint32_t s2y[32];
	for (int i = 0; i < 8; i++) {
		s2y[i] = (uint32_t)i * 8;
		s2y[8 + i] = 256 + (uint32_t)i * 8;
		s2y[16 + i] = 1024 + (uint32_t)i * 8;
		s2y[24 + i] = 1280 + (uint32_t)i * 8;
	}
	s_chr = decode2(tiles, 1024, 8, cx, cy, 128);
	s_spr16 = decode2(spr, 1024, 16, s1x, s1y, 512);
	s_spr32 = decode2(spr, 256, 32, s2x, s2y, 2048);
	if (!s_chr || !s_spr16 || !s_spr32) {
		rbx_fail("%s: out of memory (graphics)", g->set);
		cm_exit();
		return -1;
	}
	for (int i = 0; i < 256; i++) {   // cyclemb_palette
		int val = prom[i | 0x100] | (prom[i] << 4);
		int b = 0x47 * ((val >> 6) & 1) + 0x97 * ((val >> 7) & 1);
		int gg = 0x21 * ((val >> 3) & 1) + 0x47 * ((val >> 4) & 1) + 0x97 * ((val >> 5) & 1);
		int r = 0x21 * (val & 1) + 0x47 * ((val >> 1) & 1) + 0x97 * ((val >> 2) & 1);
		s_pal[i] = (uint32_t)(r << 16 | gg << 8 | b);
	}
	s_system = rbx_port_find("SYSTEM");
	s_p1_0 = rbx_port_find("P1_0");
	s_p1_1 = rbx_port_find("P1_1");
	s_p2_0 = rbx_port_find("P2_0");
	s_p2_1 = rbx_port_find("P2_1");
	s_dsw1 = rbx_port_find("DSW1");
	s_dsw2 = rbx_port_find("DSW2");
	s_dsw3 = rbx_port_find("DSW3");

	rbx_ym_config yc;
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM2203;
	yc.clock = YM_CLOCK;
	yc.gain[0] = yc.gain[1] = yc.gain[2] = yc.gain[3] = 128;   // 0.50
	if (rbx_ym_init(0, &yc)) {
		rbx_fail("%s: sound init failed", g->set);
		cm_exit();
		return -1;
	}
	s_mix_cap = (int)(rbx_ym_rate(0) / 50) + 64;
	s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	if (!s_mix || rbx_audio_setup(rbx_ym_rate(0)) || rbx_video_setup(SCREEN_W, VIS_Y1 - VIS_Y0 + 1, 256)) {
		cm_exit();
		return -1;
	}
	cm_reset();
	return 0;
}

static void cm_frame(void)
{
	// dials: relative counts of the frame (PORT_REVERSE), applied at every MCU read as MAME's update_dial
	for (int p = 0; p < 2; p++)
		s_dial_delta[p] = -rbx_analog_dpad(p + 1, RBX_IPT_DIAL, 3);   // (d-pad: key delta 16 x sensitivity 20%)
	s_line_pos = 0;
	s_audio.irq = 1;   // periodic 60 Hz IRQ (irq0_line_hold)
	for (int i = 0; i < LINES; i++) {
		int line = (VIS_Y1 + 1 + i) % LINES;
		if (line == VIS_Y1 + 1) {
			vblank_capture();
			s_main.irq = 1;
		}
		int mc = rbx_line_cycles(&s_frac_main, MAIN_CLOCK, 60000, LINES);
		int ac = rbx_line_cycles(&s_frac_audio, AUDIO_CLOCK, 60000, LINES);
		zcpu_run(&s_main, mc);
		zcpu_run(&s_audio, ac);
		s_line_pos += ac;
		ym_sync();
	}
	for (int i = 0; i < s_mix_n; i++)
		rbx_audio_put(s_mix[i]);
	memset(s_mix, 0, sizeof(int32_t) * (size_t)s_mix_cap);
	s_mix_n = 0;
	s_ym_done = 0;
}

static int zcpu_state(void *ctx, const char *tag, zcpu *c, int load)
{
	zcpu t = *c;
	if (rbx_state_io(ctx, tag, &t, sizeof(t)))
		return -1;
	if (load) {
		t.z.read_byte = c->z.read_byte;
		t.z.write_byte = c->z.write_byte;
		t.z.port_in = c->z.port_in;
		t.z.port_out = c->z.port_out;
		t.z.userdata = c->z.userdata;
		*c = t;
	}
	return 0;
}

static int cm_state(void *ctx, int load)
{
	int rc = 0;
	rc |= zcpu_state(ctx, "main", &s_main, load);
	rc |= zcpu_state(ctx, "audio", &s_audio, load);
	rc |= rbx_state_io(ctx, "ram", s_ram, sizeof(s_ram));
	rc |= rbx_state_io(ctx, "audram", s_audio_ram, sizeof(s_audio_ram));
	rc |= rbx_state_io(ctx, "mcu", s_mcu, sizeof(s_mcu));
	rc |= rbx_ym_state(0, ctx, load);
	int32_t misc[10] = { s_bank, s_sprite_page, s_screen_display, s_flip, s_latch,
		s_dial_value[0], s_dial_value[1], s_dial_reverse[0], s_dial_reverse[1], 0 };
	rc |= rbx_state_io(ctx, "misc", misc, sizeof(misc));
	if (rc)
		return -1;
	if (load) {
		s_bank = misc[0] & 3; s_sprite_page = misc[1] & 1; s_screen_display = misc[2]; s_flip = misc[3];
		s_latch = (uint8_t)misc[4];
		s_dial_value[0] = (uint8_t)misc[5]; s_dial_value[1] = (uint8_t)misc[6];
		s_dial_reverse[0] = (uint8_t)misc[7]; s_dial_reverse[1] = (uint8_t)misc[8];
	}
	return 0;
}

static int cm_save(void *ctx) { return cm_state(ctx, 0); }
static int cm_load(void *ctx) { return cm_state(ctx, 1); }
static int cm_supported(const char *machine) { return !strcmp(machine, "cyclemb") || !strcmp(machine, "skydest"); }

const rbx_driver rbx_drv_cyclemb = { "Cycle Maabou", cm_init, cm_exit, cm_reset, cm_frame, cm_save, cm_load, cm_supported, cm_render };
