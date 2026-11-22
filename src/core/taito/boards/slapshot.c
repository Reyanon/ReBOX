/*  Slap Shot / Operation Wolf 3, ported from MAME src/mame/taito/slapshot.cpp and slapshot_v.cpp (BSD-3-Clause,
    copyright-holders: David Graves). 68000 + TC0640FIO + MK48T08 timekeeper NVRAM (MAME src/devices/machine/timekpr.cpp,
    BSD-3-Clause, Aaron Giles, smf) + TC0480SCP + TC0360PRI + 6bpp TC0540OBN sprites (the F2 sprite format, no
    buffering); opwolf3 adds an ADC0809 for the light guns (IRQ3 from the EOC flip-flop).
    Sound board: Z80 + YM2610B + TC0140SYT (machine/taito_sound.c).
    Light guns (opwolf3): crosshair position of each player (MAME GUNnX / GUNnY, 0x10-0xf0) follows
    rbx_analog(player, RBX_IPT_LIGHTGUN_X / Y) (the d-pad without a pointing device); the game draws its own sights.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m68k_bus.h"
#include "../../cpu/m68k/m68k.h"
#include "../machine/taito_sound.h"
#include "../../machine/adc0808.h"
#include "../video/tc0480scp.h"
#include "../video/tc0360pri.h"
#include "../../video/zoomgfx.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define LINES 256
#define SCREEN_W 320
#define VIS_Y0 16
#define VIS_Y1 239
#define VIS_H (VIS_Y1 - VIS_Y0 + 1)
#define CPU_CLOCK 16000000

static int s_opwolf3;
static uint8_t *s_rom;
static uint32_t s_rom_size;
static uint8_t (*s_ram_hp)[0x10000];
#define s_ram (*s_ram_hp)
static uint16_t (*s_spr_hp)[0x8000];
#define s_spr (*s_spr_hp)
static uint16_t (*s_spr_buf_hp)[0x8000];
#define s_spr_buf (*s_spr_buf_hp)
static uint16_t s_ext[0x1000];
static uint16_t (*s_pal_hp)[0x4000];
#define s_pal (*s_pal_hp)
static tc0480scp_t *s_scp, *s_snap_scp;
static tc0360pri_t s_pri;
static zoomgfx_t s_gfx;
static uint8_t s_fio[8];
static int s_port_coins, s_port_buttons, s_port_system, s_port_joy, s_port_service;
static int s_port_gun[4];
static adc0808_t s_adc;
static int s_gun[4] = { 0x80, 0x80, 0x80, 0x80 };
static uint64_t s_cpu_frac, s_adc_frac;
static int s_cal_fire;                     // BUTTONS bits pressed by the calibration
static int s_autocal;                      // opwolf3 first boot: frame of the automatic gun calibration (0 = off)
static int32_t s_int6;                     // cycles to the delayed IRQ6 (0 = none)
// MK48T08
static uint8_t s_nv[0x2000];
static char s_nv_path[512];
static uint8_t s_tk_control;
static uint32_t s_tk_frames;
static int s_tk[7];                        // seconds, minutes, hours, day, date, month, year (binary)
// sprite state (MAME slapshot_state)
static int s_sprites_disabled, s_sprites_active_area, s_master_scrollx, s_master_scrolly, s_prepare_sprites;

static struct {
	uint16_t spr[0x8000];
	uint16_t ext[0x1000];
	uint16_t pal[0x4000];
	tc0360pri_t pri;
	int disabled, area, msx, msy;
} *s_snap;
static int s_r_flipscreen;                 // MAME m_sprites_flipscreen (drawing side)
static uint8_t *s_pribuf;

static inline int sext12(int v) { return (v & 0x800) ? (v | ~0xfff) : (v & 0xfff); }
static inline uint8_t bcd(int v) { return (uint8_t)((v / 10) << 4 | (v % 10)); }
static inline int unbcd(uint8_t v) { return (v >> 4) * 10 + (v & 15); }

// ---- MK48T08 timekeeper ----
static void tk_to_ram(void)
{
	s_nv[0x1ff9] = (uint8_t)((s_nv[0x1ff9] & 0x80) | bcd(s_tk[0]));
	s_nv[0x1ffa] = bcd(s_tk[1]);
	s_nv[0x1ffb] = bcd(s_tk[2]);
	s_nv[0x1ffc] = (uint8_t)((s_nv[0x1ffc] & 0xf8) | s_tk[3]);
	s_nv[0x1ffd] = bcd(s_tk[4]);
	s_nv[0x1ffe] = bcd(s_tk[5]);
	s_nv[0x1fff] = bcd(s_tk[6]);
}

static void tk_from_ram(void)
{
	s_tk[0] = unbcd(s_nv[0x1ff9] & 0x7f);
	s_tk[1] = unbcd(s_nv[0x1ffa] & 0x7f);
	s_tk[2] = unbcd(s_nv[0x1ffb] & 0x3f);
	s_tk[3] = s_nv[0x1ffc] & 7;
	s_tk[4] = unbcd(s_nv[0x1ffd] & 0x3f);
	s_tk[5] = unbcd(s_nv[0x1ffe] & 0x1f);
	s_tk[6] = unbcd(s_nv[0x1fff]);
}

static void tk_frame(void)   // one second every 60 frames (deterministic clock, no host time)
{
	if (++s_tk_frames < 60)
		return;
	s_tk_frames = 0;
	if (s_nv[0x1ff9] & 0x80)   // oscillator stopped
		return;
	static const int mdays[12] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };
	if (++s_tk[0] >= 60) {
		s_tk[0] = 0;
		if (++s_tk[1] >= 60) {
			s_tk[1] = 0;
			if (++s_tk[2] >= 24) {
				s_tk[2] = 0;
				s_tk[3] = s_tk[3] % 7 + 1;
				int m = s_tk[5] >= 1 && s_tk[5] <= 12 ? s_tk[5] - 1 : 0;
				if (++s_tk[4] > mdays[m] + (m == 1 && !(s_tk[6] & 3))) {
					s_tk[4] = 1;
					if (++s_tk[5] > 12) {
						s_tk[5] = 1;
						s_tk[6] = (s_tk[6] + 1) % 100;
					}
				}
			}
		}
	}
	if (!(s_tk_control & 0xc0))
		tk_to_ram();
}

static void tk_write(uint32_t o, uint8_t v)
{
	if (o == 0x1ff8) {
		if ((s_tk_control & 0x80) && !(v & 0x80))   // W cleared: load the counters
			tk_from_ram();
		if ((s_tk_control & 0x40) && !(v & 0x80))
			tk_to_ram();
		s_tk_control = v;
	}
	s_nv[o] = v;
}

// ---- I/O ----
static uint8_t fio_read(int o)
{
	switch (o) {
	case 1: return (uint8_t)rbx_port_read(s_port_coins);
	case 2: return (uint8_t)(rbx_port_read(s_port_buttons) & ~s_cal_fire);
	case 3: return (uint8_t)rbx_port_read(s_port_system);
	case 4: return s_fio[4];
	case 7: return (uint8_t)rbx_port_read(s_port_joy);
	}
	return o == 0 ? 0 : 0xff;
}

// first boot of opwolf3 without NVRAM: the game asks for the minimum and maximum of each gun (shoot at the top
// left, then bottom right corner); answered here as a player would, with the guns at the ends of their range
static const struct { int frame, player, value, fire; } s_cal_steps[] = {
	{ 1000, 0, 0x10, 0 }, { 1050, 0, 0x10, 1 }, { 1060, 0, 0xf0, 0 }, { 1140, 0, 0xf0, 1 },
	{ 1200, 1, 0x10, 0 }, { 1250, 1, 0x10, 1 }, { 1260, 1, 0xf0, 0 }, { 1340, 1, 0xf0, 1 }, { 1400, -1, 0, 0 } };

static int autocal_step(void)
{
	s_cal_fire = 0;
	if (!s_autocal)
		return 0;
	int f = s_autocal++;
	for (int i = (int)(sizeof(s_cal_steps) / sizeof(s_cal_steps[0])) - 1; i >= 0; i--)
		if (f >= s_cal_steps[i].frame) {
			int p = s_cal_steps[i].player;
			if (p < 0) {
				s_autocal = 0;
				for (int k = 0; k < 4; k++)
					s_gun[k] = 0x80;
				return 0;
			}
			s_gun[p * 2] = s_gun[p * 2 + 1] = s_cal_steps[i].value;
			if (s_cal_steps[i].fire && f < s_cal_steps[i].frame + 5)
				s_cal_fire = p ? 0x10 : 0x01;
			return 1;
		}
	return 1;
}

static void update_gun(void)
{
	if (autocal_step())
		return;
	for (int p = 0; p < 2; p++)
		for (int axis = 0; axis < 2; axis++) {
			int *g = &s_gun[p * 2 + axis];
			int v = *g + rbx_analog(p + 1, axis ? RBX_IPT_LIGHTGUN_Y : RBX_IPT_LIGHTGUN_X);
			*g = v < 0x10 ? 0x10 : v > 0xf0 ? 0xf0 : v;   // PORT_MINMAX(0x10, 0xf0)
		}
}

static uint16_t main_read16(uint32_t a)
{
	if (a >= 0x830000 && a <= 0x83002f) return tc0480scp_ctrl_r(s_scp, (int)((a - 0x830000) >> 1));
	if (a >= 0xa00000 && a <= 0xa03fff) return (uint16_t)(s_nv[(a - 0xa00000) >> 1] << 8);
	if (a >= 0xc00000 && a <= 0xc0000f) return (uint16_t)(fio_read((int)((a >> 1) & 7)) << 8);
	if (a >= 0xc00020 && a <= 0xc0002f) {   // service_input_r
		int o = (int)((a >> 1) & 7);
		if (o == 3)
			return (uint16_t)(((rbx_port_read(s_port_system) & 0xef) | (rbx_port_read(s_port_service) & 0x10)) << 8);
		return (uint16_t)(fio_read(o) << 8);
	}
	if (a == 0xd00002) return (uint16_t)(tc0140syt_master_comm_r(tsnd_syt()) << 8);
	if (s_opwolf3 && a >= 0xe00000 && a <= 0xe0000f) {
		m68k_bus_set_line(3, 0);   // data read clears the EOC flip-flop
		return (uint16_t)(adc0808_data_r(&s_adc) << 8);
	}
	return 0;
}

static uint8_t main_read8(uint32_t a)
{
	uint16_t v = main_read16(a & ~1u);
	return (uint8_t)((a & 1) ? v : v >> 8);
}

static void main_write16m(uint32_t a, uint16_t v, uint16_t mask)
{
	if (a >= 0x830000 && a <= 0x83002f) { tc0480scp_ctrl_w(s_scp, (int)((a - 0x830000) >> 1), v, mask); return; }
	if (a >= 0xa00000 && a <= 0xa03fff) { if (mask & 0xff00) tk_write((a - 0xa00000) >> 1, (uint8_t)(v >> 8)); return; }
	if (a >= 0xb00000 && a <= 0xb0001f) { if (mask & 0xff00) tc0360pri_w(&s_pri, (int)((a >> 1) & 15), (uint8_t)(v >> 8)); return; }
	if (a >= 0xc00000 && a <= 0xc0000f) {   // halfword_byteswap_w
		s_fio[(a >> 1) & 7] = (mask & 0xff00) ? (uint8_t)(v >> 8) : (uint8_t)v;
		return;
	}
	if (a == 0xd00000 && (mask & 0xff00)) { tc0140syt_master_port_w(tsnd_syt(), (uint8_t)(v >> 8)); return; }
	if (a == 0xd00002 && (mask & 0xff00)) { tc0140syt_master_comm_w(tsnd_syt(), (uint8_t)(v >> 8)); return; }
	if (s_opwolf3 && a >= 0xe00000 && a <= 0xe0000f) { if (mask & 0xff00) adc0808_start(&s_adc, (int)((a >> 1) & 7)); return; }
}

static void main_write16(uint32_t a, uint16_t v) { main_write16m(a, v, 0xffff); }
static void main_write8(uint32_t a, uint8_t v)
{
	if (a & 1)
		main_write16m(a & ~1u, v, 0x00ff);
	else
		main_write16m(a, (uint16_t)(v << 8), 0xff00);
}

// ---- sprites (MAME slapshot_v.cpp) ----
static void handle_sprite_buffering(void)
{
	if (s_prepare_sprites) {
		memcpy(s_spr_buf, s_spr, sizeof(s_spr_buf));
		s_prepare_sprites = 0;
	}
}

static void update_sprites_active_area(void)
{
	handle_sprite_buffering();
	const uint16_t *b = s_spr_buf;
	if (s_sprites_active_area == 0x8000 && b[(0x8000 + 6) / 2] == 0 && b[(0x8000 + 10) / 2] == 0)
		s_sprites_active_area = 0;
	for (int off = 0; off < 0x4000; off += 16) {
		int offs = off + s_sprites_active_area;
		if (b[(offs + 6) / 2] & 0x8000) {
			s_sprites_disabled = (b[(offs + 10) / 2] >> 12) & 1;
			s_sprites_active_area = 0x8000 * (b[(offs + 10) / 2] & 1);
			continue;
		}
		if ((b[(offs + 4) / 2] & 0xf000) == 0xa000) {
			s_master_scrollx = sext12(b[(offs + 4) / 2]);
			s_master_scrolly = sext12(b[(offs + 6) / 2]);
		}
	}
}

typedef struct { uint32_t code, color; int flipx, flipy, x, y, zoomx, zoomy; uint32_t primask; } tempsprite_t;
static tempsprite_t (*s_list_hp)[0x400];
#define s_list (*s_list_hp)

static void draw_sprites(uint16_t *bm, int pitch, const uint32_t *primasks, int y_offset)
{
	const uint16_t *b = s_snap->spr;
	int big_sprite = 0, last_continuation_tile = 0;
	int y_no = 0, x_no = 0, xlatch = 0, ylatch = 0;
	uint32_t zoomx, zoomy, zx = 0, zy = 0, zoomxlatch = 0, zoomylatch = 0;
	int scrollx = 0, scrolly = 0;
	tempsprite_t *sp = s_list;
	int disabled = s_snap->disabled;
	int master_scrollx = s_snap->msx, master_scrolly = s_snap->msy;
	int area = s_snap->area;
	int scroll1x = 0, scroll1y = 0;
	int x = 0, y = 0, xcurrent = 0, ycurrent = 0;
	uint32_t color = 0;
	int x_offset = s_r_flipscreen ? -3 : 3;
	if (area == 0x8000 && b[(0x8000 + 6) / 2] == 0 && b[(0x8000 + 10) / 2] == 0)
		area = 0;
	for (int off = 0; off < 0x4000; off += 16) {
		int offs = off + area;
		if (b[(offs + 6) / 2] & 0x8000) {
			disabled = (b[(offs + 10) / 2] >> 12) & 1;
			s_r_flipscreen = (b[(offs + 10) / 2] >> 13) & 1;
			x_offset = s_r_flipscreen ? -3 : 3;
			area = 0x8000 * (b[(offs + 10) / 2] & 1);
			continue;
		}
		if ((b[(offs + 4) / 2] & 0xf000) == 0xa000) {
			master_scrollx = sext12(b[(offs + 4) / 2]);
			master_scrolly = sext12(b[(offs + 6) / 2]);
		}
		if ((b[(offs + 4) / 2] & 0xf000) == 0x5000) {
			scroll1x = sext12(b[(offs + 4) / 2]);
			scroll1y = sext12(b[(offs + 6) / 2]);
		}
		if (disabled)
			continue;
		uint16_t spritedata = b[(offs + 8) / 2];
		int spritecont = (spritedata & 0xff00) >> 8;
		if (spritecont & 0x08) {
			if (!big_sprite) {
				xlatch = b[(offs + 4) / 2] & 0xfff;
				ylatch = b[(offs + 6) / 2] & 0xfff;
				x_no = y_no = 0;
				uint16_t zw = b[(offs + 2) / 2];
				zoomylatch = (zw >> 8) & 0xff;
				zoomxlatch = zw & 0xff;
				big_sprite = 1;
			}
		} else if (big_sprite) {
			last_continuation_tile = 1;
		}
		if (!(spritecont & 0x04))
			color = spritedata & 0xff;
		if (!big_sprite || !(spritecont & 0xf0)) {
			x = b[(offs + 4) / 2];
			if (x & 0x8000) {
				scrollx = -x_offset - 0x60;
				scrolly = 0;
			} else if (x & 0x4000) {
				scrollx = master_scrollx - x_offset - 0x60;
				scrolly = master_scrolly;
			} else {
				scrollx = scroll1x + master_scrollx - x_offset - 0x60;
				scrolly = scroll1y + master_scrolly;
			}
			x &= 0xfff;
			y = b[(offs + 6) / 2] & 0xfff;
			xcurrent = x;
			ycurrent = y;
		} else {
			if (!(spritecont & 0x10))
				y = ycurrent;
			else if (spritecont & 0x20) {
				y += 16;
				y_no++;
			}
			if (!(spritecont & 0x40))
				x = xcurrent;
			else if (spritecont & 0x80) {
				x += 16;
				y_no = 0;
				x_no++;
			}
		}
		if (big_sprite) {
			zoomx = zoomxlatch;
			zoomy = zoomylatch;
			zx = zy = 0x10;
			if (zoomx || zoomy) {
				x = xlatch + x_no * (int)(0x100 - zoomx) / 16;
				y = ylatch + y_no * (int)(0x100 - zoomy) / 16;
				zx = (uint32_t)(xlatch + (x_no + 1) * (int)(0x100 - zoomx) / 16 - x);
				zy = (uint32_t)(ylatch + (y_no + 1) * (int)(0x100 - zoomy) / 16 - y);
			}
		} else {
			uint16_t zw = b[(offs + 2) / 2];
			zoomy = (zw >> 8) & 0xff;
			zoomx = zw & 0xff;
			zx = (0x100 - zoomx) / 16;
			zy = (0x100 - zoomy) / 16;
		}
		if (last_continuation_tile) {
			big_sprite = 0;
			last_continuation_tile = 0;
		}
		int extoffs = offs;
		if (extoffs >= 0x8000)
			extoffs -= 0x4000;
		uint32_t code = (b[offs / 2] & 0xff) | (s_snap->ext[(extoffs >> 4) & 0xfff] & 0xff00);
		if (code == 0)
			continue;
		int flipx = spritecont & 1, flipy = (spritecont >> 1) & 1;
		int curx = sext12(x + scrollx), cury = sext12(y + scrolly);
		if (s_r_flipscreen) {
			curx = 319 - curx - (int)zx;
			cury = 256 - cury - (int)zy;
			flipx = !flipx;
			flipy = !flipy;
		}
		cury += y_offset;
		if (sp < s_list + 0x400) {
			sp->code = code;
			sp->color = color / 4;   // 6bpp: granularity 64
			sp->flipx = flipx;
			sp->flipy = flipy;
			sp->x = curx;
			sp->y = cury;
			sp->zoomx = (int)(zx << 12);
			sp->zoomy = (int)(zy << 12);
			sp->primask = primasks[(color & 0xc0) >> 6];
			sp++;
		}
	}
	zoomgfx_target t = { bm, s_pribuf, pitch, 0, VIS_Y0, SCREEN_W - 1, VIS_Y1, VIS_Y0 };
	while (sp != s_list) {
		sp--;
		zoomgfx_draw(&s_gfx, &t, sp->code, (sp->color % 64) * 64, sp->flipx, sp->flipy, sp->x, sp->y,
		             (uint32_t)sp->zoomx, (uint32_t)sp->zoomy, 1, sp->primask);
	}
}

static void slapshot_render(void)
{
	const uint16_t *p = s_snap->pal;
	for (int i = 0; i < 8192; i++)   // xRGB_888 over two words
		rbx_video.palette[i] = (uint32_t)((p[i * 2] & 0xff) << 16 | p[i * 2 + 1]);
	uint16_t *bm = rbx_video.bitmap;
	int pitch = rbx_video.pitch;
	for (int y = 0; y < VIS_H; y++)
		memset(bm + y * pitch, 0, SCREEN_W * 2);
	memset(s_pribuf, 0, (size_t)pitch * VIS_H);
	const tc0480scp_t *scp = s_snap_scp;
	const tc0360pri_t *pr = &s_snap->pri;
	int priority = tc0480scp_bg_priority(scp);
	int layer[5];
	layer[0] = (priority & 0xf000) >> 12;
	layer[1] = (priority & 0x0f00) >> 8;
	layer[2] = (priority & 0x00f0) >> 4;
	layer[3] = priority & 0x000f;
	layer[4] = 4;
	uint8_t tilepri[5], spritepri[4];
	tilepri[0] = tc0360pri_r(pr, 4) & 0x0f;
	tilepri[1] = tc0360pri_r(pr, 4) >> 4;
	tilepri[2] = tc0360pri_r(pr, 5) & 0x0f;
	tilepri[3] = tc0360pri_r(pr, 5) >> 4;
	tilepri[4] = tc0360pri_r(pr, 9) & 0x0f;
	spritepri[0] = tc0360pri_r(pr, 6) & 0x0f;
	spritepri[1] = tc0360pri_r(pr, 6) >> 4;
	spritepri[2] = tc0360pri_r(pr, 7) & 0x0f;
	spritepri[3] = tc0360pri_r(pr, 7) >> 4;
	tc0480scp_draw(scp, layer[0], bm, s_pribuf, pitch, VIS_Y0, VIS_H, 1);
	tc0480scp_draw(scp, layer[1], bm, s_pribuf, pitch, VIS_Y0, VIS_H, 2);
	tc0480scp_draw(scp, layer[2], bm, s_pribuf, pitch, VIS_Y0, VIS_H, 4);
	tc0480scp_draw(scp, layer[3], bm, s_pribuf, pitch, VIS_Y0, VIS_H, 8);
	uint32_t primasks[4] = { 0, 0, 0, 0 };
	for (int i = 0; i < 4; i++) {
		if (spritepri[i] < tilepri[layer[0] & 3]) primasks[i] |= 0xaaaa;
		if (spritepri[i] < tilepri[layer[1] & 3]) primasks[i] |= 0xcccc;
		if (spritepri[i] < tilepri[layer[2] & 3]) primasks[i] |= 0xf0f0;
		if (spritepri[i] < tilepri[layer[3] & 3]) primasks[i] |= 0xff00;
	}
	draw_sprites(bm, pitch, primasks, 0);
	tc0480scp_draw(scp, layer[4], bm, s_pribuf, pitch, VIS_Y0, VIS_H, 0);
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	handle_sprite_buffering();   // screen_update
	memcpy(s_snap->spr, s_spr_buf, sizeof(s_snap->spr));
	memcpy(s_snap->ext, s_ext, sizeof(s_snap->ext));
	memcpy(s_snap->pal, s_pal, sizeof(s_snap->pal));
	s_snap->pri = s_pri;
	s_snap->disabled = s_sprites_disabled;
	s_snap->area = s_sprites_active_area;
	s_snap->msx = s_master_scrollx;
	s_snap->msy = s_master_scrolly;
	tc0480scp_copy(s_snap_scp, s_scp);
	rbx_video_frame_done();
	// screen_vblank_no_buffer (rising edge)
	update_sprites_active_area();
	s_prepare_sprites = 1;
}

// ---- sprite graphics: 4bpp low + 2bpp high (MAME driver_init) ----
static int decode_sprites(void)
{
	uint32_t lsize, hsize;
	uint8_t *lo = rbx_region("sprites", &lsize);
	uint8_t *hi = rbx_region("sprites_hi", &hsize);
	if (!lo || !hi || zoomgfx_decode(&s_gfx, lo, lsize, ZGFX_16x16_PACKED_LSB))
		return -1;
	uint32_t n = s_gfx.count;
	if (hsize / 64 < n)
		n = s_gfx.count = hsize / 64;
	for (uint32_t i = 0; i < n; i++) {
		uint8_t *d = s_gfx.pix + (size_t)i * 256, any = 0;
		for (int j = 0; j < 256; j++) {   // layout_6bpp_hi
			int yy = j >> 4, xx = j & 15;
			uint8_t hb = hi[i * 64 + yy * 4 + (xx >> 2)];
			d[j] = (uint8_t)((d[j] & 0xf) | (((hb >> ((xx & 3) * 2)) & 3) << 4));
			any |= d[j];
		}
		s_gfx.opaque[i] = any != 0;
	}
	return 0;
}

// ---- init / frame ----
static void nv_save(void)
{
	if (!s_nv_path[0])
		return;
	FILE *f = fopen(s_nv_path, "wb");
	if (f) {
		fwrite(s_nv, 1, sizeof(s_nv), f);
		fclose(f);
	}
}

static void slapshot_reset(void)
{
	// (the palette / video RAM too: a game that has not written it yet showed the last game's, as at start it is 0)
	memset(s_pal, 0, sizeof(s_pal));
	memset(s_ram, 0, sizeof(s_ram));
	memset(s_spr, 0, sizeof(s_spr));
	memset(s_spr_buf, 0, sizeof(s_spr_buf));
	memset(s_ext, 0, sizeof(s_ext));
	memset(s_fio, 0, sizeof(s_fio));
	tc0480scp_reset(s_scp);
	tc0360pri_reset(&s_pri);
	adc0808_reset(&s_adc);
	s_sprites_disabled = 1;
	s_sprites_active_area = 0;
	s_master_scrollx = s_master_scrolly = 0;
	s_prepare_sprites = 0;
	s_r_flipscreen = 0;
	m68k_bus_irq_reset();
	m68k_pulse_reset();
	tsnd_reset();
	s_cpu_frac = s_adc_frac = 0;
	s_int6 = 0;
	for (int i = 0; i < 4; i++)
		s_gun[i] = 0x80;
}

static void slapshot_exit(void);

static int slapshot_init(const rbx_game *g)
{
	if (!s_list_hp && !(s_list_hp = calloc(1, sizeof(*s_list_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	if (!s_pal_hp && !(s_pal_hp = calloc(1, sizeof(*s_pal_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	if (!s_ram_hp && !(s_ram_hp = calloc(1, sizeof(*s_ram_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	if (!s_spr_hp && !(s_spr_hp = calloc(1, sizeof(*s_spr_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	if (!s_spr_buf_hp && !(s_spr_buf_hp = calloc(1, sizeof(*s_spr_buf_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	s_opwolf3 = !strcmp(g->machine, "opwolf3");
	s_rom = rbx_region("maincpu", &s_rom_size);
	uint32_t size;
	uint8_t *scp_gfx = rbx_region("tc0480scp", &size);
	if (!s_rom || !scp_gfx) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	m68k_bus_swap16(s_rom, s_rom_size);
	s_scp = (tc0480scp_t *)calloc(1, sizeof(tc0480scp_t));
	s_snap_scp = (tc0480scp_t *)malloc(sizeof(tc0480scp_t));
	s_snap = calloc(1, sizeof(*s_snap));
	if (!s_scp || !s_snap_scp || !s_snap || tc0480scp_init(s_scp, scp_gfx, size) || decode_sprites()) {
		rbx_fail("%s: out of memory (graphics)", g->set);
		slapshot_exit();
		return -1;
	}
	s_scp->x_offset = 30 + 3;
	s_scp->y_offset = 9;
	s_scp->text_xoffs = -1;
	s_scp->text_yoffs = -1;
	s_scp->flip_xoffs = 0;
	s_scp->flip_yoffs = 2;
	s_scp->col_base = 4096;
	*s_snap_scp = *s_scp;
	s_port_coins = rbx_port_find("COINS");
	s_port_buttons = rbx_port_find("BUTTONS");
	s_port_system = rbx_port_find("SYSTEM");
	s_port_joy = rbx_port_find("JOY");
	s_port_service = rbx_port_find("SERVICE");
	// timekeeper NVRAM (MAME default: 0xff, flags 0)
	// opwolf3: an all-0xff NVRAM skips the light gun calibration and leaves the game on a black screen; start from
	// zeros instead so that the game calibrates (done automatically below) and writes its settings
	memset(s_nv, s_opwolf3 ? 0x00 : 0xff, sizeof(s_nv));
	s_autocal = -1;
	s_nv[0x1ff0] = 0;
	s_tk[0] = 0; s_tk[1] = 0; s_tk[2] = 12; s_tk[3] = 6; s_tk[4] = 1; s_tk[5] = 7; s_tk[6] = 94;
	s_nv[0x1ff9] = 0;
	s_nv[0x1ffc] = 0;
	tk_to_ram();
	s_tk_control = 0;
	s_tk_frames = 0;
	s_nv_path[0] = 0;
	extern const char *rbx_nvram_dir(void);
	const char *dir = rbx_nvram_dir();
	if (dir) {
		snprintf(s_nv_path, sizeof(s_nv_path), "%s%s.nv", dir, g->set);
		FILE *f = fopen(s_nv_path, "rb");
		if (f) {
			if (fread(s_nv, 1, sizeof(s_nv), f) == sizeof(s_nv)) {
				tk_from_ram();
				s_autocal = 0;   // calibrated before
			}
			fclose(f);
		}
	}
	s_autocal = s_opwolf3 && s_autocal < 0 ? 1 : 0;
	// sound: YM2610B SSG 0.75 to both sides, FM 1.0
	tsnd_config sc;
	memset(&sc, 0, sizeof(sc));
	sc.map = TSND_MAP_E000;
	sc.z80_clock = 4000000;
	sc.frame_rate_mhz = 60000;
	sc.lines = LINES;
	sc.ym.type = RBX_YM2610B;
	sc.ym.clock = 8000000;
	sc.ym.gain[0] = 192;
	sc.ym.gain[1] = sc.ym.gain[2] = 128;   // (MAME's FM left and right at 1.0 each, averaged on one speaker)
	if (tsnd_init(&sc)) {
		rbx_fail("%s: sound init failed", g->set);
		slapshot_exit();
		return -1;
	}
	if (rbx_video_setup(SCREEN_W, VIS_H, 8192)) {
		slapshot_exit();
		return -1;
	}
	s_pribuf = (uint8_t *)calloc((size_t)rbx_video.pitch, VIS_H);
	if (!s_pribuf) {
		slapshot_exit();
		return -1;
	}
	m68k_init();
	m68k_set_cpu_type(M68K_CPU_TYPE_68000);
	m68k_bus_reset_map();
	m68k_bus_map(0, s_opwolf3 ? 0x1fffff : 0x0fffff, s_rom, s_rom_size, 1, 0);
	m68k_bus_map(0x500000, 0x50ffff, s_ram, sizeof(s_ram), 1, 1);
	m68k_bus_map(0x600000, 0x60ffff, s_spr, sizeof(s_spr), 1, 1);
	m68k_bus_map(0x700000, 0x701fff, s_ext, sizeof(s_ext), 1, 1);
	m68k_bus_map(0x800000, 0x80ffff, s_scp->ram, TC0480SCP_RAM_BYTES, 1, 1);
	m68k_bus_map(0x900000, 0x907fff, s_pal, sizeof(s_pal), 1, 1);
	m68k_bus.read8 = main_read8;
	m68k_bus.read16 = main_read16;
	m68k_bus.write8 = main_write8;
	m68k_bus.write16 = main_write16;
	slapshot_reset();
	return 0;
}

static void slapshot_exit(void)
{
	nv_save();
	s_nv_path[0] = 0;
	if (s_scp)
		tc0480scp_exit(s_scp);
	free(s_scp);
	free(s_snap_scp);
	s_scp = s_snap_scp = NULL;
	free(s_snap);
	s_snap = NULL;
	free(s_pribuf);
	s_pribuf = NULL;
	zoomgfx_free(&s_gfx);
	tsnd_exit();
	free(s_list_hp);
	s_list_hp = NULL;
	free(s_pal_hp);
	s_pal_hp = NULL;
	free(s_ram_hp);
	s_ram_hp = NULL;
	free(s_spr_hp);
	s_spr_hp = NULL;
	free(s_spr_buf_hp);
	s_spr_buf_hp = NULL;
}

static void slapshot_frame(void)
{
	if (s_opwolf3)
		update_gun();
	tk_frame();
	for (int line = 0; line < LINES; line++) {
		if (line == VIS_Y1 + 1) {
			vblank_capture();
			m68k_bus_hold_irq(5);
			s_int6 = 200000 - 500;   // MAME interrupt(): trigger_int6 after 199500 cycles
		}
		int cycles = rbx_line_cycles(&s_cpu_frac, CPU_CLOCK, 60000, LINES);
		if (s_int6 > 0 && s_int6 <= cycles) {
			int done = m68k_execute(s_int6);
			m68k_bus_hold_irq(6);
			s_int6 = 0;
			if (cycles - done > 0)
				m68k_execute(cycles - done);
		} else {
			m68k_execute(cycles);
			if (s_int6 > 0)
				s_int6 -= cycles;
		}
		if (s_opwolf3 && adc0808_advance(&s_adc, rbx_line_cycles(&s_adc_frac, 500000, 60000, LINES))) {
			s_adc.sar = (uint8_t)s_gun[s_adc.addr & 3];
			m68k_bus_set_line(3, 1);   // eoc_ff_callback -> IRQ3 (level)
		}
		tsnd_line();
	}
	tsnd_frame_end();
}

static int slapshot_state(void *ctx, int load)
{
	uint8_t cpu[1024];
	uint32_t csize = m68k_context_size();
	if (csize > sizeof(cpu))
		return -1;
	if (!load)
		m68k_get_context(cpu);
	uint32_t irq = m68k_bus_irq_pending();
	uint64_t fr[2] = { s_cpu_frac, s_adc_frac };
	int misc[24] = { s_sprites_disabled, s_sprites_active_area, s_master_scrollx, s_master_scrolly, s_prepare_sprites,
	                 s_r_flipscreen, s_int6, s_tk_control, (int)s_tk_frames, s_gun[0], s_gun[1], s_gun[2], s_gun[3],
	                 s_tk[0], s_tk[1], s_tk[2], s_tk[3], s_tk[4], s_tk[5], s_tk[6], s_autocal, 0, 0, 0 };
	int rc = 0;
	rc |= rbx_state_io(ctx, "m68k", cpu, csize);
	rc |= rbx_state_io(ctx, "irq", &irq, 4);
	rc |= rbx_state_io(ctx, "irqline", &m68k_bus.irq_asserted, sizeof(m68k_bus.irq_asserted));   // level lines (CPU A)
	rc |= rbx_state_io(ctx, "ram", s_ram, sizeof(s_ram));
	rc |= rbx_state_io(ctx, "spr", s_spr, sizeof(s_spr));
	rc |= rbx_state_io(ctx, "sprbuf", s_spr_buf, sizeof(s_spr_buf));
	rc |= rbx_state_io(ctx, "sprext", s_ext, sizeof(s_ext));
	rc |= rbx_state_io(ctx, "pal", s_pal, sizeof(s_pal));
	rc |= rbx_state_io(ctx, "scp", s_scp->ram, sizeof(s_scp->ram));
	rc |= rbx_state_io(ctx, "scpc", s_scp->ctrl, sizeof(s_scp->ctrl));
	rc |= rbx_state_io(ctx, "pri", s_pri.regs, sizeof(s_pri.regs));
	rc |= rbx_state_io(ctx, "fio", s_fio, sizeof(s_fio));
	rc |= rbx_state_io(ctx, "nvram", s_nv, sizeof(s_nv));
	rc |= rbx_state_io(ctx, "adc", &s_adc, sizeof(s_adc));
	rc |= rbx_state_io(ctx, "misc", misc, sizeof(misc));
	rc |= rbx_state_io(ctx, "frac", fr, sizeof(fr));
	rc |= tsnd_state(ctx, load);
	if (rc)
		return -1;
	if (load) {
		m68k_set_context(cpu);
		m68k_bus_set_irq_pending(irq);
		s_sprites_disabled = misc[0]; s_sprites_active_area = misc[1]; s_master_scrollx = misc[2];
		s_master_scrolly = misc[3]; s_prepare_sprites = misc[4]; s_r_flipscreen = misc[5]; s_int6 = misc[6];
		s_tk_control = (uint8_t)misc[7]; s_tk_frames = (uint32_t)misc[8];
		for (int i = 0; i < 4; i++)
			s_gun[i] = misc[9 + i];
		for (int i = 0; i < 7; i++)
			s_tk[i] = misc[13 + i];
		s_autocal = misc[20];
		tc0480scp_post_load(s_scp);
		s_cpu_frac = fr[0];
		s_adc_frac = fr[1];
	}
	return 0;
}

static int slapshot_save(void *ctx) { return slapshot_state(ctx, 0); }
static int slapshot_load(void *ctx) { return slapshot_state(ctx, 1); }
static int slapshot_supported(const char *machine) { return !strcmp(machine, "slapshot") || !strcmp(machine, "opwolf3"); }

const rbx_driver rbx_drv_slapshot = { "Slap Shot", slapshot_init, slapshot_exit, slapshot_reset, slapshot_frame,
                                      slapshot_save, slapshot_load, slapshot_supported, slapshot_render };
