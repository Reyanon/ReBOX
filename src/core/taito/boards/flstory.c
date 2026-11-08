/*  The FairyLand Story / Onna Sanshirou / Victorious Nine / Rumba Lumber, ported from MAME src/mame/taito/flstory.cpp
    (BSD-3-Clause, copyright-holders: Nicola Salmoria). Memory maps, the split-priority tilemap (two transparency
    groups, front / back halves), the sprites, the banked xBGR_444 palette, the Taito 68705 MCU interface
    (machine/taito68705) and the sound board (Z80 + YM2149 + MSM5232 + TA7630 volume + 8-bit DAC) follow MAME.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/z80/z80.h"
#include "../machine/taito68705.h"
#include "../../sound/ay8910.h"
#include "../../sound/msm5232.h"
#include "../../sound/ta7630.h"

#include <stdlib.h>
#include <string.h>

enum { FL_FLSTORY, FL_ONNA, FL_ONNA_MCU, FL_VICTNINE, FL_RUMBA };

// MAME set_raw(8 MHz, 510, 0, 256, 262, 16, 240)
#define LINES 262
#define VIS_Y0 16
#define VIS_Y1 239
#define SCREEN_W 256
#define RATE_MHZ 59871        // 8 MHz / (510 * 262)
#define AUDIO_CLOCK 4000000
#define MCU_CLOCK 768000      // 18.432 MHz / 6 / 4
#define SND_CLOCK 2000000
#define AUDIO_IRQ_PERIOD 32768   // 0x10000 ticks of 8 MHz in 4 MHz CPU cycles (~122 Hz)

typedef struct {
	z80 z;
	int irq;
} zcpu;

static int s_kind, s_has_mcu;
static uint32_t s_main_clock;
static uint8_t *s_main_rom, *s_audio_rom, *s_gfx;
static uint32_t s_main_size, s_audio_size, s_gfx_size;
static zcpu s_main, s_audio;
static int s_audio_reset;
static uint8_t s_vram[0x800], s_spr[0xa0], s_scrl[0x20], s_misc_ram[0x40], s_work[0x800], s_audio_ram[0x800];
static uint8_t s_pal[0x200], s_pal_ext[0x200];
static uint8_t s_gfxctrl, s_char_bank, s_pal_bank, s_flip;
static uint8_t s_latch, s_latch2;
static int s_latch_pending, s_latch2_pending, s_nmi_in1, s_nmi_line;
static taito68705_t s_mcu;
// sound
static ay8910_t s_ay;
static msm5232_t s_msm;
static int s_ta[16], s_ay_vol, s_ay_gain;
static int32_t s_dac_out;
static uint32_t s_audio_irq_count;
static int32_t *s_mix, *s_tmp;
static int s_mix_cap, s_mix_n;
static uint32_t s_snd_done, s_slice_start, s_slice_len;
static unsigned long s_cpu_start;
static int s_cpu_len;
static uint64_t s_frac_main, s_frac_audio, s_frac_mcu, s_frac_snd;
// video
static uint8_t *s_tiles, *s_sprites;
static uint32_t s_ntiles, s_nsprites;
static uint8_t s_snap_vram[0x800], s_snap_spr[0xa0], s_snap_scrl[0x20], s_snap_pal[0x200], s_snap_pal_ext[0x200];
static uint8_t s_snap_char_bank, s_snap_flip;
static uint8_t s_pri[SCREEN_W * (VIS_Y1 - VIS_Y0 + 1)];
// ports
static int s_p_sw[3], s_p_system, s_p_p1, s_p_p2, s_p_ex1, s_p_ex2;

static uint8_t port8(int p) { return (uint8_t)rbx_port_read(p); }

// ---- sound ----
static void snd_run_to(uint32_t pos)
{
	if (pos <= s_snd_done)
		return;
	uint32_t clk = pos - s_snd_done;
	int room = s_mix_cap - s_mix_n;
	if (room <= 0) {
		s_snd_done = pos;
		return;
	}
	int32_t *o = s_mix + s_mix_n;
	memset(s_tmp, 0, sizeof(int32_t) * (size_t)(ay8910_out_max(&s_ay, clk) < room ? ay8910_out_max(&s_ay, clk) : room));
	int n = ay8910_render(&s_ay, clk, s_tmp, room);
	const int vol = s_ay_vol;
	for (int k = 0; k < n; k++)
		o[k] += s_tmp[k] * vol / 256 + s_dac_out;
	msm5232_render(&s_msm, clk, o, room);
	s_mix_n += n;
	s_snd_done = pos;
}

static void snd_sync(void)
{
	uint32_t done = (uint32_t)(s_audio.z.cyc - s_cpu_start);
	if ((int)done > s_cpu_len)
		done = (uint32_t)s_cpu_len;
	snd_run_to(s_slice_start + (s_cpu_len ? (uint32_t)((uint64_t)s_slice_len * done / (uint32_t)s_cpu_len) : 0));
}

static void update_nmi(void)
{
	int line = s_latch_pending && s_nmi_in1;   // INPUT_MERGER_ALL_HIGH
	if (line && !s_nmi_line)
		s_audio.z.nmi_pending = 1;
	s_nmi_line = line;
}

static void ay_pa_w(void *u, uint8_t data) { (void)u; snd_sync(); s_ay_vol = s_ta[data >> 4]; }   // sound_control_2_w
static void ay_pb_w(void *u, uint8_t data) { (void)u; (void)data; }                               // sound_control_3_w

static uint8_t snd_flag_r(void) { return (uint8_t)((s_latch_pending ? 0 : 1) | (s_latch2_pending ? 2 : 0)); }

// ---- main CPU ----
static uint8_t mcu_status_r(void)
{
	return (uint8_t)((taito68705_host_semaphore_r(&s_mcu) ? 0 : 1) | (taito68705_mcu_semaphore_r(&s_mcu) ? 2 : 0));
}

static void audio_reset_cpu(void);

static uint8_t main_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0xc000) return a < s_main_size ? s_main_rom[a] : 0xff;
	if (a < 0xd000) return s_vram[a & 0x7ff];
	if (a >= 0xe000) return s_work[a & 0x7ff];
	if (a >= 0xdc00 && a < 0xdca0) return s_spr[a - 0xdc00];
	if (a >= 0xdca0 && a < 0xdcc0) return s_scrl[a - 0xdca0];
	if (a >= 0xdcc0 && a < 0xdd00) {
		if ((s_kind == FL_VICTNINE || s_kind == FL_RUMBA) && a == 0xdce0) return s_gfxctrl;
		if (s_kind == FL_FLSTORY || s_kind == FL_ONNA || s_kind == FL_ONNA_MCU) return s_misc_ram[a - 0xdcc0];
		return 0;
	}
	if (a >= 0xdd00 && a < 0xdf00) {
		int o = a - 0xdd00, i = (o & 0xff) + (s_pal_bank << 8);
		return (o & 0x100) ? s_pal_ext[i] : s_pal[i];
	}
	switch (a) {
	case 0xd000: return s_has_mcu ? taito68705_data_r(&s_mcu) : 0;
	case 0xd400: s_latch2_pending = 0; return s_latch2;
	case 0xd401: return snd_flag_r();
	case 0xd800: return port8(s_p_sw[0]);
	case 0xd801: return port8(s_p_sw[1]);
	case 0xd802: return port8(s_p_sw[2]);
	case 0xd803: return port8(s_p_system);
	case 0xd804: return port8(s_p_p1);
	case 0xd805:
		if (!s_has_mcu) return 0;
		if (s_kind == FL_VICTNINE) return (uint8_t)((mcu_status_r() & 3) | (port8(s_p_ex1) & 0xfc));
		return mcu_status_r();
	case 0xd806: return port8(s_p_p2);
	case 0xd807: return (s_kind == FL_VICTNINE || s_kind == FL_RUMBA) ? port8(s_p_ex2) : 0;
	}
	return 0;
}

static void main_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a < 0xc000) return;
	if (a < 0xd000) { s_vram[a & 0x7ff] = v; return; }
	if (a >= 0xe000) { s_work[a & 0x7ff] = v; return; }
	if (a >= 0xdc00 && a < 0xdca0) { s_spr[a - 0xdc00] = v; return; }
	if (a >= 0xdca0 && a < 0xdcc0) { s_scrl[a - 0xdca0] = v; return; }
	if (a >= 0xdcc0 && a < 0xdd00) {
		if (s_kind == FL_VICTNINE || s_kind == FL_RUMBA) {
			if (a == 0xdce0) {   // victnine_gfxctrl_w
				s_gfxctrl = v;
				s_pal_bank = (v & 0x20) >> 5;
				if (v & 4)
					s_flip = v & 1;
			}
		} else
			s_misc_ram[a - 0xdcc0] = v;
		return;
	}
	if (a >= 0xdd00 && a < 0xdf00) {
		int o = a - 0xdd00, i = (o & 0xff) + (s_pal_bank << 8);
		if (o & 0x100) s_pal_ext[i] = v; else s_pal[i] = v;
		return;
	}
	switch (a) {
	case 0xd000: if (s_has_mcu) taito68705_data_w(&s_mcu, v); return;
	case 0xd400: s_latch = v; s_latch_pending = 1; update_nmi(); return;
	case 0xd403: {   // snd_reset_w
		int r = v & 1;
		if (r && !s_audio_reset)
			audio_reset_cpu();
		s_audio_reset = r;
		return;
	}
	case 0xdf03:
		if (s_kind == FL_FLSTORY || s_kind == FL_ONNA || s_kind == FL_ONNA_MCU) {   // flstory_gfxctrl_w
			s_gfxctrl = v;
			s_flip = (~v) & 1;
			s_char_bank = (v & 0x10) >> 4;
			s_pal_bank = (v & 0x20) >> 5;
		}
		return;
	}
}

// ---- sound CPU ----
static uint8_t audio_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0xc000) return a < s_audio_size ? s_audio_rom[a] : 0xff;
	if (a < 0xc800) return s_audio_ram[a & 0x7ff];
	if (a == 0xd800) { s_latch_pending = 0; update_nmi(); return s_latch; }
	if (a == 0xda00) return snd_flag_r();
	if (a >= 0xe000 && a < 0xf000) return a < s_audio_size ? s_audio_rom[a] : 0xff;
	return 0;
}

static void audio_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a < 0xc000) return;
	if (a < 0xc800) { s_audio_ram[a & 0x7ff] = v; return; }
	if (a == 0xc800 || a == 0xc801) { snd_sync(); ay8910_write(&s_ay, a & 1, v); return; }
	if (a >= 0xca00 && a <= 0xca0d) { snd_sync(); msm5232_write(&s_msm, a - 0xca00, v); return; }
	switch (a) {
	case 0xcc00:   // sound_control_0_w: TA7630 on MSM5232 group 1
		snd_sync();
		for (int i = 0; i < 4; i++) s_msm.gain[i] = s_ta[v >> 4];
		return;
	case 0xce00:   // sound_control_1_w: group 2
		snd_sync();
		for (int i = 0; i < 4; i++) s_msm.gain[i + 4] = s_ta[v >> 4];
		return;
	case 0xd800: s_latch2 = v; s_latch2_pending = 1; return;
	case 0xda00: s_nmi_in1 = 1; update_nmi(); return;
	case 0xdc00: s_nmi_in1 = 0; update_nmi(); return;
	case 0xde00:   // DAC_8BIT_R2R, route 0.1
		snd_sync();
		s_dac_out = (int32_t)(32767.0 * 0.1 * (2.0 * v / 255.0 - 1.0));
		return;
	}
}

static uint8_t no_in(z80 *z, uint8_t p) { (void)z; (void)p; return 0xff; }
static void no_out(z80 *z, uint8_t p, uint8_t v) { (void)z; (void)p; (void)v; }

static void zcpu_init(zcpu *c, uint8_t (*rd)(void *, uint16_t), void (*wr)(void *, uint16_t, uint8_t))
{
	unsigned long cyc = c->z.cyc;
	z80_init(&c->z);
	c->z.read_byte = rd;
	c->z.write_byte = wr;
	c->z.port_in = no_in;
	c->z.port_out = no_out;
	c->z.int_data = 0xff;
	c->z.cyc = cyc;
	c->irq = 0;
}
static void audio_reset_cpu(void) { zcpu_init(&s_audio, audio_read, audio_write); }

static void zcpu_run(zcpu *c, int cycles)
{
	unsigned long end = c->z.cyc + (unsigned long)cycles;
	while ((long)(c->z.cyc - end) < 0) {
		c->z.int_pending = c->irq ? 1 : 0;
		z80_step(&c->z);
		if (c->irq && !c->z.int_pending)
			c->irq = 0;   // HOLD_LINE acknowledged
	}
}

// ---- video ----
static void decode_gfx(void)
{
	// ROMREGION_INVERT (not applied by the loader)
	for (uint32_t i = 0; i < s_gfx_size; i++)
		s_gfx[i] ^= 0xff;
	const uint32_t half = s_gfx_size / 2 * 8;   // in bits
	const uint32_t planes[4] = { half, half + 4, 0, 4 };
	static const int xo[16] = { 3, 2, 1, 0, 11, 10, 9, 8, 131, 130, 129, 128, 139, 138, 137, 136 };
	for (uint32_t t = 0; t < s_ntiles; t++)
		for (int y = 0; y < 8; y++)
			for (int x = 0; x < 8; x++) {
				int v = 0;
				for (int p = 0; p < 4; p++) {
					uint32_t b = t * 128 + planes[p] + (uint32_t)xo[x] + (uint32_t)y * 16;
					v = v << 1 | ((s_gfx[b >> 3] >> (7 - (b & 7))) & 1);
				}
				s_tiles[t * 64 + y * 8 + x] = (uint8_t)v;
			}
	for (uint32_t t = 0; t < s_nsprites; t++)
		for (int y = 0; y < 16; y++)
			for (int x = 0; x < 16; x++) {
				int v = 0;
				uint32_t yo = (uint32_t)(y < 8 ? y * 16 : 256 + (y - 8) * 16);
				for (int p = 0; p < 4; p++) {
					uint32_t b = t * 512 + planes[p] + (uint32_t)xo[x] + yo;
					v = v << 1 | ((s_gfx[b >> 3] >> (7 - (b & 7))) & 1);
				}
				s_sprites[t * 256 + y * 16 + x] = (uint8_t)v;
			}
}

// one pass of the tilemap: category (-1 = all), layer 0 = front half, 1 = back half (-1 = whole tile, 2 = both halves
// with the flstory priority codes), pcode ORed
// into the priority buffer
static void draw_tilemap(int category, int layer, int pcode)
{
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	const int victnine = s_kind == FL_VICTNINE, rumba = s_kind == FL_RUMBA;
	const int fl = s_snap_flip;
	for (int y = VIS_Y0; y <= VIS_Y1; y++) {
		uint16_t *d = bm + (y - VIS_Y0) * pitch;
		uint8_t *pr = s_pri + (y - VIS_Y0) * SCREEN_W;
		int t_ti = -1, t_color = 0, t_fx = 0, t_fy = 0, t_group = 0;
		const uint8_t *t_pix = s_tiles;
		for (int x = 0; x < SCREEN_W; x++) {
			// MAME tilemap flip: the screen is 510 x 262, the tilemap 256 x 256
			int lx, ly;
			if (!fl) {
				lx = x;
				int col = lx >> 3;
				ly = (y + s_snap_scrl[col]) & 0xff;
			} else {
				lx = (x - 254) & 0xff;
				lx = 255 - lx;
				int col = lx >> 3;
				ly = (y - (6 + s_snap_scrl[col])) & 0xff;
				ly = 255 - ly;
			}
			int ti = (ly >> 3) * 32 + (lx >> 3);
			if (ti != t_ti) {   // (a tile's values for the run of its pixels on this line)
				t_ti = ti;
				uint8_t code = s_snap_vram[ti * 2], attr = s_snap_vram[ti * 2 + 1];
				uint32_t num;
				t_fx = t_fy = t_group = 0;
				if (victnine) {
					num = ((attr & 0x38) << 5) + code;
					t_color = attr & 7;
					t_fx = (attr >> 6) & 1;
					t_fy = (attr >> 7) & 1;
				} else {
					num = code + ((attr & 0xc0) << 2) + 0x400 + 0x800 * s_snap_char_bank;
					t_color = attr & 0x0f;
					if (!rumba) {
						t_fx = (attr >> 3) & 1;
						t_fy = (attr >> 4) & 1;
					}
					t_group = (attr >> 5) & 1;
				}
				while (num >= s_ntiles)   // (num % s_ntiles: num < 0x1000, a few subtractions, not a divide per pixel)
					num -= s_ntiles;
				t_pix = s_tiles + num * 64;
			}
			const int color = t_color, group = t_group;
			if (category >= 0 && group != category)
				continue;
			int px = lx & 7, py = ly & 7;
			if (t_fx) px = 7 - px;
			if (t_fy) py = 7 - py;
			uint8_t p = t_pix[py * 8 + px];
			// transmask: group 0 front 0x3fff / back 0xc000, group 1 front 0x8000 / back 0x7fff
			static const uint16_t tm[2][2] = { { 0x3fff, 0xc000 }, { 0x8000, 0x7fff } };
			if (layer == 2) {
				// the four flstory passes at once (each pixel belongs to one category): back 1 / 2, front 4 / 8
				int back = !(tm[group][1] & (1 << p)), front = !(tm[group][0] & (1 << p));
				if (!back && !front)
					continue;
				pcode = (back ? (group ? 2 : 1) : 0) | (front ? (group ? 8 : 4) : 0);
			} else if (layer >= 0 && (tm[group][layer] & (1 << p)))
				continue;
			d[x] = (uint16_t)(color * 16 + p);
			pr[x] |= (uint8_t)pcode;
		}
	}
}

static void draw_sprites_fl(void)
{
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	const int fl = s_snap_flip;
	const int victnine_style = s_kind == FL_VICTNINE || s_kind == FL_RUMBA;
	for (int k = 0; k < 0x20; k++) {
		int i = victnine_style ? k : 0x1f - k;
		uint8_t pr = s_snap_spr[0xa0 - 1 - i];
		int offs = (pr & 0x1f) * 4;
		uint32_t pmask = victnine_style ? 0 : (((pr & 0x80) ? 0xff00u : (0xff00u | 0xf0f0u)) | 1u << 31);
		const uint8_t *s = s_snap_spr + offs;
		uint32_t code = victnine_style ? (uint32_t)(s[2] + ((s[1] & 0x20) << 3)) : (uint32_t)(s[2] + ((s[1] & 0x30) << 4));
		int sx = s[3], sy = s[0];
		int flipx = (s[1] >> 6) & 1, flipy = (s[1] >> 7) & 1;
		if (fl) {
			sx = victnine_style ? (240 - sx + 1) & 0xff : (240 - sx) & 0xff;
			sy = victnine_style ? sy + 1 : sy - 1;
			flipx = !flipx;
			flipy = !flipy;
		} else
			sy = victnine_style ? 240 - sy + 1 : 240 - sy - 1;
		const uint8_t *g = s_sprites + (code % s_nsprites) * 256;
		const uint16_t base = (uint16_t)(256 + (s[1] & 0x0f) * 16);
		for (int rep = 0; rep < (sx > 240 ? 2 : 1); rep++) {
			int ox = rep ? sx - 256 : sx;
			for (int ty = 0; ty < 16; ty++) {
				int y = sy + ty;
				if (y < VIS_Y0 || y > VIS_Y1) continue;
				const uint8_t *r = g + (flipy ? 15 - ty : ty) * 16;
				uint16_t *d = bm + (y - VIS_Y0) * pitch;
				uint8_t *pp = s_pri + (y - VIS_Y0) * SCREEN_W;
				for (int tx = 0; tx < 16; tx++) {
					int x = ox + tx;
					if (x < 0 || x >= SCREEN_W) continue;
					uint8_t p = r[flipx ? 15 - tx : tx];
					if (p == 15) continue;
					if (victnine_style)
						d[x] = (uint16_t)(base + p);
					else {
						if (!((1u << (pp[x] & 0x1f)) & pmask))
							d[x] = (uint16_t)(base + p);
						pp[x] = 31;
					}
				}
			}
		}
	}
}

static void fl_render(void)
{
	for (int i = 0; i < 512; i++) {   // xBGR_444: base = low byte, ext = high byte
		int c = s_snap_pal[i] | s_snap_pal_ext[i] << 8;
		rbx_video.palette[i] = (uint32_t)((c & 15) * 17 << 16 | ((c >> 4) & 15) * 17 << 8 | ((c >> 8) & 15) * 17);
	}
	uint16_t *bm = rbx_video.bitmap;
	for (int y = 0; y < rbx_video.height; y++)
		memset(bm + y * rbx_video.pitch, 0, SCREEN_W * sizeof(uint16_t));
	memset(s_pri, 0, sizeof(s_pri));
	if (s_kind == FL_VICTNINE) {
		draw_tilemap(-1, -1, 0);
		draw_sprites_fl();
	} else if (s_kind == FL_RUMBA) {
		draw_tilemap(-1, 1, 0);   // = (0, back), (1, back): each pixel is of one group, the two passes draw apart
		draw_sprites_fl();
		draw_tilemap(0, 0, 0);
		draw_sprites_fl();
		draw_tilemap(1, 0, 0);
	} else {
		draw_tilemap(-1, 2, 0);   // = (0, back, 1), (1, back, 2), (0, front, 4), (1, front, 8)
		draw_sprites_fl();
	}
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap_vram, s_vram, sizeof(s_vram));
	memcpy(s_snap_spr, s_spr, sizeof(s_spr));
	memcpy(s_snap_scrl, s_scrl, sizeof(s_scrl));
	memcpy(s_snap_pal, s_pal, sizeof(s_pal));
	memcpy(s_snap_pal_ext, s_pal_ext, sizeof(s_pal_ext));
	s_snap_char_bank = s_char_bank;
	s_snap_flip = s_flip;
	rbx_video_frame_done();
}

// ---- init / frame ----
static int kind_of(const char *m)
{
	if (!strcmp(m, "flstory")) return FL_FLSTORY;
	if (!strcmp(m, "onna34ro")) return FL_ONNA;
	if (!strcmp(m, "onna34ro_mcu")) return FL_ONNA_MCU;
	if (!strcmp(m, "victnine")) return FL_VICTNINE;
	if (!strcmp(m, "rumba")) return FL_RUMBA;
	return -1;
}

static void fl_reset(void)
{
	// (first: the sound syncs read them, from a CPU write or a port callback of a chip reset below, before a frame
	// sets them; the last game's values made a render of billions of clocks, the next game of the board hung)
	s_slice_start = s_slice_len = s_cpu_start = s_cpu_len = 0;
	memset(s_vram, 0, sizeof(s_vram));
	memset(s_spr, 0, sizeof(s_spr));
	memset(s_scrl, 0, sizeof(s_scrl));
	memset(s_misc_ram, 0, sizeof(s_misc_ram));
	memset(s_work, 0, sizeof(s_work));
	memset(s_audio_ram, 0, sizeof(s_audio_ram));
	memset(s_pal, 0, sizeof(s_pal));
	memset(s_pal_ext, 0, sizeof(s_pal_ext));
	memset(&s_main, 0, sizeof(s_main));
	memset(&s_audio, 0, sizeof(s_audio));
	zcpu_init(&s_main, main_read, main_write);
	zcpu_init(&s_audio, audio_read, audio_write);
	s_audio_reset = 0;
	// machine_reset
	s_gfxctrl = 0;
	s_char_bank = 1;
	s_pal_bank = 0;
	s_flip = 0;
	s_latch = s_latch2 = 0;
	s_latch_pending = s_latch2_pending = s_nmi_in1 = s_nmi_line = 0;
	if (s_has_mcu)
		taito68705_reset(&s_mcu);
	ay8910_reset(&s_ay);
	msm5232_reset(&s_msm);
	for (int i = 0; i < 8; i++)
		s_msm.gain[i] = 256;
	s_ay_vol = 256;
	s_dac_out = (int32_t)(32767.0 * 0.1 * -1.0);
	s_audio_irq_count = 0;
	s_frac_main = s_frac_audio = s_frac_mcu = s_frac_snd = 0;
	s_snd_done = 0;
	s_mix_n = 0;
}

static void fl_exit(void)
{
	free(s_tiles);
	free(s_sprites);
	s_tiles = s_sprites = NULL;
	free(s_mix);
	free(s_tmp);
	s_mix = s_tmp = NULL;
}

static int fl_init(const rbx_game *g)
{
	s_kind = kind_of(g->machine);
	if (s_kind < 0) {
		rbx_fail("%s: machine %s is not emulated", g->set, g->machine);
		return -1;
	}
	s_has_mcu = s_kind != FL_ONNA;
	s_main_clock = (s_kind == FL_VICTNINE || s_kind == FL_RUMBA) ? 4000000 : 5366500;
	s_main_rom = rbx_region("maincpu", &s_main_size);
	s_audio_rom = rbx_region("audiocpu", &s_audio_size);
	s_gfx = rbx_region("tiles", &s_gfx_size);
	if (!s_main_rom || !s_audio_rom || !s_gfx || s_gfx_size < 0x8000) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	if (s_has_mcu) {
		uint32_t msize;
		uint8_t *mrom = rbx_region("bmcu:mcu", &msize);
		memset(&s_mcu, 0, sizeof(s_mcu));
		if (!mrom || msize < 0x800 || taito68705_init(&s_mcu, TAITO68705_STD, mrom, 0x800)) {
			rbx_fail("%s: MCU ROM missing", g->set);
			return -1;
		}
	}
	s_p_sw[0] = rbx_port_find("SWA");
	s_p_sw[1] = rbx_port_find("SWB");
	s_p_sw[2] = rbx_port_find("SWC");
	s_p_system = rbx_port_find("SYSTEM");
	s_p_p1 = rbx_port_find("P1");
	s_p_p2 = rbx_port_find("P2");
	s_p_ex1 = rbx_port_find("EXTRA_P1");
	s_p_ex2 = rbx_port_find("EXTRA_P2");
	s_ntiles = s_gfx_size / 2 / 16;
	s_nsprites = s_gfx_size / 2 / 64;
	s_tiles = (uint8_t *)malloc((size_t)s_ntiles * 64);
	s_sprites = (uint8_t *)malloc((size_t)s_nsprites * 256);
	if (!s_tiles || !s_sprites) {
		fl_exit();
		return -1;
	}
	decode_gfx();
	ta7630_table(s_ta);
	s_ay_gain = s_kind == FL_VICTNINE ? 128 : 26;   // route 0.5 / 0.1
	ay8910_init(&s_ay, AY8910_TYPE_YM2149, SND_CLOCK, AY8910_LEGACY_OUTPUT, s_ay_gain, 4, 1);
	s_ay.port_a_write = ay_pa_w;
	s_ay.port_b_write = ay_pb_w;
	static const double caps[8] = { 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6 };
	msm5232_init(&s_msm, SND_CLOCK, caps, 2);
	uint32_t rate = ay8910_rate(&s_ay);
	s_mix_cap = (int)(rate / 50) + 64;
	s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	s_tmp = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	if (!s_mix || !s_tmp || rbx_audio_setup(rate) || rbx_video_setup(SCREEN_W, VIS_Y1 - VIS_Y0 + 1, 512)) {
		fl_exit();
		return -1;
	}
	fl_reset();
	return 0;
}

static void fl_frame(void)
{
	rbx_audio.frame_mhz = RATE_MHZ;   // (the board's frame rate: a frame's sound is played in a 60 Hz frame, as its picture)
	for (int i = 0; i < LINES; i++) {
		int line = (VIS_Y1 + 1 + i) % LINES;
		if (line == VIS_Y1 + 1) {
			vblank_capture();
			s_main.irq = 1;   // irq0_line_hold
		}
		int mc = rbx_line_cycles(&s_frac_main, s_main_clock, RATE_MHZ, LINES);
		int ac = rbx_line_cycles(&s_frac_audio, AUDIO_CLOCK, RATE_MHZ, LINES);
		int uc = rbx_line_cycles(&s_frac_mcu, MCU_CLOCK, RATE_MHZ, LINES);
		int sc = rbx_line_cycles(&s_frac_snd, SND_CLOCK, RATE_MHZ, LINES);
		zcpu_run(&s_main, mc);
		s_slice_start = s_snd_done;
		s_slice_len = (uint32_t)sc;
		s_cpu_start = s_audio.z.cyc;
		s_cpu_len = ac;
		s_audio_irq_count += (uint32_t)ac;
		if (s_audio_irq_count >= AUDIO_IRQ_PERIOD) {
			s_audio_irq_count -= AUDIO_IRQ_PERIOD;
			s_audio.irq = 1;
		}
		if (s_audio_reset)
			s_audio.z.cyc += (unsigned long)ac;
		else
			zcpu_run(&s_audio, ac);
		if (s_has_mcu)
			taito68705_execute(&s_mcu, uc);
		snd_run_to(s_slice_start + s_slice_len);
	}
	for (int i = 0; i < s_mix_n; i++)
		rbx_audio_put(s_mix[i]);
	memset(s_mix, 0, sizeof(int32_t) * (size_t)s_mix_n);
	s_mix_n = 0;
	s_snd_done = 0;
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

static int fl_state(void *ctx, int load)
{
	int rc = 0;
	rc |= zcpu_state(ctx, "main", &s_main, load);
	rc |= zcpu_state(ctx, "audio", &s_audio, load);
	rc |= rbx_state_io(ctx, "vram", s_vram, sizeof(s_vram));
	rc |= rbx_state_io(ctx, "spr", s_spr, sizeof(s_spr));
	rc |= rbx_state_io(ctx, "scrl", s_scrl, sizeof(s_scrl));
	rc |= rbx_state_io(ctx, "mram", s_misc_ram, sizeof(s_misc_ram));
	rc |= rbx_state_io(ctx, "work", s_work, sizeof(s_work));
	rc |= rbx_state_io(ctx, "aram", s_audio_ram, sizeof(s_audio_ram));
	rc |= rbx_state_io(ctx, "pal", s_pal, sizeof(s_pal));
	rc |= rbx_state_io(ctx, "palx", s_pal_ext, sizeof(s_pal_ext));
	if (s_has_mcu) {
		rc |= rbx_state_io(ctx, "tmcu", &s_mcu.st, sizeof(s_mcu.st));
		rc |= rbx_state_io(ctx, "tmcu6805", &s_mcu.mcu.st, sizeof(s_mcu.mcu.st));
	}
	rc |= rbx_state_io(ctx, "ay", &s_ay, ay8910_state_size());
	rc |= rbx_state_io(ctx, "msm", &s_msm, msm5232_state_size());
	int32_t misc[16] = { s_audio_reset, s_gfxctrl, s_char_bank, s_pal_bank, s_flip, s_latch, s_latch2, s_latch_pending,
		s_latch2_pending, s_nmi_in1, s_nmi_line, s_ay_vol, s_dac_out, (int32_t)s_audio_irq_count, 0, 0 };
	rc |= rbx_state_io(ctx, "misc", misc, sizeof(misc));
	if (rc)
		return -1;
	if (load) {
		s_audio_reset = misc[0]; s_gfxctrl = (uint8_t)misc[1]; s_char_bank = (uint8_t)misc[2]; s_pal_bank = (uint8_t)misc[3];
		s_flip = (uint8_t)misc[4]; s_latch = (uint8_t)misc[5]; s_latch2 = (uint8_t)misc[6]; s_latch_pending = misc[7];
		s_latch2_pending = misc[8]; s_nmi_in1 = misc[9]; s_nmi_line = misc[10]; s_ay_vol = misc[11]; s_dac_out = misc[12];
		s_audio_irq_count = (uint32_t)misc[13];
	}
	return 0;
}

static int fl_save(void *ctx) { return fl_state(ctx, 0); }
static int fl_load(void *ctx) { return fl_state(ctx, 1); }
static int fl_supported(const char *machine) { return kind_of(machine) >= 0; }

const rbx_driver rbx_drv_flstory = { "The FairyLand Story", fl_init, fl_exit, fl_reset, fl_frame, fl_save, fl_load, fl_supported, fl_render };
