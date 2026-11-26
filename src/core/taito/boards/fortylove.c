/*  Forty-Love / Field Day / The Undoukai, ported from MAME src/mame/taito/40love.cpp, 40love_v.cpp, 40love.h
    (GPL-2.0+, copyright-holders: Jarek Burczynski) and src/devices/machine/mb14241.cpp (BSD-3-Clause,
    copyright-holders: Tomasz Slanina). Memory maps of the three machine configs (_40love, _40lovebl, undoukai), the
    double-buffered 2bpp pixel layer, the row-scrolled 64 x 32 tilemap, the two sprite lists, the PROM palette,
    the MB14241 shifter, the Taito 68705 MCU interface (machine/taito68705) and the sound board (Z80 + YM2149 + MSM5232
    + TA7630 volume + 8-bit DAC) follow MAME.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/z80/z80.h"
#include "../../machine/taito68705.h"
#include "../../sound/ay8910.h"
#include "../../sound/msm5232.h"
#include "../../sound/ta7630.h"

#include <stdlib.h>
#include <string.h>

enum { FT_40LOVE, FT_40LOVEBL, FT_UNDOUKAI };

// MAME: 60 Hz, 512 x 256, visible 128-383 x 16-239, vblank_int at line 240
#define LINES 256
#define VIS_X0 128
#define VIS_Y0 16
#define VIS_Y1 239
#define SCREEN_W 256
#define VIS_H (VIS_Y1 - VIS_Y0 + 1)
#define RATE_MHZ 60000
#define CPU_CLOCK 4000000
#define MCU_CLOCK 768000      // 18.432 MHz / 6 / 4
#define SND_CLOCK 2000000
#define AUDIO_IRQ_HZ 120      // periodic irq0_line_hold at 2 * 60 Hz

typedef struct {
	z80 z;
	int irq;
} zcpu;

static int s_kind, s_has_mcu;
static uint8_t *s_main_rom, *s_audio_rom, *s_gfx1, *s_gfx2, *s_proms;
static uint32_t s_main_size, s_audio_size;
static uint16_t s_ram_base, s_bank_base, s_io_base, s_vid_base, s_spr2_off, s_spr2_len;
static zcpu s_main, s_audio;
static uint8_t s_work[0x800], s_vram[0x800], s_vctrl[0x40], s_spr[0x40], s_cram[0x40], s_spr2[0x40], s_audio_ram[0x800];
static uint8_t (*s_pixram_hp)[2][0x4000];
#define s_pixram (*s_pixram_hp)
static uint8_t (*s_pixbmp_hp)[2][256 * 256];   // pen (0-3) | color bank << 2
#define s_pixbmp (*s_pixbmp_hp)
static int s_pixram_sel, s_color_bank, s_screen_disable, s_flip, s_pix_redraw, s_bank;
static int16_t s_scrollx[32];
static uint16_t s_shift_data;
static uint8_t s_shift_count;
static uint8_t s_latch, s_latch2;
static int s_latch_pending, s_latch2_pending, s_nmi_in1, s_nmi_line;
static taito68705_t s_mcu;
// sound
static ay8910_t s_ay;
static msm5232_t s_msm;
static int s_ta[16], s_ay_vol;
static uint8_t s_snd_ctrl[4];
static int32_t s_dac_out;
static uint32_t s_audio_irq_acc;
static int32_t *s_mix, *s_tmp;
static int s_mix_cap, s_mix_n;
static uint32_t s_snd_done, s_slice_start, s_slice_len;
static unsigned long s_cpu_start;
static int s_cpu_len;
static uint64_t s_frac_main, s_frac_audio, s_frac_mcu, s_frac_snd;
// video
static uint8_t *s_tiles, *s_sprites;
static uint32_t s_ntiles, s_nsprites;
static uint32_t s_palette[1024];
static uint8_t s_snap_pix[256 * 256], s_snap_vram[0x800], s_snap_cram[0x40], s_snap_spr[0x40], s_snap_spr2[0x40];
static int16_t s_snap_scrollx[32];
static int s_snap_flip, s_snap_disable, s_snap_vc1, s_snap_color_bank;
static uint32_t s_rng = 0x9e3779b9u;   // machine().rand() of draw_sprites (render side only)
// ports
static int s_p_dsw[3], s_p_system, s_p_p1, s_p_p2;

static const int s_pix_color[4] = { 0x000, 0x1e3, 0x16c, 0x1ec };

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

static void ay_pa_w(void *u, uint8_t data) { (void)u; snd_sync(); s_snd_ctrl[2] = data; s_ay_vol = s_ta[data >> 4]; }   // sound_control_2_w
static void ay_pb_w(void *u, uint8_t data) { (void)u; s_snd_ctrl[3] = data; }                                           // sound_control_3_w

// ---- video memory ----
static void set_scroll_x(int offset)
{
	int i = offset & ~1;
	int x = ((s_cram[i] & 0x80) << 1) | s_cram[i + 1];   // 9 bits signed
	if (s_flip)
		x += 0x51;
	else
		x -= 0x50;
	x &= 0x1ff;
	if (x & 0x100)
		x -= 0x200;
	s_scrollx[offset / 2] = (int16_t)x;
}

static void plot_pix(int offset)
{
	const int x = (offset & 0x1f) * 8;
	const int y = (offset >> 5) & 0xff;
	const uint8_t *ram = s_pixram[s_pixram_sel];
	const int d1 = ram[offset], d2 = ram[offset + 0x2000];
	uint8_t *d = s_pixbmp[s_pixram_sel] + y * 256 + x;
	const uint8_t bank = (uint8_t)(s_color_bank ? 4 : 0);
	for (int i = 0; i < 8; i++)
		d[i] = (uint8_t)((((d2 >> i) & 1) | (((d1 >> i) & 1) << 1)) | bank);
}

static void pixram_sel_w(uint8_t data)
{
	int f = data & 1;
	int cur_col_bank = (data & 8) != 0;
	s_pixram_sel = (data & 4) >> 2;
	s_screen_disable = (data & 0x20) != 0;
	if (cur_col_bank != s_color_bank) {
		s_color_bank = cur_col_bank;
		s_pix_redraw = 1;   // redraw_pixels (the tilemap colors follow s_color_bank directly)
	}
	if (s_flip != f) {
		s_flip = f;
		s_pix_redraw = 1;
		for (int offs = 0; offs < 32; offs++)
			set_scroll_x(offs * 2);
	}
}

static void colorram_w(int offset, uint8_t data)
{
	if (s_cram[offset] != data) {
		s_cram[offset] = data;
		set_scroll_x(offset);
	}
}

// ---- main CPU ----
static uint8_t mcu_status_r(void)
{
	return (uint8_t)((taito68705_host_semaphore_r(&s_mcu) ? 0 : 1) | (taito68705_mcu_semaphore_r(&s_mcu) ? 2 : 0));
}

static uint8_t main_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x8000) return a < s_main_size ? s_main_rom[a] : 0;
	if (a >= 0xc000) return s_pixram[s_pixram_sel][a - 0xc000];
	if (a >= s_bank_base && a < s_bank_base + 0x2000) {
		uint32_t o = 0x10000u + (uint32_t)s_bank * 0x2000u + (uint32_t)(a - s_bank_base);
		return o < s_main_size ? s_main_rom[o] : 0;
	}
	if (a >= s_ram_base && a < s_ram_base + 0x800) return s_work[a - s_ram_base];
	if (a >= s_vid_base && a < s_vid_base + 0x900) {
		int o = a - s_vid_base;
		if (o < 0x800) return s_vram[o];
		o -= 0x800;
		if (o < 0x40) return s_vctrl[o];
		if (o < 0x80) return s_spr[o - 0x40];
		if (o < 0xc0) return s_cram[o - 0x80];
		if (o >= s_spr2_off) return s_spr2[o - s_spr2_off];
		return 0;
	}
	if (a >= s_io_base && a < s_io_base + 0x10) {
		switch (a - s_io_base) {
		case 0x0: return s_has_mcu ? taito68705_data_r(&s_mcu) : 0;
		case 0x1: return s_has_mcu ? mcu_status_r() : 0;
		case 0x3: return (uint8_t)((s_shift_data >> s_shift_count) & 0xff);   // mb14241 shift_result_r
		case 0x4: s_latch2_pending = 0; return s_latch2;
		case 0x5: return (uint8_t)((s_latch2_pending ? 2 : 0) | 0xfd);   // snd_flag_r
		case 0x8: return port8(s_p_dsw[2]);
		case 0x9: return port8(s_p_p1);
		case 0xa: return port8(s_p_system);
		case 0xb: return port8(s_p_p2);
		case 0xc: return port8(s_p_dsw[0]);
		case 0xd: return port8(s_p_dsw[1]);
		}
	}
	return 0;
}

static void main_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a < 0x8000) return;
	if (a >= 0xc000) {   // fortyl_pixram_w
		uint32_t o = a - 0xc000u;
		s_pixram[s_pixram_sel][o] = v;
		plot_pix((int)(o & 0x1fff));
		return;
	}
	if (a >= s_ram_base && a < s_ram_base + 0x800) { s_work[a - s_ram_base] = v; return; }
	if (a >= s_vid_base && a < s_vid_base + 0x900) {
		int o = a - s_vid_base;
		if (o < 0x800) { s_vram[o] = v; return; }
		o -= 0x800;
		if (o < 0x40) s_vctrl[o] = v;
		else if (o < 0x80) s_spr[o - 0x40] = v;
		else if (o < 0xc0) colorram_w(o - 0x80, v);
		else if (o >= s_spr2_off) s_spr2[o - s_spr2_off] = v;
		return;
	}
	if (a >= s_io_base && a < s_io_base + 0x10) {
		switch (a - s_io_base) {
		case 0x0: if (s_has_mcu) taito68705_data_w(&s_mcu, v); return;
		case 0x1: s_shift_count = (uint8_t)(~v & 7); return;   // mb14241 shift_count_w
		case 0x2: s_bank = v & 1; return;   // bank_select_w
		case 0x3: s_shift_data = (uint16_t)((s_shift_data >> 8) | ((uint16_t)v << 7)); return;   // shift_data_w
		case 0x4: s_latch = v; s_latch_pending = 1; update_nmi(); return;
		case 0xc: pixram_sel_w(v); return;
		}
	}
}

// ---- sound CPU ----
static uint8_t audio_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0xc000) return a < s_audio_size ? s_audio_rom[a] : 0;
	if (a < 0xc800) return s_audio_ram[a & 0x7ff];
	if (a == 0xd800) { s_latch_pending = 0; update_nmi(); return s_latch; }
	if (a >= 0xe000 && a < 0xf000) return a < s_audio_size ? s_audio_rom[a] : 0;
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
		s_snd_ctrl[0] = v;
		for (int i = 0; i < 4; i++) s_msm.gain[i] = s_ta[v >> 4];
		return;
	case 0xce00:   // sound_control_1_w: group 2
		snd_sync();
		s_snd_ctrl[1] = v;
		for (int i = 0; i < 4; i++) s_msm.gain[i + 4] = s_ta[v >> 4];
		return;
	case 0xd800: s_latch2 = v; s_latch2_pending = 1; return;
	case 0xda00: s_nmi_in1 = 1; update_nmi(); return;
	case 0xdc00: s_nmi_in1 = 0; update_nmi(); return;
	case 0xde00:   // DAC_8BIT_R2R, route 0.2
		snd_sync();
		s_dac_out = (int32_t)(32767.0 * 0.2 * (2.0 * v / 255.0 - 1.0));
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
static void decode_gfx(uint32_t size1, uint32_t size2)
{
	static const int xo[16] = { 3, 2, 1, 0, 11, 10, 9, 8, 131, 130, 129, 128, 139, 138, 137, 136 };
	// char_layout (gfx2): 0x400 chars, planes 2*0x2000*8 + 0 / + 4, 0, 4
	const uint32_t ch = 2 * 0x2000 * 8;
	const uint32_t cplanes[4] = { ch, ch + 4, 0, 4 };
	for (uint32_t t = 0; t < s_ntiles; t++)
		for (int y = 0; y < 8; y++)
			for (int x = 0; x < 8; x++) {
				int v = 0;
				for (int p = 0; p < 4; p++) {
					uint32_t b = t * 128 + cplanes[p] + (uint32_t)xo[x] + (uint32_t)y * 16;
					v = v << 1 | ((b >> 3) < size2 ? (s_gfx2[b >> 3] >> (7 - (b & 7))) & 1 : 0);
				}
				s_tiles[t * 64 + y * 8 + x] = (uint8_t)v;
			}
	// sprite_layout (gfx1): RGN_FRAC(1,2)
	const uint32_t half = size1 / 2 * 8;
	const uint32_t splanes[4] = { half, half + 4, 0, 4 };
	for (uint32_t t = 0; t < s_nsprites; t++)
		for (int y = 0; y < 16; y++)
			for (int x = 0; x < 16; x++) {
				int v = 0;
				uint32_t yo = (uint32_t)(y < 8 ? y * 16 : 256 + (y - 8) * 16);
				for (int p = 0; p < 4; p++) {
					uint32_t b = t * 512 + splanes[p] + (uint32_t)xo[x] + yo;
					v = v << 1 | ((s_gfx1[b >> 3] >> (7 - (b & 7))) & 1);
				}
				s_sprites[t * 256 + y * 16 + x] = (uint8_t)v;
			}
}

static void draw_sprite_list(const uint8_t *spr, int len)
{
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	const int fl = s_snap_flip;
	for (int offs = 0; offs < len; offs += 4) {
		int sx = spr[offs + 3];
		int sy = spr[offs + 0] + 1;
		if (fl)
			sx = 240 - sx;
		else
			sy = 242 - sy;
		uint32_t code = (uint32_t)((spr[offs + 1] & 0x3f) + ((spr[offs + 2] & 0x18) << 3));
		int flipx = ((spr[offs + 1] & 0x40) >> 6) ^ fl;
		int flipy = ((spr[offs + 1] & 0x80) >> 7) ^ fl;
		int color = (spr[offs + 2] & 0x07) + 0x08;
		if (spr[offs + 2] & 0xe0) {
			s_rng = s_rng * 1103515245u + 12345u;
			color = (int)(s_rng >> 16) & 0xf;
		}
		const uint8_t *g = s_sprites + (rbx_wrap(code, s_nsprites)) * 256;
		const int base = color * 16;
		// drawn at sx + m_xoffset (128) on the 512-wide screen, visible area starts at 128
		for (int ty = 0; ty < 16; ty++) {
			int y = sy + ty;
			if (y < VIS_Y0 || y > VIS_Y1) continue;
			const uint8_t *r = g + (flipy ? 15 - ty : ty) * 16;
			uint16_t *d = bm + (y - VIS_Y0) * pitch;
			for (int tx = 0; tx < 16; tx++) {
				int x = sx + tx;
				if (x < 0 || x >= SCREEN_W) continue;
				uint8_t p = r[flipx ? 15 - tx : tx];
				if (p)
					d[x] = (uint16_t)(base + p);
			}
		}
	}
}

static void ft_render(void)
{
	memcpy(rbx_video.palette, s_palette, sizeof(s_palette));
	rbx_video.blank = s_snap_disable;
	if (s_snap_disable)
		return;
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	const int fl = s_snap_flip;
	// draw_pixram: copybitmap with flipx / flipy = flipscreen ^ 1 at x offset 128
	const int pf = fl ^ 1;
	uint16_t pixpal[8];
	for (int i = 0; i < 8; i++)
		pixpal[i] = (uint16_t)(s_pix_color[i & 3] | ((i & 4) ? 0x200 : 0));
	for (int oy = 0; oy < VIS_H; oy++) {
		int y = oy + VIS_Y0;
		uint16_t *d = bm + oy * pitch;
		if (pf) {
			const uint8_t *s = s_snap_pix + (255 - y) * 256 + 255;
			for (int x = 0; x < SCREEN_W; x++)
				d[x] = pixpal[s[-x] & 7];
		} else {
			const uint8_t *s = s_snap_pix + y * 256;
			for (int x = 0; x < SCREEN_W; x++)
				d[x] = pixpal[s[x] & 7];
		}
	}
	// tilemap: 512 x 256, 32 scroll rows, scrolldy (-vc1 + 1, -vc1 - 1), pen 0 transparent
	const int cb = s_snap_color_bank ? 0x20 : 0;
	for (int oy = 0; oy < VIS_H; oy++) {
		int y = oy + VIS_Y0;
		uint16_t *d = bm + oy * pitch;
		int sy, lrow;
		if (!fl) {
			sy = (y - (1 - s_snap_vc1)) & 0xff;
			lrow = sy >> 3;
		} else {
			sy = (y - (s_snap_vc1 + 1)) & 0xff;
			lrow = 31 - (sy >> 3);
		}
		int ly = fl ? 255 - sy : sy;
		int rs = s_snap_scrollx[lrow];
		int attrib = s_snap_cram[lrow * 2];
		int hb = (attrib & 0x40) << 3, lb = (attrib & 0x18) << 3;
		int base = ((attrib & 7) | cb) * 16;
		const uint8_t *vrow = s_snap_vram + lrow * 64;
		for (int ox = 0; ox < SCREEN_W; ox++) {
			int x = ox + VIS_X0;
			int lx = fl ? 511 - ((x - rs) & 511) : (x + rs) & 511;
			int code = vrow[lx >> 3];
			if ((attrib & 0x20) && code >= 0xc0)
				code = (code & 0x3f) | lb | 0x100;
			code |= hb;
			uint8_t p = s_tiles[(rbx_wrap((uint32_t)code, s_ntiles)) * 64 + (ly & 7) * 8 + (lx & 7)];
			if (p)
				d[ox] = (uint16_t)(base + p);
		}
	}
	draw_sprite_list(s_snap_spr, 0x40);
	draw_sprite_list(s_snap_spr2, s_spr2_len);
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	if (s_pix_redraw) {   // draw_pixram: replot the selected bitmap
		s_pix_redraw = 0;
		for (int offs = 0; offs < 0x2000; offs++)
			plot_pix(offs);
	}
	memcpy(s_snap_pix, s_pixbmp[s_pixram_sel ? 0 : 1], sizeof(s_snap_pix));   // sel 1 shows bitmap 1, sel 0 bitmap 2
	memcpy(s_snap_vram, s_vram, sizeof(s_vram));
	memcpy(s_snap_cram, s_cram, sizeof(s_cram));
	memcpy(s_snap_spr, s_spr, sizeof(s_spr));
	memcpy(s_snap_spr2, s_spr2, sizeof(s_spr2));
	memcpy(s_snap_scrollx, s_scrollx, sizeof(s_scrollx));
	s_snap_flip = s_flip;
	s_snap_disable = s_screen_disable;
	s_snap_vc1 = s_vctrl[1];
	s_snap_color_bank = s_color_bank;
	rbx_video_frame_done();
}

// ---- init / frame ----
static int kind_of(const char *m)
{
	if (!strcmp(m, "_40love")) return FT_40LOVE;
	if (!strcmp(m, "_40lovebl")) return FT_40LOVEBL;
	if (!strcmp(m, "undoukai")) return FT_UNDOUKAI;
	return -1;
}

static void ft_reset(void)
{
	// (first: the sound syncs read them, from a CPU write or a port callback of a chip reset below, before a frame
	// sets them; the last game's values made a render of billions of clocks, the next game of the board hung)
	s_slice_start = s_slice_len = s_cpu_start = s_cpu_len = 0;
	memset(s_work, 0, sizeof(s_work));
	memset(s_vram, 0, sizeof(s_vram));
	memset(s_vctrl, 0, sizeof(s_vctrl));
	memset(s_spr, 0, sizeof(s_spr));
	memset(s_cram, 0, sizeof(s_cram));
	memset(s_spr2, 0, sizeof(s_spr2));
	memset(s_audio_ram, 0, sizeof(s_audio_ram));
	memset(s_pixram, 0, sizeof(s_pixram));
	memset(s_pixbmp, 0, sizeof(s_pixbmp));
	memset(s_scrollx, 0, sizeof(s_scrollx));
	memset(&s_main, 0, sizeof(s_main));
	memset(&s_audio, 0, sizeof(s_audio));
	zcpu_init(&s_main, main_read, main_write);
	zcpu_init(&s_audio, audio_read, audio_write);
	s_pixram_sel = s_screen_disable = s_flip = s_pix_redraw = s_bank = 0;
	s_color_bank = 0;   // machine_reset
	s_shift_data = 0;
	s_shift_count = 0;
	s_latch = s_latch2 = 0;
	s_latch_pending = s_latch2_pending = s_nmi_in1 = s_nmi_line = 0;
	if (s_has_mcu)
		taito68705_reset(&s_mcu);
	ay8910_reset(&s_ay);
	msm5232_reset(&s_msm);
	for (int i = 0; i < 8; i++)
		s_msm.gain[i] = 256;
	s_ay_vol = 256;
	memset(s_snd_ctrl, 0, sizeof(s_snd_ctrl));
	s_dac_out = (int32_t)(32767.0 * 0.2 * -1.0);
	s_audio_irq_acc = 0;
	s_frac_main = s_frac_audio = s_frac_mcu = s_frac_snd = 0;
	s_snd_done = 0;
	s_mix_n = 0;
}

static void ft_exit(void)
{
	free(s_tiles);
	free(s_sprites);
	s_tiles = s_sprites = NULL;
	free(s_mix);
	free(s_tmp);
	s_mix = s_tmp = NULL;
	free(s_pixbmp_hp);
	s_pixbmp_hp = NULL;
	free(s_pixram_hp);
	s_pixram_hp = NULL;
}

static int ft_init(const rbx_game *g)
{
	if (!s_pixbmp_hp && !(s_pixbmp_hp = calloc(1, sizeof(*s_pixbmp_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	if (!s_pixram_hp && !(s_pixram_hp = calloc(1, sizeof(*s_pixram_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	s_kind = kind_of(g->machine);
	if (s_kind < 0) {
		rbx_fail("%s: machine %s is not emulated", g->set, g->machine);
		return -1;
	}
	s_has_mcu = s_kind != FT_40LOVEBL;
	if (s_kind == FT_UNDOUKAI) {
		s_bank_base = 0x8000; s_ram_base = 0xa000; s_io_base = 0xa800; s_vid_base = 0xb000;
		s_spr2_off = 0xe0; s_spr2_len = 0x20;
	} else {
		s_ram_base = 0x8000; s_io_base = 0x8800; s_vid_base = 0x9000; s_bank_base = 0xa000;
		s_spr2_off = 0xc0; s_spr2_len = 0x40;
	}
	uint32_t size1, size2, psize;
	s_main_rom = rbx_region("maincpu", &s_main_size);
	s_audio_rom = rbx_region("audiocpu", &s_audio_size);
	s_gfx1 = rbx_region("gfx1", &size1);
	s_gfx2 = rbx_region("gfx2", &size2);
	s_proms = rbx_region("proms", &psize);
	if (!s_main_rom || !s_audio_rom || !s_gfx1 || !s_gfx2 || !s_proms || size1 < 0x8000 || size2 < 0x8000 || psize < 0xc00) {
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
	s_p_dsw[0] = rbx_port_find("DSW1");
	s_p_dsw[1] = rbx_port_find("DSW2");
	s_p_dsw[2] = rbx_port_find("DSW3");
	s_p_system = rbx_port_find("SYSTEM");
	s_p_p1 = rbx_port_find("P1");
	s_p_p2 = rbx_port_find("P2");
	// PALETTE RGB_444_PROMS, 1024 entries
	for (int i = 0; i < 1024; i++) {
		int r = s_proms[i] & 15, gg = s_proms[i + 0x400] & 15, b = s_proms[i + 0x800] & 15;
		s_palette[i] = (uint32_t)(r * 17 << 16 | gg * 17 << 8 | b * 17);
	}
	s_ntiles = 0x400;
	s_nsprites = size1 / 2 / 64;
	s_tiles = (uint8_t *)malloc((size_t)s_ntiles * 64);
	s_sprites = (uint8_t *)malloc((size_t)s_nsprites * 256);
	if (!s_tiles || !s_sprites) {
		ft_exit();
		return -1;
	}
	decode_gfx(size1, size2);
	ta7630_table(s_ta);
	ay8910_init(&s_ay, AY8910_TYPE_YM2149, SND_CLOCK, AY8910_LEGACY_OUTPUT, 26, 4, 1);   // route 0.1
	s_ay.port_a_write = ay_pa_w;
	s_ay.port_b_write = ay_pb_w;
	static const double caps[8] = { 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6 };
	msm5232_init(&s_msm, SND_CLOCK, caps, 2);
	uint32_t rate = ay8910_rate(&s_ay);
	s_mix_cap = (int)(rate / 50) + 64;
	s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	s_tmp = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	if (!s_mix || !s_tmp || rbx_audio_setup(rate) || rbx_video_setup(SCREEN_W, VIS_H, 1024)) {
		ft_exit();
		return -1;
	}
	ft_reset();
	return 0;
}

static void ft_frame(void)
{
	for (int i = 0; i < LINES; i++) {
		int line = (VIS_Y1 + 1 + i) % LINES;
		if (line == VIS_Y1 + 1) {
			vblank_capture();
			s_main.irq = 1;   // irq0_line_hold
		}
		int mc = rbx_line_cycles(&s_frac_main, CPU_CLOCK, RATE_MHZ, LINES);
		int ac = rbx_line_cycles(&s_frac_audio, CPU_CLOCK, RATE_MHZ, LINES);
		int uc = rbx_line_cycles(&s_frac_mcu, MCU_CLOCK, RATE_MHZ, LINES);
		int sc = rbx_line_cycles(&s_frac_snd, SND_CLOCK, RATE_MHZ, LINES);
		zcpu_run(&s_main, mc);
		s_slice_start = s_snd_done;
		s_slice_len = (uint32_t)sc;
		s_cpu_start = s_audio.z.cyc;
		s_cpu_len = ac;
		s_audio_irq_acc += (uint32_t)ac * AUDIO_IRQ_HZ;
		if (s_audio_irq_acc >= CPU_CLOCK) {
			s_audio_irq_acc -= CPU_CLOCK;
			s_audio.irq = 1;
		}
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

static int ft_state(void *ctx, int load)
{
	int rc = 0;
	rc |= zcpu_state(ctx, "main", &s_main, load);
	rc |= zcpu_state(ctx, "audio", &s_audio, load);
	rc |= rbx_state_io(ctx, "work", s_work, sizeof(s_work));
	rc |= rbx_state_io(ctx, "vram", s_vram, sizeof(s_vram));
	rc |= rbx_state_io(ctx, "vctrl", s_vctrl, sizeof(s_vctrl));
	rc |= rbx_state_io(ctx, "spr", s_spr, sizeof(s_spr));
	rc |= rbx_state_io(ctx, "cram", s_cram, sizeof(s_cram));
	rc |= rbx_state_io(ctx, "spr2", s_spr2, sizeof(s_spr2));
	rc |= rbx_state_io(ctx, "aram", s_audio_ram, sizeof(s_audio_ram));
	rc |= rbx_state_io(ctx, "pixram", s_pixram, sizeof(s_pixram));
	rc |= rbx_state_io(ctx, "pixbmp", s_pixbmp, sizeof(s_pixbmp));
	rc |= rbx_state_io(ctx, "scrollx", s_scrollx, sizeof(s_scrollx));
	if (s_has_mcu) {
		rc |= rbx_state_io(ctx, "tmcu", &s_mcu.st, sizeof(s_mcu.st));
		rc |= rbx_state_io(ctx, "tmcu6805", &s_mcu.mcu.st, sizeof(s_mcu.mcu.st));
	}
	rc |= rbx_state_io(ctx, "ay", &s_ay, ay8910_state_size());
	rc |= rbx_state_io(ctx, "msm", &s_msm, msm5232_state_size());
	rc |= rbx_state_io(ctx, "sndctrl", s_snd_ctrl, sizeof(s_snd_ctrl));
	int32_t misc[20] = { s_pixram_sel, s_color_bank, s_screen_disable, s_flip, s_pix_redraw, s_bank, s_shift_data,
		s_shift_count, s_latch, s_latch2, s_latch_pending, s_latch2_pending, s_nmi_in1, s_nmi_line, s_ay_vol, s_dac_out,
		(int32_t)s_audio_irq_acc, 0, 0, 0 };
	rc |= rbx_state_io(ctx, "misc", misc, sizeof(misc));
	if (rc)
		return -1;
	if (load) {
		s_pixram_sel = misc[0]; s_color_bank = misc[1]; s_screen_disable = misc[2]; s_flip = misc[3];
		s_pix_redraw = misc[4]; s_bank = misc[5]; s_shift_data = (uint16_t)misc[6]; s_shift_count = (uint8_t)misc[7];
		s_latch = (uint8_t)misc[8]; s_latch2 = (uint8_t)misc[9]; s_latch_pending = misc[10];
		s_latch2_pending = misc[11]; s_nmi_in1 = misc[12]; s_nmi_line = misc[13]; s_ay_vol = misc[14];
		s_dac_out = misc[15]; s_audio_irq_acc = (uint32_t)misc[16];
	}
	return 0;
}

static int ft_save(void *ctx) { return ft_state(ctx, 0); }
static int ft_load(void *ctx) { return ft_state(ctx, 1); }
static int ft_supported(const char *machine) { return kind_of(machine) >= 0; }

const rbx_driver rbx_drv_fortylove = { "Forty-Love", ft_init, ft_exit, ft_reset, ft_frame, ft_save, ft_load, ft_supported, ft_render };
