/*  The Legend of Kage, ported from MAME src/mame/taito/lkage.cpp (BSD-3-Clause, copyright-holders: Phil Stroffolino).
    Memory maps, the three tilemaps with their layer priorities, the sprites, the Taito 68705 MCU interface
    (machine/taito68705), the bootleg fake MCU and both sound boards follow MAME:
    lkage: Z80 + 2 x YM2203 (rbx_ym); lkagem: Z80 + 2 x YM2149 + MSM5232 with the TA7630 volume control.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/z80/z80.h"
#include "../../machine/taito68705.h"
#include "../../sound/rbx_ym.h"
#include "../../sound/ay8910.h"
#include "../../sound/msm5232.h"
#include "../../sound/ta7630.h"

#include <stdlib.h>
#include <string.h>

enum { LK_MCU, LK_MSM, LK_BOOT };

// MAME: 60 Hz, 256x256, visible (16, 16) - (255, 239)
#define LINES 256
#define VIS_X0 16
#define VIS_Y0 16
#define VIS_Y1 239
#define SCREEN_W 240
#define MAIN_CLOCK 6000000
#define AUDIO_CLOCK 4000000
#define MCU_CLOCK 750000      // 3 MHz / 4
#define SND_CLOCK 2000000     // YM2149 / MSM5232 input clock (lkagem)
#define RATE_MHZ 60000
#define SLICES 2

typedef struct {
	z80 z;
	int irq;
} zcpu;

static int s_kind;
static uint8_t *s_main_rom, *s_audio_rom, *s_data, *s_gfx;
static uint32_t s_main_size, s_audio_size, s_data_size, s_gfx_size;
static zcpu s_main, s_audio;
static uint8_t s_ram[0x800], s_pal[0x800], s_vreg[4], s_scroll[6], s_spr[0x60], s_vram[0xc00], s_audio_ram[0x800];
static uint8_t s_soundlatch, s_exrom_offs[2];
static int s_latch_pending, s_nmi_enable, s_nmi_line;
static uint8_t s_mcu_val, s_mcu_ready;   // bootleg fake MCU
static taito68705_t s_mcu;
// sound
static int s_ym_irq;
static ay8910_t s_ay[2];
static msm5232_t s_msm;
static int s_ta[16], s_ay_vol[2];
static uint32_t s_audio_irq_count;
static int32_t *s_mix, *s_tmp;
static int s_mix_cap, s_mix_n;
static uint32_t s_snd_done, s_slice_start, s_slice_len;   // in sound clocks (YM / AY input clock) of this frame
static unsigned long s_cpu_start;
static int s_cpu_len;
static uint64_t s_frac_main, s_frac_audio, s_frac_mcu, s_frac_snd;
// video
static uint8_t *s_tiles, *s_sprites;   // decoded 8x8 (2048) and 16x16 (512), one byte per pixel
static uint8_t s_snap_pal[0x800], s_snap_vreg[4], s_snap_scroll[6], s_snap_spr[0x60], s_snap_vram[0xc00];
static int s_sprite_dx[2];
// ports
static int s_p_dsw[3], s_p_system, s_p_p1, s_p_p2;

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
	if (s_kind != LK_MSM) {
		int n = rbx_ym_advance(0, clk, o, room);
		rbx_ym_advance(1, clk, o, room);
		s_mix_n += n;
	} else {
		// TA7630 device volume on each YM2149, mixed through a scratch buffer
		int n = 0;
		for (int i = 0; i < 2; i++) {
			memset(s_tmp, 0, sizeof(int32_t) * (size_t)(ay8910_out_max(&s_ay[i], clk) < room ? ay8910_out_max(&s_ay[i], clk) : room));
			n = ay8910_render(&s_ay[i], clk, s_tmp, room);
			for (int k = 0; k < n; k++)
				o[k] += s_tmp[k] * s_ay_vol[i] / 256;
		}
		msm5232_render(&s_msm, clk, o, room);
		s_mix_n += n;
	}
	s_snd_done = pos;
}

static void snd_sync(void)
{
	uint32_t done = (uint32_t)(s_audio.z.cyc - s_cpu_start);
	if ((int)done > s_cpu_len)
		done = (uint32_t)s_cpu_len;
	snd_run_to(s_slice_start + (s_cpu_len ? (uint32_t)((uint64_t)s_slice_len * done / (uint32_t)s_cpu_len) : 0));
}

static void ym_irq(int state) { s_ym_irq = state; }

// sound NMI: INPUT_MERGER_ALL_HIGH of (latch pending, NMI enable)
static void update_nmi(void)
{
	int line = s_latch_pending && s_nmi_enable;
	if (line && !s_nmi_line)
		s_audio.z.nmi_pending = 1;
	s_nmi_line = line;
}

static void ay_vol_w(void *u, uint8_t data)
{
	snd_sync();
	s_ay_vol[u == &s_ay[1]] = s_ta[data >> 4];
}

static void msm_volume_w(uint8_t data)
{
	snd_sync();
	for (int i = 0; i < 4; i++) {
		s_msm.gain[i] = s_ta[data & 0xf];
		s_msm.gain[i + 4] = s_ta[data >> 4];
	}
}

// ---- main CPU ----
static uint8_t mcu_status_r(void)
{
	return (uint8_t)((taito68705_host_semaphore_r(&s_mcu) ? 0 : 1) | (taito68705_mcu_semaphore_r(&s_mcu) ? 2 : 0));
}

static uint8_t fake_mcu_r(void)
{
	switch (s_mcu_val) {
	case 0x01: return (uint8_t)(s_mcu_val - 1);
	case 0x90: return (uint8_t)(s_mcu_val + 0x43);
	case 0xa6: return (uint8_t)(s_mcu_val + 0x27);
	case 0x34: return (uint8_t)(s_mcu_val + 0x7f);
	case 0x48: return (uint8_t)(s_mcu_val + 0xb7);
	}
	return s_mcu_val;
}

static uint8_t main_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0xe000) return a < s_main_size ? s_main_rom[a] : 0xff;
	if (a < 0xe800) return s_ram[a & 0x7ff];
	if (a < 0xf000) return s_pal[a & 0x7ff];
	if (a >= 0xf400) return s_vram[a - 0xf400];
	if (a < 0xf004) return s_vreg[a & 3];
	if (a >= 0xf0c0 && a < 0xf0c6) return s_scroll[a - 0xf0c0];
	switch (a) {
	case 0xf061: return 0xff;   // sound_status_r
	case 0xf080: return port8(s_p_dsw[0]);
	case 0xf081: return port8(s_p_dsw[1]);
	case 0xf082: return port8(s_p_dsw[2]);
	case 0xf083: return port8(s_p_system);
	case 0xf084: return port8(s_p_p1);
	case 0xf086: return port8(s_p_p2);
	case 0xf062:
		if (s_kind == LK_MCU) return taito68705_data_r(&s_mcu);
		if (s_kind == LK_BOOT) return fake_mcu_r();
		return 0;
	case 0xf087:
		if (s_kind == LK_MCU) return mcu_status_r();
		if (s_kind == LK_BOOT) return s_mcu_ready;
		return 0;
	case 0xf0a0:
		if (s_kind == LK_MSM && s_data) return s_data[rbx_wrap(((s_exrom_offs[1] & 0x3f) << 8 | s_exrom_offs[0]), s_data_size)];
		return 0;
	case 0xf0e0: return s_kind == LK_MSM ? taito68705_data_r(&s_mcu) : 0;
	case 0xf0e1: return s_kind == LK_MSM ? mcu_status_r() : 0;
	}
	return 0;
}

static void main_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a < 0xe000) return;
	if (a < 0xe800) { s_ram[a & 0x7ff] = v; return; }
	if (a < 0xf000) { s_pal[a & 0x7ff] = v; return; }
	if (a >= 0xf400) { s_vram[a - 0xf400] = v; return; }
	if (a < 0xf004) { s_vreg[a & 3] = v; return; }
	if (a >= 0xf0c0 && a < 0xf0c6) { s_scroll[a - 0xf0c0] = v; return; }
	if (a >= 0xf100 && a < 0xf160) {
		if (s_kind == LK_MSM && a >= 0xf140) return;   // (unmapped on lkagem)
		s_spr[a - 0xf100] = v;
		return;
	}
	if (a >= 0xf160 && a < 0xf180) {
		if (s_kind == LK_MSM) s_spr[(a - 0xf160) | 0x40] = v;
		return;
	}
	switch (a) {
	case 0xf060: s_soundlatch = v; s_latch_pending = 1; update_nmi(); return;
	case 0xf062:
		if (s_kind == LK_MCU) taito68705_data_w(&s_mcu, v);
		else if (s_kind == LK_BOOT) s_mcu_val = v;
		return;
	case 0xf0a0: case 0xf0a1:
		if (s_kind == LK_MSM) s_exrom_offs[a & 1] = v;
		return;
	case 0xf0e0:
		if (s_kind == LK_MSM) taito68705_data_w(&s_mcu, v);
		return;
	}
}

static uint8_t main_in(z80 *z, uint8_t p)
{
	(void)p;
	uint16_t a = z->mem_ptr;   // full port address
	if (s_kind != LK_MSM && a >= 0x4000 && a < 0x8000 && s_data && (uint32_t)(a - 0x4000) < s_data_size)
		return s_data[a - 0x4000];
	return 0xff;
}

// ---- sound CPU ----
static uint8_t latch_r(void)
{
	s_latch_pending = 0;
	update_nmi();
	return s_soundlatch;
}

static uint8_t audio_read(void *u, uint16_t a)
{
	(void)u;
	if (s_kind != LK_MSM) {
		if (a < 0x8000) return a < s_audio_size ? s_audio_rom[a] : 0xff;
		if (a < 0x8800) return s_audio_ram[a & 0x7ff];
		if (a == 0x9000 || a == 0x9001) { snd_sync(); return rbx_ym_read(0, a & 1); }
		if (a == 0xa000 || a == 0xa001) { snd_sync(); return rbx_ym_read(1, a & 1); }
		if (a == 0xb000) return latch_r();
		if (a >= 0xe000 && a < 0xf000) return a < s_audio_size ? s_audio_rom[a] : 0xff;
		return 0;
	}
	if (a < 0x4000) return a < s_audio_size ? s_audio_rom[a] : 0xff;
	if (a < 0x4800) return s_audio_ram[a & 0x7ff];
	if (a == 0xc000) return latch_r();
	if (a >= 0xe000 && a < 0xf000) return a < s_audio_size ? s_audio_rom[a] : 0xff;
	return 0;
}

static void audio_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (s_kind != LK_MSM) {
		if (a >= 0x8000 && a < 0x8800) { s_audio_ram[a & 0x7ff] = v; return; }
		if (a == 0x9000 || a == 0x9001) { snd_sync(); rbx_ym_write(0, a & 1, v); return; }
		if (a == 0xa000 || a == 0xa001) { snd_sync(); rbx_ym_write(1, a & 1, v); return; }
		if (a == 0xb001) { s_nmi_enable = 1; update_nmi(); return; }
		if (a == 0xb002) { s_nmi_enable = 0; update_nmi(); return; }
		return;
	}
	if (a >= 0x4000 && a < 0x4800) { s_audio_ram[a & 0x7ff] = v; return; }
	if (a >= 0x8000 && a < 0x8004) { snd_sync(); ay8910_write(&s_ay[(a >> 1) & 1], a & 1, v); return; }
	if (a >= 0x8010 && a <= 0x801d) { snd_sync(); msm5232_write(&s_msm, a - 0x8010, v); return; }
	if (a == 0x8020) { msm_volume_w(v); return; }
	if (a == 0xc001) { s_nmi_enable = 1; update_nmi(); return; }
	if (a == 0xc002) { s_nmi_enable = 0; update_nmi(); return; }
}

static uint8_t no_in(z80 *z, uint8_t p) { (void)z; (void)p; return 0xff; }
static void no_out(z80 *z, uint8_t p, uint8_t v) { (void)z; (void)p; (void)v; }

static void zcpu_init(zcpu *c, uint8_t (*rd)(void *, uint16_t), void (*wr)(void *, uint16_t, uint8_t),
                      uint8_t (*in)(z80 *, uint8_t))
{
	unsigned long cyc = c->z.cyc;
	z80_init(&c->z);
	c->z.read_byte = rd;
	c->z.write_byte = wr;
	c->z.port_in = in;
	c->z.port_out = no_out;
	c->z.int_data = 0xff;
	c->z.cyc = cyc;
	c->irq = 0;
}

static void zcpu_run(zcpu *c, int cycles, int level)
{
	unsigned long end = c->z.cyc + (unsigned long)cycles;
	while ((long)(c->z.cyc - end) < 0) {
		c->z.int_pending = (level >= 0 ? level : c->irq) ? 1 : 0;
		z80_step(&c->z);
		if (level < 0 && c->irq && !c->z.int_pending)
			c->irq = 0;   // HOLD_LINE acknowledged
	}
}

// ---- video ----
static void decode_gfx(void)
{
	const uint32_t q = s_gfx_size / 4;
	// planes (MSB first): 1/4, 0/4, 3/4, 2/4
	const uint32_t pl[4] = { q, 0, 3 * q, 2 * q };
	for (uint32_t t = 0; t < 2048; t++)
		for (int y = 0; y < 8; y++) {
			uint32_t o = t * 8 + (uint32_t)y;
			for (int x = 0; x < 8; x++) {
				int v = 0;
				for (int p = 0; p < 4; p++)
					v = v << 1 | (o < q ? (s_gfx[pl[p] + o] >> x) & 1 : 0);
				s_tiles[t * 64 + y * 8 + x] = (uint8_t)v;
			}
		}
	for (uint32_t t = 0; t < 512; t++)
		for (int y = 0; y < 16; y++)
			for (int x = 0; x < 16; x++) {
				uint32_t o = t * 32 + (uint32_t)(y & 7) + ((y & 8) ? 16 : 0) + ((x & 8) ? 8 : 0);
				int v = 0;
				for (int p = 0; p < 4; p++)
					v = v << 1 | (o < q ? (s_gfx[pl[p] + o] >> (x & 7)) & 1 : 0);
				s_sprites[t * 256 + y * 16 + x] = (uint8_t)v;
			}
}

static uint8_t (*s_pri_hp)[SCREEN_W * (VIS_Y1 - VIS_Y0 + 1)];
#define s_pri (*s_pri_hp)

// a tilemap: dx / dxf = MAME set_scrolldx(dx, dx_flipped); opaque = draw pen 0 too; pcode ORed into the priority buffer
static void draw_tilemap(const uint8_t *vram, int bank_add, int pal, int sx, int sy, int dx, int dxf, int opaque, int pcode,
                         int fx, int fy)
{
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	// MAME effective scroll: not flipped dx - scroll, flipped 256 - 256 - (dxf - scroll); dy = 0
	const int ex = fx ? (sx - dxf) : (dx - sx);
	const int ey = fy ? sy : -sy;
	for (int y = VIS_Y0; y <= VIS_Y1; y++) {
		int ly = (y - ey) & 0xff;
		if (fy) ly = 255 - ly;
		uint16_t *d = bm + (y - VIS_Y0) * pitch;
		uint8_t *pr = s_pri + (y - VIS_Y0) * SCREEN_W;
		const uint8_t *row = vram + (ly >> 3) * 32;
		int last = -1;
		const uint8_t *trow = NULL;
		for (int x = VIS_X0; x < 256; x++) {
			int lx = (x - ex) & 0xff;
			if (fx) lx = 255 - lx;
			if ((lx >> 3) != last) {   // (a tile looked up once for its 8 pixels)
				last = lx >> 3;
				trow = s_tiles + ((row[lx >> 3] + bank_add) & 2047) * 64 + (ly & 7) * 8;
			}
			const uint8_t p = trow[lx & 7];
			if (p || opaque) {
				d[x - VIS_X0] = (uint16_t)(pal + p);
				pr[x - VIS_X0] |= (uint8_t)pcode;
			}
		}
	}
}

static void draw_sprites(int fx, int fy)
{
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	for (int s = 0; s < 0x60; s += 4) {
		const uint8_t *src = s_snap_spr + s;
		int attr = src[2];
		int color = (attr >> 4) & 7;
		int flipx = attr & 1, flipy = attr & 2;
		int height = (attr & 8) ? 2 : 1;
		int sx = src[0] + s_sprite_dx[0];
		int sy = 255 - 16 * height - src[1];
		int num = src[3] + ((attr & 4) << 6);
		uint32_t pmask = ((attr & 0x80) ? (0xf0 | 0xcc) : 0xf0) | 1u << 31;   // (the high bit is implicitly on: earlier sprites win)
		if (fx) { sx = 256 - src[0] + s_sprite_dx[1]; flipx = !flipx; }
		if (fy) { sy = 254 - 16 * height - sy; flipy = !flipy; }
		if (height == 2 && !flipy)
			num ^= 1;
		for (int yy = 0; yy < height; yy++) {
			const uint8_t *g = s_sprites + ((num ^ yy) & 511) * 256;
			int ox = sx & 0xff, oy = sy + 16 * yy;
			for (int ty = 0; ty < 16; ty++) {
				int y = oy + ty;
				if (y < VIS_Y0 || y > VIS_Y1) continue;
				const uint8_t *r = g + (flipy ? 15 - ty : ty) * 16;
				uint16_t *d = bm + (y - VIS_Y0) * pitch;
				uint8_t *pr = s_pri + (y - VIS_Y0) * SCREEN_W;
				for (int tx = 0; tx < 16; tx++) {
					int x = ox + tx;
					if (x < VIS_X0 || x > 255) continue;
					uint8_t p = r[flipx ? 15 - tx : tx];
					if (!p) continue;
					uint8_t *pp = pr + x - VIS_X0;
					if (!((1u << (*pp & 0x1f)) & pmask))
						d[x - VIS_X0] = (uint16_t)(color * 16 + p);
					*pp = 31;
				}
			}
		}
	}
}

static void lk_render(void)
{
	for (int i = 0; i < 1024; i++) {   // xRGB_444, little endian
		int c = s_snap_pal[i * 2] | s_snap_pal[i * 2 + 1] << 8;
		rbx_video.palette[i] = (uint32_t)(((c >> 8) & 15) * 17 << 16 | ((c >> 4) & 15) * 17 << 8 | (c & 15) * 17);
	}
	const uint8_t *vr = s_snap_vreg;
	const int fx = ~vr[2] & 1, fy = (~vr[2] & 2) != 0;
	const int bgpal = 0x300 + (vr[1] & 0xf0), fgpal = 0x200 + (vr[1] & 0xf0), txpal = 0x100 + (vr[1] & 0xf0);
	uint16_t *bm = rbx_video.bitmap;
	for (int y = 0; y < rbx_video.height; y++)
		for (int x = 0; x < SCREEN_W; x++)
			bm[y * rbx_video.pitch + x] = (uint16_t)bgpal;
	memset(s_pri, 0, sizeof(s_pri));
	if (vr[2] & 0x40)
		draw_tilemap(s_snap_vram + 0x800, 256 * ((vr[1] & 0x08) ? 5 : 1), bgpal, s_snap_scroll[4], s_snap_scroll[5], -5, -5 + 32, 1, 1, fx, fy);
	if (vr[2] & 0x20)
		draw_tilemap(s_snap_vram + 0x400, 256 * ((vr[0] & 0x04) ? 1 : 0), fgpal, s_snap_scroll[2], s_snap_scroll[3], -3, -3 + 28, 0,
		             (vr[1] & 2) ? 2 : 4, fx, fy);
	if (vr[2] & 0x10)
		draw_tilemap(s_snap_vram, 256 * ((vr[0] & 0x02) ? 4 : 0), txpal, s_snap_scroll[0], s_snap_scroll[1], -1, -1 + 24, 0, 4, fx, fy);
	if (vr[2] & 0x80)
		draw_sprites(fx, fy);
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap_pal, s_pal, sizeof(s_pal));
	memcpy(s_snap_vreg, s_vreg, sizeof(s_vreg));
	memcpy(s_snap_scroll, s_scroll, sizeof(s_scroll));
	memcpy(s_snap_spr, s_spr, sizeof(s_spr));
	memcpy(s_snap_vram, s_vram, sizeof(s_vram));
	rbx_video_frame_done();
}

// ---- init / frame ----
static int kind_of(const char *machine)
{
	if (!strcmp(machine, "lkage")) return LK_MCU;
	if (!strcmp(machine, "lkagem")) return LK_MSM;
	if (!strcmp(machine, "lkagebl")) return LK_BOOT;
	return -1;
}

static void lk_reset(void)
{
	// (first: the sound syncs read them, from a CPU write or a port callback of a chip reset below, before a frame
	// sets them; the last game's values made a render of billions of clocks, the next game of the board hung)
	s_slice_start = s_slice_len = s_cpu_start = s_cpu_len = 0;
	memset(s_ram, 0, sizeof(s_ram));
	memset(s_pal, 0, sizeof(s_pal));
	memset(s_vreg, 0, sizeof(s_vreg));
	memset(s_scroll, 0, sizeof(s_scroll));
	memset(s_spr, 0, sizeof(s_spr));
	memset(s_vram, 0xff, sizeof(s_vram));   // video_start
	memset(s_audio_ram, 0, sizeof(s_audio_ram));
	memset(&s_main, 0, sizeof(s_main));
	memset(&s_audio, 0, sizeof(s_audio));
	zcpu_init(&s_main, main_read, main_write, main_in);
	zcpu_init(&s_audio, audio_read, audio_write, no_in);
	s_soundlatch = 0;
	s_latch_pending = s_nmi_enable = s_nmi_line = 0;
	s_exrom_offs[0] = s_exrom_offs[1] = 0;
	s_mcu_ready = 3;
	s_mcu_val = 0;
	s_ym_irq = 0;
	s_audio_irq_count = 0;
	if (s_kind != LK_BOOT)
		taito68705_reset(&s_mcu);
	if (s_kind != LK_MSM) {
		rbx_ym_reset(0);
		rbx_ym_reset(1);
	} else {
		ay8910_reset(&s_ay[0]);
		ay8910_reset(&s_ay[1]);
		msm5232_reset(&s_msm);
		s_ay_vol[0] = s_ay_vol[1] = 256;
		for (int i = 0; i < 8; i++)
			s_msm.gain[i] = 256;   // route 1.0
	}
	s_frac_main = s_frac_audio = s_frac_mcu = s_frac_snd = 0;
	s_snd_done = 0;
	s_mix_n = 0;
}

static void lk_exit(void)
{
	free(s_tiles);
	free(s_sprites);
	s_tiles = s_sprites = NULL;
	free(s_mix);
	free(s_tmp);
	s_mix = s_tmp = NULL;
	rbx_ym_exit(0);
	rbx_ym_exit(1);
	free(s_pri_hp);
	s_pri_hp = NULL;
}

static int lk_init(const rbx_game *g)
{
	if (!s_pri_hp && !(s_pri_hp = calloc(1, sizeof(*s_pri_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	s_kind = kind_of(g->machine);
	if (s_kind < 0) {
		rbx_fail("%s: machine %s is not emulated", g->set, g->machine);
		return -1;
	}
	s_main_rom = rbx_region("maincpu", &s_main_size);
	s_audio_rom = rbx_region("audiocpu", &s_audio_size);
	s_data = rbx_region("data", &s_data_size);
	s_gfx = rbx_region("gfx", &s_gfx_size);
	if (!s_main_rom || !s_audio_rom || !s_gfx || s_gfx_size < 0x10000) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	if (s_kind != LK_BOOT) {
		uint32_t msize;
		uint8_t *mrom = rbx_region("bmcu:mcu", &msize);
		memset(&s_mcu, 0, sizeof(s_mcu));
		if (!mrom || msize < 0x800 || taito68705_init(&s_mcu, TAITO68705_STD, mrom, 0x800)) {
			rbx_fail("%s: MCU ROM missing", g->set);
			return -1;
		}
	}
	s_sprite_dx[0] = s_kind == LK_MSM ? 8 : -14;
	s_sprite_dx[1] = s_kind == LK_MSM ? -4 : -14;
	s_p_dsw[0] = rbx_port_find("DSW1");
	s_p_dsw[1] = rbx_port_find("DSW2");
	s_p_dsw[2] = rbx_port_find("DSW3");
	s_p_system = rbx_port_find("SYSTEM");
	s_p_p1 = rbx_port_find("P1");
	s_p_p2 = rbx_port_find("P2");
	s_tiles = (uint8_t *)malloc(2048 * 64);
	s_sprites = (uint8_t *)malloc(512 * 256);
	if (!s_tiles || !s_sprites) {
		lk_exit();
		return -1;
	}
	decode_gfx();
	uint32_t rate;
	if (s_kind != LK_MSM) {
		for (int i = 0; i < 2; i++) {
			rbx_ym_config yc;
			memset(&yc, 0, sizeof(yc));
			yc.type = RBX_YM2203;
			yc.clock = 4000000;
			yc.irq = i == 0 ? ym_irq : NULL;
			yc.gain[0] = yc.gain[1] = yc.gain[2] = 38;   // 0.15
			yc.gain[3] = 102;                            // 0.40
			// (the YMs' samples made on the worker: rbx_ym_async_take at the frame's end)
			if (rbx_ym_init(i, &yc) || rbx_ym_set_async(i, 1)) {
				rbx_fail("%s: sound init failed", g->set);
				lk_exit();
				return -1;
			}
		}
		rate = rbx_ym_rate(0);
	} else {
		ta7630_table(s_ta);
		ay8910_init(&s_ay[0], AY8910_TYPE_YM2149, SND_CLOCK, AY8910_LEGACY_OUTPUT, 51, 4, 1);   // route 0.20
		ay8910_init(&s_ay[1], AY8910_TYPE_YM2149, SND_CLOCK, AY8910_LEGACY_OUTPUT, 51, 4, 1);
		s_ay[0].port_a_write = ay_vol_w;
		s_ay[0].user = &s_ay[0];
		s_ay[1].port_a_write = ay_vol_w;
		s_ay[1].user = &s_ay[1];
		static const double caps[8] = { 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6 };
		msm5232_init(&s_msm, SND_CLOCK, caps, 2);
		rate = ay8910_rate(&s_ay[0]);   // = MSM5232 rate (2 MHz / 8 / 4 = 2 MHz / 16 / 2)
	}
	s_mix_cap = (int)(rate / 50) + 64;
	s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	s_tmp = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	if (!s_mix || !s_tmp || rbx_audio_setup(rate) || rbx_video_setup(SCREEN_W, VIS_Y1 - VIS_Y0 + 1, 1024)) {
		lk_exit();
		return -1;
	}
	lk_reset();
	return 0;
}

static void lk_frame(void)
{
	const uint32_t snd_clock = s_kind == LK_MSM ? SND_CLOCK : 4000000;
	// the frame starts at vblank (MAME updates the inputs there)
	for (int i = 0; i < LINES; i++) {
		int line = (VIS_Y1 + 1 + i) % LINES;
		if (line == VIS_Y1 + 1) {
			vblank_capture();
			s_main.irq = 1;   // irq0_line_hold
		}
		for (int s = 0; s < SLICES; s++) {
			int mc = rbx_line_cycles(&s_frac_main, MAIN_CLOCK, RATE_MHZ, LINES * SLICES);
			int ac = rbx_line_cycles(&s_frac_audio, AUDIO_CLOCK, RATE_MHZ, LINES * SLICES);
			int uc = rbx_line_cycles(&s_frac_mcu, MCU_CLOCK, RATE_MHZ, LINES * SLICES);
			int sc = rbx_line_cycles(&s_frac_snd, snd_clock, RATE_MHZ, LINES * SLICES);
			zcpu_run(&s_main, mc, -1);
			s_slice_start = s_snd_done;
			s_slice_len = (uint32_t)sc;
			s_cpu_start = s_audio.z.cyc;
			s_cpu_len = ac;
			if (s_kind == LK_MSM) {
				// periodic IRQ at 60 Hz (HOLD_LINE)
				s_audio_irq_count += (uint32_t)ac;
				if (s_audio_irq_count >= AUDIO_CLOCK / 60) {
					s_audio_irq_count -= AUDIO_CLOCK / 60;
					s_audio.irq = 1;
				}
				zcpu_run(&s_audio, ac, -1);
			} else
				zcpu_run(&s_audio, ac, s_ym_irq);
			if (s_kind != LK_BOOT)
				taito68705_execute(&s_mcu, uc);
			snd_run_to(s_slice_start + s_slice_len);
		}
	}
	if (s_kind != LK_MSM) {
		rbx_ym_async_take(0, s_mix, s_mix_n);
		rbx_ym_async_take(1, s_mix, s_mix_n);
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

static int lk_state(void *ctx, int load)
{
	int rc = 0;
	rc |= zcpu_state(ctx, "main", &s_main, load);
	rc |= zcpu_state(ctx, "audio", &s_audio, load);
	rc |= rbx_state_io(ctx, "ram", s_ram, sizeof(s_ram));
	rc |= rbx_state_io(ctx, "pal", s_pal, sizeof(s_pal));
	rc |= rbx_state_io(ctx, "vreg", s_vreg, sizeof(s_vreg));
	rc |= rbx_state_io(ctx, "scroll", s_scroll, sizeof(s_scroll));
	rc |= rbx_state_io(ctx, "spr", s_spr, sizeof(s_spr));
	rc |= rbx_state_io(ctx, "vram", s_vram, sizeof(s_vram));
	rc |= rbx_state_io(ctx, "aram", s_audio_ram, sizeof(s_audio_ram));
	if (s_kind != LK_BOOT) {
		rc |= rbx_state_io(ctx, "tmcu", &s_mcu.st, sizeof(s_mcu.st));
		rc |= rbx_state_io(ctx, "tmcu6805", &s_mcu.mcu.st, sizeof(s_mcu.mcu.st));
	}
	if (s_kind != LK_MSM) {
		rc |= rbx_ym_state(0, ctx, load);
		rc |= rbx_ym_state(1, ctx, load);
	} else {
		rc |= rbx_state_io(ctx, "ay0", &s_ay[0], ay8910_state_size());
		rc |= rbx_state_io(ctx, "ay1", &s_ay[1], ay8910_state_size());
		rc |= rbx_state_io(ctx, "msm", &s_msm, msm5232_state_size());
	}
	int32_t misc[12] = { s_soundlatch, s_exrom_offs[0], s_exrom_offs[1], s_latch_pending, s_nmi_enable, s_nmi_line,
		s_mcu_val, s_mcu_ready, s_ym_irq, s_ay_vol[0], s_ay_vol[1], (int32_t)s_audio_irq_count };
	rc |= rbx_state_io(ctx, "misc", misc, sizeof(misc));
	if (rc)
		return -1;
	if (load) {
		s_soundlatch = (uint8_t)misc[0]; s_exrom_offs[0] = (uint8_t)misc[1]; s_exrom_offs[1] = (uint8_t)misc[2];
		s_latch_pending = misc[3]; s_nmi_enable = misc[4]; s_nmi_line = misc[5];
		s_mcu_val = (uint8_t)misc[6]; s_mcu_ready = (uint8_t)misc[7]; s_ym_irq = misc[8];
		s_ay_vol[0] = misc[9]; s_ay_vol[1] = misc[10]; s_audio_irq_count = (uint32_t)misc[11];
	}
	return 0;
}

static int lk_save(void *ctx) { return lk_state(ctx, 0); }
static int lk_load(void *ctx) { return lk_state(ctx, 1); }
static int lk_supported(const char *machine) { return kind_of(machine) >= 0; }

const rbx_driver rbx_drv_lkage = { "The Legend of Kage", lk_init, lk_exit, lk_reset, lk_frame, lk_save, lk_load, lk_supported, lk_render };
