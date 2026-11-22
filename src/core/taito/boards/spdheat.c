/*  Super Dead Heat hardware, ported from MAME src/mame/taito/spdheat.cpp (BSD-3-Clause, copyright-holders:
    Philip Bennett). Memory maps, machine configuration, sound communication and the video follow MAME.
    68000 main CPU (irq1_line_hold at vblank), Z80 sound CPU (two YM2203 with merged IRQ, two YM2149, command
    latches), Z80 sub CPU driving an 8-bit R2R DAC (NMI pulsed by the sound CPU).
    Screens: the cabinet has four 256x224 monitors, each with its own text layer and palette, the sprites spread
    over two 512-pixel wide pairs. They are composed into one 512x448 bitmap as MAME's default layout view
    ("Quadruple (1P/2P side)"): screens 2 | 3 on the top row, 0 | 1 on the bottom row.
    Sound: as MAME (all chips to one mono speaker at 0.3; the PC010SA filter / volume ports are not emulated, MAME
    leaves them as no-ops too). The DAC is centered on 0x80 here (MAME keeps its DC offset).
    Interleave: one scanline for the 68000, four slices per scanline between the two Z80s (MAME: 600 Hz quantum).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../rbx_core.h"
#include "../../cpu/m68k/m68k.h"
#include "../../cpu/m68k_bus.h"
#include "../../cpu/z80/z80.h"
#include "../../sound/rbx_ym.h"
#include "../../sound/ay8910.h"

#include <stdlib.h>
#include <string.h>

#define LINES 256
#define VIS_Y0 16
#define VIS_Y1 239
#define VIS_H (VIS_Y1 - VIS_Y0 + 1)
#define SCREEN_W 256
#define MAIN_CLOCK 8000000
#define Z80_CLOCK 4000000
#define FM_CLOCK 2000000
#define SLICES 4

static uint8_t *s_rom;
static uint32_t s_rom_size;
static uint8_t *s_snd_rom, *s_sub_rom;
static uint32_t s_snd_rom_size, s_sub_rom_size;
// 68000 memory (host-order words)
static uint16_t (*s_ram_hp)[0x2000];       // 080000-083fff
#define s_ram (*s_ram_hp)
static uint16_t s_fg[4][0x400];      // 088000 / 088800 / 08c000 / 08c800
static uint16_t s_spr[0x400];        // 090000-0907ff
static uint16_t s_pal[4][0x400];     // 098000 / 099000 / 09a000 / 09b000
// sound side
static z80 s_snd, s_sub;
static uint8_t s_snd_ram[0x800], s_snd_d700, s_sub_ram[0x800];
static uint32_t s_sound_data[4], s_sound_status, s_sub_data, s_sub_status;
static int s_ym_irq[2];
static ay8910_t s_ay[2];
static int s_dac;
static uint64_t s_frac_main, s_frac_z80, s_frac_ym;
// mixing (in YM2203 input clocks)
static int32_t *s_mix_fm, *s_mix_ay;
static int s_mix_cap, s_nfm, s_nay;
static uint32_t s_snd_done, s_slice_start, s_slice_len;
static unsigned long s_cpu_start;
static int s_slice_cycles;
static z80 *s_cur;
// video
static uint8_t *s_chars, *s_sprites, *s_spr_used;
static struct {
	uint16_t fg[4][0x400], spr[0x400], pal[4][0x400];
} *s_snap;
// ports
static int s_p_handle[4], s_p_accel[4], s_p_in[8];   // IN0, IN1, SYSTEM, IN2, DSWA-D
static int s_handle[4], s_accel[4];

// ---- sound ----
static int s_fm_taken;   // slots at the front of s_mix_fm that have their YM2203 samples (async: given at the frame's end)

// The YM2149s' samples made on the rbx_aux2 worker (on the Vita, 2 x 500k native samples a second were half of the
// emulation thread's time): s_ay, the chips the Z80 reads, are given every write and make no samples (ay_count moves
// their sample clock the same way ay8910_render does); s_ay_w, given the same writes and clocks in the same order by
// a queue, make them; they are added to s_mix_ay at the frame's end (s_ay_taken). The sound is the same
enum { AYQ_ADV, AYQ_ADDR, AYQ_DATA };
enum { AYQ_SIZE = 8192, AYO_SIZE = 8192 };   // (powers of two)
static struct { uint16_t clocks; uint8_t kind, chip, v; } s_ayq[AYQ_SIZE];
static unsigned s_ayq_in, s_ayq_done;        // (emulation thread / worker)
static ay8910_t s_ay_w[2];
static int32_t (*s_ay_out_hp)[AYO_SIZE];
#define s_ay_out (*s_ay_out_hp)
static unsigned s_ay_out_in, s_ay_out_take;
static int s_ay_taken;   // slots at the front of s_mix_ay that have their YM2149 samples

static void ay_job(void)
{
	const unsigned end = __atomic_load_n(&s_ayq_in, __ATOMIC_ACQUIRE);
	unsigned i = s_ayq_done;
	for (; i != end; i++) {
		const unsigned k = i & (AYQ_SIZE - 1);
		switch (s_ayq[k].kind) {
		case AYQ_ADV: {
			int32_t tmp[128];   // (at most 4096 clocks a step: under 64 samples)
			const int max = ay8910_out_max(&s_ay_w[0], s_ayq[k].clocks);
			for (int j = 0; j < max; j++)
				tmp[j] = 0;
			const int m = ay8910_render(&s_ay_w[0], s_ayq[k].clocks, tmp, max);
			ay8910_render(&s_ay_w[1], s_ayq[k].clocks, tmp, max);
			for (int j = 0; j < m; j++)
				s_ay_out[(s_ay_out_in + (unsigned)j) & (AYO_SIZE - 1)] = tmp[j];
			__atomic_store_n(&s_ay_out_in, s_ay_out_in + (unsigned)m, __ATOMIC_RELEASE);
			break;
		}
		case AYQ_ADDR: ay8910_address_w(&s_ay_w[s_ayq[k].chip], s_ayq[k].v); break;
		case AYQ_DATA: ay8910_data_w(&s_ay_w[s_ayq[k].chip], s_ayq[k].v); break;
		}
	}
	__atomic_store_n(&s_ayq_done, i, __ATOMIC_RELEASE);
}

static void ay_push(uint8_t kind, int chip, uint8_t v, uint32_t clocks)
{
	if (s_ayq_in - __atomic_load_n(&s_ayq_done, __ATOMIC_ACQUIRE) >= AYQ_SIZE)
		rbx_aux2_sync();   // (full: let the worker catch up)
	const unsigned k = s_ayq_in & (AYQ_SIZE - 1);
	s_ayq[k].kind = kind;
	s_ayq[k].chip = (uint8_t)chip;
	s_ayq[k].v = v;
	s_ayq[k].clocks = (uint16_t)clocks;
	__atomic_store_n(&s_ayq_in, s_ayq_in + 1, __ATOMIC_RELEASE);
}

// ay8910_render's sample count without the samples (its clock arithmetic)
static int ay_count(ay8910_t *ay, uint32_t clocks)
{
	int n = 0;
	uint32_t total = ay->frac + clocks;
	while (total >= ay->clk_per_sample) {
		total -= ay->clk_per_sample;
		if (++ay->div_count < ay->divider)
			continue;
		ay->div_count = 0;
		n++;
	}
	ay->frac = total;
	return n;
}

static void ay_write(int chip, int data, uint8_t v)
{
	if (data)
		ay8910_data_w(&s_ay[chip], v);
	else
		ay8910_address_w(&s_ay[chip], v);
	ay_push(data ? AYQ_DATA : AYQ_ADDR, chip, v, 0);
}

// the worker's samples so far into s_mix_ay (s_ay_taken to s_nay)
static void ay_take(void)
{
	const unsigned want = (unsigned)(s_nay - s_ay_taken);
	if (__atomic_load_n(&s_ay_out_in, __ATOMIC_ACQUIRE) - s_ay_out_take < want)
		rbx_aux2_sync();
	for (unsigned i = 0; i < want; i++)
		s_mix_ay[s_ay_taken + (int)i] += s_ay_out[(s_ay_out_take + i) & (AYO_SIZE - 1)];
	s_ay_out_take += want;
	s_ay_taken = s_nay;
}

// the queue emptied and both copies the same as s_ay (reset, state load)
static void ay_restart(void)
{
	rbx_aux2_sync();
	s_ayq_in = s_ayq_done = 0;
	s_ay_out_in = s_ay_out_take = 0;
	s_ay_taken = 0;
	for (int i = 0; i < 2; i++)
		memcpy(&s_ay_w[i], &s_ay[i], sizeof(ay8910_t));
}

static void snd_run_to(uint32_t pos)
{
	if (pos <= s_snd_done)
		return;
	uint32_t d = pos - s_snd_done;
	int n = rbx_ym_advance(0, d, s_mix_fm + s_nfm, s_mix_cap - s_nfm);
	rbx_ym_advance(1, d, s_mix_fm + s_nfm, s_mix_cap - s_nfm);
	int32_t dac = (s_dac - 0x80) * 77;   // DAC_8BIT_R2R, route 0.3
	for (int i = 0; i < n; i++)
		s_mix_fm[s_nfm + i] += dac;
	s_nfm += n;
	for (uint32_t c = d * 2; c; ) {   // YM2149s at 4 MHz (the samples on the worker)
		const uint32_t k = c < 4096 ? c : 4096;
		s_nay += ay_count(&s_ay[0], k);
		s_ay[1].frac = s_ay[0].frac;
		s_ay[1].div_count = s_ay[0].div_count;
		ay_push(AYQ_ADV, 0, 0, k);
		c -= k;
	}
	rbx_aux2_kick();
	s_snd_done = pos;
}

static void snd_sync(void)
{
	if (!s_cur)
		return;
	uint32_t done = (uint32_t)(s_cur->cyc - s_cpu_start);
	if ((int)done > s_slice_cycles)
		done = (uint32_t)s_slice_cycles;
	snd_run_to(s_slice_start + (s_slice_cycles ? s_slice_len * done / (uint32_t)s_slice_cycles : 0));
}

static void update_snd_irq(void) { s_snd.int_pending = (s_ym_irq[0] || s_ym_irq[1]) ? 1 : 0; }
static void ym1_irq(int state) { s_ym_irq[0] = state; }
static void ym2_irq(int state) { s_ym_irq[1] = state; }

static uint8_t snd_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x8000) return a < s_snd_rom_size ? s_snd_rom[a] : 0;
	if (a >= 0xe000) return a < s_snd_rom_size ? s_snd_rom[a] : 0;   // diagnostic ROM (not dumped)
	if (a >= 0xc000 && a < 0xc800) return s_snd_ram[a - 0xc000];
	switch (a) {
	case 0xd000: case 0xd001: snd_sync(); return rbx_ym_read(0, a & 1);
	case 0xd100: case 0xd101: snd_sync(); return rbx_ym_read(1, a & 1);
	case 0xd201: snd_sync(); return ay8910_data_r(&s_ay[0]);
	case 0xd301: snd_sync(); return ay8910_data_r(&s_ay[1]);
	case 0xd600: return (uint8_t)(s_sound_status ^ 0xf);
	case 0xd700: return s_snd_d700;
	case 0xd800: case 0xd801: case 0xd802: case 0xd803:
		s_sound_status |= 1u << (a & 3);
		return (uint8_t)s_sound_data[a & 3];
	case 0xd807: return s_sub_status ? 0x80 : 0;
	}
	return 0;
}

static void snd_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0xc000 && a < 0xc800) { s_snd_ram[a - 0xc000] = v; return; }
	switch (a) {
	case 0xd000: case 0xd001: snd_sync(); rbx_ym_write(0, a & 1, v); return;
	case 0xd100: case 0xd101: snd_sync(); rbx_ym_write(1, a & 1, v); return;
	case 0xd200: snd_sync(); ay_write(0, 0, v); return;
	case 0xd201: snd_sync(); ay_write(0, 1, v); return;
	case 0xd300: snd_sync(); ay_write(1, 0, v); return;
	case 0xd301: snd_sync(); ay_write(1, 1, v); return;
	case 0xd700: s_snd_d700 = v; return;
	case 0xd800:   // sub_nmi_w
		s_sub_data = v;
		z80_gen_nmi(&s_sub);
		s_sub_status = 1;
		return;
	}   // (d400-d5e0: filter ports, no-ops in MAME)
}

static uint8_t sub_read(void *u, uint16_t a)
{
	(void)u;
	if (a >= 0xf800) return s_sub_ram[a - 0xf800];
	return a < s_sub_rom_size ? s_sub_rom[a] : 0;
}

static void sub_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0xf800) { s_sub_ram[a - 0xf800] = v; return; }
	if (a == 0x8000) {
		snd_sync();
		s_dac = v;
	}
}

static uint8_t sub_in(z80 *z, uint8_t p) { (void)z; return p == 0xff ? (uint8_t)s_sub_data : 0; }
static void sub_out(z80 *z, uint8_t p, uint8_t v) { (void)z; (void)v; if (p == 0xff) s_sub_status = 0; }
static uint8_t no_in(z80 *z, uint8_t p) { (void)z; (void)p; return 0; }
static void no_out(z80 *z, uint8_t p, uint8_t v) { (void)z; (void)p; (void)v; }

static void set_z80_handlers(void)
{
	s_snd.read_byte = snd_read;
	s_snd.write_byte = snd_write;
	s_snd.port_in = no_in;
	s_snd.port_out = no_out;
	s_snd.userdata = NULL;
	s_sub.read_byte = sub_read;
	s_sub.write_byte = sub_write;
	s_sub.port_in = sub_in;
	s_sub.port_out = sub_out;
	s_sub.userdata = NULL;
}

// ---- main CPU ----
static uint16_t main_read16(uint32_t a)
{
	if (a == 0x094000) return (uint16_t)s_sound_status;
	if (a >= 0x09c000 && a <= 0x09c01f) {
		int i = (int)((a - 0x09c000) >> 1);
		if (i < 8) {   // HANDLE / ACCEL 1-4
			int p = i >> 1;
			return (uint16_t)((i & 1) ? s_accel[p] : (s_handle[p] & 0xff));
		}
		int port = s_p_in[i - 8];
		return port >= 0 ? (uint16_t)rbx_port_read(port) : 0;
	}
	return 0;
}

static uint8_t main_read8(uint32_t a) { return (uint8_t)(main_read16(a & ~1u) >> ((a & 1) ? 0 : 8)); }

static void main_write16(uint32_t a, uint16_t v)
{
	if (a >= 0x094000 && a <= 0x094007) {
		int n = (int)((a - 0x094000) >> 1);
		s_sound_data[n] = v;
		s_sound_status &= ~(1u << n);
	}   // (09c020-09c027: no-ops)
}

static void main_write8(uint32_t a, uint8_t v) { main_write16(a & ~1u, (a & 1) ? v : (uint16_t)(v << 8)); }

// ---- video ----
static uint32_t pal_rgb(uint16_t w)   // xBGR_555
{
	int r = w & 31, g = (w >> 5) & 31, b = (w >> 10) & 31;
	r = (r << 3) | (r >> 2);
	g = (g << 3) | (g >> 2);
	b = (b << 3) | (b >> 2);
	return (uint32_t)(r << 16 | g << 8 | b);
}

static void draw_sprite16(uint16_t *bm, int pitch, int code, uint16_t base, int fx, int fy, int sx, int sy)
{
	code &= 0x3ff;
	if (!s_spr_used[code])
		return;
	const uint8_t *src = s_sprites + code * 256;
	for (int y = 0; y < 16; y++) {
		int dy = sy + y - VIS_Y0;
		if (dy < 0 || dy >= VIS_H)
			continue;
		const uint8_t *r = src + (fy ? 15 - y : y) * 16;
		uint16_t *d = bm + dy * pitch;
		for (int x = 0; x < 16; x++) {
			int dx = sx + x;
			uint8_t p = r[fx ? 15 - x : x];
			if (p && dx >= 0 && dx < SCREEN_W)
				d[dx] = (uint16_t)(base + p);
		}
	}
}

static void sh_render(void)
{
	for (int s = 0; s < 4; s++)
		for (int i = 0; i < 0x400; i++)
			rbx_video.palette[s * 1024 + i] = pal_rgb(s_snap->pal[s][i]);
	const int pitch = rbx_video.pitch;
	for (int which = 0; which < 4; which++) {
		// layout: screens 2 | 3 on top, 0 | 1 below
		uint16_t *bm = rbx_video.bitmap + ((which & 2) ? 0 : VIS_H * pitch) + (which & 1) * SCREEN_W;
		const uint16_t pbase = (uint16_t)(which * 1024);
		const uint16_t *fg = s_snap->fg[which];
		// text layer: opaque, 32x32 tiles, colors 256-319
		for (int row = VIS_Y0 / 8; row <= VIS_Y1 / 8; row++)
			for (int col = 0; col < 32; col++) {
				uint16_t data = fg[row * 32 + col];
				int code = data & 0x7ff, color = (data & 0x3800) >> 12, fx = (data >> 14) & 1, fy = (data >> 15) & 1;
				const uint8_t *src = s_chars + code * 64;
				uint16_t base = (uint16_t)(pbase + 256 + color * 16);
				for (int y = 0; y < 8; y++) {
					uint16_t *d = bm + (row * 8 + y - VIS_Y0) * pitch + col * 8;
					const uint8_t *r = src + (fy ? 7 - y : y) * 8;
					for (int x = 0; x < 8; x++)
						d[x] = (uint16_t)(base + r[fx ? 7 - x : x]);
				}
			}
		// sprites (MAME draw_sprites with xo = which & 1, yo = which >> 1)
		const int xo = which & 1, yo = which >> 1;
		const uint16_t *spr = s_snap->spr;
		for (int offs = 0x400 - 4; offs >= 0; offs -= 4) {
			int code = (spr[offs + 2] & 0x3ff) ^ 0x200, color = (spr[offs + 2] >> 10) & 0x0f;
			int y = 256 - (spr[offs + 1] & 0xff), x = spr[offs + 3] & 0x3ff;
			int fx = (spr[offs + 2] >> 14) & 1, fy = (spr[offs + 2] >> 15) & 1;
			if (xo == 1)
				x -= 0x200;
			else if (x & 0x200)
				x -= 0x400;
			if (yo != ((spr[offs + 1] >> 9) & 1))
				continue;
			uint16_t base = (uint16_t)(pbase + color * 16);
			if (!(spr[offs + 1] & 0x8000)) {
				if (fy) {   // 16 x 32
					draw_sprite16(bm, pitch, code, base, fx, fy, x, y);
					draw_sprite16(bm, pitch, code + 1, base, fx, fy, x, y - 16);
				} else {
					draw_sprite16(bm, pitch, code, base, fx, fy, x, y - 16);
					draw_sprite16(bm, pitch, code + 1, base, fx, fy, x, y);
				}
			} else
				draw_sprite16(bm, pitch, code, base, fx, fy, x, y);
		}
	}
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap->fg, s_fg, sizeof(s_fg));
	memcpy(s_snap->spr, s_spr, sizeof(s_spr));
	memcpy(s_snap->pal, s_pal, sizeof(s_pal));
	rbx_video_frame_done();
}

// planar graphics: chars 2048 x 8x8 (planes 3, 2, 1, 0 x 0x4000 bytes), sprites 1024 x 16x16 (planes x 0x8000)
static int decode_gfx(const uint8_t *g1, const uint8_t *g2)
{
	s_chars = (uint8_t *)malloc(2048 * 64);
	s_sprites = (uint8_t *)malloc(1024 * 256);
	s_spr_used = (uint8_t *)calloc(1024, 1);
	if (!s_chars || !s_sprites || !s_spr_used)
		return -1;
	for (int t = 0; t < 2048; t++)
		for (int y = 0; y < 8; y++)
			for (int x = 0; x < 8; x++) {
				int o = t * 8 + y, b = 7 - x, p = 0;
				for (int pl = 0; pl < 4; pl++)   // plane 0 (MSB) at 3 * 0x4000
					p |= ((g1[(3 - pl) * 0x4000 + o] >> b) & 1) << (3 - pl);
				s_chars[t * 64 + y * 8 + x] = (uint8_t)p;
			}
	for (int t = 0; t < 1024; t++)
		for (int y = 0; y < 16; y++)
			for (int x = 0; x < 16; x++) {
				int o = t * 32 + (y < 8 ? y : 16 + y - 8) + (x < 8 ? 0 : 8), b = 7 - (x & 7), p = 0;
				for (int pl = 0; pl < 4; pl++)
					p |= ((g2[(3 - pl) * 0x8000 + o] >> b) & 1) << (3 - pl);
				s_sprites[t * 256 + y * 16 + x] = (uint8_t)p;
				if (p)
					s_spr_used[t] = 1;
			}
	return 0;
}

// ---- init / frame ----
static void sh_reset(void)
{
	// (first: the sound syncs read them, from a CPU write or a port callback of a chip reset below, before a frame
	// sets them; the last game's values made a render of billions of clocks, the next game of the board hung)
	s_slice_start = s_slice_len = s_cpu_start = 0;
	memset(s_ram, 0, sizeof(s_ram));
	memset(s_fg, 0, sizeof(s_fg));
	memset(s_spr, 0, sizeof(s_spr));
	memset(s_pal, 0, sizeof(s_pal));
	memset(s_snd_ram, 0, sizeof(s_snd_ram));
	memset(s_sub_ram, 0, sizeof(s_sub_ram));
	s_snd_d700 = 0;
	memset(s_sound_data, 0, sizeof(s_sound_data));
	s_sound_status = 0x000f;   // MAME machine_reset
	s_sub_data = 0;
	s_sub_status = 0;
	s_ym_irq[0] = s_ym_irq[1] = 0;
	z80_init(&s_snd);
	z80_init(&s_sub);
	s_snd.int_data = s_sub.int_data = 0xff;
	set_z80_handlers();
	rbx_ym_reset(0);
	rbx_ym_reset(1);
	ay8910_reset(&s_ay[0]);
	ay8910_reset(&s_ay[1]);
	ay_restart();
	s_dac = 0x80;
	s_frac_main = s_frac_z80 = s_frac_ym = 0;
	s_snd_done = 0;
	s_nfm = s_nay = s_fm_taken = 0;
	s_cur = NULL;
	m68k_bus_irq_reset();
	m68k_pulse_reset();
}

static void sh_exit(void)
{
	free(s_chars); s_chars = NULL;
	free(s_sprites); s_sprites = NULL;
	free(s_spr_used); s_spr_used = NULL;
	free(s_snap); s_snap = NULL;
	free(s_mix_fm); s_mix_fm = NULL;
	free(s_mix_ay); s_mix_ay = NULL;
	rbx_aux2_set_job(NULL);
	rbx_ym_exit(0);
	rbx_ym_exit(1);
	free(s_ay_out_hp);
	s_ay_out_hp = NULL;
	free(s_ram_hp);
	s_ram_hp = NULL;
}

static int sh_init(const rbx_game *g)
{
	if (!s_ay_out_hp && !(s_ay_out_hp = calloc(1, sizeof(*s_ay_out_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	if (!s_ram_hp && !(s_ram_hp = calloc(1, sizeof(*s_ram_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	uint32_t g1s = 0, g2s = 0;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_snd_rom = rbx_region("audiocpu", &s_snd_rom_size);
	s_sub_rom = rbx_region("subcpu", &s_sub_rom_size);
	uint8_t *g1 = rbx_region("gfx1", &g1s), *g2 = rbx_region("gfx2", &g2s);
	if (!s_rom || !s_snd_rom || !s_sub_rom || !g1 || !g2 || g1s < 0x10000 || g2s < 0x20000) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	m68k_bus_swap16(s_rom, s_rom_size);
	s_snap = calloc(1, sizeof(*s_snap));
	if (!s_snap || decode_gfx(g1, g2)) {
		rbx_fail("%s: out of memory (graphics)", g->set);
		sh_exit();
		return -1;
	}
	static const char *const in_tags[8] = { "IN0", "IN1", "SYSTEM", "IN2", "DSWA", "DSWB", "DSWC", "DSWD" };
	for (int i = 0; i < 8; i++)
		s_p_in[i] = rbx_port_find(in_tags[i]);
	// sound: 2 x YM2203 at 2 MHz (all routes 0.3), 2 x YM2149 at 4 MHz (0.3), DAC (0.3)
	rbx_ym_config yc;
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM2203;
	yc.clock = FM_CLOCK;
	yc.gain[0] = yc.gain[1] = yc.gain[2] = yc.gain[3] = 77;
	yc.irq = ym1_irq;
	if (rbx_ym_init(0, &yc)) {
		rbx_fail("%s: sound init failed", g->set);
		sh_exit();
		return -1;
	}
	yc.irq = ym2_irq;
	// (the YM2203s' samples made on the worker: rbx_ym_async_take at the frame's end, s_fm_taken)
	if (rbx_ym_init(1, &yc) || rbx_ym_set_async(0, 1) || rbx_ym_set_async(1, 1)) {
		rbx_fail("%s: sound init failed", g->set);
		sh_exit();
		return -1;
	}
	uint32_t rate = rbx_ym_rate(0);
	// YM2149 output at the same rate: 4 MHz / 8 / divider (the YM2203 rate is 2 MHz / 36 or similar)
	int div = (int)((Z80_CLOCK / 8 + rate / 2) / rate);
	for (int i = 0; i < 2; i++)
		ay8910_init(&s_ay[i], AY8910_TYPE_YM2149, Z80_CLOCK, AY8910_LEGACY_OUTPUT, 77, div, 1);
	rbx_aux2_set_job(ay_job);
	rbx_aux2_set_alone(1);
	if (ay8910_rate(&s_ay[0]) != rate)
		rbx_log("spdheat: YM2149 rate %u differs from the YM2203 rate %u", ay8910_rate(&s_ay[0]), rate);
	s_mix_cap = (int)(rate / 30) + 256;
	s_mix_fm = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	s_mix_ay = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	if (!s_mix_fm || !s_mix_ay || rbx_audio_setup(rate) || rbx_video_setup(SCREEN_W * 2, VIS_H * 2, 4096)) {
		sh_exit();
		return -1;
	}
	m68k_init();
	m68k_set_cpu_type(M68K_CPU_TYPE_68000);
	m68k_bus_reset_map();
	m68k_bus_map(0, 0x02ffff, s_rom, s_rom_size, 1, 0);
	m68k_bus_map(0x080000, 0x083fff, s_ram, sizeof(s_ram), 1, 1);
	m68k_bus_map(0x088000, 0x088fff, s_fg[0], 0x1000, 1, 1);   // fg_ram0 + fg_ram1
	m68k_bus_map(0x08c000, 0x08cfff, s_fg[2], 0x1000, 1, 1);   // fg_ram2 + fg_ram3
	m68k_bus_map(0x090000, 0x090fff, s_spr, sizeof(s_spr), 1, 1);
	for (int i = 0; i < 4; i++)
		m68k_bus_map(0x098000 + 0x1000u * (uint32_t)i, 0x098fff + 0x1000u * (uint32_t)i, s_pal[i], sizeof(s_pal[i]), 1, 1);
	m68k_bus.read8 = main_read8;
	m68k_bus.read16 = main_read16;
	m68k_bus.write8 = main_write8;
	m68k_bus.write16 = main_write16;
	for (int p = 0; p < 4; p++) {
		s_handle[p] = 0;
		s_accel[p] = 0x40;
	}
	sh_reset();
	return 0;
}

static void run_z80(z80 *z, int cycles, int is_snd)
{
	unsigned long end = z->cyc + (unsigned long)cycles;
	s_cur = z;
	s_cpu_start = z->cyc;
	while ((long)(z->cyc - end) < 0) {
		if (is_snd)
			update_snd_irq();
		z80_step(z);
	}
	s_cur = NULL;
}

static void sh_frame(void)
{
	for (int p = 0; p < 4; p++) {
		s_handle[p] = (s_handle[p] - rbx_analog_dpad(p + 1, RBX_IPT_DIAL, 3)) & 0xff;   // PORT_REVERSE (d-pad: 10 x 25%)
		int d = rbx_analog(p + 1, RBX_IPT_PEDAL);
		if (!d)   // (no pedal: d-pad up presses it, let go it returns; key delta 10 x 25%)
			d = (rbx_player_input(p + 1) & RBX_IN_UP) ? 3 : -3;
		int a = s_accel[p] + d;
		s_accel[p] = a < 0x40 ? 0x40 : a > 0xff ? 0xff : a;
	}
	for (int line = 0; line < LINES; line++) {
		if (line == VIS_Y1 + 1) {
			vblank_capture();
			m68k_bus_hold_irq(1);
		}
		m68k_execute(rbx_line_cycles(&s_frac_main, MAIN_CLOCK, 60000, LINES));
		int zc = rbx_line_cycles(&s_frac_z80, Z80_CLOCK, 60000, LINES);
		int yc = rbx_line_cycles(&s_frac_ym, FM_CLOCK, 60000, LINES);
		uint32_t line_start = s_snd_done;
		for (int s = 0; s < SLICES; s++) {
			int z0 = zc * s / SLICES, z1 = zc * (s + 1) / SLICES;
			int y0 = yc * s / SLICES, y1 = yc * (s + 1) / SLICES;
			s_slice_cycles = z1 - z0;
			s_slice_start = line_start + (uint32_t)y0;
			s_slice_len = (uint32_t)(y1 - y0);
			run_z80(&s_snd, z1 - z0, 1);
			run_z80(&s_sub, z1 - z0, 0);
			snd_run_to(line_start + (uint32_t)y1);
		}
	}
	rbx_ym_async_take(0, s_mix_fm + s_fm_taken, s_nfm - s_fm_taken);   // (the FM samples of the slots not yet given them)
	rbx_ym_async_take(1, s_mix_fm + s_fm_taken, s_nfm - s_fm_taken);
	s_fm_taken = s_nfm;
	ay_take();
	int n = s_nfm < s_nay ? s_nfm : s_nay;
	for (int i = 0; i < n; i++)
		rbx_audio_put(s_mix_fm[i] + s_mix_ay[i]);
	// keep what one chip produced past the other (the two rates match, this is at most a sample)
	memmove(s_mix_fm, s_mix_fm + n, sizeof(int32_t) * (size_t)(s_nfm - n));
	memmove(s_mix_ay, s_mix_ay + n, sizeof(int32_t) * (size_t)(s_nay - n));
	s_nfm -= n;
	s_nay -= n;
	s_fm_taken -= n;
	s_ay_taken -= n;
	memset(s_mix_fm + s_nfm, 0, sizeof(int32_t) * (size_t)(s_mix_cap - s_nfm));
	memset(s_mix_ay + s_nay, 0, sizeof(int32_t) * (size_t)(s_mix_cap - s_nay));
	s_snd_done = 0;
}

static int sh_state(void *ctx, int load)
{
	uint8_t cpu[1024];
	uint32_t csize = m68k_context_size();
	if (csize > sizeof(cpu))
		return -1;
	if (!load)
		m68k_get_context(cpu);
	uint32_t irq = m68k_bus_irq_pending();
	z80 z[2] = { s_snd, s_sub };
	uint32_t misc[12] = { s_sound_data[0], s_sound_data[1], s_sound_data[2], s_sound_data[3], s_sound_status, s_sub_data,
		s_sub_status, (uint32_t)s_ym_irq[0], (uint32_t)s_ym_irq[1], (uint32_t)s_dac, s_snd_d700, 0 };
	int32_t in[8] = { s_handle[0], s_handle[1], s_handle[2], s_handle[3], s_accel[0], s_accel[1], s_accel[2], s_accel[3] };
	uint64_t fr[3] = { s_frac_main, s_frac_z80, s_frac_ym };
	int rc = 0;
	rc |= rbx_state_io(ctx, "m68k", cpu, csize);
	rc |= rbx_state_io(ctx, "irq", &irq, 4);
	rc |= rbx_state_io(ctx, "ram", s_ram, sizeof(s_ram));
	rc |= rbx_state_io(ctx, "fg", s_fg, sizeof(s_fg));
	rc |= rbx_state_io(ctx, "spr", s_spr, sizeof(s_spr));
	rc |= rbx_state_io(ctx, "pal", s_pal, sizeof(s_pal));
	rc |= rbx_state_io(ctx, "z80", z, sizeof(z));
	rc |= rbx_state_io(ctx, "sndram", s_snd_ram, sizeof(s_snd_ram));
	rc |= rbx_state_io(ctx, "subram", s_sub_ram, sizeof(s_sub_ram));
	rc |= rbx_state_io(ctx, "misc", misc, sizeof(misc));
	rc |= rbx_state_io(ctx, "in", in, sizeof(in));
	rc |= rbx_state_io(ctx, "frac", fr, sizeof(fr));
	rbx_aux2_sync();   // (the worker's copies hold the chips' whole state: their queue done first)
	rc |= rbx_state_io(ctx, "ay1", &s_ay_w[0], ay8910_state_size());
	rc |= rbx_state_io(ctx, "ay2", &s_ay_w[1], ay8910_state_size());
	rc |= rbx_ym_state(0, ctx, load);
	rc |= rbx_ym_state(1, ctx, load);
	if (rc)
		return -1;
	if (load) {
		m68k_set_context(cpu);
		m68k_bus_set_irq_pending(irq);
		s_snd = z[0];
		s_sub = z[1];
		set_z80_handlers();
		for (int i = 0; i < 4; i++)
			s_sound_data[i] = misc[i];
		s_sound_status = misc[4]; s_sub_data = misc[5]; s_sub_status = misc[6];
		s_ym_irq[0] = (int)misc[7]; s_ym_irq[1] = (int)misc[8]; s_dac = (int)misc[9]; s_snd_d700 = (uint8_t)misc[10];
		for (int p = 0; p < 4; p++) {
			s_handle[p] = in[p];
			s_accel[p] = in[4 + p];
		}
		s_frac_main = fr[0];
		s_frac_z80 = fr[1];
		s_frac_ym = fr[2];
		for (int i = 0; i < 2; i++)
			memcpy(&s_ay[i], &s_ay_w[i], ay8910_state_size());
		ay_restart();
	}
	return 0;
}

static int sh_save(void *ctx) { return sh_state(ctx, 0); }
static int sh_load(void *ctx) { return sh_state(ctx, 1); }
static int sh_supported(const char *machine) { return !strcmp(machine, "spdheat"); }

const rbx_driver rbx_drv_spdheat = { "Super Dead Heat", sh_init, sh_exit, sh_reset, sh_frame, sh_save, sh_load, sh_supported, sh_render };
