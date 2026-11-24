/*  Data East / Wood Place Fire Trap, from MAME src/mame/dataeast/firetrap.cpp (BSD-3-Clause, copyright-holders:
    Nicola Salmoria).
    A Z80 at 6 MHz (its NMI at the frame's first line when enabled; its IRQ from the MCU, acknowledged by a write),
    the i8751 (the coins on P0, its replies on P1, the commands on P2 with INT0, P3: the Z80's IRQ and the INT0 ack;
    INT1 for a line at the frame's start; MAME: perfect sync, here sixteenths of a line; the bootleg: a coin read
    hack instead), two 16x16 backgrounds (x / y scrolled), an 8x8 text layer (column order), 16x16 / 16x32 sprites,
    the PROM palette. The sound: a 6502 at 1.5 MHz with the YM3526 and an MSM5205 fed a byte at a time through a
    74LS157 (the IRQ on every second VCK when enabled), the latch on its NMI, a banked ROM.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../cheat_cpu.h"
#include "../../rbx.h"
#include "../../cpu/m6502/m6502.h"
#include "../../cpu/mcs51/mcs51.h"
#include "../../cpu/z80/z80.h"
#include "../../sound/msm5205u.h"
#include "../../sound/rbx_ym.h"
#include "../../video/gfxlayout.h"

#include <stdlib.h>
#include <string.h>

#define CPU_CLOCK 6000000        // 12 MHz / 2
#define SND_CLOCK 1500000
#define MCU_CLOCK (8000000 / 12)
#define YM_CLOCK 3000000         // YM3526
#define FRAME_MHZ 57445          // set_raw(12 MHz / 2, 384, 0, 256, 272, 8, 248)
#define LINES 272
#define VIS_Y0 8
#define VIS_H 240
#define VIS_W 256
#define SLICES 16

static int s_boot;
static uint8_t *s_rom, *s_mcu_rom;
static uint32_t s_rom_size, s_mcu_size;
static uint8_t s_ram[0x1000], s_bg[2][0x800], s_fg[0x800], s_spr[0x180];
static uint8_t s_scroll_x[2][2], s_scroll_y[2][2];
static int s_bank, s_flip, s_nmi_enable, s_nmi, s_irq, s_coin_command_pending;
static z80 s_cpu;
static const uint8_t *s_zmap[256];
static rbx_gfx s_gfx8, s_gfxt[2], s_gfxspr;
static uint32_t s_palette[256];
static int s_port_in[3], s_port_dsw[2], s_port_coins;
static int s_line;
static uint64_t s_cpu_frac, s_snd_frac, s_mcu_frac, s_ym_frac;
static struct {
	uint8_t bg[2][0x800], fg[0x800], spr[0x180];
	int scrollx[2], scrolly[2], flip;
} *s_snap;

// ---- the i8751 ----
static mcs51_t s_mcu;
static uint8_t s_mcu_to_main, s_main_to_mcu, s_mcu_p3;

static uint8_t mcu_in(void *u, int port)
{
	(void)u;
	if (port == 0) {   // mcu_p0_r: the coins (bits 1-3) and "none in" (bit 0)
		const uint8_t c = (uint8_t)rbx_port_read(s_port_coins);
		return (uint8_t)((c & 0x0e) | ((c & 0x0e) == 0x0e));
	}
	if (port == 2) return s_main_to_mcu;
	return 0xff;
}
static void mcu_out(void *u, int port, uint8_t data)
{
	(void)u;
	if (port == 1)
		s_mcu_to_main = data;
	else if (port == 3) {   // mcu_p3_w: P3.0 falling the Z80's IRQ, P3.1 falling the INT0 ack
		if ((s_mcu_p3 & 1) && !(data & 1))
			s_irq = 1;
		if ((s_mcu_p3 & 2) && !(data & 2))
			mcs51_set_input(&s_mcu, MCS51_INT0_LINE, 0);
		s_mcu_p3 = data;
	}
}

// ---- sound: 6502, YM3526, MSM5205 ----
static m6502_t s_snd;
static uint8_t s_sram[0x800];
static uint8_t *s_zrom;
static uint32_t s_zrom_size;
static uint8_t s_latch, s_adpcm_ba;
static int s_snd_bank, s_sound_irq_enable, s_adpcm_toggle;
static msm5205u_t s_msm;
static int32_t *s_mix;
static int s_mix_n, s_mix_cap;

static void msm_vck(msm5205u_t *m, void *user)   // adpcm_int: the LS157's select toggled, the IRQ when enabled
{
	(void)user;
	s_adpcm_toggle ^= 1;
	msm5205u_data_w(m, (uint8_t)(s_adpcm_toggle ? s_adpcm_ba >> 4 : s_adpcm_ba & 0x0f));
	if (s_sound_irq_enable && s_adpcm_toggle)
		m6502_set_irq(&s_snd, 1);
}
static void set_snd_bank(int b)
{
	s_snd_bank = b & 1;
	for (int p = 0; p < 0x40; p++)
		s_snd.rpage[0x40 + p] = s_zrom + 0x8000 + s_snd_bank * 0x4000 + p * 256;
}
static uint8_t snd_r(void *ctx, uint16_t a)
{
	(void)ctx;
	if (a == 0x3400) {   // (reading the latch clears its NMI)
		m6502_set_nmi(&s_snd, 0);
		return s_latch;
	}
	return 0;
}
static void snd_w(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	switch (a) {
	case 0x1000: case 0x1001: rbx_ym_write(0, a & 1, v); break;
	case 0x2000:   // adpcm_data_w: the byte to the LS157, the IRQ cleared
		m6502_set_irq(&s_snd, 0);
		s_adpcm_ba = v;
		break;
	case 0x2400:   // sound_flip_flop_w: the MSM's reset, the IRQ enable
		msm5205u_reset_w(&s_msm, !(v & 1));
		s_sound_irq_enable = (v >> 1) & 1;
		if (!s_sound_irq_enable)
			m6502_set_irq(&s_snd, 0);
		break;
	case 0x2800: set_snd_bank(v); break;
	}
}

// ---- bus ----
static int vblank(void) { return s_line < VIS_Y0 || s_line >= VIS_Y0 + VIS_H; }
static void set_bank(int b)   // (16 KB banks from 0x10000 of the CPU's region)
{
	s_bank = b & 3;
	for (int p = 0; p < 0x40; p++)
		s_zmap[0x80 + p] = s_rom + 0x10000 + (uint32_t)s_bank * 0x4000 + (uint32_t)p * 256;
}
static uint8_t coin_r(void)   // (the bootleg: its coin read, as MAME's hack)
{
	const uint8_t port = (uint8_t)(rbx_port_read(s_port_in[2]) & 0x70);
	if (s_cpu.pc == 0x1188)   // (the PC past the reading instruction, as MAME's)
		return (uint8_t)~s_coin_command_pending;
	if (port != 0x70) {
		int coin = 0;
		if (!(port & 0x20)) coin = 1;
		if (!(port & 0x40)) coin = 2;
		if (!(port & 0x10)) coin = 3;
		s_coin_command_pending = coin;
		return 0xff;
	}
	return 0;
}
static uint8_t cpu_r(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x8000 || (s_boot && a >= 0xf800 && a < 0xf900)) return s_rom[a];
	if (a < 0xc000) return s_zmap[a >> 8][a & 0xff];
	if (a < 0xd000) return s_ram[a - 0xc000];
	if (a < 0xe000) return s_bg[(a >> 11) & 1][a & 0x7ff];
	if (a < 0xe800) return s_fg[a - 0xe000];
	if (a < 0xe980) return s_spr[a - 0xe800];
	switch (a) {
	case 0xf010: return (uint8_t)rbx_port_read(s_port_in[0]);
	case 0xf011: return (uint8_t)rbx_port_read(s_port_in[1]);
	case 0xf012:
		rbx_port_custom(s_port_in[2], 0x80, vblank() ? 0x80 : 0);
		return (uint8_t)rbx_port_read(s_port_in[2]);
	case 0xf013: return (uint8_t)rbx_port_read(s_port_dsw[0]);
	case 0xf014: return (uint8_t)rbx_port_read(s_port_dsw[1]);
	case 0xf016: return s_boot ? coin_r() : s_mcu_to_main;
	}
	return 0;
}
static void cpu_w(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a < 0xc000) return;
	if (a < 0xd000) { s_ram[a - 0xc000] = v; return; }
	if (a < 0xe000) { s_bg[(a >> 11) & 1][a & 0x7ff] = v; return; }
	if (a < 0xe800) { s_fg[a - 0xe000] = v; return; }
	if (a < 0xe980) { s_spr[a - 0xe800] = v; return; }
	switch (a) {
	case 0xf000: s_irq = 0; break;   // irqack_w
	case 0xf001: s_latch = v; m6502_set_nmi(&s_snd, 1); break;
	case 0xf002: set_bank(v); break;
	case 0xf003: s_flip = v != 0; break;
	case 0xf004:   // nmi_disable_w
		s_nmi_enable = ~v & 1;
		if (!s_nmi_enable)
			s_nmi = 0;
		break;
	case 0xf005:   // mcu_w: the command, INT0
		if (!s_boot) {
			s_main_to_mcu = v;
			mcs51_set_input(&s_mcu, MCS51_INT0_LINE, 1);
		}
		break;
	case 0xf008: case 0xf009: s_scroll_x[0][a & 1] = v; break;
	case 0xf00a: case 0xf00b: s_scroll_y[0][a & 1] = v; break;
	case 0xf00c: case 0xf00d: s_scroll_x[1][a & 1] = v; break;
	case 0xf00e: case 0xf00f: s_scroll_y[1][a & 1] = v; break;
	}
}
static uint8_t cpu_in(z80 *z, uint8_t port) { (void)z; (void)port; return 0xff; }
static void cpu_out(z80 *z, uint8_t port, uint8_t v) { (void)z; (void)port; (void)v; }

// ---- video ----
static void spr_blit(uint32_t code, int colour, int fx, int fy, int sx, int sy, uint16_t *bm, int pitch)
{
	if (!s_gfxspr.count)
		return;
	code %= s_gfxspr.count;
	const uint8_t *src = s_gfxspr.pix + code * 256;
	for (int y = 0; y < 16; y++) {
		const int dy = sy + y - VIS_Y0;
		if (dy < 0 || dy >= VIS_H)
			continue;
		const uint8_t *row = src + (fy ? 15 - y : y) * 16;
		for (int x = 0; x < 16; x++) {
			const int dx = sx + x;
			if (dx < 0 || dx >= VIS_W)
				continue;
			const uint8_t p = row[fx ? 15 - x : x];
			if (p)
				bm[dy * pitch + dx] = (uint16_t)(0x40 + colour * 16 + p);
		}
	}
}
// a background (32x32 of 16x16, the odd mapper with a hole at bit 8; colour bits 2-3 flip the tile in x / y);
// opaque or pen 0 clear; MAME's tilemap flip (the screen 384 x 272) when flipped
static void bg_draw(int which, uint16_t *bm, int pitch, int opaque)
{
	const int flip = s_snap->flip, sx = s_snap->scrollx[which], sy = s_snap->scrolly[which];
	const uint8_t *v = s_snap->bg[which];
	const rbx_gfx *g = &s_gfxt[which];
	for (int y = 0; y < VIS_H; y++) {
		uint16_t *d = bm + y * pitch;
		const int py = flip ? 511 - ((y + VIS_Y0 - (272 - 512 + sy)) & 0x1ff) : (y + VIS_Y0 + sy) & 0x1ff;
		for (int x = 0; x < VIS_W; x++) {
			const int px = flip ? 511 - ((x - (384 - 512 + sx)) & 0x1ff) : (x + sx) & 0x1ff;
			const int col = px >> 4, row = py >> 4;
			const int idx = ((row & 0x0f) ^ 0x0f) | ((col & 0x0f) << 4) | ((row & 0x10) << 5) | ((col & 0x10) << 6);
			const int attr = v[idx + 0x100];
			int tx = px & 15, ty = py & 15;
			if (attr & 4) tx = 15 - tx;
			if (attr & 8) ty = 15 - ty;
			const uint32_t code = rbx_wrap((uint32_t)(v[idx] + ((attr & 3) << 8)), g->count);
			const uint8_t p = g->pix[(code * 16 + (uint32_t)ty) * 16 + (uint32_t)tx];
			if (opaque || p)
				d[x] = (uint16_t)((which ? 0xc0 : 0x80) + ((attr & 0x30) >> 4) * 16 + p);
		}
	}
}

static void ft_render(void)
{
	memcpy(rbx_video.palette, s_palette, sizeof(s_palette));
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch, flip = s_snap->flip;
	bg_draw(1, bm, pitch, 1);
	bg_draw(0, bm, pitch, 0);
	const uint8_t *sr = s_snap->spr;
	for (int offs = 0; offs < 0x180; offs += 4) {   // draw_sprites (in the list's order, wrapping in x)
		int sy = sr[offs], sx = sr[offs + 2];
		uint32_t code = (uint32_t)(sr[offs + 3] + 4 * (sr[offs + 1] & 0xc0));
		const int colour = ((sr[offs + 1] & 0x08) >> 2) | (sr[offs + 1] & 0x01);
		int fx = (sr[offs + 1] & 4) != 0, fy = (sr[offs + 1] & 2) != 0;
		if (flip) {
			sx = 240 - sx;
			sy = 240 - sy;
			fx = !fx;
			fy = !fy;
		}
		if (sr[offs + 1] & 0x10) {   // double
			if (flip)
				sy -= 16;
			for (int w = 0; w <= 256; w += 256) {
				spr_blit(code & ~1u, colour, fx, fy, sx - w, fy ? sy : sy + 16, bm, pitch);
				spr_blit(code | 1, colour, fx, fy, sx - w, fy ? sy + 16 : sy, bm, pitch);
			}
		} else
			for (int w = 0; w <= 256; w += 256)
				spr_blit(code, colour, fx, fy, sx - w, sy, bm, pitch);
	}
	for (int y = 0; y < VIS_H; y++) {   // the text: 32x32 of 8x8 by columns (rows reversed), pen 0 clear
		uint16_t *d = bm + y * pitch;
		for (int x = 0; x < VIS_W; x++) {
			int px = x, py = y + VIS_Y0;
			if (flip) {
				px = 255 - px;
				py = 255 - py;
			}
			const int off = ((py >> 3) ^ 0x1f) + ((px >> 3) << 5), attr = s_snap->fg[off + 0x400];
			const uint32_t code = rbx_wrap((uint32_t)(s_snap->fg[off] | ((attr & 1) << 8)), s_gfx8.count);
			const uint8_t p = s_gfx8.pix[(code * 8 + (uint32_t)(py & 7)) * 8 + (uint32_t)(px & 7)];
			if (p)
				d[x] = (uint16_t)((attr >> 4) * 4 + p);
		}
	}
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap->bg, s_bg, sizeof(s_bg));
	memcpy(s_snap->fg, s_fg, sizeof(s_fg));
	memcpy(s_snap->spr, s_spr, sizeof(s_spr));
	for (int i = 0; i < 2; i++) {
		s_snap->scrollx[i] = s_scroll_x[i][0] | (s_scroll_x[i][1] << 8);
		s_snap->scrolly[i] = -(s_scroll_y[i][0] | (s_scroll_y[i][1] << 8));
	}
	s_snap->flip = s_flip;
	rbx_video_frame_done();
}

// ---- init / frame ----
static void ft_reset(void)
{
	memset(s_ram, 0, sizeof(s_ram));
	memset(s_bg, 0, sizeof(s_bg));
	memset(s_fg, 0, sizeof(s_fg));
	memset(s_spr, 0, sizeof(s_spr));
	memset(s_scroll_x, 0, sizeof(s_scroll_x));
	memset(s_scroll_y, 0, sizeof(s_scroll_y));
	s_flip = s_nmi_enable = s_nmi = s_irq = s_coin_command_pending = 0;
	set_bank(0);
	z80_init(&s_cpu);
	s_cpu.read_byte = cpu_r;
	s_cpu.write_byte = cpu_w;
	s_cpu.port_in = cpu_in;
	s_cpu.port_out = cpu_out;
	s_cpu.read_map = s_zmap;
	s_cpu.int_data = 0xff;
	s_mcu_to_main = s_main_to_mcu = 0;
	s_mcu_p3 = 0xff;
	if (!s_boot)
		mcs51_reset(&s_mcu);
	memset(s_sram, 0, sizeof(s_sram));
	s_latch = s_adpcm_ba = 0;
	s_sound_irq_enable = s_adpcm_toggle = 0;
	set_snd_bank(0);
	m6502_reset(&s_snd);
	m6502_set_irq(&s_snd, 0);
	m6502_set_nmi(&s_snd, 0);
	rbx_ym_reset(0);
	msm5205u_reset(&s_msm);
	s_mix_n = 0;
	s_line = 0;
	s_cpu_frac = s_snd_frac = s_mcu_frac = s_ym_frac = 0;
}

static void ft_exit(void);

static int ft_init(const rbx_game *g)
{
	rbx_cheat_prefer("z80");   // (the cheats' main CPU: another CPU is set up first)
	s_boot = !strcmp(g->machine, "firetrapbl");
	uint32_t csize, t1size, t2size, ssize, psize;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_zrom = rbx_region("audiocpu", &s_zrom_size);
	s_mcu_rom = s_boot ? NULL : rbx_region("mcu", &s_mcu_size);
	uint8_t *chars = rbx_region("chars", &csize), *t1 = rbx_region("tiles1", &t1size), *t2 = rbx_region("tiles2", &t2size),
	        *spr = rbx_region("sprites", &ssize), *prom = rbx_region("proms", &psize);
	if (!s_rom || s_rom_size < 0x20000 || !s_zrom || s_zrom_size < 0x10000 || (!s_boot && (!s_mcu_rom || s_mcu_size < 0x1000)) ||
	    !chars || csize < 0x2000 || !t1 || t1size < 0x20000 || !t2 || t2size < 0x20000 || !spr || ssize < 0x20000 || !prom ||
	    psize < 0x200) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	{	// charlayout (512, 2 bpp in the nibbles, bit-reversed, the right half the region's second; rows reversed),
		// tilelayout (1024, 4 bpp: 0, 4 and the region's half + 0, + 4; the quarters across x; rows reversed),
		// spritelayout (1024, 4 bpp: the quarters; bit-reversed; rows reversed)
		static uint32_t xc[8], yc[8], xt[16], yt[16], xs[16];
		for (int i = 0; i < 4; i++) {
			xc[i] = (uint32_t)(3 - i);
			xc[4 + i] = (uint32_t)(0x1000 * 8 + 3 - i);
			xt[i] = (uint32_t)(3 - i);
			xt[4 + i] = (uint32_t)(0x8000 * 8 + 3 - i);
			xt[8 + i] = (uint32_t)(16 * 8 + 3 - i);
			xt[12 + i] = (uint32_t)(0x8000 * 8 + 16 * 8 + 3 - i);
		}
		for (int i = 0; i < 8; i++) {
			yc[i] = (uint32_t)((7 - i) * 8);
			xs[i] = (uint32_t)(7 - i);
			xs[8 + i] = (uint32_t)(16 * 8 + 7 - i);
		}
		for (int i = 0; i < 16; i++)
			yt[i] = (uint32_t)((15 - i) * 8);
		static const uint32_t pc[2] = { 0, 4 }, pt[4] = { 0, 4, 0x10000 * 8, 0x10000 * 8 + 4 },
		                      ps[4] = { 0, 0x8000 * 8, 0x10000 * 8, 0x18000 * 8 };
		const rbx_gfx_layout l8 = { 8, 8, 2, pc, xc, yc, 8 * 8 };
		const rbx_gfx_layout lt = { 16, 16, 4, pt, xt, yt, 32 * 8 }, ls = { 16, 16, 4, ps, xs, yt, 32 * 8 };
		if (rbx_gfx_decode(&s_gfx8, &l8, chars, 512 * 8) || rbx_gfx_decode(&s_gfxt[0], &lt, t1, 1024 * 32) ||
		    rbx_gfx_decode(&s_gfxt[1], &lt, t2, 1024 * 32) || rbx_gfx_decode(&s_gfxspr, &ls, spr, 1024 * 32)) {
			rbx_fail("%s: out of memory (graphics)", g->set);
			ft_exit();
			return -1;
		}
	}
	for (int i = 0; i < 256; i++) {   // palette: red the low nibble, green the high; blue the second PROM
		static const uint8_t w[4] = { 0x0e, 0x1f, 0x43, 0x8f };
		int r = 0, gg = 0, b = 0;
		for (int k = 0; k < 4; k++) {
			r += ((prom[i] >> k) & 1) * w[k];
			gg += ((prom[i] >> (4 + k)) & 1) * w[k];
			b += ((prom[256 + i] >> k) & 1) * w[k];
		}
		s_palette[i] = (uint32_t)(r << 16 | gg << 8 | b);
	}
	if (!s_boot) {
		mcs51_init(&s_mcu, s_mcu_rom, 0x1000);
		s_mcu.port_in = mcu_in;
		s_mcu.port_out = mcu_out;
	}
	s_snap = calloc(1, sizeof(*s_snap));
	rbx_ym_config yc;   // (YM3526 3 MHz 1.0, its IRQ not wired)
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM3526;
	yc.clock = YM_CLOCK;
	yc.gain[0] = 256;
	int rc = !s_snap || rbx_ym_init(0, &yc) || rbx_ym_set_async(0, 1);
	if (!rc) {
		s_mix_cap = (int)(rbx_ym_rate(0) / 50) + 256;
		s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
		rc = !s_mix || rbx_audio_setup(rbx_ym_rate(0)) || rbx_video_setup(VIS_W, VIS_H, 256);
	}
	if (rc) {
		rbx_fail("%s: out of memory", g->set);
		ft_exit();
		return -1;
	}
	msm5205u_init(&s_msm, 375000, MSM5205_S48_4B, msm_vck, NULL);   // (12 MHz / 32: 7.8125 kHz)
	m6502_init(&s_snd, M6502_NMOS);
	for (int p = 0; p < 8; p++)
		s_snd.rpage[p] = s_snd.wpage[p] = s_sram + p * 256;
	for (int p = 0x80; p < 0x100; p++)
		s_snd.rpage[p] = s_zrom + (p - 0x80) * 256;
	s_snd.read = snd_r;
	s_snd.write = snd_w;
	memset(s_zmap, 0, sizeof(s_zmap));
	for (int p = 0; p < 0x80; p++)
		s_zmap[p] = s_rom + p * 256;
	s_port_in[0] = rbx_port_find("IN0");
	s_port_in[1] = rbx_port_find("IN1");
	s_port_in[2] = rbx_port_find("IN2");
	s_port_dsw[0] = rbx_port_find("DSW0");
	s_port_dsw[1] = rbx_port_find("DSW1");
	s_port_coins = rbx_port_find("COINS");
	ft_reset();
	return 0;
}

static void ft_exit(void)
{
	rbx_ym_exit(0);
	free(s_mix);
	s_mix = NULL;
	rbx_gfx_free(&s_gfx8);
	rbx_gfx_free(&s_gfxt[0]);
	rbx_gfx_free(&s_gfxt[1]);
	rbx_gfx_free(&s_gfxspr);
	free(s_snap);
	s_snap = NULL;
}

static void run_main(unsigned long end)
{
	if (s_nmi) {   // (the NMI line held: an edge taken once)
		if (s_nmi == 1) {
			z80_gen_nmi(&s_cpu);
			s_nmi = 2;
		}
	}
	z80_run(&s_cpu, end, &s_irq, 0);
}

static void ft_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	for (int line = 0; line < LINES; line++) {
		s_line = line;
		if (line == VIS_Y0 + VIS_H)
			vblank_capture();
		if (line == 0) {   // the scanline timer: line 0 the NMI (when enabled) and the MCU's INT1, line 1 its end
			if (s_nmi_enable && !s_nmi)
				s_nmi = 1;
			if (!s_boot)
				mcs51_set_input(&s_mcu, MCS51_INT1_LINE, 1);
		} else if (line == 1 && !s_boot)
			mcs51_set_input(&s_mcu, MCS51_INT1_LINE, 0);
		const int cyc = rbx_line_cycles(&s_cpu_frac, CPU_CLOCK, FRAME_MHZ, LINES);
		if (s_boot)
			run_main(s_cpu.cyc + (unsigned long)cyc);
		else {   // (MAME: perfect sync with the MCU)
			const int mc = rbx_line_cycles(&s_mcu_frac, MCU_CLOCK, FRAME_MHZ, LINES);
			const unsigned long t0 = s_cpu.cyc;
			for (int k = 0; k < SLICES; k++) {
				run_main(t0 + (unsigned long)(cyc * (k + 1) / SLICES));
				mcs51_execute(&s_mcu, mc * (k + 1) / SLICES - mc * k / SLICES);
			}
		}
		const int sc = rbx_line_cycles(&s_snd_frac, SND_CLOCK, FRAME_MHZ, LINES);
		m6502_run_until(&s_snd, s_snd.cycles + sc);
		const int n = rbx_ym_advance(0, (uint32_t)rbx_line_cycles(&s_ym_frac, YM_CLOCK, FRAME_MHZ, LINES), s_mix + s_mix_n,
		                             s_mix_cap - s_mix_n);
		msm5205u_render(&s_msm, s_mix + s_mix_n, n, rbx_ym_rate(0), 77);   // 0.30 (its VCKs in step with the 6502)
		s_mix_n += n;
	}
	rbx_ym_async_take(0, s_mix, s_mix_n);
	for (int i = 0; i < s_mix_n; i++)
		rbx_audio_put(s_mix[i]);
	memset(s_mix, 0, sizeof(int32_t) * (size_t)s_mix_n);
	s_mix_n = 0;
}

static int ft_state(void *ctx, int load)
{
	uint8_t snd[M6502_STATE_SIZE];
	if (!load)
		memcpy(snd, &s_snd, M6502_STATE_SIZE);
	z80 z = s_cpu;
	uint64_t fr[4] = { s_cpu_frac, s_snd_frac, s_mcu_frac, s_ym_frac };
	int32_t misc[14] = { s_bank, s_flip, s_nmi_enable, s_nmi, s_irq, s_coin_command_pending, s_mcu_to_main, s_main_to_mcu,
	                     s_mcu_p3, s_latch, s_adpcm_ba, s_snd_bank, s_sound_irq_enable, s_adpcm_toggle };
	int rc = 0;
	rc |= rbx_state_io(ctx, "z80", &z, sizeof(z));
	rc |= rbx_state_io(ctx, "ram", s_ram, sizeof(s_ram));
	rc |= rbx_state_io(ctx, "bg", s_bg, sizeof(s_bg));
	rc |= rbx_state_io(ctx, "fg", s_fg, sizeof(s_fg));
	rc |= rbx_state_io(ctx, "spr", s_spr, sizeof(s_spr));
	rc |= rbx_state_io(ctx, "scx", s_scroll_x, sizeof(s_scroll_x));
	rc |= rbx_state_io(ctx, "scy", s_scroll_y, sizeof(s_scroll_y));
	if (!s_boot) {
		rc |= rbx_state_io(ctx, "mcu", &s_mcu.st, sizeof(s_mcu.st));
		rc |= rbx_state_io(ctx, "mcuf", s_mcu.forced_inputs, sizeof(s_mcu.forced_inputs));
	}
	rc |= rbx_state_io(ctx, "m6502", snd, M6502_STATE_SIZE);
	rc |= rbx_state_io(ctx, "sram", s_sram, sizeof(s_sram));
	rc |= rbx_state_io(ctx, "msm", &s_msm, MSM5205_STATE_SIZE);
	rc |= rbx_ym_state(0, ctx, load);
	rc |= rbx_state_io(ctx, "frac", fr, sizeof(fr));
	rc |= rbx_state_io(ctx, "misc", misc, sizeof(misc));
	if (!rc && load) {
		z.read_byte = cpu_r;
		z.write_byte = cpu_w;
		z.port_in = cpu_in;
		z.port_out = cpu_out;
		z.userdata = NULL;
		z.read_map = s_zmap;
		s_cpu = z;
		memcpy(&s_snd, snd, M6502_STATE_SIZE);
		s_cpu_frac = fr[0];
		s_snd_frac = fr[1];
		s_mcu_frac = fr[2];
		s_ym_frac = fr[3];
		set_bank(misc[0]);
		s_flip = misc[1];
		s_nmi_enable = misc[2];
		s_nmi = misc[3];
		s_irq = misc[4];
		s_coin_command_pending = misc[5];
		s_mcu_to_main = (uint8_t)misc[6];
		s_main_to_mcu = (uint8_t)misc[7];
		s_mcu_p3 = (uint8_t)misc[8];
		s_latch = (uint8_t)misc[9];
		s_adpcm_ba = (uint8_t)misc[10];
		set_snd_bank(misc[11]);
		s_sound_irq_enable = misc[12];
		s_adpcm_toggle = misc[13];
	}
	return rc ? -1 : 0;
}

static int ft_save(void *ctx) { return ft_state(ctx, 0); }
static int ft_load(void *ctx) { return ft_state(ctx, 1); }
static int ft_supported(const char *machine) { return !strcmp(machine, "firetrap") || !strcmp(machine, "firetrapbl"); }

const rbx_driver rbx_drv_firetrap = { "DECO Fire Trap", ft_init, ft_exit, ft_reset, ft_frame, ft_save, ft_load,
                                      ft_supported, ft_render, NULL };
