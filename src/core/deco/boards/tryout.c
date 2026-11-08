/*  Data East Pro Baseball Skill Tryout, from MAME src/mame/dataeast/tryout.cpp (BSD-3-Clause, copyright-holders:
    Pierpaolo Prazzoli, Bryan McPhail).
    A 6502 at 2 MHz (its NMI on a coin, acknowledged by a write), a 6502 at 1.5 MHz with the YM2203 (the latch on its
    IRQ, a 1 kHz NMI for the music's tempo). The background's 16x16 tiles are uploaded by the CPU into banked video
    RAM (MAME's vram_w regroups their bit planes as they are written), a 1024x256 map of them (x / y scrolled), an
    8x8 text layer (column order), 16x16 / 16x32 sprites, the PROM palette (32 entries). MAME's screen: 60 Hz,
    256x256 with a 2.5 ms vblank (here 288 lines: 8 .. 247 shown, 248 .. 287 the vblank).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m6502/m6502.h"
#include "../../sound/rbx_ym.h"
#include "../../video/gfxlayout.h"

#include <stdlib.h>
#include <string.h>

#define CPU_CLOCK 2000000
#define SND_CLOCK 1500000
#define YM_CLOCK 1500000
#define FRAME_MHZ 60000
#define LINES 288
#define VIS_Y0 8
#define VIS_H 240
#define VIS_W 256

static uint8_t *s_rom;
static uint32_t s_rom_size;
static uint8_t s_ram[0x800], s_videoram[0x800], s_spr0[0x100], s_spr1[0x100];   // (the sprites: 0x80 used)
static uint8_t s_vram[8 * 0x800], s_vram_gfx[0x6000];
static uint8_t s_gfx_control[3], s_vram_bank;
static int s_bank, s_flip, s_nmi, s_coin_prev;
static m6502_t s_cpu;
static rbx_gfx s_gfx8, s_gfxspr;
static uint8_t s_bgpix[128 * 256];         // (the background's tiles, decoded from s_vram_gfx at render)
static uint32_t s_palette[32];
static int s_port_dsw, s_port_p[2], s_port_sys;
static int s_line;
static uint64_t s_cpu_frac, s_snd_frac, s_ym_frac, s_tempo_frac;
static int32_t *s_mix;
static int s_mix_n, s_mix_cap;
static struct {
	uint8_t videoram[0x800], spr0[0x100], spr1[0x100], map[0x400], gfx[0x6000], gfx_control[3];
	int flip;
} *s_snap;

// ---- sound: 6502, YM2203 ----
static m6502_t s_snd;
static uint8_t s_sram[0x800];
static uint8_t *s_zrom;
static uint32_t s_zrom_size;
static uint8_t s_latch;

static uint8_t snd_r(void *ctx, uint16_t a)
{
	(void)ctx;
	if (a == 0x4000 || a == 0x4001) return rbx_ym_read(0, a & 1);
	if (a == 0xa000) {   // (reading the latch clears its IRQ)
		m6502_set_irq(&s_snd, 0);
		return s_latch;
	}
	return 0;
}
static void snd_w(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	if (a == 0x4000 || a == 0x4001) rbx_ym_write(0, a & 1, v);
}

// ---- bus ----
static int vblank(void) { return s_line >= VIS_Y0 + VIS_H; }
static void set_bank(int b)
{
	s_bank = b & 1;
	for (int p = 0; p < 0x20; p++)
		s_cpu.rpage[0x20 + p] = s_rom + 0x10000 + (uint32_t)s_bank * 0x2000 + (uint32_t)p * 256;
}
static void vram_w(uint32_t offset, uint8_t data)   // (MAME vram_w: the tile map, or the tiles' planes regrouped)
{
	const int bank = (s_vram_bank >> 1) & 7;
	if ((bank == 0 || bank == 2 || bank == 4 || bank == 6) && (offset & 0x7ff) < 0x400) {
		s_vram[offset & 0x3ff] = data;
		return;
	}
	offset = (offset & 0x7ff) | ((uint32_t)bank << 11);
	s_vram[offset] = data;
	const uint32_t o = offset & 0x3ff, hi = (offset & 0x2000) >> 1;
	const uint8_t lo4 = (uint8_t)(~data & 0x0f), hi4 = (uint8_t)((~data & 0xf0) >> 4);
	static const uint16_t base[8][2] = {   // (by offset & 0x1c00: the low and the high nibble's destinations)
		{ 0, 0 }, { 0x0000, 0x2000 }, { 0x4000, 0x4400 }, { 0x0400, 0x2400 },
		{ 0, 0 }, { 0x0800, 0x2800 }, { 0x4800, 0x4c00 }, { 0x0c00, 0x2c00 } };
	const int k = (int)((offset & 0x1c00) >> 10);
	if (k == 0 || k == 4)
		return;
	s_vram_gfx[o + base[k][0] + hi] = lo4;
	s_vram_gfx[o + base[k][1] + hi] = hi4;
}
static uint8_t cpu_r(void *ctx, uint16_t a)
{
	(void)ctx;
	if (a >= 0xd000 && a < 0xd800) return s_vram[a - 0xd000];   // vram_r ("debug only")
	switch (a) {
	case 0xe000: return (uint8_t)rbx_port_read(s_port_dsw);
	case 0xe001: return (uint8_t)rbx_port_read(s_port_p[0]);
	case 0xe002: return (uint8_t)rbx_port_read(s_port_p[1]);
	case 0xe003:
		rbx_port_custom(s_port_sys, 0x80, vblank() ? 0x80 : 0);
		return (uint8_t)rbx_port_read(s_port_sys);
	}
	if (a >= 0xfff0) return s_rom[0xbff0 + (a - 0xfff0)];   // (the vectors: the region's 0xbff0)
	return 0;
}
static void cpu_w(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	if (a >= 0xd000 && a < 0xd800) {
		vram_w(a - 0xd000, v);
		return;
	}
	switch (a) {
	case 0xe301: s_flip = v & 1; break;
	case 0xe302: set_bank(v); break;
	case 0xe401: s_vram_bank = v; break;
	case 0xe402: case 0xe403: case 0xe404: s_gfx_control[a - 0xe402] = v; break;
	case 0xe414: s_latch = v; m6502_set_irq(&s_snd, 1); break;
	case 0xe417: s_nmi = 0; m6502_set_nmi(&s_cpu, 0); break;   // nmi_ack_w
	}
}
static void coin_update(void)   // (a coin's press: the main CPU's NMI)
{
	const int c = (int)(rbx_port_read(s_port_sys) & 0x18);
	if (s_coin_prev & ~c & 0x18) {
		s_nmi = 1;
		m6502_set_nmi(&s_cpu, 1);
	}
	s_coin_prev = c;
}

// ---- video ----
// MAME's tilemap scroll: the map's x (y) of screen x (y) for a width (height) w, flipped as a whole when flip
static inline int tm_pos(int x, int scroll, int w, int flip)
{
	if (!flip)
		return (x + scroll) & (w - 1);
	const int value = 256 - w + scroll;   // (the screen: 256 wide / high)
	return w - 1 - ((x - value) & (w - 1));
}
static void spr_blit(uint32_t code, int fx, int fy, int sx, int sy, uint16_t *bm, int pitch)
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
				bm[dy * pitch + dx] = p;
		}
	}
}

static void to_render(void)
{
	memcpy(rbx_video.palette, s_palette, sizeof(s_palette));
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch, flip = s_snap->flip;
	const uint8_t *gc = s_snap->gfx_control;
	if (!(gc[0] & 8)) {   // (the screen disabled)
		for (int y = 0; y < VIS_H; y++)
			for (int x = 0; x < VIS_W; x++)
				bm[y * pitch + x] = 0x10;
		return;
	}
	// the background's tiles: vramlayout (128, 3 bpp: planes 0, 0x2000, 0x4000; 4 pixels a byte's low nibble, 4
	// bytes of 16 rows each, rows from the bottom)
	for (int t = 0; t < 128; t++)
		for (int y = 0; y < 16; y++)
			for (int x = 0; x < 16; x++) {
				const int byte = t * 64 + (x >> 2) * 16 + (15 - y), bit = x & 3;
				uint8_t p = 0;
				for (int pl = 0; pl < 3; pl++)
					p = (uint8_t)(p << 1 | ((s_snap->gfx[pl * 0x2000 + byte] >> bit) & 1));
				s_bgpix[(t * 16 + y) * 16 + x] = p;
			}
	int scrollx = gc[1] + ((gc[0] & 1) << 8) + ((gc[0] & 4) << 7) - ((gc[0] & 2) ? 0 : 0x100);
	if (gc[1] == 0)
		scrollx += 0x100;
	scrollx += 2;
	const int scrolly = -gc[2];
	for (int y = 0; y < VIS_H; y++) {
		uint16_t *d = bm + y * pitch;
		const int py = tm_pos(y + VIS_Y0, scrolly, 256, flip);
		for (int x = 0; x < VIS_W; x++) {   // the background: 64x16 of 16x16, opaque, colour 2
			const int px = tm_pos(x, scrollx, 1024, flip);
			const int col = px >> 4, row = py >> 4;
			const int a = (7 - (row & 7)) + ((0x8 - (row & 0x8)) << 4) + ((col & 0xf) << 3) + ((col & 0x10) << 4) +
			              ((col & 0x20) << 4);
			const int t = s_snap->map[a & 0x3ff] & 0x7f;
			d[x] = (uint16_t)(2 * 8 + s_bgpix[(t * 16 + (py & 15)) * 16 + (px & 15)]);
		}
		const int qy = tm_pos(y + VIS_Y0, 0, 256, flip);
		for (int x = 0; x < VIS_W; x++) {   // the text: 32x32 of 8x8 by columns (rows reversed), pen 0 clear
			const int qx = tm_pos(x, flip ? -8 : 16, 256, flip);
			const int off = ((qy >> 3) ^ 0x1f) + ((qx >> 3) << 5), attr = s_snap->videoram[off + 0x400];
			const uint32_t code = s_gfx8.count ? (uint32_t)(s_snap->videoram[off] | ((attr & 3) << 8)) % s_gfx8.count : 0;
			const uint8_t p = s_gfx8.pix[(code * 8 + (uint32_t)(qy & 7)) * 8 + (uint32_t)(qx & 7)];
			if (p)
				d[x] = (uint16_t)((((attr & 4) >> 2) + 6) * 4 + p);
		}
	}
	for (int offs = 0; offs < 0x7f; offs += 4) {   // draw_sprites (in the list's order)
		const uint8_t *s = s_snap->spr0 + offs;
		if (!(s[0] & 1))
			continue;
		const uint32_t sprite = (uint32_t)(s[1] + ((s_snap->spr1[offs] & 7) << 8));
		int x = s[3] - 3, y = s[2], fx = (s[0] & 8) >> 3, fy = 0, inc = 16;
		if (flip) {
			x = 240 - x;
			fx = !fx;
			y = 240 - y;
			fy = 1;
			inc = -16;
		}
		if (s[0] & 0x10) {   // double height
			spr_blit(sprite, fx, fy, x, y + inc, bm, pitch);
			spr_blit(sprite + 1, fx, fy, x, y, bm, pitch);
		} else
			spr_blit(sprite, fx, fy, x, y, bm, pitch);
	}
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap->videoram, s_videoram, sizeof(s_videoram));
	memcpy(s_snap->spr0, s_spr0, sizeof(s_spr0));
	memcpy(s_snap->spr1, s_spr1, sizeof(s_spr1));
	memcpy(s_snap->map, s_vram, sizeof(s_snap->map));
	memcpy(s_snap->gfx, s_vram_gfx, sizeof(s_vram_gfx));
	memcpy(s_snap->gfx_control, s_gfx_control, sizeof(s_gfx_control));
	s_snap->flip = s_flip;
	rbx_video_frame_done();
}

// ---- init / frame ----
static void to_reset(void)
{
	memset(s_ram, 0, sizeof(s_ram));
	memset(s_videoram, 0, sizeof(s_videoram));
	memset(s_spr0, 0, sizeof(s_spr0));
	memset(s_spr1, 0, sizeof(s_spr1));
	memset(s_vram, 0, sizeof(s_vram));
	memset(s_vram_gfx, 0, sizeof(s_vram_gfx));
	memset(s_gfx_control, 0, sizeof(s_gfx_control));
	s_vram_bank = 0;
	s_flip = s_nmi = 0;
	s_coin_prev = 0x18;
	set_bank(0);
	m6502_reset(&s_cpu);
	m6502_set_irq(&s_cpu, 0);
	m6502_set_nmi(&s_cpu, 0);
	memset(s_sram, 0, sizeof(s_sram));
	s_latch = 0;
	m6502_reset(&s_snd);
	m6502_set_irq(&s_snd, 0);
	m6502_set_nmi(&s_snd, 0);
	rbx_ym_reset(0);
	s_mix_n = 0;
	s_line = 0;
	s_cpu_frac = s_snd_frac = s_ym_frac = s_tempo_frac = 0;
}

static void to_exit(void);

static int to_init(const rbx_game *g)
{
	uint32_t csize, ssize, psize;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_zrom = rbx_region("audiocpu", &s_zrom_size);
	uint8_t *chars = rbx_region("chars", &csize), *spr = rbx_region("sprites", &ssize), *prom = rbx_region("proms", &psize);
	if (!s_rom || s_rom_size < 0x14000 || !s_zrom || s_zrom_size < 0x10000 || !chars || csize < 0x4000 || !spr ||
	    ssize < 0x24000 || !prom || psize < 0x20) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	{	// charlayout (1024, 2 bpp in the nibbles, bit-reversed; the right half the region's second; rows reversed),
		// spritelayout (1536, 3 bpp: the thirds 2, 1, 0; bit-reversed; rows reversed)
		static uint32_t x8[8], y8[8], xs[16], ys[16];
		for (int i = 0; i < 4; i++) {
			x8[i] = (uint32_t)(3 - i);
			x8[4 + i] = (uint32_t)(0x2000 * 8 + 3 - i);
		}
		for (int i = 0; i < 8; i++) {
			y8[i] = (uint32_t)((7 - i) * 8);
			xs[i] = (uint32_t)(7 - i);
			xs[8 + i] = (uint32_t)(16 * 8 + 7 - i);
		}
		for (int i = 0; i < 16; i++)
			ys[i] = (uint32_t)((15 - i) * 8);
		static const uint32_t pc[2] = { 0, 4 }, ps[3] = { 0x18000 * 8, 0xc000 * 8, 0 };
		const rbx_gfx_layout l8 = { 8, 8, 2, pc, x8, y8, 8 * 8 }, ls = { 16, 16, 3, ps, xs, ys, 32 * 8 };
		if (rbx_gfx_decode(&s_gfx8, &l8, chars, 1024 * 8) || rbx_gfx_decode(&s_gfxspr, &ls, spr, 1536 * 32)) {
			rbx_fail("%s: out of memory (graphics)", g->set);
			to_exit();
			return -1;
		}
	}
	for (int i = 0; i < 32; i++) {   // palette: 3-3-2 (0x21, 0x47, 0x97; blue's low bit 0)
		const int c = prom[i];
		const int r = 0x21 * (c & 1) + 0x47 * ((c >> 1) & 1) + 0x97 * ((c >> 2) & 1);
		const int gg = 0x21 * ((c >> 3) & 1) + 0x47 * ((c >> 4) & 1) + 0x97 * ((c >> 5) & 1);
		const int b = 0x47 * ((c >> 6) & 1) + 0x97 * ((c >> 7) & 1);
		s_palette[i] = (uint32_t)(r << 16 | gg << 8 | b);
	}
	s_snap = calloc(1, sizeof(*s_snap));
	rbx_ym_config yc;   // (YM2203 1.5 MHz, all outputs 0.50, its IRQ not wired)
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM2203;
	yc.clock = YM_CLOCK;
	yc.gain[0] = yc.gain[1] = yc.gain[2] = yc.gain[3] = 128;
	int rc = !s_snap || rbx_ym_init(0, &yc) || rbx_ym_set_async(0, 1);
	if (!rc) {
		s_mix_cap = (int)(rbx_ym_rate(0) / 50) + 256;
		s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
		rc = !s_mix || rbx_audio_setup(rbx_ym_rate(0)) || rbx_video_setup(VIS_W, VIS_H, 32);
	}
	if (rc) {
		rbx_fail("%s: out of memory", g->set);
		to_exit();
		return -1;
	}
	m6502_init(&s_snd, M6502_NMOS);
	for (int p = 0; p < 8; p++)
		s_snd.rpage[p] = s_snd.wpage[p] = s_sram + p * 256;
	for (int p = 0xc0; p < 0x100; p++)
		s_snd.rpage[p] = s_zrom + p * 256;
	s_snd.read = snd_r;
	s_snd.write = snd_w;
	m6502_init(&s_cpu, M6502_NMOS);
	for (int p = 0; p < 8; p++) {
		s_cpu.rpage[p] = s_cpu.wpage[p] = s_ram + p * 256;
		s_cpu.rpage[0x10 + p] = s_cpu.wpage[0x10 + p] = s_videoram + p * 256;
	}
	for (int p = 0x40; p < 0xc0; p++)
		s_cpu.rpage[p] = s_rom + p * 256;
	s_cpu.rpage[0xc8] = s_cpu.wpage[0xc8] = s_spr0;
	s_cpu.rpage[0xcc] = s_cpu.wpage[0xcc] = s_spr1;
	s_cpu.read = cpu_r;
	s_cpu.write = cpu_w;
	s_port_dsw = rbx_port_find("DSW");
	s_port_p[0] = rbx_port_find("P1");
	s_port_p[1] = rbx_port_find("P2");
	s_port_sys = rbx_port_find("SYSTEM");
	to_reset();
	return 0;
}

static void to_exit(void)
{
	rbx_ym_exit(0);
	free(s_mix);
	s_mix = NULL;
	rbx_gfx_free(&s_gfx8);
	rbx_gfx_free(&s_gfxspr);
	free(s_snap);
	s_snap = NULL;
}

static void to_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	coin_update();
	for (int line = 0; line < LINES; line++) {
		s_line = line;
		if (line == VIS_Y0 + VIS_H)
			vblank_capture();
		s_cpu_frac += (uint64_t)CPU_CLOCK * 1000;
		const uint64_t den = (uint64_t)FRAME_MHZ * LINES;
		const int64_t cyc = (int64_t)(s_cpu_frac / den);
		s_cpu_frac -= (uint64_t)cyc * den;
		m6502_run_until(&s_cpu, s_cpu.cycles + cyc);
		s_snd_frac += (uint64_t)SND_CLOCK * 1000;
		const int64_t sc = (int64_t)(s_snd_frac / den);
		s_snd_frac -= (uint64_t)sc * den;
		m6502_run_until(&s_snd, s_snd.cycles + sc);
		s_tempo_frac += 1000ull * 1000;   // (a 1 kHz NMI pulse: the music's tempo)
		while (s_tempo_frac >= den) {
			s_tempo_frac -= den;
			m6502_set_nmi(&s_snd, 1);
			m6502_set_nmi(&s_snd, 0);
		}
		s_mix_n += rbx_ym_advance(0, (uint32_t)rbx_line_cycles(&s_ym_frac, YM_CLOCK, FRAME_MHZ, LINES), s_mix + s_mix_n,
		                          s_mix_cap - s_mix_n);
	}
	rbx_ym_async_take(0, s_mix, s_mix_n);
	for (int i = 0; i < s_mix_n; i++)
		rbx_audio_put(s_mix[i]);
	memset(s_mix, 0, sizeof(int32_t) * (size_t)s_mix_n);
	s_mix_n = 0;
}

static int to_state(void *ctx, int load)
{
	uint8_t cpu[M6502_STATE_SIZE], snd[M6502_STATE_SIZE];
	if (!load) {
		memcpy(cpu, &s_cpu, M6502_STATE_SIZE);
		memcpy(snd, &s_snd, M6502_STATE_SIZE);
	}
	uint64_t fr[4] = { s_cpu_frac, s_snd_frac, s_ym_frac, s_tempo_frac };
	int32_t misc[6] = { s_bank, s_flip, s_nmi, s_coin_prev, s_latch, s_vram_bank };
	int rc = 0;
	rc |= rbx_state_io(ctx, "m6502", cpu, M6502_STATE_SIZE);
	rc |= rbx_state_io(ctx, "ram", s_ram, sizeof(s_ram));
	rc |= rbx_state_io(ctx, "video", s_videoram, sizeof(s_videoram));
	rc |= rbx_state_io(ctx, "spr0", s_spr0, sizeof(s_spr0));
	rc |= rbx_state_io(ctx, "spr1", s_spr1, sizeof(s_spr1));
	rc |= rbx_state_io(ctx, "vram", s_vram, sizeof(s_vram));
	rc |= rbx_state_io(ctx, "vgfx", s_vram_gfx, sizeof(s_vram_gfx));
	rc |= rbx_state_io(ctx, "gfxc", s_gfx_control, sizeof(s_gfx_control));
	rc |= rbx_state_io(ctx, "m6502s", snd, M6502_STATE_SIZE);
	rc |= rbx_state_io(ctx, "sram", s_sram, sizeof(s_sram));
	rc |= rbx_ym_state(0, ctx, load);
	rc |= rbx_state_io(ctx, "frac", fr, sizeof(fr));
	rc |= rbx_state_io(ctx, "misc", misc, sizeof(misc));
	if (!rc && load) {
		memcpy(&s_cpu, cpu, M6502_STATE_SIZE);
		memcpy(&s_snd, snd, M6502_STATE_SIZE);
		s_cpu_frac = fr[0];
		s_snd_frac = fr[1];
		s_ym_frac = fr[2];
		s_tempo_frac = fr[3];
		set_bank(misc[0]);
		s_flip = misc[1];
		s_nmi = misc[2];
		s_coin_prev = misc[3];
		s_latch = (uint8_t)misc[4];
		s_vram_bank = (uint8_t)misc[5];
	}
	return rc ? -1 : 0;
}

static int to_save(void *ctx) { return to_state(ctx, 0); }
static int to_load(void *ctx) { return to_state(ctx, 1); }
static int to_supported(const char *machine) { return !strcmp(machine, "tryout"); }

const rbx_driver rbx_drv_tryout = { "DECO Tryout", to_init, to_exit, to_reset, to_frame, to_save, to_load,
                                    to_supported, to_render, NULL };
