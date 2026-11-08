/*  Data East Karnov / Chelnov / Wonder Planet. Ported from MAME src/mame/dataeast/karnov.cpp (BSD-3-Clause,
    copyright-holders: Bryan McPhail), decrmc3.cpp (the PROM palette).
    68000 at 10 MHz (IRQ 7 at vblank when enabled, IRQ 6 from the MCU), the i8751 (the command and return words, the
    coins on its INT0; MAME runs it in perfect sync: here interleaved by sixteenths of a line), a 16x16 background
    (scroll) and an 8x8 text layer, the Karnov sprites (their list buffered by a write), the palette from PROMs. The
    sound: 6502 at 1.5 MHz with YM2203 and YM3526 (on its IRQ), the latch on its NMI.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m68k_bus.h"
#include "../../cpu/m68k/m68k.h"
#include "../../cpu/m6502/m6502.h"
#include "../../cpu/mcs51/mcs51.h"
#include "../../sound/rbx_ym.h"
#include "../../video/gfxlayout.h"
#include "../video/deckarn.h"

#include <stdlib.h>
#include <string.h>

#define CPU_CLOCK 10000000       // 20 MHz / 2
#define SND_CLOCK 1500000        // 12 MHz / 8
#define MCU_CLOCK (8000000 / 12) // i8751 machine cycles
#define YM1_CLOCK 1500000        // YM2203
#define YM2_CLOCK 3000000        // YM3526
#define FRAME_MHZ 60000          // set_refresh_hz(60)
#define LINES 256                // set_size(32*8, 32*8)
#define VIS_Y0 8                 // set_visarea(0, 255, 8, 247)
#define VIS_H 240
#define VIS_W 256
#define VBL_LINES 38             // vblank 2500 us (into the next frame)
#define SLICES 16

static int s_wndrplnt;
static uint8_t *s_rom, *s_mcu_rom;
static uint32_t s_rom_size, s_mcu_size;
static uint16_t s_ram[0x2000];             // 0x060000 .. 0x063fff
static uint16_t s_spriteram[0x800], s_spriteram_buf[0x800];
static uint16_t s_tx_vram[0x400], s_bg_vram[0x400];
static uint16_t s_scroll[2];
static int s_vint_en;
static rbx_gfx s_gfx8, s_gfx16, s_gfxspr;
static uint32_t s_pal[1024];
static int s_port_p1p2, s_port_sys, s_port_coin, s_port_dsw;
static int s_line;
static uint32_t s_frame;
static uint64_t s_cpu_frac, s_snd_frac, s_mcu_frac, s_ym1_frac, s_ym2_frac;
static struct {
	uint16_t spriteram[0x800], tx[0x400], bg[0x400], scroll[2];
	uint32_t frame;
} *s_snap;

// ---- the i8751 ----
static mcs51_t s_mcu;
static uint8_t s_mcu_p0, s_mcu_p1, s_mcu_p2;
static uint16_t s_mcu_to_main, s_main_to_mcu;
static int s_coin_state;

static uint8_t mcu_in(void *u, int port)
{
	(void)u;
	switch (port) {
	case 0: return s_mcu_p0;
	case 1: return s_mcu_p1;
	case 3: return (uint8_t)rbx_port_read(s_port_coin);
	}
	return 0xff;
}
static void mcu_out(void *u, int port, uint8_t data)
{
	(void)u;
	if (port == 0) s_mcu_p0 = data;
	else if (port == 1) s_mcu_p1 = data;
	else if (port == 2) {   // mcu_p2_w: the falling edges
		const uint8_t fall = (uint8_t)(~data & s_mcu_p2);
		s_mcu_p2 = data;
		if (fall & 0x01) mcs51_set_input(&s_mcu, MCS51_INT0_LINE, 0);
		if (fall & 0x02) mcs51_set_input(&s_mcu, MCS51_INT1_LINE, 0);
		if (fall & 0x04) m68k_bus_set_line(6, 1);
		if (fall & 0x10) s_mcu_p0 = (uint8_t)(s_main_to_mcu & 0xff);
		if (fall & 0x20) s_mcu_p1 = (uint8_t)(s_main_to_mcu >> 8);
		if (fall & 0x40) s_mcu_to_main = (uint16_t)((s_mcu_to_main & 0xff00) | s_mcu_p0);
		if (fall & 0x80) s_mcu_to_main = (uint16_t)((s_mcu_to_main & 0x00ff) | (s_mcu_p1 << 8));
	}
}
// the coin inputs on an input merger (any low) to the MCU's INT0
static void coin_update(void)
{
	const int state = (rbx_port_read(s_port_coin) & 0xe0) != 0xe0;
	if (state && !s_coin_state)
		mcs51_set_input(&s_mcu, MCS51_INT0_LINE, 1);
	s_coin_state = state;
}

// ---- sound: 6502, YM2203, YM3526 ----
static m6502_t s_snd;
static uint8_t s_sram[0x800];
static uint8_t *s_zrom;
static uint32_t s_zrom_size;
static uint8_t s_latch;
static int32_t *s_mix, *s_ym2buf;
static int s_mix_n, s_mix_cap, s_ym2_n, s_ym2_cap;

static void ym2_irq(int state) { m6502_set_irq(&s_snd, state); }
static uint8_t snd_r(void *ctx, uint16_t a)
{
	(void)ctx;
	if (a < 0x0600) return s_sram[a];
	if (a == 0x0800) {
		m6502_set_nmi(&s_snd, 0);
		return s_latch;
	}
	if (a >= 0x8000) return a < s_zrom_size ? s_zrom[a] : 0;
	return 0;
}
static void snd_w(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	if (a < 0x0600) s_sram[a] = v;
	else if (a == 0x1000 || a == 0x1001) rbx_ym_write(0, a & 1, v);
	else if (a == 0x1800 || a == 0x1801) rbx_ym_write(1, a & 1, v);
}
static void sound_line(void)
{
	s_snd_frac += (uint64_t)SND_CLOCK * 1000;
	const uint64_t den = (uint64_t)FRAME_MHZ * LINES;
	const int64_t cyc = (int64_t)(s_snd_frac / den);
	s_snd_frac -= (uint64_t)cyc * den;
	m6502_run_until(&s_snd, s_snd.cycles + cyc);
	s_mix_n += rbx_ym_advance(0, (uint32_t)rbx_line_cycles(&s_ym1_frac, YM1_CLOCK, FRAME_MHZ, LINES), s_mix + s_mix_n,
	                          s_mix_cap - s_mix_n);
	s_ym2_n += rbx_ym_advance(1, (uint32_t)rbx_line_cycles(&s_ym2_frac, YM2_CLOCK, FRAME_MHZ, LINES), s_ym2buf + s_ym2_n,
	                          s_ym2_cap - s_ym2_n);
}
static void sound_frame_end(void)
{
	rbx_ym_async_take(0, s_mix, s_mix_n);
	rbx_ym_async_take(1, s_ym2buf, s_ym2_n);
	if (s_ym2_n > 0)
		for (int i = 0; i < s_mix_n; i++)
			s_mix[i] += s_ym2buf[(int)((int64_t)i * s_ym2_n / s_mix_n)];
	for (int i = 0; i < s_mix_n; i++)
		rbx_audio_put(s_mix[i]);
	memset(s_mix, 0, sizeof(int32_t) * (size_t)s_mix_n);
	memset(s_ym2buf, 0, sizeof(int32_t) * (size_t)s_ym2_n);
	s_mix_n = s_ym2_n = 0;
}

// ---- bus ----
static int vblank(void) { return ((s_line - (VIS_Y0 + VIS_H) + LINES) % LINES) < VBL_LINES; }
static void wmask(uint16_t *p, uint16_t v, uint16_t mask) { *p = (uint16_t)((*p & ~mask) | (v & mask)); }

static uint16_t main_read16(uint32_t a)
{
	a &= 0xffffff;
	switch (a) {
	case 0x0c0000: return (uint16_t)rbx_port_read(s_port_p1p2);
	case 0x0c0002:
		rbx_port_custom(s_port_sys, 0x80, vblank() ? 0x80 : 0);
		return (uint16_t)rbx_port_read(s_port_sys);
	case 0x0c0004: return (uint16_t)rbx_port_read(s_port_dsw);
	case 0x0c0006: return s_mcu_to_main;   // mcu_r
	}
	if (a >= 0x0a0000 && a < 0x0a0800) return s_tx_vram[(a - 0x0a0000) >> 1];
	return 0;
}
static uint8_t main_read8(uint32_t a)
{
	const uint16_t v = main_read16(a & ~1u);
	return (uint8_t)(a & 1 ? v : v >> 8);
}
static void main_write16m(uint32_t a, uint16_t v, uint16_t mask)
{
	a &= 0xffffff;
	if (a >= 0x0a0000 && a < 0x0a1000) wmask(&s_tx_vram[((a - 0x0a0000) >> 1) & 0x3ff], v, mask);   // (mirror 0x800)
	else if (a >= 0x0a1000 && a < 0x0a1800) wmask(&s_bg_vram[(a - 0x0a1000) >> 1], v, mask);
	else if (a >= 0x0a1800 && a < 0x0a2000) {   // bg_col_w: the column major view
		const uint32_t o = (a - 0x0a1800) >> 1;
		wmask(&s_bg_vram[((o & 0x1f) << 5) | ((o & 0x3e0) >> 5)], v, mask);
	} else if (a == 0x0c0000) m68k_bus_set_line(6, 0);   // mcu_ack_w
	else if (a == 0x0c0002) { if (mask & 0xff) { s_latch = (uint8_t)v; m6502_set_nmi(&s_snd, 1); } }
	else if (a == 0x0c0004) memcpy(s_spriteram_buf, s_spriteram, sizeof(s_spriteram_buf));
	else if (a == 0x0c0006) {   // mcu_w
		s_main_to_mcu = v;
		mcs51_set_input(&s_mcu, MCS51_INT1_LINE, 1);
	} else if (a >= 0x0c0008 && a < 0x0c000c) wmask(&s_scroll[(a >> 1) & 1], v, mask);
	else if (a >= 0x0c000c && a < 0x0c0010) {   // vintctl_w (any write clears the line)
		s_vint_en = (int)((a >> 1) & 1);
		m68k_bus_set_line(7, 0);
	}
}
static void main_write16(uint32_t a, uint16_t v) { main_write16m(a, v, 0xffff); }
static void main_write8(uint32_t a, uint8_t v)
{
	const int sh = a & 1 ? 0 : 8;
	main_write16m(a & ~1u, (uint16_t)(v << sh), (uint16_t)(0xff << sh));
}

// ---- video ----
static void kn_render(void)
{
	memcpy(rbx_video.palette, s_pal, sizeof(s_pal));
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	const int flip = (s_snap->scroll[0] >> 15) & 1;
	// the background: 32x32 tiles of 16x16 by rows, opaque, scrolled (the flip turns the picture round)
	const int sx0 = s_snap->scroll[0] & 0x1ff, sy0 = s_snap->scroll[1] & 0x1ff;
	for (int y = 0; y < VIS_H; y++) {
		uint16_t *d = bm + y * pitch;
		for (int x = 0; x < VIS_W; x++) {
			int px = (x + sx0) & 0x1ff, py = (y + VIS_Y0 + sy0) & 0x1ff;
			if (flip) {   // (MAME tilemap flip: the scroll taken on the turned picture)
				px = (511 - ((x + sx0) & 0x1ff));
				py = (511 - ((y + VIS_Y0 + sy0) & 0x1ff));
			}
			const uint16_t t = s_snap->bg[(py >> 4) * 32 + (px >> 4)];
			const uint32_t code = s_gfx16.count ? (t & 0x7ffu) % s_gfx16.count : 0;
			const uint8_t p = s_gfx16.pix[(code * 16 + (uint32_t)(py & 15)) * 16 + (uint32_t)(px & 15)];
			d[x] = (uint16_t)(512 + (t >> 12) * 16 + p);
		}
	}
	deckarn_draw(&s_gfxspr, 256, s_snap->spriteram, 0x800, flip, bm, pitch, VIS_W, VIS_H, VIS_Y0);
	// the text layer: 32x32 of 8x8 (Wonder Planet: by columns), 3 bpp, colour 2 bits, pen 0 clear
	for (int y = 0; y < VIS_H; y++) {
		uint16_t *d = bm + y * pitch;
		for (int x = 0; x < VIS_W; x++) {
			int px = x, py = y + VIS_Y0;
			if (flip) {
				px = 255 - px;
				py = 255 - py;
			}
			const int col = px >> 3, row = py >> 3;
			const uint16_t t = s_snap->tx[s_wndrplnt ? col * 32 + row : row * 32 + col];
			const uint32_t code = s_gfx8.count ? (t & 0xfffu) % s_gfx8.count : 0;
			const uint8_t p = s_gfx8.pix[(code * 8 + (uint32_t)(py & 7)) * 8 + (uint32_t)(px & 7)];
			if (p)
				d[x] = (uint16_t)((t >> 14) * 8 + p);
		}
	}
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap->spriteram, s_spriteram_buf, sizeof(s_spriteram_buf));
	memcpy(s_snap->tx, s_tx_vram, sizeof(s_tx_vram));
	memcpy(s_snap->bg, s_bg_vram, sizeof(s_bg_vram));
	memcpy(s_snap->scroll, s_scroll, sizeof(s_scroll));
	s_snap->frame = s_frame;
	rbx_video_frame_done();
}

// ---- init / frame ----
static void kn_reset(void)
{
	memset(s_ram, 0, sizeof(s_ram));   // (Chelnov likes the RAM cleared on reset)
	memset(s_spriteram, 0, sizeof(s_spriteram));
	memset(s_spriteram_buf, 0, sizeof(s_spriteram_buf));
	memset(s_tx_vram, 0, sizeof(s_tx_vram));
	memset(s_bg_vram, 0, sizeof(s_bg_vram));
	s_scroll[0] = s_scroll[1] = 0;
	s_vint_en = 0;
	s_mcu_p0 = s_mcu_p1 = s_mcu_p2 = 0;
	s_mcu_to_main = s_main_to_mcu = 0;
	s_coin_state = 0;
	mcs51_reset(&s_mcu);
	memset(s_sram, 0, sizeof(s_sram));
	s_latch = 0;
	m6502_reset(&s_snd);
	m6502_set_irq(&s_snd, 0);
	m6502_set_nmi(&s_snd, 0);
	rbx_ym_reset(0);
	rbx_ym_reset(1);
	s_mix_n = s_ym2_n = 0;
	m68k_bus_irq_reset();
	m68k_pulse_reset();
	s_line = 0;
	s_frame = 0;
	s_cpu_frac = s_snd_frac = s_mcu_frac = s_ym1_frac = s_ym2_frac = 0;
}

static void kn_exit(void);

static int kn_init(const rbx_game *g)
{
	s_wndrplnt = !strcmp(g->machine, "wndrplnt");
	uint32_t csize, tsize, ssize, psize;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_zrom = rbx_region("audiocpu", &s_zrom_size);
	s_mcu_rom = rbx_region("mcu", &s_mcu_size);
	uint8_t *chars = rbx_region("char", &csize), *tiles = rbx_region("tiles", &tsize),
	        *spr = rbx_region("sprites", &ssize), *proms = rbx_region("proms", &psize);
	if (!s_rom || !s_zrom || !s_mcu_rom || s_mcu_size < 0x1000 || !chars || !tiles || !spr || !proms || psize < 0x800) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	{	// chars (3 bpp, planes the quarters 3, 2, 1), tiles (planes 3, 0, 1, 2 quarters)
		static uint32_t x8[8], y8[8], x16[16], y16[16];
		for (int i = 0; i < 8; i++) {
			x8[i] = (uint32_t)i;
			y8[i] = (uint32_t)(i * 8);
			x16[i] = (uint32_t)(16 * 8 + i);
			x16[8 + i] = (uint32_t)i;
		}
		for (int i = 0; i < 16; i++)
			y16[i] = (uint32_t)(i * 8);
		const uint32_t qc = csize * 2, qt = tsize * 2, qs = ssize * 2;
		const uint32_t pc[3] = { 3 * qc, 2 * qc, qc };
		const uint32_t pt[4] = { 3 * qt, 0, qt, 2 * qt }, ps[4] = { 3 * qs, 0, qs, 2 * qs };
		const rbx_gfx_layout l8 = { 8, 8, 3, pc, x8, y8, 8 * 8 };
		const rbx_gfx_layout lt = { 16, 16, 4, pt, x16, y16, 16 * 16 }, ls = { 16, 16, 4, ps, x16, y16, 16 * 16 };
		if (rbx_gfx_decode(&s_gfx8, &l8, chars, csize / 4) || rbx_gfx_decode(&s_gfx16, &lt, tiles, tsize / 4) ||
		    rbx_gfx_decode(&s_gfxspr, &ls, spr, ssize / 4)) {
			rbx_fail("%s: out of memory (graphics)", g->set);
			kn_exit();
			return -1;
		}
	}
	for (int i = 0; i < 1024; i++) {   // deco_rmc3 palette_init_proms (the weighted resistor ladder)
		const uint8_t r4 = proms[i] & 0xf, g4 = (proms[i] >> 4) & 0xf, b4 = proms[i + 1024] & 0xf;
#define W(v) (0x0e * ((v) & 1) + 0x1f * (((v) >> 1) & 1) + 0x43 * (((v) >> 2) & 1) + 0x8f * (((v) >> 3) & 1))
		s_pal[i] = (uint32_t)(W(r4) << 16 | W(g4) << 8 | W(b4));
#undef W
	}
	mcs51_init(&s_mcu, s_mcu_rom, 0x1000);
	s_mcu.port_in = mcu_in;
	s_mcu.port_out = mcu_out;
	s_snap = calloc(1, sizeof(*s_snap));
	// sound: YM2203 1.5 MHz 0.25, YM3526 3 MHz 1.0
	rbx_ym_config yc;
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM2203;
	yc.clock = YM1_CLOCK;
	yc.gain[0] = yc.gain[1] = yc.gain[2] = yc.gain[3] = 64;
	int rc = !s_snap || rbx_ym_init(0, &yc) || rbx_ym_set_async(0, 1);
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM3526;
	yc.clock = YM2_CLOCK;
	yc.gain[0] = 256;
	yc.irq = ym2_irq;
	rc = rc || rbx_ym_init(1, &yc) || rbx_ym_set_async(1, 1);
	if (!rc) {
		s_mix_cap = (int)(rbx_ym_rate(0) / 50) + 256;
		s_ym2_cap = (int)(rbx_ym_rate(1) / 50) + 256;
		s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
		s_ym2buf = (int32_t *)calloc((size_t)s_ym2_cap, sizeof(int32_t));
		rc = !s_mix || !s_ym2buf || rbx_audio_setup(rbx_ym_rate(0)) || rbx_video_setup(VIS_W, VIS_H, 1024);
	}
	if (rc) {
		rbx_fail("%s: out of memory", g->set);
		kn_exit();
		return -1;
	}
	m6502_init(&s_snd, M6502_NMOS);
	for (int p = 0; p < 6; p++)
		s_snd.rpage[p] = s_snd.wpage[p] = s_sram + p * 256;
	for (int p = 0x80; p < 0x100; p++)
		if ((uint32_t)p * 256 + 256 <= s_zrom_size)
			s_snd.rpage[p] = s_zrom + p * 256;
	s_snd.read = snd_r;
	s_snd.write = snd_w;
	s_port_p1p2 = rbx_port_find("P1_P2");
	s_port_sys = rbx_port_find("SYSTEM");
	s_port_coin = rbx_port_find("COIN");
	s_port_dsw = rbx_port_find("DSW");
	m68k_bus_swap16(s_rom, s_rom_size);
	m68k_init();
	m68k_set_cpu_type(M68K_CPU_TYPE_68000);
	m68k_bus_reset_map();
	m68k_bus_map(0, 0x5ffff, s_rom, s_rom_size, 1, 0);
	m68k_bus_map(0x060000, 0x063fff, s_ram, sizeof(s_ram), 1, 1);
	m68k_bus_map(0x080000, 0x080fff, s_spriteram, sizeof(s_spriteram), 1, 1);
	m68k_bus.read8 = main_read8;
	m68k_bus.read16 = main_read16;
	m68k_bus.write8 = main_write8;
	m68k_bus.write16 = main_write16;
	kn_reset();
	return 0;
}

static void kn_exit(void)
{
	rbx_ym_exit(0);
	rbx_ym_exit(1);
	free(s_mix);
	free(s_ym2buf);
	s_mix = s_ym2buf = NULL;
	rbx_gfx_free(&s_gfx8);
	rbx_gfx_free(&s_gfx16);
	rbx_gfx_free(&s_gfxspr);
	free(s_snap);
	s_snap = NULL;
}

static void kn_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	coin_update();
	for (int line = 0; line < LINES; line++) {
		s_line = line;
		if (line == VIS_Y0 + VIS_H) {   // screen_vblank: IRQ 7 when enabled
			vblank_capture();
			if (s_vint_en)
				m68k_bus_set_line(7, 1);
		}
		const int cyc = rbx_line_cycles(&s_cpu_frac, CPU_CLOCK, FRAME_MHZ, LINES);
		const int mc = rbx_line_cycles(&s_mcu_frac, MCU_CLOCK, FRAME_MHZ, LINES);
		for (int k = 0; k < SLICES; k++) {   // (MAME: perfect sync between the 68000 and the MCU)
			m68k_execute(cyc * (k + 1) / SLICES - cyc * k / SLICES);
			mcs51_execute(&s_mcu, mc * (k + 1) / SLICES - mc * k / SLICES);
		}
		sound_line();
	}
	sound_frame_end();
	s_frame++;
}

static int kn_state(void *ctx, int load)
{
	const uint32_t csize = m68k_context_size();
	uint8_t *cpu = (uint8_t *)malloc(csize);
	if (!cpu)
		return -1;
	if (!load)
		m68k_get_context(cpu);
	uint32_t irq[2] = { m68k_bus_irq_pending(), m68k_bus_cur->irq_asserted };
	uint64_t fr[5] = { s_cpu_frac, s_snd_frac, s_mcu_frac, s_ym1_frac, s_ym2_frac };
	int32_t misc[10] = { (int32_t)s_frame, s_vint_en, s_mcu_p0, s_mcu_p1, s_mcu_p2, s_mcu_to_main, s_main_to_mcu,
	                     s_coin_state, s_latch, 0 };
	uint8_t snd[M6502_STATE_SIZE];
	if (!load)
		memcpy(snd, &s_snd, M6502_STATE_SIZE);
	int rc = 0;
	rc |= rbx_state_io(ctx, "m68k", cpu, csize);
	rc |= rbx_state_io(ctx, "irq", irq, sizeof(irq));
	rc |= rbx_state_io(ctx, "ram", s_ram, sizeof(s_ram));
	rc |= rbx_state_io(ctx, "spr", s_spriteram, sizeof(s_spriteram));
	rc |= rbx_state_io(ctx, "sprb", s_spriteram_buf, sizeof(s_spriteram_buf));
	rc |= rbx_state_io(ctx, "tx", s_tx_vram, sizeof(s_tx_vram));
	rc |= rbx_state_io(ctx, "bg", s_bg_vram, sizeof(s_bg_vram));
	rc |= rbx_state_io(ctx, "scroll", s_scroll, sizeof(s_scroll));
	rc |= rbx_state_io(ctx, "mcu", &s_mcu.st, sizeof(s_mcu.st));
	rc |= rbx_state_io(ctx, "m6502", snd, M6502_STATE_SIZE);
	rc |= rbx_state_io(ctx, "sram", s_sram, sizeof(s_sram));
	rc |= rbx_ym_state(0, ctx, load);
	rc |= rbx_ym_state(1, ctx, load);
	rc |= rbx_state_io(ctx, "frac", fr, sizeof(fr));
	rc |= rbx_state_io(ctx, "misc", misc, sizeof(misc));
	if (!rc && load) {
		m68k_set_context(cpu);
		m68k_bus_set_irq_pending(irq[0]);
		m68k_bus_cur->irq_asserted = irq[1];
		m68k_bus_update_irq();
		memcpy(&s_snd, snd, M6502_STATE_SIZE);
		s_cpu_frac = fr[0];
		s_snd_frac = fr[1];
		s_mcu_frac = fr[2];
		s_ym1_frac = fr[3];
		s_ym2_frac = fr[4];
		s_frame = (uint32_t)misc[0];
		s_vint_en = misc[1];
		s_mcu_p0 = (uint8_t)misc[2];
		s_mcu_p1 = (uint8_t)misc[3];
		s_mcu_p2 = (uint8_t)misc[4];
		s_mcu_to_main = (uint16_t)misc[5];
		s_main_to_mcu = (uint16_t)misc[6];
		s_coin_state = misc[7];
		s_latch = (uint8_t)misc[8];
	}
	free(cpu);
	return rc ? -1 : 0;
}

static int kn_save(void *ctx) { return kn_state(ctx, 0); }
static int kn_load(void *ctx) { return kn_state(ctx, 1); }
static int kn_supported(const char *machine) { return !strcmp(machine, "karnov") || !strcmp(machine, "wndrplnt"); }

const rbx_driver rbx_drv_karnov = { "DECO Karnov", kn_init, kn_exit, kn_reset, kn_frame, kn_save, kn_load,
                                    kn_supported, kn_render, NULL };
